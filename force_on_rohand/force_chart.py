import os
import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
import math
import sys

from queue import Queue
from collections import deque
from matplotlib.ticker import MultipleLocator
from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from serial.tools import list_ports

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from common.roh_registers_v2 import *
from common.heat_map_dot import *

# Force configuration
NUM_FINGERS = 6
NODE_ID = 2
PALM_INDEX = 5

ROH_HARDWARE_TYPE_AP = 0x2001
TACS_3D_FORCE = 1
TACS_DOT_MATRIX = 0

FORCE_TYPE = TACS_DOT_MATRIX

# global variable
data_queue = Queue(maxsize=5)
_force_point_location = None
_force_img = None
heatmap = None
_height, _width = 0, 0


def interpolate(n, from_min, from_max, to_min, to_max):
    return (n - from_min) / (from_max - from_min) * (to_max - to_min) + to_min


def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


def find_comport(port_name):
    """
    Find available serial port automatically
    :param port_name: Characterization of the port description, such as "CH340"
    :return: Comport of device if successful, None otherwise
    """
    ports = list_ports.comports()
    for port in ports:
        if port_name in port.description:
            return port.device
    return None


def write_registers(client, address, values):
    """
    Write data to Modbus device.
    :param client: Modbus client instance
    :param address: Register address
    :param values: Data to be written
    :return: True if successful, False otherwise
    """
    try:
        resp = client.write_registers(address, values, NODE_ID)
        if resp.isError():
            print("client.write_registers() returned", resp)
            return False
        return True
    except ModbusException as e:
        print("ModbusException:{0}".format(e))
        return False


def read_registers(client, address, count):
    """
    Read data from Modbus device.
    :param client: Modbus client instance
    :param address: Register address
    :param count: Register count to be read
    :return: List of registers if successful, None otherwise
    """
    try:
        resp = client.read_holding_registers(address, count, NODE_ID)
        if resp.isError():
            return None
        return resp.registers
    except ModbusException as e:
        print("ModbusException:{0}".format(e))
        return None


def img_init(hand_type, heatmap_dot):
    """
    Init force image.
    :param hand_type: left hand(0), right hand(1), Used to find the corresponding image
    :param heatmap_dot: HeatMapDot instance, that contains force sensor configuration information
    """
    global _force_point_location
    global _force_img
    file_path = os.path.abspath(os.path.dirname(__file__))

    cv2.namedWindow("Heatmap", cv2.WINDOW_NORMAL)  # Create a window with adjustable size

    if int(hand_type) == 0:
        pic_path = "/pic/force_left.png"
        _force_point_location = heatmap_dot.LEFT_FORCE_POINT
    else:
        pic_path = "/pic/force_right.png"
        _force_point_location = heatmap_dot.RIGHT_FORCE_POINT

    image_path = file_path + pic_path
    _force_img = cv2.imread(image_path)

    if _force_img is None:
        raise ValueError("Failed to load image, please check the path")
    else:
        return _force_img


def data_reading(client, heatmap_dot):
    """
    Independent thread: Continuously read force sensor data and place it in the queue
    :param client: Modbus client instance
    :param heatmap_dot: HeatMapDot instance, that contains force sensor configuration information
    """
    try:
        finger_force = [[] for _ in range(NUM_FINGERS)]
        finger_force_sum = [0 for _ in range(NUM_FINGERS)]
        # Fingers force data acquisition
        for i in range(NUM_FINGERS - 1):
            reg_cnt = heatmap_dot.FORCE_VALUE_LENGTH[i]
            resp = read_registers(client, ROH_FINGER_FORCE_EX0 + i * FORCE_GROUP_SIZE, reg_cnt)

            if resp is not None and len(resp) == reg_cnt:
                val = []

                if heatmap_dot.SENSOR_TYPE == TACS_3D_FORCE:
                    for j in range(reg_cnt):
                        val = ((resp[j] & 0xFF) << 8) | ((resp[j] >> 8) & 0xFF)
                        # Avoid invalid data
                        if val < 65535:
                            finger_force[i].append(val)
                        else:
                            finger_force[i].append(0)
                    # print(finger_force)
                    finger_force_sum[i] = math.sqrt(finger_force[i][0] ** 2 + finger_force[i][1] ** 2)
                    # print(finger_force_sum[i])
                    if reg_cnt >= 6:
                        finger_force_sum[i] += math.sqrt(finger_force[i][3] ** 2 + finger_force[i][4] ** 2)
                elif heatmap_dot.SENSOR_TYPE == TACS_DOT_MATRIX:
                    for j in range(reg_cnt):
                        val.append((resp[j] >> 8) & 0xFF)
                        val.append(resp[j] & 0xFF)
                        finger_force_sum[i] += val[j]
                    # print(val)
                    finger_force[i] = val
        # Palm force data acquisition
        reg_cnt = heatmap_dot.FORCE_VALUE_LENGTH[PALM_INDEX]
        resp = read_registers(client, ROH_FINGER_FORCE_EX0 + PALM_INDEX * FORCE_GROUP_SIZE, reg_cnt)

        if resp is not None and len(resp) == reg_cnt:
            val = []
            for i in range(reg_cnt):
                val.append((resp[i] >> 8) & 0xFF)
                val.append(resp[i] & 0xFF)
                finger_force_sum[PALM_INDEX] += val[i]
            # print(val)
            finger_force[PALM_INDEX] = val

        if not data_queue.full():
            data_queue.put((finger_force, finger_force_sum))

    except Exception as e:
        print(f"Data reading thread err: {e}")


def curve_init(heatmap_dot):
    """
    Init force curve chart.
    :return: None
    """
    global _data_queues, _x_data, _fig, _ax, _lines, _annotations, _finger_labels
    limit_max = heatmap_dot.MAX_FORCE_SUM  # Max force value
    limit_min = 0
    line_width = 2

    plt.ion()
    _fig, _ax = plt.subplots(1, 1, figsize=(10, 6), sharex=True)
    _fig.subplots_adjust(bottom=0.15)
    _data_queues = [deque(maxlen=10) for _ in range(NUM_FINGERS)]
    _x_data = deque(maxlen=10)  # Store the last 100 timestamps

    _lines = []
    _annotations = []
    _finger_labels = ["Thumb", "Index", "Middle", "Ring", "Little", "Palm"]
    colors = ["r-", "b-", "g-", "c-", "m-", "y-"]

    for i in range(NUM_FINGERS):
        (line,) = _ax.plot([], [], colors[i], label=f"{_finger_labels[i]}", linewidth=line_width)
        _lines.append(line)

        # Set the initial position of the annotation to (0,0), do not specify transform, use the data coordinate system
        annotation = _ax.text(0, 0, "", ha="left", va="center", bbox=dict(facecolor="white", pad=0.7), fontsize=10)
        _annotations.append(annotation)

    _ax.set_ylabel("Force Value")
    _ax.set_xlabel("Time/s")
    _ax.set_ylim(limit_min, limit_max)
    _ax.yaxis.set_major_locator(MultipleLocator(limit_max / 5))
    _ax.grid(True)
    _ax.legend(loc="upper right")
    _ax.set_title("Real-time Force Data")
    _fig.show()


def update_heatmap(finger_force, heatmap_dot):
    """
    Update the heat map based on the force sensor data, and display it in fusion with the hand reference image
    :param finger_force: Multidimensional List.Store the force sensor data of each finger
    :param heatmap_dot: HeatMapDot instance, that contains force sensor configuration information
    """
    # Clear heat map
    heatmap = np.zeros((_height, _width), dtype=np.uint8)
    # Show heat map
    for finger_id, force_points in enumerate(_force_point_location):
        data = finger_force[finger_id] if finger_id < len(finger_force) else []

        for dot_index in range(len(force_points)):
            x, y = force_points[dot_index]

            if heatmap_dot.SENSOR_TYPE == TACS_3D_FORCE:
                if 0 <= x < _width and 0 <= y < _height and finger_id != 5:
                    # nf
                    value = data[dot_index * 3] * heatmap_dot.COLOR_SCALE
                    radius = heatmap_dot.POINT_RADIUS + round(interpolate(value, 0, heatmap_dot.MAX_FORCE, 0, 10))
                    color = interpolate(value, 0, heatmap_dot.MAX_FORCE, 120, 1)
                    color = clamp(color, 1, 120)
                    cv2.circle(heatmap, (x, y), radius, color, -1)

                    # tf
                    value = data[dot_index * 3 + 1] * heatmap_dot.COLOR_SCALE
                    length = round(interpolate(value, 0, heatmap_dot.MAX_FORCE, 0, 100))
                    color = interpolate(value, 0, heatmap_dot.MAX_FORCE, 120, 1)
                    color = clamp(color, 1, 120)

                    # dir
                    value = data[dot_index * 3 + 2] - 90
                    value = math.radians(value)  # convert to radians
                    arrowStart = (x, y)
                    arrowEnd = (
                        x + int(length * math.cos(value) * heatmap_dot.ARROW_SCALE),
                        y + int(length * math.sin(value) * heatmap_dot.ARROW_SCALE)
                    )
                    cv2.arrowedLine(heatmap, arrowStart, arrowEnd, color, 5, tipLength=0.3)

            elif heatmap_dot.SENSOR_TYPE == TACS_DOT_MATRIX:
                if 0 <= x < _width and 0 <= y < _height:
                    if dot_index < len(data):
                        value = data[dot_index] * heatmap_dot.COLOR_SCALE
                    else:
                        value = 0
                    color = interpolate(value, 0, heatmap_dot.MAX_FORCE, 120, 1)
                    color = clamp(color, 1, 120)
                    if finger_id == 5:
                        cv2.circle(heatmap, (x, y), heatmap_dot.PALM_POINT_RADIUS, color, -1)
                    else:
                        cv2.circle(heatmap, (x, y), heatmap_dot.POINT_RADIUS, color, -1)

    heatmap_colored = cv2.applyColorMap(heatmap, cv2.COLORMAP_HSV)
    heatmap_colored = cv2.resize(heatmap_colored, (_force_img.shape[1], _force_img.shape[0]))
    mask = heatmap > 0
    mask = np.uint8(mask * 255)

    if _force_img is not None and heatmap_colored is not None:
        result = _force_img.copy()
        try:
            result[mask > 0] = cv2.addWeighted(_force_img[mask > 0], 0.2, heatmap_colored[mask > 0], 0.8, 0)
        except Exception as e:
            print(f"Error when blending images: {e}")
        else:
            cv2.imshow("Heatmap", result)


def update_curve(finger_force_sum):
    """
    Real time update of force data curve chart, displaying the trend of force value changes for each finger
    """
    for i in range(NUM_FINGERS):
        _data_queues[i].append(finger_force_sum[i])
    # Update graphics
    for i in range(NUM_FINGERS):
        if _data_queues[i]:  # Update numerical labels
            _lines[i].set_data(_x_data, _data_queues[i])
            # get latest point position
            x = _x_data[-1]
            y = _data_queues[i][-1]
            # Annotations displayed at the end of the curve
            _annotations[i].set_position((x, y))
            _annotations[i].set_text(f"{_finger_labels[i]}: {y:.1f}")
            # update labels
            _lines[i].set_label(f"{_finger_labels[i]} Value: {y:.1f}")

    _ax.relim()
    _ax.autoscale_view(scalex=True, scaley=False)

    # Refresh view
    _ax.legend(loc="upper right")

    _fig.canvas.draw()
    _fig.canvas.flush_events()


def main():
    global _width, _height

    # sub_model = 0
    heatmap_dot = HeatMapDot(FORCE_TYPE)
    heatmap_dot.init_dot_info()

    start_time = time.time()
    hand_type = input("select left hand(0) or right hand(1)")

    client = ModbusSerialClient(find_comport("CH340") or find_comport("USB"), FramerType.RTU, 115200)
    if not client.connect():
        print("Failed to connect to Modbus device")
        exit(-1)

    resp = read_registers(client, ROH_HW_VERSION, 1)

    if resp is None or resp[0] != ROH_HARDWARE_TYPE_AP:
        print("Failed to read hardware version or unsupported hardware type")
        exit(-1)

    img_init(hand_type, heatmap_dot)  # choose left hand
    curve_init(heatmap_dot)
    # Create a blank heatmap (single channel)
    _height, _width = _force_img.shape[:2]

    if _force_img is None:
        print("Error: Image initialization failed, unable to continue running")
        exit(-1)

    if not write_registers(client, ROH_RESET_FORCE, 1):
        print("Failed to reset force")

    while True:
        data_reading(client, heatmap_dot)
        finger_force, finger_force_sum = [], []

        if not data_queue.empty():
            finger_force, finger_force_sum = data_queue.get()

        # heatmap
        if finger_force is None:
            continue
        update_heatmap(finger_force, heatmap_dot)

        # curve
        current_time = time.time() - start_time
        _x_data.append(current_time)
        update_curve(finger_force_sum)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    plt.close()
    client.close()


if __name__ == "__main__":
    main()
