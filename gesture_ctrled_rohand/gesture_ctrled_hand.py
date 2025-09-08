import os
import cv2
import numpy as np
import sys
import math
import queue
import threading

from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from serial.tools import list_ports
from queue import Queue

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from common.roh_registers_v2 import *
from common.heat_map_dot import *
from HandTrackingModule import HandDetector

# Hand configuration
NODE_ID = 2
NUM_FINGERS = 6
THUMB_ROOT_ID = 5
PALM_INDEX = 5

# Rohand hardware type which supports force feedback
ROH_HARDWARE_TYPE_AP = 0x2001
# Force setting
TACS_3D_FORCE = 1
TACS_DOT_MATRIX = 0
FORCE_TYPE = TACS_DOT_MATRIX
FORCE_TARGET = 200

# global variable
file_path = os.path.abspath(os.path.dirname(__file__))
data_queue = Queue(maxsize=5)
gesture_queue = queue.Queue(maxsize=NUM_FINGERS)
image_queue = queue.Queue(maxsize=1)
_video = None
_detector = None


def interpolate(n, from_min, from_max, to_min, to_max):
    return (n - from_min) / (from_max - from_min) * (to_max - to_min) + to_min


def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


def img_init(hand_type, heatmap_dot):
    """
    Init force image.
    :param hand_type: 0 for left hand, other for right hand
    :param heatmap_dot: HeatMapDot instance, that contains force sensor configuration information
    """
    global _force_img, _force_point_location, _height, _width
    # Create a window with adjustable size
    cv2.namedWindow("Heatmap", cv2.WINDOW_NORMAL)

    if hand_type == 0:
        pic_path = "/gestures/force_left.png"
        _force_point_location = heatmap_dot.LEFT_FORCE_POINT
    else:
        pic_path = "/gestures/force_right.png"
        _force_point_location = heatmap_dot.RIGHT_FORCE_POINT

    image_path = file_path + pic_path
    _force_img = cv2.imread(image_path)

    if _force_img is None:
        raise ValueError(
            "Failed to load image, please check the path"
        )

    _height, _width = _force_img.shape[:2]


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


def get_finger_force(client, heatmap_dot):
    """
    Continuously read force sensor data and place it in the queue
    :param client: Modbus client instance
    :param heatmap_dot: HeatMapDot instance, that contains force sensor configuration information
    """
    try:
        finger_force = [[] for _ in range(NUM_FINGERS)]
        finger_force_sum = [0 for _ in range(NUM_FINGERS)]
        # Fingers force data acquisition
        for i in range(NUM_FINGERS - 1):
            reg_cnt = heatmap_dot.FORCE_VALUE_LENGTH[i]
            resp = read_registers(
                client, ROH_FINGER_FORCE_EX0 + i * FORCE_GROUP_SIZE, reg_cnt
            )

            if resp is not None and len(resp) == reg_cnt:
                val = []

                if heatmap_dot.SENSOR_TYPE == TACS_3D_FORCE:
                    for j in range(reg_cnt):
                        val = ((resp[j] & 0xFF) << 8) | (
                            (resp[j] >> 8) & 0xFF
                        )

                        # Avoid invalid data
                        if val < 65535:
                            finger_force[i].append(val)
                        else:
                            finger_force[i].append(0)
                    # print(finger_force)
                    finger_force_sum[i] = math.sqrt(
                        finger_force[i][0] ** 2 + finger_force[i][1] ** 2
                    )

                    # print(finger_force_sum[i])
                    if reg_cnt >= 6:
                        finger_force_sum[i] += math.sqrt(
                            finger_force[i][3] ** 2 + finger_force[i][4] ** 2
                        )
                elif heatmap_dot.SENSOR_TYPE == TACS_DOT_MATRIX:
                    for j in range(reg_cnt):
                        val.append((resp[j] >> 8) & 0xFF)
                        val.append(resp[j] & 0xFF)
                        finger_force_sum[i] += val[j]
                    # print(val)
                    finger_force[i] = val
        # Palm force data acquisition
        reg_cnt = heatmap_dot.FORCE_VALUE_LENGTH[PALM_INDEX]
        resp = read_registers(
            client, ROH_FINGER_FORCE_EX0 + PALM_INDEX * FORCE_GROUP_SIZE, reg_cnt
        )

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
                    value = data[dot_index * 3]
                    radius = heatmap_dot.POINT_RADIUS + round(
                        interpolate(value, 0, heatmap_dot.MAX_FORCE, 0, 10)
                    )
                    color = interpolate(value, 0, heatmap_dot.MAX_FORCE, 120, 1)
                    color = clamp(color, 1, 120)
                    cv2.circle(heatmap, (x, y), radius, color, -1)

                    # tf
                    value = data[dot_index * 3 + 1]
                    length = round(interpolate(value, 0, heatmap_dot.MAX_FORCE, 0, 100))
                    color = interpolate(value, 0, heatmap_dot.MAX_FORCE, 120, 1)
                    color = clamp(color, 1, 120)

                    # dir
                    value = data[dot_index * 3 + 2] - 90
                    value = math.radians(value)  # convert to radians
                    arrowStart = (x, y)
                    arrowEnd = (
                        x + int(length * math.cos(value)),
                        y + int(length * math.sin(value)),
                    )
                    cv2.arrowedLine(
                        heatmap, arrowStart, arrowEnd, color, 5, tipLength=0.3
                    )

            elif heatmap_dot.SENSOR_TYPE == TACS_DOT_MATRIX:
                if 0 <= x < _width and 0 <= y < _height:
                    if dot_index < len(data):
                        value = data[dot_index]
                    else:
                        value = 0
                    color = interpolate(value, 0, heatmap_dot.MAX_FORCE, 120, 1)
                    color = clamp(color, 1, 120)
                    if finger_id == 5:
                        cv2.circle(
                            heatmap, (x, y), heatmap_dot.PALM_POINT_RADIUS, color, -1
                        )
                    else:
                        cv2.circle(heatmap, (x, y), heatmap_dot.POINT_RADIUS, color, -1)

    heatmap_colored = cv2.applyColorMap(heatmap, cv2.COLORMAP_HSV)
    heatmap_colored = cv2.resize(
        heatmap_colored, (_force_img.shape[1], _force_img.shape[0])
    )
    mask = heatmap > 0
    mask = np.uint8(mask * 255)

    if _force_img is not None and heatmap_colored is not None:
        result = _force_img.copy()
        try:
            result[mask > 0] = cv2.addWeighted(
                _force_img[mask > 0], 0.2, heatmap_colored[mask > 0], 0.8, 0
            )
        except Exception as e:
            print(f"Error when blending images: {e}")
        else:
            cv2.imshow("Heatmap", result)


def camera_thread():
    """
    Camera processing thread: Real-time. Continuously capture images from the camera and recognize gestures through a hand detector,
    And place the recognized gesture information into the queue
    """
    timer = 0
    interval = 10
    original_gesture0 = 0

    while True:
        _, img = _video.read()
        img = cv2.flip(img, 1)
        hand = _detector.findHands(img, draw=True)
        gesture_pic = cv2.imread(file_path + "/gestures/unknown.png")
        gesture = [45000, 65535, 65535, 65535, 65535, 65535]

        if hand:
            lmlist = hand[0]

            if lmlist and lmlist[0]:
                try:
                    finger_up = _detector.fingersUp(lmlist[0])

                    for i in range(len(finger_up)):
                        gesture[i] = int(gesture[i] * (1 - finger_up[i]))

                except Exception as e:
                    print(str(e))

                if finger_up[:5] == [0, 0, 0, 0, 0]:
                    gesture_pic = cv2.imread(file_path + "/gestures/0.png")
                elif finger_up[:5] == [0, 1, 0, 0, 0]:
                    gesture_pic = cv2.imread(file_path + "/gestures/1.png")
                elif finger_up[:5] == [0, 1, 1, 0, 0]:
                    gesture_pic = cv2.imread(file_path + "/gestures/2.png")
                elif finger_up[:5] == [0, 1, 1, 1, 0]:
                    gesture_pic = cv2.imread(file_path + "/gestures/3.png")
                elif finger_up[:5] == [0, 1, 1, 1, 1]:
                    gesture_pic = cv2.imread(file_path + "/gestures/4.png")
                elif finger_up[:5] == [1, 1, 1, 1, 1]:
                    gesture_pic = cv2.imread(file_path + "/gestures/5.png")
            else:
                gesture = [0, 0, 0, 0, 0, 0]

        if gesture_pic.any():
            gesture_pic = cv2.resize(gesture_pic, (161, 203))
            img[0:203, 0:161] = gesture_pic

        if gesture[1] == 65535 and gesture[5] == 65535 and gesture[0] == 45000:
            if timer == 0:
                original_gesture0 = gesture[0]
            timer += 1

            if timer <= interval:
                gesture[0] = 0
            else:
                gesture[0] = original_gesture0
        else:
            if timer > 0:
                gesture[0] = original_gesture0
            timer = 0

        if not gesture_queue.full():
            gesture_queue.put(gesture)
        if not image_queue.full():
            image_queue.put(img)


def init_video_and_detector():
    """
    Initialize the camera and hand detector
    """
    global _video, _detector
    # init window
    _video = cv2.VideoCapture(0)

    # Get the resolution of the camera
    window_width = int(_video.get(cv2.CAP_PROP_FRAME_WIDTH))  # Camera frame width
    window_height = int(_video.get(cv2.CAP_PROP_FRAME_HEIGHT))  # Camera frame height

    _detector = HandDetector(maxHands=1, detectionCon=0.8)

    # Create a window with adjustable size
    cv2.namedWindow("Video", cv2.WINDOW_NORMAL)

    # Set window size to camera resolution
    cv2.resizeWindow("Video", window_width, window_height)


def main():
    prev_gesture = [0, 0, 0, 0, 0, 0]
    finger_force, finger_force_sum = [], []
    # Flag that indicates setting force target for the first time, to avoid repeated setting
    first_set = [True for _ in range(NUM_FINGERS - 1)]
    force_control_mode = False

    # sub_model = 0
    heatmap_dot = HeatMapDot(FORCE_TYPE)
    heatmap_dot.init_dot_info()

    force_sensor = False
    client = ModbusSerialClient(find_comport("CH340"), FramerType.RTU, 115200)
    if not client.connect():
        print("Failed to connect to Modbus device")
        exit(-1)

    resp = read_registers(client, ROH_HW_VERSION, 1)

    if resp is None:
        print("Failed to read hardware version or unsupported hardware type")
        exit(-1)
    elif resp[0] != ROH_HARDWARE_TYPE_AP:
        force_sensor = False
    else:
        force_sensor = True

    if force_sensor:
        hand_type = input("select left hand(0) or right hand(1)")
        print("Press '1' to enter force control mode, '2' to exit force control mode, 'q' to exit program\n")
        img_init(hand_type, heatmap_dot)
        cv2.namedWindow("Heatmap", cv2.WINDOW_NORMAL)

        heatmap = np.zeros((_height, _width), dtype=np.float32)
        if not write_registers(client, ROH_RESET_FORCE, 1):
            print("Failed to reset force")

    init_video_and_detector()
    threading.Thread(target=camera_thread, daemon=True).start()

    while True:
        gesture = gesture_queue.get()

        if not image_queue.empty():
            img = image_queue.get()
            cv2.putText(img, "Try with gestures", (16, 272), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 5)
            cv2.imshow("Video", img)

        # Choose different control mode
        if force_control_mode:
            for i in range(NUM_FINGERS - 1):
                if gesture[i] > 0:
                    # print(resp[0])
                    if first_set[i]:
                        write_registers(client, ROH_FINGER_FORCE_TARGET0 + i, FORCE_TARGET)
                        first_set[i] = False
                else:
                    if not first_set[i]:
                        write_registers(client, ROH_FINGER_FORCE_TARGET0 + i, 0)
                        first_set[i] = True

            # thumb root control
            if prev_gesture[THUMB_ROOT_ID] != gesture[THUMB_ROOT_ID]:
                write_registers(client, ROH_FINGER_POS_TARGET5, gesture[THUMB_ROOT_ID])
                prev_gesture[THUMB_ROOT_ID] = gesture[THUMB_ROOT_ID]
        # default:position control mode
        else:
            if prev_gesture != gesture:
                if not write_registers(client, ROH_FINGER_POS_TARGET0, gesture):
                    print("Failed to write target position")
                prev_gesture = gesture

        # heatmap
        if force_sensor:
            get_finger_force(client, heatmap_dot)
            if not data_queue.empty():
                finger_force, finger_force_sum = data_queue.get()
            update_heatmap(finger_force, heatmap_dot)
            # print(finger_force_sum)

        # set control mode by keybord
        key = cv2.waitKey(1) & 0xFF

        if force_sensor:
            if key == ord("1") and not force_control_mode:
                force_control_mode = True
                print("Use force control mode")
            elif key == ord("2") and force_control_mode:
                force_control_mode = False
                print("Use position control mode")
                for i in range(NUM_FINGERS - 1):
                    write_registers(client, ROH_FINGER_FORCE_TARGET0 + i, 0)

        if key == ord("q"):
            break

    _video.release()
    cv2.destroyAllWindows()
    client.close()

if __name__ == "__main__":
    main()
