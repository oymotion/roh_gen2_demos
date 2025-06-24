import os

import cv2
import time
import numpy as np
import matplotlib.pyplot as plt

from collections import deque
from matplotlib.ticker import MultipleLocator
from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from serial.tools import list_ports
from roh_registers_v2 import *
from heat_map_dot import *

def interpolate(n, from_min, from_max, to_min, to_max):
    return (n - from_min) / (from_max - from_min) * (to_max - to_min) + to_min

def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

# Force configuration
NUM_FINGERS = 6
NODE_ID = 2

def find_comport(port_name):
    """自动查找可用串口"""
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
    
def img_init(hand_type):
    """
    Init force image.
    :param hand_type: 0 for left hand, other for right hand
    :return: None
    """
    file_path = os.path.abspath(os.path.dirname(__file__))
    global force_img, force_point

    cv2.namedWindow("Heatmap", cv2.WINDOW_NORMAL)# Create a window with adjustable size

    if hand_type == 0:
        pic_path = "/pic/force_left.png"
        force_point = LEFT_FORCE_POINT
    else:
        pic_path = "/pic/force_right.png"
        force_point = RIGHT_FORCE_POINT

    image_path = file_path + pic_path
    force_img = cv2.imread(image_path)

    if force_img is None:
        raise ValueError("无法加载图片，请检查路径是否正确\n Failed to load image, please check the path")
    
def curve_init():
    """
    Init force curve chart.
    :return: None
    """
    global data_queues, x_data, fig, axs, lines, annotations
    limit_max = 3000 # Max force value
    limit_min = 0
    line_width = 3

    plt.ioff()  # Turn off interactive mode
    fig, axs = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    fig.subplots_adjust(hspace=0.4)
    data_queues = [deque(maxlen=100) for _ in range(NUM_FINGERS)]
    x_data = deque(maxlen=100)  # Store the last 100 timestamps

    lines = []
    annotations = []
    finger_labels = ['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'Palm']
    colors = ['r-', 'b-', 'g-', 'c-', 'm-', 'y-']
    y_labels = ["Thumb", "IndexF", "MiddleF", "RingF", "LittleF", "Palm"]

    for i, ax in enumerate(axs):
        line, = ax.plot([], [], colors[i], label=f'{finger_labels[i]} Value', linewidth=line_width)
        lines.append(line)

        annotation = ax.text(1, 0, '', transform=ax.transAxes, 
                           ha='right', va='bottom', bbox=dict(facecolor='white', alpha=0.7))
        annotations.append(annotation)

        ax.set_ylabel(y_labels[i])
        ax.set_ylim(limit_min, limit_max) # Set max force value
        ax.yaxis.set_major_locator(MultipleLocator(600)) # Set major tick interval
        ax.grid(True) # Enable grid
        ax.legend(loc='upper right')

    axs[-1].set_xlabel('Time/s')
    axs[0].set_title('Real-time ROHand Finger Force Data')

def main():
    start_time = time.time()
    img_init(0) # choose left hand
    curve_init()

    # Create a blank heatmap (single channel)
    height, width = force_img.shape[:2]
    heatmap = np.zeros((height, width), dtype=np.float32)
    finger_force_sum = [0 for _ in range(NUM_FINGERS)] 
    intensity = [0, 0, 0, 0, 0, 0]
    # fingers_point_force :list[list[int]] = [[]]
    fingers_point_force = [[] for _ in range(NUM_FINGERS)] 

    client = ModbusSerialClient(find_comport("CH340"), FramerType.RTU, 115200)
    if not client.connect():
        print("连接Modbus设备失败\nFailed to connect to Modbus device")
        exit(-1)

    if not write_registers(client, ROH_RESET_FORCE, 1):
        print("力量清零失败\nFailed to reset force")

    while True:

        for i in range(NUM_FINGERS):
            reg_cnt = FORCE_VALUE_LENGTH[i]
            resp = client.read_holding_registers(ROH_FINGER_FORCE_EX0 + i * FORCE_GROUP_SIZE, reg_cnt, NODE_ID)

            if len(resp.registers) == reg_cnt:
                items = []
                finger_force_sum[i] = 0
                for j in range(reg_cnt):
                    items.append((resp.registers[j] >> 8) & 0xff)
                    items.append(resp.registers[j] & 0xff)
                    finger_force_sum[i] += items[j]
                # print(items)
                fingers_point_force.append(items)

        for i, finger_points in enumerate(force_point):
            force_values = fingers_point_force[i] if i < len(fingers_point_force) else []
            for j, point in enumerate(finger_points):
                x, y = point
                x, y = int(x), int(y)
                if 0 <= x < width and 0 <= y < height:
                    if j < len(force_values):
                        intensity = force_values[j]
                    else:
                        intensity = 0
                    intensity = interpolate(intensity, 0, 200, 120, 0)
                    intensity = clamp(intensity, 0, 120)
                    if i == 5:
                        cv2.circle(heatmap, (x, y), PALM_POINT_RADIUS, intensity, -1)
                    else:
                        cv2.circle(heatmap, (x, y), POINT_RADIUS, intensity, -1)

        fingers_point_force = []

        heatmap = np.uint8(heatmap)

        heatmap_colored = cv2.applyColorMap(heatmap, cv2.COLORMAP_HSV)
        heatmap_colored = cv2.resize(heatmap_colored, (force_img.shape[1], force_img.shape[0]))
        mask = heatmap > 0
        mask = np.uint8(mask * 255)

        if force_img is not None and heatmap_colored is not None:
            result = force_img.copy()
            try:
                result[mask > 0] = cv2.addWeighted(force_img[mask > 0], 0.2, heatmap_colored[mask > 0], 0.8, 0)
            except Exception as e:
                print(f"混合图像时出错\nError when blending images: {e}")
            else:
                cv2.imshow('Heatmap', result)
        
        # Record timestamp and value
        current_time = time.time() - start_time
        x_data.append(current_time)

        for i in range(NUM_FINGERS):
            data_queues[i].append(finger_force_sum[i])
        # Update graphics
        for i in range(NUM_FINGERS):
            if data_queues[i]:  # Update numerical labels
                annotations[i].set_text(f'{data_queues[i][-1]:.1f}')

            lines[i].set_data(x_data,data_queues[i])
            if data_queues[i] and x_data[0] != x_data[-1]:
                axs[i].set_xlim(x_data[0], x_data[-1])  # Only called when data changes

        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.show(block=False)
        fig.set_facecolor('none')
        fig.canvas.manager.set_window_title("ROHand Force Curve")
        fig.canvas.toolbar = None

        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()
    client.close()

if __name__ == "__main__":
    main()
