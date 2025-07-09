import os
import cv2
import numpy as np

from roh_registers_v2 import *
from heat_map_dot import *
from HandTrackingModule import HandDetector
from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from serial.tools import list_ports

# Hand configuration
NUM_FINGERS = 6
NODE_ID = 2

# Rohand hardware type which supports force feedback
ROH_HARDWARE_TYPE = 0x2001

file_path = os.path.abspath(os.path.dirname(__file__))

def img_init():
    """
    Init force image.
    :return: None
    """
    file_path = os.path.abspath(os.path.dirname(__file__))
    global force_img, force_point, height, width
    
    # Switch to the correct hand
    while True:
        # 显示选择提示
        print("请选择配置:1. 右手 (Right) 2. 左手 (Left)")

        # 获取用户输入
        choice = input("请输入选项(1或2): ")
        if choice == "1":
            pic_path = "/gestures/force_right.png"
            force_point = RIGHT_FORCE_POINT
            break
        elif choice == "2":
            pic_path = "/gestures/force_left.png"
            force_point = LEFT_FORCE_POINT
            break
        else:
            print("无效输入，请输入1或2")

    image_path = file_path + pic_path
    force_img = cv2.imread(image_path)

    if force_img is None:
        raise ValueError("无法加载图片，请检查路径是否正确\n Failed to load image, please check the path")

    height, width = force_img.shape[:2]

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

def main():
    # 初始化图像
    img_init()

    # 初始化窗口
    video = cv2.VideoCapture(0)

    # 获取摄像头的分辨率
    window_width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))  # 摄像头帧宽度
    window_height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))  # 摄像头帧高度

    detector = HandDetector(maxHands=1, detectionCon=0.8)

    # 创建可调整大小的窗口
    cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Heatmap", cv2.WINDOW_NORMAL)

    # 设置窗口大小为摄像头分辨率
    cv2.resizeWindow("Video", window_width, window_height)

    client = ModbusSerialClient(find_comport("CH340"), FramerType.RTU, 115200)
    if not client.connect():
        print("连接Modbus设备失败\nFailed to connect to Modbus device")
        exit(-1)
    
    # 创建空白热力图(单通道)
    heatmap = np.zeros((height, width), dtype=np.float32)

    # 获取硬件版本信息
    resp = read_registers(client, ROH_HW_VERSION, 1)
    if resp is None or resp[0] != ROH_HARDWARE_TYPE:
        print("读取硬件版本失败，或不支持的硬件版本\nFailed to read hardware version or unsupported hardware type")
        exit(-1)

    prev_gesture = [0, 0, 0, 0, 0, 0]
    intensity = [0, 0, 0, 0, 0, 0]
    fingers_point_force :list[list[int]] = [[]]
    if not write_registers(client, ROH_RESET_FORCE, 1):
        print("力量清零失败\nFailed to reset force")

    while True:
        _, img = video.read()
        img = cv2.flip(img, 1)
        hand = detector.findHands(img, draw=True)
        gesture_pic = cv2.imread(file_path + "/gestures/unknown.png")
        gesture = [45000, 65535, 65535, 65535, 65535, 65535]

        if hand:
            lmlist = hand[0]

            if lmlist and lmlist[0]:
                try:
                    finger_up = detector.fingersUp(lmlist[0])
                    for i in range(len(finger_up)):
                        gesture[i] = int(gesture[i] * (1 - finger_up[i]))
                except Exception as e:
                    print(str(e))

                # print(gesture)

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

        # print(gesture_pic)
        if gesture_pic.any():
            gesture_pic = cv2.resize(gesture_pic, (161, 203))
            img[0:203, 0:161] = gesture_pic
            cv2.putText(img, "Try with gestures", (16, 272), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 5)
            cv2.imshow("Video", img)

        if (prev_gesture != gesture):
            if not write_registers(client, ROH_FINGER_POS_TARGET0, gesture):
                print("写入目标位置失败\nFailed to write target position")
            prev_gesture = gesture

        for i in range(NUM_FINGERS):
            reg_cnt = FORCE_VALUE_LENGTH[i]
            resp = read_registers(client, ROH_FINGER_FORCE_EX0 + i * FORCE_GROUP_SIZE, reg_cnt)
            if resp is None:
                print("读取力量失败,退出\nFailed to read force, exit")
                exit(-1)

            if len(resp) == reg_cnt:
                items = []
                for j in range(reg_cnt):
                    # 第一个点：高八位 第二个点低八位
                    items.append((resp[j] >> 8) & 0xff)
                    items.append(resp[j] & 0xff)
                # print(items)
                fingers_point_force.append(items)
        # print(finger_force)

        for i, finger_points in enumerate(force_point):
            force_values = fingers_point_force[i] if i < len(fingers_point_force) else []
            for j, point in enumerate(finger_points):
                x, y = point
                # 确保坐标在图片范围内
                x, y = int(x), int(y)
                if 0 <= x < width and 0 <= y < height:
                    # 获取对应的强度值
                    if j < len(force_values):
                        intensity = force_values[j]
                    else:
                        intensity = 0  # 如果没有对应的强度值，使用默认值0
                    # 把数据0->200 => hsv的120->0
                    intensity = interpolate(intensity, 0, 200, 120, 0)
                    intensity = clamp(intensity, 0, 120)
                    # 绘制圆形点
                    if i == 5:
                        cv2.circle(heatmap, (x, y), PALM_POINT_RADIUS, intensity, -1)
                    else:
                        cv2.circle(heatmap, (x, y), POINT_RADIUS, intensity, -1)

        fingers_point_force = []
        heatmap = np.uint8(heatmap)

        # 应用颜色映射(这里使用HSV颜色)
        heatmap_colored = cv2.applyColorMap(heatmap, cv2.COLORMAP_HSV)

        # 调整热力图尺寸使其与原始图像一致
        heatmap_colored = cv2.resize(heatmap_colored, (force_img.shape[1], force_img.shape[0]))

        # 创建一个掩码，只在有数据的区域显示热力图
        mask = heatmap > 0
        mask = np.uint8(mask * 255)

        # 检查 force_img 和 heatmap_colored 是否正确生成
        if force_img is not None and heatmap_colored is not None:
            # 仅在掩码区域混合热力图和原始图像
            result = force_img.copy()
            try:
                result[mask > 0] = cv2.addWeighted(force_img[mask > 0], 0.2, heatmap_colored[mask > 0], 0.8, 0)
            except Exception as e:
                print(f"混合图像时出错\nError when blending images: {e}")
            else:
                # 显示结果
                cv2.imshow('Heatmap', result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    video.release()
    cv2.destroyAllWindows()
    client.close()

if __name__ == "__main__":
    main()
