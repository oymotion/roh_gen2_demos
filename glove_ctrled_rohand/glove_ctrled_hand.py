# Sample code to get glove data and controls ROHand via ModBus-RTU protocol
import asyncio
import os
import signal

import cv2
import numpy as np

from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from serial.tools import list_ports

from roh_registers_v2 import *
from heat_map_dot import *

# Choose input device. ONLY ONE of the following should be uncommented.
# Uncomment following line to use BLE Glove
from pos_input_ble_glove import PosInputBleGlove as PosInput

# Or
# Uncomment following line to use USB Glove
# from pos_input_usb_glove import PosInputUsbGlove as PosInput


# ROHand configuration
NODE_ID = 2
NUM_FINGERS = 6

# Rohand hardware type which supports force feedback
ROH_HARDWARE_TYPE = 0x2001


TOLERANCE = round(65536 / 32)  # 判断目标位置变化的阈值，位置控制模式时为整数，角度控制模式时为浮点数
SPEED_CONTROL_THRESHOLD = 8192  # 位置变化低于该值时，线性调整手指运动速度

file_path = os.path.abspath(os.path.dirname(__file__))


def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


def interpolate(n, from_min, from_max, to_min, to_max):
    return (n - from_min) / (from_max - from_min) * (to_max - to_min) + to_min


class Application:
    def __init__(self):
        signal.signal(signal.SIGINT, lambda signal, frame: self._signal_handler())
        self.terminated = False

    def _signal_handler(self):
        print("You pressed ctrl-c, exit")
        self.terminated = True

    def find_comport(self, port_name):
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

    def write_registers(self, client, address, values):
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

    def read_registers(self, client, address, count):
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

    async def main(self):
        glove_usb = False
        if self.find_comport("STM Serial") or self.find_comport("串行设备"):
            glove_usb = True
            while True:
                # 显示选择提示
                print("请选择灵巧手配置:1. 右手 (Right) 2. 左手 (Left)")

                # 获取用户输入
                choice = input("请输入选项(1或2): ")
                if choice == "1":
                    pic_path = "/pic/force_right.png"
                    FORCE_POINT = RIGHT_FORCE_POINT
                    break
                elif choice == "2":
                    pic_path = "/pic/force_left.png"
                    FORCE_POINT = LEFT_FORCE_POINT
                    break
                else:
                    print("无效输入，请输入1或2")
        prev_finger_data = [65535 for _ in range(NUM_FINGERS)]
        finger_data = [0 for _ in range(NUM_FINGERS)]
        prev_dir = [0 for _ in range(NUM_FINGERS)]
        fingers_point_force = [[] for _ in range(NUM_FINGERS)]
        pos_input = PosInput()

        # 连接到Modbus设备
        client = ModbusSerialClient(self.find_comport("CH340"), FramerType.RTU, 115200)
        if not client.connect():
            print("连接Modbus设备失败\nFailed to connect to Modbus device")
            exit(-1)

        # 获取硬件版本信息
        resp = self.read_registers(client, ROH_HW_VERSION, 1)
        if resp is None or resp[0] != ROH_HARDWARE_TYPE:
            print("读取硬件版本失败，或不支持的硬件版本\nFailed to read hardware version or unsupported hardware type")
            exit(-1)

        # 重置力传感器
        if not self.write_registers(client, ROH_RESET_FORCE, 1):
            print("力量清零失败\nFailed to reset force")

        if glove_usb:
            # 热力图绘制
            # 创建可调整大小的窗口
            cv2.namedWindow("Heatmap", cv2.WINDOW_NORMAL)

            image_path = file_path + pic_path
            force_img = cv2.imread(image_path)

            if force_img is None:
                raise ValueError("无法加载图片，请检查路径是否正确\n Failed to load image, please check the path")
            height, width = force_img.shape[:2]

            # 创建空白热力图(单通道)
            heatmap = np.zeros((height, width), dtype=np.float32)

        finger_id = 0

        if not await pos_input.start():
            print("初始化失败,退出\nFailed to initialize, exit.")
            exit(-1)

        while not self.terminated:
            finger_data = await pos_input.get_position()

            dir = [0 for _ in range(NUM_FINGERS)]
            pos = [0 for _ in range(NUM_FINGERS)]
            target_changed = False

            for i in range(NUM_FINGERS):
                if finger_data[i] > prev_finger_data[i] + TOLERANCE:
                    prev_finger_data[i] = finger_data[i]
                    dir[i] = 1
                elif finger_data[i] < prev_finger_data[i] - TOLERANCE:
                    prev_finger_data[i] = finger_data[i]
                    dir[i] = -1

                # 只在方向发生变化时发送目标位置/角度
                if dir[i] != prev_dir[i]:
                    prev_dir[i] = dir[i]
                    target_changed = True

                if dir[i] == -1:
                    pos[i] = 0
                elif dir[i] == 0:
                    pos[i] = finger_data[i]
                else:
                    pos[i] = 65535

            if target_changed:
                # Read current position
                curr_pos = [0 for _ in range(NUM_FINGERS)]
                resp = self.read_registers(client, ROH_FINGER_POS0, NUM_FINGERS)

                if resp is not None:
                    curr_pos = resp
                else:
                    print("读取位置指令发送失败\nFailed to send read pos command")
                    print(f"read_registers({ROH_FINGER_POS0}, {NUM_FINGERS}, {NODE_ID}) returned {resp})")
                    continue

                speed = [0 for _ in range(NUM_FINGERS)]

                for i in range(NUM_FINGERS):
                    temp = interpolate(abs(curr_pos[i] - finger_data[i]), 0, SPEED_CONTROL_THRESHOLD, 0, 65535)
                    speed[i] = clamp(round(temp), 0, 65535)

                # Set speed
                if not self.write_registers(client, ROH_FINGER_SPEED0, speed):
                    print("设置速度失败\nFailed to set speed")

                # Control the ROHand
                if not self.write_registers(client, ROH_FINGER_POS_TARGET0, pos):
                    print("设置位置失败\nFailed to set pos")

            if glove_usb:
                reg_cnt = FORCE_VALUE_LENGTH[finger_id]
                resp = self.read_registers(client, ROH_FINGER_FORCE_EX0 + finger_id * FORCE_GROUP_SIZE, reg_cnt)
                if resp is None:
                    print("读取力量失败,退出\nFailed to read force, exit")
                    exit(-1)

                if len(resp) == reg_cnt:
                    items = []
                    for j in range(reg_cnt):
                        # 第一个点：高八位 第二个点低八位
                        items.append((resp[j] >> 8) & 0xFF)
                        items.append(resp[j] & 0xFF)
                    # print(items)

                    fingers_point_force[finger_id] = items

                if fingers_point_force[finger_id] is not None:
                    for i, finger_points in enumerate(FORCE_POINT):
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

                finger_id += 1

                if finger_id >= NUM_FINGERS:
                    finger_id = 0

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
                        cv2.imshow("Heatmap", result)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        await pos_input.stop()
        client.close()


if __name__ == "__main__":
    app = Application()
    asyncio.run(app.main())
