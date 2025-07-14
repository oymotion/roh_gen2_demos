import time
import serial
import os
import sys
import asyncio
import cv2
import numpy as np

from roh_registers_v2 import *
from heat_map_dot import *
from serial.tools import list_ports
from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException

from PyInstaller.utils.hooks import collect_dynamic_libs
binaries = collect_dynamic_libs('cv2')

# Force configuration
NUM_FINGERS = 6
file_path = os.path.abspath(os.path.dirname(__file__))

# Rohand hardware type which supports force feedback
ROH_HARDWARE_TYPE = 0x2001

# Constants
MAX_PROTOCOL_DATA_SIZE = 64

# Protocol states
WAIT_ON_HEADER_0 = 0
WAIT_ON_HEADER_1 = 1
WAIT_ON_BYTE_COUNT = 2
WAIT_ON_DATA = 3
WAIT_ON_LRC = 4

# Protocol byte name
DATA_CNT_BYTE_NUM = 0
DATA_START_BYTE_NUM = 1

# ROHand configuration
NODE_ID = 2

TOLERANCE = round(65536 / 32)  # 判断目标位置变化的阈值，位置控制模式时为整数，角度控制模式时为浮点数
SPEED_CONTROL_THRESHOLD = 8192 # 位置变化低于该值时，线性调整手指运动速度


current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)


def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


def interpolate(n, from_min, from_max, to_min, to_max):
    return (n - from_min) / (from_max - from_min) * (to_max - to_min) + to_min


# OHand bus context
class OGlove:
    def __init__(self, serial, timeout):
        """
        Initialize OGlove.

        Parameters
        ----------
        serial : str
            Path to serial port
        timeout : int
            Timeout in in milliseconds
        """
        self.serial_port = serial
        self.timeout = timeout
        self.is_whole_packet = False
        self.decode_state = WAIT_ON_HEADER_0
        self.packet_data = bytearray(MAX_PROTOCOL_DATA_SIZE + 2)  # Including byte_cnt, data[], lrc
        self.send_buf = bytearray(MAX_PROTOCOL_DATA_SIZE + 4)  # Including header0, header1, nb_data, lrc
        self.byte_count = 0

    def calc_lrc(ctx, lrcBytes, lrcByteCount):
        """
        Calculate the LRC for a given sequence of bytes
        :param lrcBytes: sequence of bytes to calculate LRC over
        :param lrcByteCount: number of bytes in the sequence
        :return: calculated LRC value
        """
        lrc = 0
        for i in range(lrcByteCount):
            lrc ^= lrcBytes[i]
        return lrc

    def on_data(self, data):
        """
        Called when a new byte is received from the serial port. This function implements
        a state machine to decode the packet. If a whole packet is received, is_whole_packet
        is set to 1 and the packet is stored in packet_data.

        Args:
            data (int): The newly received byte

        Returns:
            None
        """
        if self is None:
            return

        if self.is_whole_packet:
            return  # Old packet is not processed, ignore

        # State machine implementation
        if self.decode_state == WAIT_ON_HEADER_0:
            if data == 0x55:
                self.decode_state = WAIT_ON_HEADER_1

        elif self.decode_state == WAIT_ON_HEADER_1:
            self.decode_state = WAIT_ON_BYTE_COUNT if data == 0xAA else WAIT_ON_HEADER_0

        elif self.decode_state == WAIT_ON_BYTE_COUNT:
            self.packet_data[DATA_CNT_BYTE_NUM] = data
            self.byte_count = data

            if self.byte_count > MAX_PROTOCOL_DATA_SIZE:
                self.decode_state = WAIT_ON_HEADER_0
            elif self.byte_count > 0:
                self.decode_state = WAIT_ON_DATA
            else:
                self.decode_state = WAIT_ON_LRC

        elif self.decode_state == WAIT_ON_DATA:
            self.packet_data[DATA_START_BYTE_NUM + self.packet_data[DATA_CNT_BYTE_NUM] - self.byte_count] = data
            self.byte_count -= 1

            if self.byte_count == 0:
                self.decode_state = WAIT_ON_LRC

        elif self.decode_state == WAIT_ON_LRC:
            self.packet_data[DATA_START_BYTE_NUM + self.packet_data[DATA_CNT_BYTE_NUM]] = data
            self.is_whole_packet = True
            self.decode_state = WAIT_ON_HEADER_0

        else:
            self.decode_state = WAIT_ON_HEADER_0

    def get_data(self, resp_bytes) -> bool:
        """
        Retrieve a complete packet from the serial port and validate it.

        Args:
            resp_bytes (bytearray): A bytearray to store the response data.

        Returns:
            bool: True if a valid packet is received, False otherwise.
        """
        # Check if self or self.serial_port is None
        if self is None or self.serial_port is None:
            return False

        self.serial_port.reset_input_buffer()

        # 记录开始等待的时间
        wait_start = time.time()
        # 计算等待超时时间
        wait_timeout = wait_start + self.timeout / 1000

        # 循环等待完整的数据包
        while not self.is_whole_packet:
            # time.sleep(0.001)
            # print(f"in_waiting: {self.serial_port.in_waiting}")

            # 如果串口有数据可读
            while self.serial_port.in_waiting > 0:
                # 读取串口数据
                data_bytes = self.serial_port.read(self.serial_port.in_waiting)
                # print("data_bytes: ", len(data_bytes))

                # 遍历读取到的数据
                for ch in data_bytes:
                    # print(f"data: {hex(ch)}")
                    # 处理数据
                    self.on_data(ch)
                # 如果已经读取到完整的数据包，跳出循环
                if self.is_whole_packet:
                    break

            # 如果还没有读取到完整的数据包，并且已经超时，跳出循环
            if (not self.is_whole_packet) and (wait_timeout < time.time()):
                # print(f"wait time out: {wait_timeout}, now: {time.time()}")
                # 重置解码状态
                self.decode_state = WAIT_ON_HEADER_0
                return False

        # Validate LRC
        lrc = self.calc_lrc(self.packet_data, self.packet_data[DATA_CNT_BYTE_NUM] + 1)
        if lrc != self.packet_data[DATA_START_BYTE_NUM + self.packet_data[DATA_CNT_BYTE_NUM]]:
            self.is_whole_packet = False
            return False

        # Copy response data
        if resp_bytes is not None:
            packet_byte_count = self.packet_data[DATA_CNT_BYTE_NUM]
            resp_bytes.clear()
            resp_data = self.packet_data[DATA_START_BYTE_NUM: DATA_START_BYTE_NUM + packet_byte_count]
            for v in resp_data:
                resp_bytes.append(v)

        self.is_whole_packet = False
        return True

    def do_calibration(self, cali_min, cali_max, offset) -> bool:
        """
        Do calibration to obtain user-appropriate glove data.

        Args:
            cali_min: Minimum of calicration data.
            cali_min: Maximum of calicration data.
            offset: 0-general glove, 1-left or right glove

        Returns: 
            bool: True if calibration data is valid, False otherwise.
        """
        cali_min[:] = [65535 for _ in range(NUM_FINGERS)]
        cali_max[:] = [0 for _ in range(NUM_FINGERS)]

        glove_raw_data = bytearray()

        print("校正模式，请执行握拳和张开动作若干次\nCalibrating mode, please perform a fist and open action several times")

        for _ in range(512):
            self.get_data(glove_raw_data)
            glove_data = []

            for i in range(int(len(glove_raw_data) / 2)):
                glove_data.append((glove_raw_data[offset + i * 2]) | (glove_raw_data[offset + i * 2 + 1] << 8)) # 每两个字节为一个数据

            glove_data_sum = [0 for _ in range(len(glove_data))]

            for _ in range(len(glove_data)):
                for i in range(len(glove_data)):
                    glove_data_sum[i] += glove_data[i]

            # 更新最大最小值
            for i in range(len(glove_data)):
                temp = glove_data_sum[i] / len(glove_data)
                cali_max[i] = max(cali_max[i], temp)
                cali_min[i] = min(cali_min[i], temp)

        for i in range(NUM_FINGERS):
            print("MIN/MAX of finger {0}: {1}-{2}".format(i, cali_min[i], cali_max[i]))
            if (cali_min[i] >= cali_max[i]):
                print("无效数据，退出.\nInvalid data, exit.")
                return False  
                              
        return True

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


async def main():
    
    # 选择正确的手
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

    # 配置串口参数（根据实际设备修改）
    glove_serial_port = serial.Serial(
        port=find_comport("串行设备") or find_comport("STM"),
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
    )
    print(f"手套使用端口：\nGlove using serial port: {glove_serial_port.name}")

    # 初始化数据
    finger_data = [0 for _ in range(NUM_FINGERS)]  # 灵巧手手指位置
    prev_finger_data = [65535 for _ in range(NUM_FINGERS)]
    prev_dir = [0 for _ in range(NUM_FINGERS)]

    fingers_point_force = [[] for _ in range(NUM_FINGERS)]

    # 连接到Modbus设备
    client = ModbusSerialClient(find_comport("CH340"), FramerType.RTU, 115200)

    if not client.connect():
        print("连接Modbus设备失败\nFailed to connect to Modbus device")
        exit(-1)


    # 获取硬件版本信息
    resp = read_registers(client, ROH_HW_VERSION, 1)
    if resp is None or resp[0] != ROH_HARDWARE_TYPE:
        print("读取硬件版本失败，或不支持的硬件版本\nFailed to read hardware version or unsupported hardware type")
        exit(-1)

    # 重置力传感器
    if not write_registers(client, ROH_RESET_FORCE, 1):
        print("力量清零失败\nFailed to reset force")


    # 初始化手套
    oglove = OGlove(serial=glove_serial_port, timeout=2000)
    glove_raw_data = bytearray() # 手套原始数据，单字节形式
    # 区分左右手套
    left_or_right = None
    offset = 0
    
    if oglove.get_data(glove_raw_data):
        if len(glove_raw_data) & 0x01 == 1:
            left_or_right = glove_raw_data[0]
            offset = 1
    
    if left_or_right == 0:
        print("使用左手手套\nUse left glove")
    elif left_or_right == 1:
        print("使用右手手套\nUse right glove")
    else:
        print("使用通用手套\nUse general glove")

    # 开始校正
    cali_min = [65535 for _ in range(NUM_FINGERS)]
    cali_max = [0 for _ in range(NUM_FINGERS)]
    if not (oglove.do_calibration(cali_min, cali_max, offset)):
        exit(-1)
    
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

    try:
        finger_id = 0

        while True:
            # 读取串口数据
            if oglove.get_data(glove_raw_data):
                glove_data = []  # 手套完整数据，两个字节

                # 处理数据
                for i in range(int(len(glove_raw_data) / 2)):
                    glove_data.append(
                        ((glove_raw_data[offset + i * 2]) | (glove_raw_data[offset + i * 2 + 1] << 8)))  # 每两个字节为一个数据

                glove_data_sum = [0 for _ in range(len(glove_data))]

                for j in range(len(glove_data)):
                    for i in range(len(glove_data)):
                        glove_data_sum[i] += glove_data[i]

                for i in range(NUM_FINGERS):
                    glove_data[i] = (glove_data[i] * 3 + glove_data_sum[i] / len(glove_data)) / 4  # 平滑
                    # 映射到灵巧手位置
                    finger_data[i] = round(interpolate(glove_data[i], cali_min[i], cali_max[i], 65535, 0))
                    finger_data[i] = clamp(finger_data[i], 0, 65535)  # 限制在最大最小范围内

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
                    resp = read_registers(client, ROH_FINGER_POS0, NUM_FINGERS)

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
                    if not write_registers(client, ROH_FINGER_SPEED0, speed):
                        print("设置速度失败\nFailed to set speed")

                    # Control the ROHand
                    if not write_registers(client, ROH_FINGER_POS_TARGET0, pos):
                        print("设置位置失败\nFailed to set pos")


                reg_cnt = FORCE_VALUE_LENGTH[finger_id]
                resp = read_registers(client, ROH_FINGER_FORCE_EX0 + finger_id * FORCE_GROUP_SIZE, reg_cnt)
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

            if (finger_id >= NUM_FINGERS):
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
                    cv2.imshow('Heatmap', result)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

  
    except KeyboardInterrupt:
        print("用户终止程序\nUser terminated the program")
    finally:
        glove_serial_port.close()
        print("串口已关闭\nSerial port closed")


if __name__ == "__main__":
    asyncio.run(main())
