# Sample code to get glove data and controls ROHand via ModBus-RTU protocol
import asyncio
import os
import signal
import cv2
import numpy as np
import math
import sys

from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from serial.tools import list_ports
from queue import Queue

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from common.roh_registers_v2 import *
from common.heat_map_dot import *



# ROHand configuration
NODE_ID = 2
NUM_FINGERS = 6
PALM_INDEX = 5

TACS_3D_FORCE = 1
TACS_DOT_MATRIX = 0

FORCE_TYPE = TACS_DOT_MATRIX

# Rohand hardware type which supports force feedback
ROH_HARDWARE_TYPE_AP = 0x2001

# The threshold for judging changes in target position is an integer in position control mode,
# and a floating-point number in angle control mode
TOLERANCE = round(65536 / 32)
SPEED_CONTROL_THRESHOLD = 8192  # When the position change is below this value, linearly adjust the finger movement speed

# global variable
file_path = os.path.abspath(os.path.dirname(__file__))
heatmap = None
data_queue = Queue(maxsize=NUM_FINGERS)


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

    def img_init(self, hand_type, heatmap_dot):
        """
        Init force image.
        :param hand_type: 0 for left hand, other for right hand
        :param heatmap_dot: HeatMapDot instance, that contains force sensor configuration information
        """
        # file_path = os.path.abspath(os.path.dirname(__file__))
        global _force_img, _force_point_location

        # Create a window with adjustable size
        cv2.namedWindow("Heatmap", cv2.WINDOW_NORMAL)

        if hand_type == 0 or "0":
            pic_path = "/pic/force_left.png"
            _force_point_location = heatmap_dot.LEFT_FORCE_POINT
        else:
            pic_path = "/pic/force_right.png"
            _force_point_location = heatmap_dot.RIGHT_FORCE_POINT

        image_path = file_path + pic_path
        _force_img = cv2.imread(image_path)

        if _force_img is None:
            raise ValueError("Failed to load image, please check the path")

    def data_reading(self, client, heatmap_dot):
        """
        Independent thread: Continuously read force sensor data and place it in the queue
        :param client: Modbus client instance
        :param heatmap_dot: HeatMapDot instance, that contains force sensor configuration information
        """
        try:
            finger_force = [[] for _ in range(NUM_FINGERS)]
            # Fingers force data acquisition
            for i in range(NUM_FINGERS - 1):
                reg_cnt = heatmap_dot.FORCE_VALUE_LENGTH[i]
                resp = self.read_registers(client, ROH_FINGER_FORCE_EX0 + i * FORCE_GROUP_SIZE, reg_cnt)

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
                    elif heatmap_dot.SENSOR_TYPE == TACS_DOT_MATRIX:
                        for j in range(reg_cnt):
                            val.append((resp[j] >> 8) & 0xFF)
                            val.append(resp[j] & 0xFF)
                        # print(val)
                        finger_force[i] = val
            # Palm force data acquisition
            reg_cnt = heatmap_dot.FORCE_VALUE_LENGTH[PALM_INDEX]
            resp = self.read_registers(client, ROH_FINGER_FORCE_EX0 + PALM_INDEX * FORCE_GROUP_SIZE, reg_cnt)

            if resp is not None and len(resp) == reg_cnt:
                val = []

                for i in range(reg_cnt):
                    val.append((resp[i] >> 8) & 0xFF)
                    val.append(resp[i] & 0xFF)
                # print(val)
                finger_force[PALM_INDEX] = val

            if not data_queue.full():
                data_queue.put((finger_force))
            # time.sleep(0.001)

        except Exception as e:
            print(f"Data reading thread err: {e}")

    def update_heatmap(self, finger_force, heatmap_dot):
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
                        length = round(interpolate(value, 0, heatmap_dot.MAX_FORCE, 0, 120))
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

    async def main(self):
        global _height, _width
        # The wireless glove does not display force because Bluetooth and graphics
        # must be processed simultaneously on the main thread
        glove_usb = False
        force_sensor = False

        if self.find_comport("STM Serial") or self.find_comport("串行设备"):
            glove_usb = True
            from pos_input_usb_glove import PosInputUsbGlove as PosInput
        else:
            from pos_input_ble_glove import PosInputBleGlove as PosInput

        # connect to Modbus device
        client = ModbusSerialClient(self.find_comport("CH340"), FramerType.RTU, 115200)
        if not client.connect():
            print("Failed to connect to Modbus device")
            exit(-1)

        resp = self.read_registers(client, ROH_HW_VERSION, 1)

        if resp is None:
            print("Failed to read hardware version or unsupported hardware type")
            exit(-1)
        elif resp[0] == ROH_HARDWARE_TYPE_AP:
            force_sensor = True

        if force_sensor and glove_usb:
            # sub_model = 0
            force_type = int(input("select force type: DOT_MATRIX(0) or 3D_FORCE(1)"))

            heatmap_dot = HeatMapDot(force_type)
            heatmap_dot.init_dot_info()
            hand_type = input("select left hand(0) or right hand(1)")
            self.img_init(hand_type, heatmap_dot)  # choose left hand
            _height, _width = _force_img.shape[:2]

            # reset force
            if not self.write_registers(client, ROH_RESET_FORCE, 1):
                print("Failed to reset force")

        prev_finger_data = [65535 for _ in range(NUM_FINGERS)]
        finger_data = [0 for _ in range(NUM_FINGERS)]
        prev_dir = [0 for _ in range(NUM_FINGERS)]
        pos_input = PosInput()

        if not await pos_input.start():
            print("Failed to initialize, exit.")
            exit(-1)

        while not self.terminated:
            # Finger control
            finger_data = await pos_input.get_position()

            if finger_data is None:
                continue

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

                # Only send the target position/angle when the direction changess
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
                    print("Failed to send read pos command")
                    print(f"read_registers({ROH_FINGER_POS0}, {NUM_FINGERS}, {NODE_ID}) returned {resp})")
                    continue

                speed = [0 for _ in range(NUM_FINGERS)]

                for i in range(NUM_FINGERS):
                    temp = interpolate(
                        abs(curr_pos[i] - finger_data[i]),
                        0,
                        SPEED_CONTROL_THRESHOLD,
                        0,
                        65535,
                    )
                    speed[i] = clamp(round(temp), 0, 65535)

                # Set speed
                if not self.write_registers(client, ROH_FINGER_SPEED0, speed):
                    print("Failed to set speed")

                # Control the ROHand
                if not self.write_registers(client, ROH_FINGER_POS_TARGET0, pos):
                    print("Failed to set pos")

            if force_sensor and glove_usb:
                self.data_reading(client, heatmap_dot)
                finger_force = []

                if not data_queue.empty():
                    finger_force = data_queue.get()

                # heatmap
                if finger_force is None:
                    continue
                self.update_heatmap(finger_force, heatmap_dot)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        await pos_input.stop()
        client.close()


if __name__ == "__main__":
    app = Application()
    asyncio.run(app.main())
