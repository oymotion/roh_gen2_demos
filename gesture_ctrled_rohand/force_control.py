from roh_registers_v2 import *
from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from serial.tools import list_ports

# Hand configuration
NUM_FINGERS = 5
NODE_ID = 2
FORCE_VALUE_LENGTH = [17, 30, 30, 30, 16, 27]

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
    client = ModbusSerialClient(find_comport("CH340"), FramerType.RTU, 115200)
    if not client.connect():
        print("连接Modbus设备失败\nFailed to connect to Modbus device")
        exit(-1)

    if not write_registers(client, ROH_RESET_FORCE, 1):
        print("力量清零失败\nFailed to reset force")

    if not write_registers(client, ROH_FINGER_FORCE_TARGET0, [500, 500, 500, 500, 500]):
        print("写入目标力量失败\nFailed to write target position")

    resp = read_registers(client, ROH_FINGER_FORCE0, NUM_FINGERS)
    if resp is None:
        print("读取合力失败,退出\nFailed to read force summation")
    else:
        print(resp)

    for i in range(NUM_FINGERS):
        reg_cnt = FORCE_VALUE_LENGTH[i]
        resp = read_registers(client, ROH_FINGER_FORCE_EX0 + i * FORCE_GROUP_SIZE, reg_cnt)
        if resp is None:
            print("读取单点力量失败,退出\nFailed to read single dot force")

        if len(resp) == reg_cnt:
            force_dot = []
            for j in range(reg_cnt):
                # 第一个点：高八位 第二个点低八位
                force_dot.append((resp[j] >> 8) & 0xff)
                force_dot.append(resp[j] & 0xff)
            print(force_dot)

    if not write_registers(client, ROH_FINGER_FORCE_TARGET0, [0, 0, 0, 0, 0]):
        print("写入目标力量失败\nFailed to write target position")

    client.close()

if __name__ == "__main__":
    main()
