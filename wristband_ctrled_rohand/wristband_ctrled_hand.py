import asyncio
import signal
import sys
import os

from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
from serial.tools import list_ports

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sensor import *
from common.roh_registers_v2 import *

SCAN_DEVICE_PERIOD_IN_MS = 3000
PACKAGE_COUNT = 5
POWER_REFRESH_PERIOD_IN_MS = 5000
MAX_RETRIES = 50

GESTURES = {
   "REST":     [20000, 10000, 10000, 10000, 10000,     0],
   "FIST":     [45000, 65535, 65535, 65535, 65535,     0],
   "POINT":    [45000,     0, 65535, 65535, 65535,     0],
   "VICTORY":  [45000,     0,     0, 65535, 65535,     0],
   "SPREAD":   [    0,     0,     0,     0,     0,     0],
   "ROCK":     [    0,     0, 65535, 65535,     0,     0],
   "PINCH":    [45000, 45000, 45000, 45000, 45000,     0],
   "SHOOT":    [    0,     0, 65535, 65535, 65535,     0],
   "ITC":      [30000, 30000,     0,     0,     0, 65535],
   "SIX":      [    0, 65535, 65535, 65535,     0,     0],
   "TRIGGER":  [38000, 30000, 32000, 65535, 65535, 65535],
   "THUMBUP":  [    0, 65535, 65535, 65535, 65535,     0],
   "GRASP":    [27525, 29491, 32768, 27525, 24903, 65535]
}
NODE_ID = 2
NUM_FINGERS = 5

terminated = False
gestureID = 0

def terminate():
    global terminated
    terminated = True


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
    global terminated, gestureID
    signal.signal(signal.SIGINT, lambda signal, frame: terminate())

    # init rohand
    client = ModbusSerialClient(find_comport("CH340") or find_comport("USB"), FramerType.RTU, 115200)
    if not client.connect():
        print("Failed to connect to Modbus device")
        exit(-1)

    resp = read_registers(client, ROH_HW_VERSION, 1)

    if resp is None:
        print("Failed to read hardware version or unsupported hardware type")
        exit(-1)

    async def gestures_control(gesture):
        if not write_registers(client, ROH_FINGER_POS_TARGET0, GESTURES["REST"]):
            print("Failed to send control command")
        await asyncio.sleep(1)

        if not write_registers(client, ROH_FINGER_POS_TARGET0, gesture):
            print("Failed to send control command")


    # init OYWW
    if not SensorControllerInstance.isEnable:
        print("please open bluetooth")
        return

    deviceList = await SensorControllerInstance.asyncScan(3000)

    filteredDevice = filter(
        lambda x: x.RSSI > -80 and (x.Name.startswith("OYWW")),
        deviceList,
    )
    for device in filteredDevice:
        sensor = SensorControllerInstance.requireSensor(device)
        if sensor == None:
            continue

        print("found: " + sensor.BLEDevice.Name)
        sensor.onDataCallback = onDataCallback
        sensor.onPowerChanged = onPowerChanged
        sensor.onStateChanged = onStateChanged
        sensor.onErrorCallback = onErrorCallback

        # check state & connect
        if sensor.deviceState != DeviceStateEx.Ready:
            print("connecting: " + sensor.BLEDevice.Address)
            if not await sensor.asyncConnect():
                print("connect device: " + sensor.BLEDevice.Name + " failed")
                continue

        await asyncio.sleep(1)

        # init & start data transfer
        if sensor.deviceState == DeviceStateEx.Ready and not sensor.hasInited:
            await sensor.asyncSetParam("DEBUG_BLE_DATA_PATH", "d:/temp/test.csv")
            await sensor.asyncSetParam("NTF_ECG", "OFF")
            await sensor.asyncSetParam("NTF_IMU", "OFF")
            await sensor.asyncSetParam("FILTER_50HZ", "OFF")
            await sensor.asyncSetParam("FILTER_60HZ", "OFF")
            await sensor.asyncSetParam("FILTER_HPF", "OFF")
            await sensor.asyncSetParam("FILTER_LPF", "OFF")

            init_success = False
            for attempt in range(1, MAX_RETRIES):
                print(f"Initializing {sensor.BLEDevice.Name}, attempt {attempt}...")
                if await sensor.asyncInit(PACKAGE_COUNT, POWER_REFRESH_PERIOD_IN_MS):
                    init_success = True
                    break
                print(f"Init attempt {attempt} failed, retrying in 1s...")
                await asyncio.sleep(1)

            if not init_success:
                print(f"init device {sensor.BLEDevice.Name} failed after {MAX_RETRIES} attempts, skip")
                continue

            await sensor.asyncSetParam("NTF_GEST", "ON")
            deviceInfo = sensor.getDeviceInfo()
            print("deviceInfo: Model: " + str(deviceInfo.ModelName))

        if sensor.hasInited:
            print("start data transfer")
            if not await sensor.asyncStartDataNotification():
                print("start data transfer with device: " + sensor.BLEDevice.Name + " failed")
                continue

        pre_gestID = 0
        while not terminated:
            await asyncio.sleep(0.5)
            print(f"gesture ID: {gestureID}, pre_gestID: {pre_gestID}")
            if (gestureID != pre_gestID):
                match (gestureID):
                    case 0:
                        continue
                    case 1:
                        await gestures_control(GESTURES["FIST"])
                    case 2:
                        await gestures_control(GESTURES["SPREAD"])
                    case 3:
                        await gestures_control(GESTURES["SHOOT"])
                    case 4:
                        await gestures_control(GESTURES["ROCK"])
                    case 5:
                        continue

                pre_gestID = gestureID
            else:
                continue
            pass

        await sensor.stopDataNotification()
        await sensor.disconnect()
        SensorControllerInstance.terminate()

    # SensorControllerInstance.terminate()


def onDataCallback(sensor: SensorProfile, data: SensorData):
    global gestureID
    if data.dataType == DataType.NTF_GEST:
        gestureID = data.channelSamples[0][0].data
    pass


def onPowerChanged(sensor: SensorProfile, power: int):
    # print("connected sensor: " + sensor.BLEDevice.Name + " power: " + str(power))
    pass


def onStateChanged(sensor: SensorProfile, newstate: DeviceStateEx):
    print("device: " + sensor.BLEDevice.Name + str(newstate))


def onErrorCallback(sensor: SensorProfile, reason: str):
    print("device: " + sensor.BLEDevice.Name + reason)
    pass


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
