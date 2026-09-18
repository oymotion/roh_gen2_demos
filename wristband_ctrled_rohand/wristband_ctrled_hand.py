import asyncio
import signal
import sys
import os
import time
import math

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
   "REST":     [0    , 30000, 30000, 30000, 30000,     0],
   "FIST":     [10000, 65535, 65535, 65535, 65535,     0],
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
   "GRASP":    [27525, 29491, 32768, 27525, 24903, 65535],
   "INDEX":    [    0, 65535, 30000, 30000, 30000,     0],
   "MIDDLE":   [    0, 30000, 65535, 30000, 30000,     0],
   "RING":     [    0, 30000, 30000, 65535, 30000,     0],
   "LITTLE":   [    0, 30000, 30000, 30000, 65535,     0]
}
NODE_ID = 2
NUM_FINGERS = 5

THUMB, INDEX, MIDDLE, RING, LITTLE, THUMB_ROOT = range(6)
MAX_POSITION = 65535
REST_POSE = GESTURES["REST"]
# natural linkage table
COUPLING = {
    INDEX:  {MIDDLE: 0.15},
    MIDDLE: {RING:   0.15},
    RING:   {MIDDLE: 0.25, LITTLE: 0.25},
    LITTLE: {RING:   0.50},
}
GESTURES_THREEHOLD = [80, 100, 80, 90, 100, 100]
GESTURES_APPLY = [GESTURES["INDEX"], GESTURES["MIDDLE"],
                  GESTURES["RING"], GESTURES["LITTLE"],
                  GESTURES["FIST"], GESTURES["SPREAD"]]

SMOOTH_TAU = 0.12
current_pose = [float(v) for v in GESTURES["REST"]]
LOOP_DT = 0.02

terminated = False
gestureID = 0
ges_strength = 0

GEST_DEBUG = True

def interpolate(n, from_min, from_max, to_min, to_max):
    return (n - from_min) / (from_max - from_min) * (to_max - to_min) + to_min

def clamp(v, low, high):
    return max(low, min(high, v))

def scale_gesture(pose, ratio):
    r = clamp(ratio, 0.0, 1.0)
    return [int(clamp(65535 * r, 0, 65535)) if v == 65535 else v for v in pose]

def scale_spread(pose, ratio):
    r = clamp(ratio, 0.0, 1.0)
    return [int(clamp(REST_POSE[i] + (pose[i] - REST_POSE[i]) * r, 0, MAX_POSITION))
            for i in range(len(pose))]

def apply_coupling(pose):
    out = list(pose)
    for src, dragged in COUPLING.items():
        curl = pose[src] - REST_POSE[src]
        if curl <= 0:
            continue
        for dst, factor in dragged.items():
            out[dst] = int(clamp(out[dst] + curl * factor, 0, MAX_POSITION))
    return out


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
        if not write_registers(client, ROH_FINGER_POS_TARGET0, gesture):
            print("Failed to send control command")

    async def smooth_gestures_control(target_pose, dt):
        global current_pose
        alpha = 1.0 - math.exp(-dt / SMOOTH_TAU)
        current_pose = [cur + (tgt - cur) * alpha
                        for cur, tgt in zip(current_pose, target_pose)]
        out = [int(clamp(round(v), 0, MAX_POSITION)) for v in current_pose]
        await gestures_control(out)

    print("start init oyww1000")
    # init OYWW
    if not SensorControllerInstance.isEnable:
        print("please open bluetooth")
        return

    deviceList = await SensorControllerInstance.asyncScan(3000)

    filteredDevice = filter(
        lambda x: x.RSSI > -80 and (x.Name.startswith("gForceU") or x.Name.startswith("OYWW1000")),
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

            gest_result = await sensor.asyncSetParam("NTF_GEST", "ON")
            print(f"setParam NTF_GEST result: {gest_result}")
            deviceInfo = sensor.getDeviceInfo()
            print("deviceInfo: Model: " + str(deviceInfo.ModelName))

        if sensor.hasInited:
            print("start data transfer")
            start_result = await sensor.asyncStartDataNotification()
            print(f"startDataNotification result: {start_result}, isTransfering: {sensor.isDataTransfering}")
            if not start_result:
                print("start data transfer with device: " + sensor.BLEDevice.Name + " failed")
                continue

        loop_count = 0
        last_timer = time.monotonic()
        while not terminated:
            await asyncio.sleep(LOOP_DT)
            now = time.monotonic()
            dt = now - last_timer
            last_t = now

            strength_ratio = 0.0

            match gestureID:
                case 0:
                    await smooth_gestures_control(GESTURES["REST"], dt)
                case 1:
                    strength_ratio = interpolate(ges_strength, 0, GESTURES_THREEHOLD[0], 0, 1)
                    await smooth_gestures_control(apply_coupling(scale_gesture(GESTURES_APPLY[0], strength_ratio)), dt)
                case 2:
                    strength_ratio = interpolate(ges_strength, 0, GESTURES_THREEHOLD[1], 0, 1)
                    await smooth_gestures_control(apply_coupling(scale_gesture(GESTURES_APPLY[1], strength_ratio)), dt)
                case 3:
                    strength_ratio = interpolate(ges_strength, 0, GESTURES_THREEHOLD[2], 0, 1)
                    await smooth_gestures_control(apply_coupling(scale_gesture(GESTURES_APPLY[2], strength_ratio)), dt)
                case 4:
                    strength_ratio = interpolate(ges_strength, 0, GESTURES_THREEHOLD[3], 0, 1)
                    await smooth_gestures_control(apply_coupling(scale_gesture(GESTURES_APPLY[3], strength_ratio)), dt)
                case 5:
                    strength_ratio = interpolate(ges_strength, 0, GESTURES_THREEHOLD[4], 0, 1)
                    await smooth_gestures_control(scale_gesture(GESTURES_APPLY[4], strength_ratio), dt)
                case 6:
                    strength_ratio = interpolate(ges_strength, 0, GESTURES_THREEHOLD[5], 0, 1)
                    await smooth_gestures_control(scale_spread(GESTURES_APPLY[5], strength_ratio), dt)
                case _:
                    pass

        await sensor.stopDataNotification()
        await sensor.disconnect()
        SensorControllerInstance.terminate()

    # SensorControllerInstance.terminate()


def onDataCallback(sensor: SensorProfile, data_list: list):
    global gestureID, ges_strength
    for data in data_list:
        if data.getDataType() != DataType.NTF_GEST:
            continue
        if not data.channelSamples or not data.channelSamples[0]:
            continue
        try:
            new_id = int(data.getData(0, 0))
            raw_id = int(data.getRawData(0, 0))
        except IndexError:
            continue

        if GEST_DEBUG:
            print(f"[GEST] data={new_id}, rawData={raw_id}, "
                  f"impedance(possibility)={data.getImpedance(0, 0)}, "
                  f"saturation(strength)={data.getSaturation(0, 0)}")
        gestureID = new_id
        ges_strength = data.getSaturation(0, 0)

def onPowerChanged(sensor: SensorProfile, power: int):
    # print("connected sensor: " + sensor.BLEDevice.Name + " power: " + str(power))
    pass


def onStateChanged(sensor: SensorProfile, newstate: DeviceStateEx):
    print("device: " + sensor.BLEDevice.Name + str(newstate))


def onErrorCallback(sensor: SensorProfile, reason: str):
    print(f"[ERROR] device: {sensor.BLEDevice.Name} reason: {reason}")


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
