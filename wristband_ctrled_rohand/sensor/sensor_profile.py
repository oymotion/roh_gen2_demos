

from concurrent.futures import ThreadPoolExecutor
from enum import Enum, IntEnum
from queue import Queue
import threading
import time
from typing import Callable, Optional
import uuid

import asyncio
import sys

from sensor import sensor_utils
from sensor.sensor_data import SensorData

from sensor.sensor_device import BLEChipType, BLEDevice, DeviceInfo, DeviceStateEx
from sensor.sensor_utils import async_call, sync_call, async_exec
from sensor.sdk_log import SdkLog

_TAG = "SensorProfile"

SERVICE_GUID = "0000ffd0-0000-1000-8000-00805f9b34fb"
RFSTAR_SERVICE_GUID = "00001812-0000-1000-8000-00805f9b34fb"

_STATE_NAME_MAP = {
    "Disconnected": DeviceStateEx.Disconnected,
    "Connecting": DeviceStateEx.Connecting,
    "Connected": DeviceStateEx.Connected,
    "Ready": DeviceStateEx.Ready,
    "Disconnecting": DeviceStateEx.Disconnecting,
}


class SensorProfile:


    def __init__(
        self,
        device=None,
        adv=None,
        mac=None,
        serialized=None,
        bleak_host=None,
    ):

        self._bleak_host = bleak_host

        if serialized is not None:
            self._device_address = serialized["address"]
            self._device_name = serialized["name"]
            self._device_mac = serialized["mac"]
            self._device_rssi = serialized["rssi"]
            self._service_data = {
                k: bytes.fromhex(v) if isinstance(v, str) else v
                for k, v in serialized["service_data"].items()
            }
        else:
            if device is not None:
                if hasattr(device, "Name"):
                    # Our BLEDevice wrapper
                    self._device_name = device.Name
                    self._device_mac = device.Address if hasattr(device, "Address") else mac
                    self._device_rssi = device.RSSI if hasattr(device, "RSSI") else 0
                else:
                    # bleak BLEDevice
                    self._device_name = device.name
                    self._device_mac = mac
                    self._device_rssi = adv.rssi if adv else 0
                self._device_address = getattr(device, "address", self._device_mac)
            else:
                self._device_name = ""
                self._device_mac = mac
                self._device_rssi = 0
                self._device_address = mac

            self._service_data = {}
            if adv is not None and hasattr(adv, "service_data"):
                self._service_data = adv.service_data

        self._device = BLEDevice(self._device_name, self._device_mac, self._device_rssi)
        self._device_state = DeviceStateEx.Disconnected
        self._on_state_changed: Callable[["SensorProfile", DeviceStateEx], None] = None
        self._on_error_callback: Callable[["SensorProfile", str], None] = None
        self._on_data_callback: Callable[["SensorProfile", SensorData], None] = None
        self._on_power_changed: Callable[["SensorProfile", int], None] = None
        self._power = -1
        self._power_interval = 0
        self._is_starting = False
        self._is_setting_param = False
        self._has_inited = False
        self._is_data_transfering = False
        self._device_info: Optional[DeviceInfo] = None
        self._chip_type: BLEChipType = BLEChipType.Unknown

        # 用于在独立线程中执行用户回调，避免阻塞 BLE/数据解析线程
        # onDataCallback 使用单线程池，保证数据回调严格按到达顺序执行
        self._callback_executor = ThreadPoolExecutor(max_workers=4)
        self._data_callback_executor = ThreadPoolExecutor(max_workers=1)

        # 数据回调 epoch：每次开始/停止数据流时递增，用于丢弃停止前已提交但未执行的旧回调
        self._data_callback_epoch = 0

    def __del__(self) -> None:

        if not sys.is_finalizing():
            self._destroy()

    def _destroy(self):
        try:
            if self._device_state == DeviceStateEx.Connected or self._device_state == DeviceStateEx.Ready:
                self.disconnect()
        except Exception as e:
            SdkLog.e(_TAG, f"Error occurred while destroying SensorProfile: {e}")
        self._is_starting = False
        self._is_setting_param = False
        # 递增 epoch，丢弃尚未执行的数据回调
        self._data_callback_epoch += 1
        try:
            self._callback_executor.shutdown(wait=False)
            self._data_callback_executor.shutdown(wait=False)
        except Exception as e:
            SdkLog.e(_TAG, f"Error occurred while shutting down callback executors: {e}")

    # ------------------------------------------------------------------
    # Command helpers
    # ------------------------------------------------------------------
    def _send_cmd_sync(self, cmd: dict, timeout: float = 10.0) -> dict:

        if self._bleak_host is None:
            return {}
        cmd["device_mac"] = self._device.Address
        return self._bleak_host.send_command_sync(cmd, timeout=timeout)

    async def _send_cmd_async(self, cmd: dict, timeout: float = 10.0) -> dict:

        if self._bleak_host is None:
            return {}
        cmd["device_mac"] = self._device.Address
        return await self._bleak_host.send_command_async(cmd, timeout=timeout)

    def _submit_callback(
        self,
        callback: Callable,
        *args,
        executor: ThreadPoolExecutor = None,
        error_msg: str = "",
        epoch: int = None,
    ):
        """将用户回调提交到线程池执行，避免阻塞 BLE/数据解析线程。

        当传入 epoch 且与当前数据回调 epoch 不一致时，说明数据流已经停止/重启，
        该回调会被静默丢弃。
        """
        if callback is None:
            return

        if executor is None:
            executor = self._callback_executor

        captured_epoch = epoch

        def _run():
            if captured_epoch is not None and captured_epoch != self._data_callback_epoch:
                return
            try:
                callback(*args)
            except Exception as e:
                if error_msg:
                    SdkLog.e(_TAG, f"{error_msg}: {e}")

        try:
            executor.submit(_run)
        except Exception as e:
            SdkLog.e(_TAG, f"Error occurred while submitting callback: {e}")

    def _on_subprocess_message(self, msg: dict):

        msg_type = msg.get("type")
        if msg_type == "state_changed":
            state_name = msg.get("state")
            new_state = _STATE_NAME_MAP.get(state_name, self._device_state)
            self._set_device_state(new_state)
        elif msg_type == "power_changed":
            power = msg.get("power", -1)
            if power < self._power or self._power == -1:
                self._power = power
            SdkLog.d(_TAG, f"onPowerChanged triggered: {self._device_mac} power={self._power}")
            self._submit_callback(
                self._on_power_changed, self, self._power
            )
        elif msg_type == "sensor_data":
            # 数据流未在进行时，丢弃延迟到达的数据消息
            if not self._is_data_transfering:
                return
            self._submit_callback(
                self._on_data_callback,
                self,
                msg.get("data"),
                executor=self._data_callback_executor,
                error_msg="Error occurred while processing sensor data",
                epoch=self._data_callback_epoch,
            )
        elif msg_type == "error":
            SdkLog.d(_TAG, f"onErrorCallback triggered: {self._device_mac} message={msg.get('message', '')}")
            self._submit_callback(
                self._on_error_callback,
                self,
                msg.get("message", ""),
                error_msg="Error occurred while processing error message",
            )
        elif msg_type == "chip_type":
            chip_value = msg.get("chip_type")
            self._chip_type = BLEChipType(chip_value) if chip_value is not None else BLEChipType.Unknown

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    @property
    def deviceState(self) -> DeviceStateEx:

        return self._device_state

    def _set_device_state(self, newState: DeviceStateEx):
        if self._device_state != newState:
            self._device_state = newState
            if newState == DeviceStateEx.Disconnected:
                self._has_inited = False
                self._is_data_transfering = False
            if self._on_state_changed is not None:
                SdkLog.d(_TAG, f"onStateChanged triggered: {self._device_mac} state={newState}")
                try:
                    self._on_state_changed(self, newState)
                except Exception as e:
                    SdkLog.e(_TAG, f"Error occurred while processing state change: {e}")
                    raise RuntimeError("Set device state %s fail: %s" % (self.BLEDevice.Name , e))

    @property
    def hasInited(self) -> bool:

        return self._has_inited

    @property
    def isDataTransfering(self) -> bool:

        return self._is_data_transfering

    @property
    def BLEDevice(self) -> BLEDevice:

        return self._device

    @property
    def onStateChanged(self) -> Callable[["SensorProfile", DeviceStateEx], None]:

        return self._on_state_changed

    @onStateChanged.setter
    def onStateChanged(self, callback: Callable[["SensorProfile", DeviceStateEx], None]):
        SdkLog.d(_TAG, "onStateChanged registered")
        self._on_state_changed = callback

    @property
    def onErrorCallback(self) -> Callable[["SensorProfile", str], None]:

        return self._on_error_callback

    @onErrorCallback.setter
    def onErrorCallback(self, callback: Callable[["SensorProfile", str], None]):
        SdkLog.d(_TAG, "onErrorCallback registered")
        self._on_error_callback = callback

    @property
    def onDataCallback(self) -> Callable[["SensorProfile", SensorData], None]:

        return self._on_data_callback

    @onDataCallback.setter
    def onDataCallback(self, callback: Callable[["SensorProfile", SensorData], None]):
        SdkLog.d(_TAG, "onDataCallback registered")
        self._on_data_callback = callback

    @property
    def onPowerChanged(self) -> Callable[["SensorProfile", int], None]:

        return self._on_power_changed

    @onPowerChanged.setter
    def onPowerChanged(self, callback: Callable[["SensorProfile", int], None]):
        SdkLog.d(_TAG, "onPowerChanged registered")
        self._on_power_changed = callback

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------
    @staticmethod
    def _connect_timeout() -> float:
        # bumble 后端经 USB dongle 建链 + 服务发现 + MTU 交换耗时更长，
        # 放宽连接超时，与子进程侧 25s 上限对齐
        if sensor_utils._ble_backend_name == "bumble":
            return 25.0
        return sensor_utils._TIMEOUT

    async def _connect(self) -> bool:
        from sensor import sensor_utils

        if sensor_utils._terminated:
            return False
        if self.deviceState == DeviceStateEx.Connected or self.deviceState == DeviceStateEx.Ready:
            return True

        self._set_device_state(DeviceStateEx.Connecting)

        cmd = {
            "type": "connect",
            "device_address": self._device_address,
            "name": self._device_name,
            "service_data": self._service_data,
        }
        result = await self._send_cmd_async(cmd, timeout=self._connect_timeout())
        success = result.get("success", False) if result else False
        return success

    def connect(self) -> bool:
        SdkLog.d(_TAG, f"connect called: {self._device_mac}")
        result = sync_call(self._connect(), _timeout=self._connect_timeout())
        return result

    async def asyncConnect(self) -> bool:
        SdkLog.d(_TAG, f"asyncConnect called: {self._device_mac}")
        return await async_call(self._connect(), _timeout=self._connect_timeout())

    async def _disconnect(self) -> bool:
        if self.deviceState != DeviceStateEx.Connected and self.deviceState != DeviceStateEx.Ready:
            return True

        # 与 Android SDK 对齐：断开前若正在推流，先停止数据通知
        if self._is_data_transfering:
            await self._stopDataNotification()

        cmd = {"type": "disconnect"}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        self._has_inited = False
        self._is_data_transfering = False
        return True

    def disconnect(self) -> bool:
        SdkLog.d(_TAG, f"disconnect called: {self._device_mac}")
        return sync_call(self._disconnect())

    async def asyncDisconnect(self) -> bool:
        SdkLog.d(_TAG, f"asyncDisconnect called: {self._device_mac}")
        return await async_call(self._disconnect())

    # ------------------------------------------------------------------
    # Data notification
    # ------------------------------------------------------------------
    async def _startDataNotification(self) -> bool:
        if self.deviceState != DeviceStateEx.Ready:
            return False
        if not self._has_inited:
            return False

        cmd = {"type": "start_notification"}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        success = result.get("success", False) if result else False
        if success:
            self._is_data_transfering = True
            # 开始新数据流时递增 epoch，丢弃旧流尚未执行的回调
            self._data_callback_epoch += 1
        return success

    def startDataNotification(self) -> bool:
        SdkLog.d(_TAG, f"startDataNotification called: {self._device_mac}")
        if self._is_starting:
            return False

        try:
            self._is_starting = True
            ret = sync_call(self._startDataNotification())
            self._is_starting = False
            return ret
        except Exception as e:
            self._is_starting = False
            SdkLog.exception(_TAG, "startDataNotification failed")
            raise

    async def asyncStartDataNotification(self) -> bool:
        SdkLog.d(_TAG, f"asyncStartDataNotification called: {self._device_mac}")
        if self._is_starting:
            return False

        try:
            self._is_starting = True
            ret = await async_call(self._startDataNotification())
            self._is_starting = False
            return ret
        except Exception as e:
            self._is_starting = False
            SdkLog.exception(_TAG, "asyncStartDataNotification failed")
            raise

    async def _stopDataNotification(self) -> bool:
        if self.deviceState != DeviceStateEx.Ready:
            return False
        if not self._has_inited:
            return False

        cmd = {"type": "stop_notification"}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        success = result.get("success", False) if result else False
        if success:
            self._is_data_transfering = False
            # 停止数据流时递增 epoch，丢弃已提交但未执行的旧数据回调
            self._data_callback_epoch += 1
        return success

    def stopDataNotification(self) -> bool:
        SdkLog.d(_TAG, f"stopDataNotification called: {self._device_mac}")
        if self._is_starting:
            return False

        try:
            self._is_starting = True
            ret = sync_call(self._stopDataNotification())
            self._is_starting = False
            return ret
        except Exception as e:
            self._is_starting = False
            SdkLog.exception(_TAG, "stopDataNotification failed")
            raise

    async def asyncStopDataNotification(self) -> bool:
        SdkLog.d(_TAG, f"asyncStopDataNotification called: {self._device_mac}")
        if self._is_starting:
            return False

        try:
            self._is_starting = True
            ret = await async_call(self._stopDataNotification())
            self._is_starting = False
            return ret
        except Exception as e:
            self._is_starting = False
            SdkLog.exception(_TAG, "asyncStopDataNotification failed")
            raise

    # ------------------------------------------------------------------
    # Init
    # ------------------------------------------------------------------
    async def _init(self, packageSampleCount: int, powerRefreshInterval: int) -> bool:
        if self.deviceState != DeviceStateEx.Ready:
            return False

        self._power_interval = powerRefreshInterval

        cmd = {
            "type": "init",
            "package_sample_count": packageSampleCount,
            "power_refresh_interval": powerRefreshInterval,
        }
        result = await self._send_cmd_async(cmd, timeout=20.0)
        success = result.get("success", False) if result else False
        if success:
            self._has_inited = True
            self._device_info = result.get("device_info")
        return success

    def init(self, packageSampleCount: int, powerRefreshInterval: int) -> bool:
        SdkLog.d(_TAG, f"init called: {self._device_mac} packageSampleCount={packageSampleCount} powerRefreshInterval={powerRefreshInterval}")
        return sync_call(
            self._init(packageSampleCount, powerRefreshInterval),
            20,
        )

    async def asyncInit(self, packageSampleCount: int, powerRefreshInterval: int) -> bool:
        SdkLog.d(_TAG, f"asyncInit called: {self._device_mac} packageSampleCount={packageSampleCount} powerRefreshInterval={powerRefreshInterval}")
        return await async_call(
            self._init(packageSampleCount, powerRefreshInterval),
            20,
        )

    # ------------------------------------------------------------------
    # Battery
    # ------------------------------------------------------------------
    async def _asyncGetBatteryLevel(self) -> int:
        if self.deviceState != DeviceStateEx.Ready:
            return -1
        if not self._has_inited:
            return -1

        cmd = {"type": "get_battery"}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        if result and result.get("success"):
            return result.get("result", -1)
        return -1

    async def asyncGetBatteryLevel(self) -> int:
        SdkLog.d(_TAG, f"asyncGetBatteryLevel called: {self._device_mac}")
        return await async_call(self._asyncGetBatteryLevel())

    def getBatteryLevel(self) -> int:
        SdkLog.d(_TAG, f"getBatteryLevel called: {self._device_mac}")
        return self._power

    def getDeviceInfo(self) -> Optional[DeviceInfo]:
        SdkLog.d(_TAG, f"getDeviceInfo called: {self._device_mac}")
        if self.hasInited:
            return self._device_info
        return None

    # ------------------------------------------------------------------
    # Neucir
    # ------------------------------------------------------------------
    async def _asyncSet_neucir_app_control(self, open: bool, close: bool, stop: bool) -> str:
        if self.deviceState != DeviceStateEx.Ready:
            return "Error: Please connect first"
        if not self._has_inited:
            return "Error: Not initialized"

        cmd = {
            "type": "set_neucir_app_control",
            "open": open,
            "close": close,
            "stop": stop,
        }
        result = await self._send_cmd_async(cmd, timeout=10.0)
        if result and result.get("success"):
            return result.get("result", "OK")
        return result.get("result", "Error: Unknown error") if result else "Error: Unknown error"

    async def _asyncSet_neucir_mode(self, mode: int) -> str:
        if self.deviceState != DeviceStateEx.Ready:
            return "Error: Please connect first"
        if not self._has_inited:
            return "Error: Not initialized"

        cmd = {"type": "set_neucir_mode", "mode": mode}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        if result and result.get("success"):
            return result.get("result", "OK")
        return result.get("result", "Error: Unknown error") if result else "Error: Unknown error"

    # ------------------------------------------------------------------
    # SetParam
    # ------------------------------------------------------------------
    async def _setParam(self, key: str, value: str) -> str:
        if self.deviceState != DeviceStateEx.Ready:
            return "Error: Please connect first"

        if key == "DEBUG_LOG_PATH":
            if value == "False" or value == "":
                SdkLog.set_log_path("")
            elif value == "True":
                value = SdkLog.get_default_log_path(prefix=self.BLEDevice.Name)
                SdkLog.set_log_path(value)
            else:
                SdkLog.set_log_path(value)

        if key == "DEBUG_BLE_DATA_PATH":
            if value == "True":
                # 默认导出路径改为 .bin（bin 导出而非实时 CSV）
                value = SdkLog.get_default_data_log_path(prefix=self.BLEDevice.Name)[:-4] + ".bin"
            # "False"/"" 保持原值，由子进程关闭

        cmd = {"type": "set_param", "key": key, "value": value}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        if result:
            return result.get("result", "Error: Unknown error")
        return "Error: Timeout"

    def setParam(self, key: str, value: str) -> str:
        SdkLog.d(_TAG, f"setParam called: {self._device_mac} key={key} value={value}")
        if self._is_setting_param:
            return "Error: Please wait for the previous operation to complete"

        try:
            self._is_setting_param = True
            ret = sync_call(
                self._setParam(key, value),
                10,
            )
            self._is_setting_param = False
            return ret
        except Exception as e:
            self._is_setting_param = False
            SdkLog.exception(_TAG, f"setParam({key}) failed")
            raise

    async def asyncSetParam(self, key: str, value: str) -> str:
        SdkLog.d(_TAG, f"asyncSetParam called: {self._device_mac} key={key} value={value}")
        if self._is_setting_param:
            return "Error: Please wait for the previous operation to complete"

        try:
            self._is_setting_param = True
            ret = await async_call(
                self._setParam(key, value),
                10,
            )
            self._is_setting_param = False
            return ret
        except Exception as e:
            self._is_setting_param = False
            SdkLog.exception(_TAG, f"asyncSetParam({key}) failed")
            raise

    # ------------------------------------------------------------------
    # GetParam
    # ------------------------------------------------------------------
    async def _getParam(self, key: str) -> str:
        if self.deviceState != DeviceStateEx.Ready:
            return "Error: Please connect first"

        if key == "DEBUG_LOG_PATH":
            return SdkLog.get_log_path() or ""

        cmd = {"type": "get_param", "key": key}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        if result:
            return result.get("result", "Error: Unknown error")
        return "Error: Timeout"

    def getParam(self, key: str) -> str:
        SdkLog.d(_TAG, f"getParam called: {self._device_mac} key={key}")
        return sync_call(
            self._getParam(key),
            10,
        )

    async def asyncGetParam(self, key: str) -> str:
        SdkLog.d(_TAG, f"asyncGetParam called: {self._device_mac} key={key}")
        return await async_call(
            self._getParam(key),
            10,
        )
