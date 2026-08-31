import asyncio
from concurrent.futures import ThreadPoolExecutor
import multiprocessing
import os
import queue as queue_module
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from sensor import sensor_profile
from sensor import sensor_utils
from sensor.bin_recorder import (
    BIN_RECORD_CONFIG,
    BIN_RECORD_DATA,
    BIN_RECORD_HEADER,
    _HEADER_PAYLOAD_SIZE,
    _HEADER_PAYLOAD_STRUCT,
    decode_bin_config,
    iter_bin_records,
)
from sensor.bleak_host import BleakHost
from sensor.sensor_device import BLEDevice
from sensor.sensor_profile import DeviceStateEx, SensorProfile

from sensor.sensor_utils import async_call, sync_call, async_exec
from sensor.sdk_log import SdkLog

_TAG = "SensorController"

SERVICE_GUID = "0000ffd0-0000-1000-8000-00805f9b34fb"
RFSTAR_SERVICE_GUID = "00001812-0000-1000-8000-00805f9b34fb"


class SensorController:
    _instance_lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        if not hasattr(SensorController, "_instance"):
            with SensorController._instance_lock:
                if not hasattr(SensorController, "_instance"):
                    SensorController._instance = object.__new__(cls)

        return SensorController._instance

    """
    SensorController 类的操作包括扫描蓝牙设备以及回调，创建SensorProfile等。
    """

    def __init__(self):

        self._is_scanning = False
        self._device_callback: Callable[[List[sensor_profile.BLEDevice]], None] = None
        self._device_callback_period = 0
        self._enable_callback: Callable[[bool], None] = None
        self._sensor_profiles: Dict[str, SensorProfile] = dict()
        self._profiles_lock = threading.Lock()


        self._bleak_host = BleakHost()
        self._bleak_host_started = False


        self._scan_once_event = threading.Event()
        self._scan_once_result: List[dict] = None
        self._scan_once_devices: List[sensor_profile.BLEDevice] = None


        self._callback_executor = ThreadPoolExecutor(max_workers=2)

        # 当前 BLE 后端名称（"bleak"/"bumble"），由子进程 backend_info 消息更新
        self._ble_backend_name = "bleak"

    def _ensure_bleak_host(self):

        if not self._bleak_host_started:
            self._bleak_host.start()
            self._bleak_host.on_scan_once_result = self._on_bleak_scan_once_result
            self._bleak_host.on_scan_result = self._on_bleak_scan_result
            self._bleak_host.on_device_message = self._on_bleak_device_message
            self._bleak_host.on_backend_info = self._on_bleak_backend_info
            self._bleak_host_started = True

    def __del__(self) -> None:
        pass

    def terminate(self):
        if getattr(self, '_terminated', False):
            return
        self._terminated = True
        SdkLog.controller(_TAG, "terminate called")
        sensor_utils._terminated = True

        for sensor in self._sensor_profiles.values():
            if sensor.deviceState == DeviceStateEx.Connected or sensor.deviceState == DeviceStateEx.Ready:
                sensor._destroy()


        self._callback_executor.shutdown(wait=False)


        if self._bleak_host_started:
            self._bleak_host.stop()
            self._bleak_host_started = False

        sensor_utils.Terminate()

        # 停止日志监听器，确保队列中的日志全部落盘
        SdkLog.stop()

    @property
    def isScanning(self) -> bool:

        return self._is_scanning

    @property
    def isEnable(self) -> bool:

        return True

    @isEnable.setter
    def onEnableCallback(self, callback: Callable[[bool], None]):

        self._enable_callback = callback

    @property
    def hasDeviceFoundCallback(self) -> bool:

        return self._device_callback != None

    @hasDeviceFoundCallback.setter
    def onDeviceFoundCallback(self, callback: Callable[[List[sensor_profile.BLEDevice]], None]):

        self._device_callback = callback

    def _on_bleak_scan_once_result(self, msg: dict):
        serialized_devices = msg.get("devices", [])
        self._scan_once_result = serialized_devices
        devices: List[sensor_profile.BLEDevice] = list()
        with self._profiles_lock:
            deviceMap: Dict[str, SensorProfile] = self._sensor_profiles.copy()
            for serialized in serialized_devices:
                mac = serialized.get("mac")
                if mac is None:
                    continue
                if deviceMap.get(mac) is not None:
                    self._sensor_profiles[mac].BLEDevice.RSSI = serialized.get("rssi")
                    devices.append(self._sensor_profiles[mac].BLEDevice)
                else:
                    newSensor = SensorProfile(serialized=serialized, bleak_host=self._bleak_host)
                    deviceMap[mac] = newSensor
                    devices.append(newSensor.BLEDevice)
            self._sensor_profiles = deviceMap
        self._scan_once_devices = devices
        self._scan_once_event.set()

    def _on_bleak_scan_result(self, msg: dict):
        serialized_devices = msg.get("devices", [])
        devices: List[sensor_profile.BLEDevice] = list()
        with self._profiles_lock:
            deviceMap: Dict[str, SensorProfile] = self._sensor_profiles.copy()
            for serialized in serialized_devices:
                mac = serialized.get("mac")
                if mac is None:
                    continue
                if deviceMap.get(mac) is not None:
                    # 已注册设备名称以首次注册为准，空名广播只更新 RSSI
                    self._sensor_profiles[mac].BLEDevice.RSSI = serialized.get("rssi")
                    devices.append(self._sensor_profiles[mac].BLEDevice)
                else:
                    # 新设备：空名（只收到广播包、scan response 的名字还没到）
                    # 不注册也不回调，等后续轮次拿到名字再出现
                    if not serialized.get("name"):
                        SdkLog.controller(_TAG, f"skip nameless device: {mac}")
                        continue
                    newSensor = SensorProfile(serialized=serialized, bleak_host=self._bleak_host)
                    deviceMap[mac] = newSensor
                    devices.append(newSensor.BLEDevice)
            self._sensor_profiles = deviceMap
        # 过滤空名设备后本轮无可回调设备时不回调（避免空列表打扰应用层）
        if not sensor_utils._terminated and self._device_callback and devices:
            SdkLog.controller(_TAG, f"onDeviceFoundCallback triggered with {len(devices)} devices")
            try:
                self._callback_executor.submit(self._device_callback, devices)
            except Exception as e:
                raise RuntimeError("Scan device fail: %s" % (e))
        if not sensor_utils._terminated and self._is_scanning:
            try:
                self._bleak_host.start_scan(self._device_callback_period)
            except Exception as e:
                SdkLog.exception(_TAG, "Error restarting scan")

    def _on_bleak_device_message(self, device_mac: str, msg: dict):
        if device_mac is not None and device_mac in self._sensor_profiles:
            try:
                self._sensor_profiles[device_mac]._on_subprocess_message(msg)
            except Exception as e:
                SdkLog.exception(_TAG, f"Error handling device message for {device_mac}")

    def _on_bleak_backend_info(self, msg: dict):
        backend = msg.get("backend", "bleak")
        self._ble_backend_name = backend
        sensor_utils._ble_backend_name = backend
        SdkLog.controller(_TAG, f"BLE backend: {backend} {msg.get('transport', '')}")

    def getBLEBackendName(self) -> str:
        """返回当前 BLE 后端名称（"bleak" 原生后端 / "bumble" bleak_bumble 后端）。"""
        return self._ble_backend_name

    def getVersion(self) -> str:
        """返回 SDK 版本号（sensor/__init__.py 的 __version__，惰性导入避免循环依赖）。"""
        from sensor import __version__

        return __version__

    def checkSetupDongle(self) -> str:
        """检查 USB BLE dongle 是否已装好驱动/权限、可供 bumble 后端使用。

        逻辑实现在 sensor_utils.checkSetupDongle()（也可经 sensor 包顶层
        导出直接调用，无需 controller 实例）；本方法额外处理已连接场景：
        已有设备连接时不跑安装脚本，直接按当前检测结果返回（连接占用的
        dongle 必然可用；检测不到时退化为无数量的 "OK"，如 bleak 后端）。
        返回 "OK: N"（N 为可用 dongle 数量）或 "OK"；失败返回 "Error: ..."。
        """
        if self.getConnectedSensors():
            from sensor import bumble_dongle

            specs = bumble_dongle.detect_usb_dongle_specs()
            return f"OK: {len(specs)}" if specs else "OK"
        return sensor_utils.checkSetupDongle()

    def scan(self, period) -> List[sensor_profile.BLEDevice]:
        SdkLog.controller(_TAG, f"scan called: period={period}")
        self._ensure_bleak_host()
        self._scan_once_event.clear()
        self._scan_once_result = None
        self._bleak_host.scan_once(period)

        timeout = sensor_utils._TIMEOUT + period / 1000
        if not self._scan_once_event.wait(timeout=timeout):
            return []

        if self._scan_once_devices is None:
            return []

        return self._scan_once_devices

    async def asyncScan(self, period) -> List[sensor_profile.BLEDevice]:
        SdkLog.controller(_TAG, f"asyncScan called: period={period}")
        self._ensure_bleak_host()
        self._scan_once_event.clear()
        self._scan_once_result = None
        self._bleak_host.scan_once(period)

        timeout = sensor_utils._TIMEOUT + period / 1000
        start = time.time()
        while not self._scan_once_event.is_set() and time.time() - start < timeout:
            await asyncio.sleep(0.05)

        if self._scan_once_devices is None:
            return []

        return self._scan_once_devices

    def startScan(self, periodInMs: int) -> bool:
        SdkLog.controller(_TAG, f"startScan called: period={periodInMs}")
        if self._is_scanning:
            return True

        self._ensure_bleak_host()
        self._is_scanning = True
        self._device_callback_period = periodInMs

        self._bleak_host.start_scan(periodInMs)
        return True

    def stopScan(self) -> None:
        SdkLog.controller(_TAG, "stopScan called")
        if not self._is_scanning:
            return

        self._is_scanning = False
        try:
            self._bleak_host.stop_scan()
        except Exception as e:
            SdkLog.exception(_TAG, "Error stopping scan")

    def requireSensor(self, device: sensor_profile.BLEDevice) -> Optional[SensorProfile]:
        SdkLog.controller(_TAG, f"requireSensor called: {device.Address if device else None}")
        with self._profiles_lock:
            if self._sensor_profiles.get(device.Address) == None:
                newSensor = SensorProfile(device=device, bleak_host=self._bleak_host)
                self._sensor_profiles[device.Address] = newSensor

            return self._sensor_profiles[device.Address]

    def getSensor(self, deviceMac: str) -> Optional[SensorProfile]:
        SdkLog.controller(_TAG, f"getSensor called: {deviceMac}")
        with self._profiles_lock:
            return self._sensor_profiles.get(deviceMac)

    def getConnectedSensors(self) -> List[SensorProfile]:
        SdkLog.controller(_TAG, "getConnectedSensors called")
        sensors: List[SensorProfile] = list()
        with self._profiles_lock:
            for sensor in self._sensor_profiles.values():
                if sensor.deviceState == DeviceStateEx.Connected or sensor.deviceState == DeviceStateEx.Ready:
                    sensors.append(sensor)

        return sensors

    def getConnectedDevices(self) -> List[sensor_profile.BLEDevice]:
        SdkLog.controller(_TAG, "getConnectedDevices called")
        devices: List[sensor_profile.BLEDevice] = list()
        with self._profiles_lock:
            for sensor in self._sensor_profiles.values():
                if sensor.deviceState == DeviceStateEx.Connected or sensor.deviceState == DeviceStateEx.Ready:
                    devices.append(sensor.BLEDevice)

        return devices

    # ------------------------------------------------------------------
    # Bin file replay
    # ------------------------------------------------------------------
    def getBinFileInfo(self, file_path: str) -> Optional[dict]:
        """读取 bin 文件信息：第一条配置记录（device_mac、device_name、device_info、
        各数据类型解析配置等）加上 ``replay_duration`` 字段（录制时长，秒）。

        新版 bin 文件第一条记录是头部记录，时长在文件关闭时写入，读取很快；
        没有头部记录的旧文件回退为扫描整个文件估算时长。
        文件不存在或没有配置记录时返回 None。
        """
        if not file_path or not os.path.isfile(file_path):
            return None
        config = None
        header_duration = None
        first_ts = None
        last_ts = None
        try:
            for rec_type, ts, payload in iter_bin_records(file_path):
                if rec_type == BIN_RECORD_HEADER:
                    # 头部记录：时长已在关闭时写入，无需继续扫描数据记录
                    if header_duration is None and len(payload) >= _HEADER_PAYLOAD_SIZE:
                        f_ts, l_ts = _HEADER_PAYLOAD_STRUCT.unpack(payload[:_HEADER_PAYLOAD_SIZE])
                        if l_ts > 0 and l_ts >= f_ts:
                            header_duration = (l_ts - f_ts) / 1000.0
                elif rec_type == BIN_RECORD_CONFIG:
                    if config is None:
                        config = decode_bin_config(payload)
                elif rec_type == BIN_RECORD_DATA:
                    if first_ts is None:
                        first_ts = ts
                    last_ts = ts
                if config is not None and header_duration is not None:
                    break
        except Exception:
            pass
        if config is None:
            return None
        if header_duration is not None:
            config["replay_duration"] = header_duration
        elif first_ts is not None and last_ts is not None:
            # 旧文件（无头部记录）：扫描首末数据记录估算
            config["replay_duration"] = max(0.0, (last_ts - first_ts) / 1000.0)
        else:
            config["replay_duration"] = 0.0
        return config

    def replayBinFile(
        self,
        file_path: str,
        sensor: Optional[SensorProfile] = None,
        realtime: bool = True,
        timeout: Optional[float] = None,
    ) -> Optional[SensorProfile]:
        """回放 bin 文件中的原始蓝牙数据，解析结果通过 sensor 的 onDataCallback 正常回调。

        Args:
            file_path: bin 文件路径（设备连接期间自动记录在 SDK 日志目录下）。
            sensor: 已有的 SensorProfile；为 None 时根据 bin 文件中的配置记录
                自动创建（或复用同 MAC 的已有 profile）。
            realtime: True 按记录时间间隔回放；False 全速回放。
            timeout: 等待回放完成的超时时间（秒）；为 None 时按 bin 文件时长
                自动估算（realtime）或使用默认值 600 秒。
                超时返回时回放可能仍在后台继续。

        Returns:
            用于回放的 SensorProfile；无法开始回放时返回 None。

        注意：
            - 目标 sensor 正在传输实时数据时回放会被拒绝。
            - ``sensor=None`` 自动创建模式下，profile 在回放结束时才返回，
              本次回放的数据回调无法接收；可保存返回的 profile 再次回放，
              或先通过 ``getSensor(mac)`` / ``requireSensor(device)`` 创建并注册回调。
        """
        SdkLog.controller(_TAG, f"replayBinFile called: {file_path} realtime={realtime}")
        if not file_path or not os.path.isfile(file_path):
            SdkLog.e(_TAG, f"replayBinFile: file not found: {file_path}")
            return None
        self._ensure_bleak_host()

        info = self.getBinFileInfo(file_path)
        if timeout is None:
            if realtime:
                duration = (info or {}).get("replay_duration", 0.0)
                timeout = max(60.0, duration + 30.0)
            else:
                timeout = 600.0

        if sensor is None:
            config = info
            if config is None:
                SdkLog.e(_TAG, f"replayBinFile: no config record in {file_path}")
                return None
            mac = config.get("device_mac")
            if not mac:
                SdkLog.e(_TAG, f"replayBinFile: config record missing device_mac")
                return None
            name = config.get("device_name") or ""
            with self._profiles_lock:
                sensor = self._sensor_profiles.get(mac)
                if sensor is None:
                    sensor = SensorProfile(device=BLEDevice(name, mac, 0), bleak_host=self._bleak_host)
                    self._sensor_profiles[mac] = sensor

        prev_transfering = sensor._is_data_transfering
        sensor._set_data_transfering(True)
        recv_baseline = sensor._received_sensor_data_msgs
        try:
            cmd = {"type": "replay_bin", "path": os.path.abspath(file_path), "realtime": bool(realtime)}
            result = sensor._send_cmd_sync(cmd, timeout=timeout)
            if result and result.get("success"):
                # 全速回放时主进程消费滞后，command_result（优先通道）会先于
                # 数据队列里滞留的 sensor_data 到达；等本次回放发布的消息全部
                # 到达后再恢复传输标志，否则滞留消息会被当作流外迟到数据丢弃
                expected = result.get("published_data_msgs") or 0
                if expected > 0:
                    self._wait_replay_data_arrived(sensor, recv_baseline, expected)
        finally:
            sensor._set_data_transfering(prev_transfering)

        if not result:
            SdkLog.w(_TAG, f"replayBinFile: no result within timeout={timeout}s, replay may still be running")
            return sensor
        if not result.get("success"):
            SdkLog.e(_TAG, f"replayBinFile failed: {result.get('result')}")
            return None
        SdkLog.controller(_TAG, f"replayBinFile finished: {result.get('result')}")
        return sensor

    @staticmethod
    def _wait_replay_data_arrived(sensor: SensorProfile, baseline: int, expected: int,
                                  timeout: float = 30.0):
        """等待本次回放发布的 sensor_data 消息全部到达主进程（上限 timeout 秒）。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            arrived = sensor._received_sensor_data_msgs - baseline
            if arrived >= expected:
                return
            time.sleep(0.05)
        SdkLog.w(_TAG, f"replayBinFile: replay published {expected} data messages but only "
                       f"{sensor._received_sensor_data_msgs - baseline} arrived within {timeout}s")

    def pauseBinReplay(self, sensor: SensorProfile) -> str:
        """暂停 sensor 上正在进行的 bin 文件回放。"""
        return self._setBinReplayPaused(sensor, True)

    def resumeBinReplay(self, sensor: SensorProfile) -> str:
        """恢复 sensor 上已暂停的 bin 文件回放。"""
        return self._setBinReplayPaused(sensor, False)

    def _setBinReplayPaused(self, sensor: SensorProfile, paused: bool) -> str:
        SdkLog.controller(_TAG, f"setBinReplayPaused: {paused} {sensor.BLEDevice.Address if sensor else None}")
        if sensor is None:
            return "Error: no sensor"
        if not self._bleak_host_started:
            return "Error: BLE host not started"
        result = sensor._send_cmd_sync({"type": "pause_replay_bin", "paused": paused}, timeout=10)
        if not result:
            return "Error: Timeout"
        return result.get("result", "Error: Unknown error")

    def stopBinReplay(self, sensor: SensorProfile) -> str:
        """停止 sensor 上正在进行的 bin 文件回放（阻塞中的 replayBinFile 会随之返回）。"""
        SdkLog.controller(_TAG, f"stopBinReplay: {sensor.BLEDevice.Address if sensor else None}")
        if sensor is None:
            return "Error: no sensor"
        if not self._bleak_host_started:
            return "Error: BLE host not started"
        result = sensor._send_cmd_sync({"type": "stop_replay_bin"}, timeout=10)
        if not result:
            return "Error: Timeout"
        return result.get("result", "Error: Unknown error")

    # ------------------------------------------------------------------
    # Synchronized multi-device data notification
    # ------------------------------------------------------------------
    def _prepare_multi_start(self, sensors):
        """校验并登记同步起流参与者，返回 (participants, results)。

        不合法（未注册 / 非 Ready / 未 init / 正在起流）的设备记入 results=False
        并排除；参与者的 _is_starting 置位，由调用方在命令结束后复位。
        """
        results = {}
        participants = []
        seen = set()
        for sensor in sensors or []:
            if sensor is None:
                continue
            mac = sensor.BLEDevice.Address
            if mac in seen:
                continue
            seen.add(mac)
            with self._profiles_lock:
                registered = self._sensor_profiles.get(mac)
            if registered is not sensor:
                SdkLog.w(_TAG, f"multiStartDataNotification: unregistered sensor {mac}")
                results[mac] = False
                continue
            if not sensor.isReady or not sensor._has_inited:
                SdkLog.w(_TAG, f"multiStartDataNotification: not ready or not inited {mac}")
                results[mac] = False
                continue
            if sensor._is_starting:
                SdkLog.w(_TAG, f"multiStartDataNotification: already starting {mac}")
                results[mac] = False
                continue
            sensor._is_starting = True
            participants.append(sensor)
        return participants, results

    def _finish_multi_start(self, participants, results, cmd_result):
        """按子进程汇总结果更新各 profile 状态并填充 mac->bool 结果。"""
        per_device = (cmd_result or {}).get("result") or {}
        for sensor in participants:
            mac = sensor.BLEDevice.Address
            ok = per_device.get(mac) is True
            if ok:
                sensor._apply_start_notification_success()
            elif not cmd_result:
                # 超时无结果：子进程可能仍在执行（与 replayBinFile 超时语义一致）
                SdkLog.w(_TAG, f"multiStartDataNotification: no result within timeout, "
                               f"{mac} may still be starting")
            else:
                SdkLog.w(_TAG, f"multiStartDataNotification failed: {mac}: {per_device.get(mac)}")
                # restart 语义下设备可能已被停流且起流失败，清除传输标志
                # 避免 profile 状态停留在"以为在流"的假象
                sensor._set_data_transfering(False)
            results[mac] = ok
        return results

    def multiStartDataNotification(self, sensors: List[SensorProfile], timeout: float = 30.0, maxDelayDispersionMs: int = 5, maxAttempts: int = 3) -> Dict[str, bool]:
        """同步启动多个设备的数据通知，返回 {MAC: 是否成功}。

        所有 sensor 须已 connect 到 Ready 且 init 成功（hasInited）。bumble
        （USB dongle）后端下，各设备的起流写命令（OYM 的 CCCD 写 / RFSTAR 的
        set_subscription 写）在 bumble 发送层等待同一个 SyncWriteGate，全部
        就绪后几乎同时下发；bleak 原生后端退化为并发起流（无底层对齐）。
        已在流的设备会先同步停流再统一起流（restart 语义）。
        起流后内部校验各设备首包 delay 的离散值（max-min），超过
        maxDelayDispersionMs（默认 5ms；传 -1 时不做离散度校验，
        只要求各设备起流成功并出首包）
        或有设备 2s 未出首包时自动整体停流重试，最多 maxAttempts 次
        （默认 3），仍不达标
        则停掉数据流并返回失败。不合法的设备不影响其余设备起流，
        按 MAC 逐个返回结果。默认 timeout 30s 覆盖 3 次重试的最坏耗时，
        调大 maxAttempts 时应相应调大 timeout。
        """
        SdkLog.controller(
            _TAG,
            f"multiStartDataNotification called: "
            f"{[s.BLEDevice.Address for s in sensors or [] if s]}",
        )
        participants, results = self._prepare_multi_start(sensors)
        if participants:
            self._ensure_bleak_host()
            cmd = {
                "type": "multi_start_notification",
                "device_macs": [s.BLEDevice.Address for s in participants],
                "max_delay_dispersion_ms": maxDelayDispersionMs,
                "max_attempts": maxAttempts,
            }
            try:
                cmd_result = self._bleak_host.send_command_sync(cmd, timeout=timeout)
            finally:
                for s in participants:
                    s._is_starting = False
            self._finish_multi_start(participants, results, cmd_result)
        return results

    async def asyncMultiStartDataNotification(self, sensors: List[SensorProfile], timeout: float = 30.0, maxDelayDispersionMs: int = 5, maxAttempts: int = 3) -> Dict[str, bool]:
        """multiStartDataNotification 的异步变体，语义相同。"""
        SdkLog.controller(
            _TAG,
            f"asyncMultiStartDataNotification called: "
            f"{[s.BLEDevice.Address for s in sensors or [] if s]}",
        )
        participants, results = self._prepare_multi_start(sensors)
        if participants:
            self._ensure_bleak_host()
            cmd = {
                "type": "multi_start_notification",
                "device_macs": [s.BLEDevice.Address for s in participants],
                "max_delay_dispersion_ms": maxDelayDispersionMs,
                "max_attempts": maxAttempts,
            }
            try:
                cmd_result = await self._bleak_host.send_command_async(cmd, timeout=timeout)
            finally:
                for s in participants:
                    s._is_starting = False
            self._finish_multi_start(participants, results, cmd_result)
        return results

    def _prepare_multi_stop(self, sensors):
        """校验并登记同步停流参与者，返回 (participants, results)。

        不合法（未注册 / 非 Ready / 未 init / 正在起停流）的设备记入
        results=False 并排除；参与者的 _is_starting 置位，由调用方在命令
        结束后复位。用户主动停流同时取消各设备的自动恢复标记。
        """
        results = {}
        participants = []
        seen = set()
        for sensor in sensors or []:
            if sensor is None:
                continue
            mac = sensor.BLEDevice.Address
            if mac in seen:
                continue
            seen.add(mac)
            with self._profiles_lock:
                registered = self._sensor_profiles.get(mac)
            if registered is not sensor:
                SdkLog.w(_TAG, f"multiStopDataNotification: unregistered sensor {mac}")
                results[mac] = False
                continue
            if not sensor.isReady or not sensor._has_inited:
                SdkLog.w(_TAG, f"multiStopDataNotification: not ready or not inited {mac}")
                results[mac] = False
                continue
            if sensor._is_starting:
                SdkLog.w(_TAG, f"multiStopDataNotification: already starting/stopping {mac}")
                results[mac] = False
                continue
            sensor._is_starting = True
            # 用户主动停流：取消自动恢复
            sensor._resume_pending = False
            participants.append(sensor)
        return participants, results

    def _finish_multi_stop(self, participants, results, cmd_result):
        """按子进程汇总结果更新各 profile 状态并填充 mac->bool 结果。"""
        per_device = (cmd_result or {}).get("result") or {}
        for sensor in participants:
            mac = sensor.BLEDevice.Address
            ok = per_device.get(mac) is True
            if ok:
                sensor._apply_stop_notification_success()
            elif not cmd_result:
                # 超时无结果：子进程可能仍在执行（与 replayBinFile 超时语义一致）
                SdkLog.w(_TAG, f"multiStopDataNotification: no result within timeout, "
                               f"{mac} may still be stopping")
            else:
                SdkLog.w(_TAG, f"multiStopDataNotification failed: {mac}: {per_device.get(mac)}")
            results[mac] = ok
        return results

    def multiStopDataNotification(self, sensors: List[SensorProfile], timeout: float = 10.0) -> Dict[str, bool]:
        """同步停止多个设备的数据通知，返回 {MAC: 是否成功}。

        与 multiStartDataNotification 同一机制：bumble（USB dongle）后端下，
        各设备的停流写命令（OYM 的 CCCD 写 / RFSTAR 的 set_subscription 写）
        在 bumble 发送层等待同一个 SyncWriteGate，全部就绪后几乎同时下发；
        bleak 原生后端退化为并发停流。未在流的设备视为成功（无需再停），
        不合法的设备不影响其余设备，按 MAC 逐个返回结果。
        """
        SdkLog.controller(
            _TAG,
            f"multiStopDataNotification called: "
            f"{[s.BLEDevice.Address for s in sensors or [] if s]}",
        )
        participants, results = self._prepare_multi_stop(sensors)
        if participants:
            self._ensure_bleak_host()
            cmd = {
                "type": "multi_stop_notification",
                "device_macs": [s.BLEDevice.Address for s in participants],
            }
            try:
                cmd_result = self._bleak_host.send_command_sync(cmd, timeout=timeout)
            finally:
                for s in participants:
                    s._is_starting = False
            self._finish_multi_stop(participants, results, cmd_result)
        return results

    async def asyncMultiStopDataNotification(self, sensors: List[SensorProfile], timeout: float = 10.0) -> Dict[str, bool]:
        """multiStopDataNotification 的异步变体，语义相同。"""
        SdkLog.controller(
            _TAG,
            f"asyncMultiStopDataNotification called: "
            f"{[s.BLEDevice.Address for s in sensors or [] if s]}",
        )
        participants, results = self._prepare_multi_stop(sensors)
        if participants:
            self._ensure_bleak_host()
            cmd = {
                "type": "multi_stop_notification",
                "device_macs": [s.BLEDevice.Address for s in participants],
            }
            try:
                cmd_result = await self._bleak_host.send_command_async(cmd, timeout=timeout)
            finally:
                for s in participants:
                    s._is_starting = False
            self._finish_multi_stop(participants, results, cmd_result)
        return results

    def parseBinToCsv(self, bin_path: str, csv_path: str = None) -> str:
        """离线解析 bin 文件并输出 CSV，返回 CSV 文件路径。

        解析走与在线一致的管线（配置记录 + 原始数据包重放），
        输出与历史实时 DEBUG_BLE_DATA CSV 相同的格式。
        """
        SdkLog.controller(_TAG, f"parseBinToCsv called: {bin_path} -> {csv_path}")
        from sensor.bin_to_csv import bin_to_csv

        return bin_to_csv(bin_path, csv_path)

    # ------------------------------------------------------------------
    # Logging controls (SdkLog is not exposed publicly)
    # ------------------------------------------------------------------
    def setDebugEnabled(self, enabled: bool):
        """开启或关闭 SDK 调试日志。

        开启时若文件输出已启用，自动在日志目录创建 controller log
        （``sensor_controller_log_YYYYMMDD_HHMMSS.txt``，公共日志都写入该文件）；
        关闭时同时关闭 controller log。
        """
        SdkLog.set_debug_enabled(enabled)
        self._sync_debug_enabled_to_subprocess()

    def log(self, message: str, level: str = "I"):
        """记录一条应用日志到 SDK 日志（controller log 公共通道）。

        便于应用把自己的事件（用户操作、业务状态等）与 SDK 日志写进同一文件，
        统一时间线排查问题。

        Args:
            message: 日志内容。
            level: "D"/"I"/"W"/"E"（大小写不敏感），默认 "I"；
                "D" 受 setDebugEnabled 开关控制，其余级别总是输出。
        """
        try:
            SdkLog.log(level, "App", str(message))
        except Exception:
            pass

    def _sync_debug_enabled_to_subprocess(self):
        """把 debug 开关同步给 BLE 子进程（子进程同步创建/关闭 controller log）。"""
        if not self._bleak_host_started:
            return
        try:
            self._bleak_host.send_command(
                {"type": "set_debug_enabled", "enabled": SdkLog.is_debug_enabled(),
                 "controller_log_path": SdkLog.get_controller_log_path()}
            )
        except Exception:
            SdkLog.exception(_TAG, "sync debug enabled to subprocess failed")

    def setLogPath(self, enabled: bool = True, path: str = ""):
        """设置 SDK 日志输出目录（必须是目录）。

        controller log、默认 profile log（DEBUG_LOG_PATH=True）与默认 bin
        导出（DEBUG_BLE_DATA_PATH=True）都写在该目录下。

        Args:
            enabled: True 开启文件输出，False 关闭（controller log 关闭且不再
                自动创建，默认 profile log / 默认 bin 导出随之禁用；显式绝对
                路径的 profile log 仍可用）。
            path: 日志目录；为 "" 时使用默认目录 ``~/Documents/sensorsdklog``。
                目录不存在时自动创建；若指向一个已存在的文件则拒绝本次设置。
        """
        SdkLog.set_log_dir(path if path else None, enabled=enabled)
        self._sync_log_dir_to_subprocess()

    def _sync_log_dir_to_subprocess(self):
        """把当前日志目录与文件输出开关同步给 BLE 子进程，
        保证子进程日志写入同一目录下的同名文件。"""
        if not self._bleak_host_started:
            return
        try:
            self._bleak_host.send_command({
                "type": "set_log_dir",
                "dir": SdkLog.get_log_dir(),
                "enabled": SdkLog.is_file_output_enabled(),
                "controller_log_path": SdkLog.get_controller_log_path(),
            })
        except Exception:
            SdkLog.exception(_TAG, "sync log dir to subprocess failed")


SensorControllerInstance = SensorController()
