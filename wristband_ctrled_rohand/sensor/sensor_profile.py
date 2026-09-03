

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

# 电量读数稳定带（%）：有效读数与当前显示值之差小于该值时保持显示值不变，
# 过滤 ADC ±2 抖动；慢速漂移（每次刷新降 2%）每 _POWER_STABLE_BAND 步显现一次
_POWER_STABLE_BAND = 4

# 自动重连恢复卡死看门狗：恢复流程结束后每隔该秒数检查一次流是否恢复，
# 连续 _RECOVERY_STUCK_MAX 次未恢复（链路在但设备不应答）则强制断链，
# 交给链路层重连滚动重来
_RECOVERY_STUCK_CHECK_SECONDS = 40.0
_RECOVERY_STUCK_MAX = 3
# onAutoReconnect 异步应答超时：超时未应答按 answer(False) 走默认恢复（对齐 C++ SDK）
_AUTO_RECONNECT_ANSWER_TIMEOUT_SECONDS = 10.0


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
        # 绑定 profile 日志（注册并路由到该 profile 的日志文件，未开启时回落公共通道）
        self._log = SdkLog.bind(self._device_mac)
        self._device_state = DeviceStateEx.Disconnected
        self._on_state_changed: Callable[["SensorProfile", DeviceStateEx], None] = None
        self._on_error_callback: Callable[["SensorProfile", str], None] = None
        self._on_data_callback: Callable[["SensorProfile", "list[SensorData]"], None] = None
        self._on_power_changed: Callable[["SensorProfile", int], None] = None
        # DeviceInfo 更新事件：连接参数/MTU 等链路信息变化时触发（bumble 后端）
        self._on_device_info_update: Callable[["SensorProfile", DeviceInfo], None] = None
        # 数据流开关状态变化事件：只在真实 起流/停流 变化时触发
        # （startDataNotification 成功 / stopDataNotification / 断连 / 回放开始与结束）
        self._on_data_transfer_state_change: Callable[["SensorProfile", bool], None] = None
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
        # 已到达主进程的 sensor_data 消息总数（无论是否被传输标志拦截）：
        # replayBinFile 用它等待本次回放的消息全部到达，避免 IPC 滞留消息被丢弃
        self._received_sensor_data_msgs = 0

        # 自动重连恢复（autoReconnect=True 且断连前正在数据传输时）：
        # 远端异常断连/长时间无数据异常断连后，自动完成 连接→初始化→恢复参数→开始传输
        self._auto_reconnect = True
        self._resume_pending = False       # 断连前正在传输，待恢复
        self._recovering = False           # 恢复流程进行中
        self._last_init_args = None        # 上次成功 init 的 (packageSampleCount, powerRefreshInterval)
        self._saved_params: dict = {}      # 上次传输期间成功设置的 setParam 参数（按设置顺序恢复）
        # 自动重连找到断连设备时的回调（None=走默认恢复流程）
        self._on_auto_reconnect: Optional[Callable[..., None]] = None
        # 恢复卡死看门狗：重连成功但恢复（init/起流）迟迟未完成时计时，
        # 连续卡住达到上限后强制断链，让链路层重连滚动重来
        self._recovery_stuck_timer: Optional[threading.Timer] = None
        self._recovery_stuck_count = 0

    def __del__(self) -> None:

        if not sys.is_finalizing():
            self._destroy()

    def _destroy(self):
        try:
            self._cancel_recovery_stuck_check()
            if self._device_state == DeviceStateEx.Connected or self._device_state == DeviceStateEx.Ready:
                self.disconnect()
        except Exception as e:
            self._log.e(_TAG, f"Error occurred while destroying SensorProfile: {e}")
        self._is_starting = False
        self._is_setting_param = False
        # 递增 epoch，丢弃尚未执行的数据回调
        self._data_callback_epoch += 1
        try:
            self._callback_executor.shutdown(wait=False)
            self._data_callback_executor.shutdown(wait=False)
        except Exception as e:
            self._log.e(_TAG, f"Error occurred while shutting down callback executors: {e}")

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
                    self._log.e(_TAG, f"{error_msg}: {e}")

        try:
            executor.submit(_run)
        except Exception as e:
            self._log.e(_TAG, f"Error occurred while submitting callback: {e}")

    def _on_subprocess_message(self, msg: dict):

        msg_type = msg.get("type")
        if msg_type == "state_changed":
            state_name = msg.get("state")
            new_state = _STATE_NAME_MAP.get(state_name, self._device_state)
            self._set_device_state(new_state)
        elif msg_type == "power_changed":
            power = msg.get("power", -1)
            # 无效读数（-1：读取失败/链路异常）不更新也不上报——onPowerChanged
            # 只传递有效电量；显式 getBatteryLevel() 查询仍可能返回 -1（未读到）
            if power < 0:
                self._log.d(_TAG, f"invalid battery reading ignored: {self._device_mac}")
                return
            # 电量滤波：首次读数直接接受；有效读数与当前值之差不足稳定带时
            # 保持当前值，避免 ±2 抖动使显示来回跳
            if self._power < 0 or abs(power - self._power) >= _POWER_STABLE_BAND:
                self._power = power
            self._log.d(_TAG, f"onPowerChanged triggered: {self._device_mac} power={self._power}")
            self._submit_callback(
                self._on_power_changed, self, self._power
            )
        elif msg_type == "sensor_data":
            self._received_sensor_data_msgs += 1
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
            self._log.d(_TAG, f"onErrorCallback triggered: {self._device_mac} message={msg.get('message', '')}")
            self._submit_callback(
                self._on_error_callback,
                self,
                msg.get("message", ""),
                error_msg="Error occurred while processing error message",
            )
        elif msg_type == "chip_type":
            chip_value = msg.get("chip_type")
            self._chip_type = BLEChipType(chip_value) if chip_value is not None else BLEChipType.Unknown
        elif msg_type == "device_info_update":
            # DeviceInfo 字段变化（连接参数/MTU/采样率等）：就地更新后触发事件。
            # _device_info 为空时新建再应用：在线场景 init 结果会整体覆盖它；
            # 回放场景没有 init，这里是 profile 拿到 DeviceInfo 的唯一途径
            fields = msg.get("fields") or {}
            if fields:
                if self._device_info is None:
                    self._device_info = DeviceInfo()
                applied = {}
                for key, value in fields.items():
                    if hasattr(self._device_info, key):
                        setattr(self._device_info, key, value)
                        applied[key] = value
                if applied:
                    self._log.d(_TAG, f"onDeviceInfoUpdate triggered: {self._device_mac} {applied}")
                    self._submit_callback(
                        self._on_device_info_update, self, self._device_info,
                        error_msg="Error occurred while processing device info update",
                    )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    @property
    def deviceState(self) -> DeviceStateEx:

        return self._device_state

    @property
    def isReady(self) -> bool:
        """设备是否处于 Ready 状态（已连接且可调用 init/setParam 等）。"""
        return self._device_state == DeviceStateEx.Ready

    def _set_data_transfering(self, transferring: bool):
        """_is_data_transfering 的唯一变更门控写入口：状态真实变化时
        触发 onDataTransferStateChange 事件（回调在线程池执行）。"""
        if self._is_data_transfering == transferring:
            return
        self._is_data_transfering = transferring
        self._log.d(_TAG, f"onDataTransferStateChange triggered: {self._device_mac} isTransferring={transferring}")
        self._submit_callback(
            self._on_data_transfer_state_change, self, transferring,
            error_msg="Error occurred while processing data transfer state change",
        )

    def _set_device_state(self, newState: DeviceStateEx):
        if self._device_state != newState:
            if (newState == DeviceStateEx.Disconnected and self._is_data_transfering
                    and self._auto_reconnect):
                # 断连前正在数据传输（远端异常断连/长时间无数据异常断连）：标记待恢复
                self._resume_pending = True
            self._device_state = newState
            if newState == DeviceStateEx.Disconnected:
                self._has_inited = False
                self._set_data_transfering(False)
            if self._on_state_changed is not None:
                self._log.d(_TAG, f"onStateChanged triggered: {self._device_mac} state={newState}")
                try:
                    self._on_state_changed(self, newState)
                except Exception as e:
                    self._log.e(_TAG, f"Error occurred while processing state change: {e}")
                    raise RuntimeError("Set device state %s fail: %s" % (self.BLEDevice.Name , e))
            if newState == DeviceStateEx.Ready:
                # 自动重连成功：恢复上次传输（init→参数→开始传输）
                self._maybe_start_recovery()

    # ------------------------------------------------------------------
    # 自动重连恢复（autoReconnect）
    # ------------------------------------------------------------------
    @property
    def autoReconnect(self) -> bool:
        """远端异常断连/长时间无数据异常断连后，是否自动重连并恢复上次的数据传输
        （连接→初始化→恢复 setParam 参数→开始传输）。默认 True；
        置 False 后不再自动恢复（已在进行的链路级重连不受影响）。"""
        return self._auto_reconnect

    @autoReconnect.setter
    def autoReconnect(self, enabled: bool):
        self._log.d(_TAG, f"autoReconnect set: {self._device_mac} enabled={bool(enabled)}")
        self._auto_reconnect = bool(enabled)
        if not enabled:
            self._resume_pending = False

    @property
    def onAutoReconnect(self) -> Optional[Callable[..., None]]:
        """自动重连找到断连设备（回到 Ready、即将恢复）时的回调；未设置（None）时走默认恢复流程。

        签名（异步应答式，对齐 C++ SDK）：``callback(sensor, restore: bool, answer) -> None``
        - ``restore=True``：存在上次会话的 init 参数与 setParam 设置（可保留和恢复）；
          ``restore=False``：无上次会话（全新初始化场景）。
        - 应用须在**任意线程**、之后的任意时刻**恰好调用一次** ``answer(handled)``：
          ``answer(True)`` 表示应用自行接管恢复（全新初始化或自定义流程），SDK 不再执行
          默认恢复；待恢复标记在数据流真正恢复（``startDataNotification`` 成功）时才清除——
          若应用的恢复失败，标记保留，下一次重连成功时会再次触发本回调重试恢复；
          ``answer(False)`` 回落到默认恢复流程（连接→init→回放参数→开始传输）。
        - 10 秒无应答按 ``answer(False)`` 处理（告警日志）。
        - 兼容旧的二参形式 ``callback(sensor, restore) -> bool``：返回值即应答。
        - 回调跑在 SDK 专用恢复线程上，回调内允许阻塞调用（``init()``/``setParam()`` 等）。
        """
        return self._on_auto_reconnect

    @onAutoReconnect.setter
    def onAutoReconnect(self, callback: Optional[Callable[..., None]]):
        self._log.d(_TAG, "onAutoReconnect registered" if callback is not None
                    else "onAutoReconnect cleared")
        self._on_auto_reconnect = callback

    def _maybe_start_recovery(self):
        if not self._auto_reconnect or not self._resume_pending or self._recovering:
            return
        if self._on_auto_reconnect is None and self._last_init_args is None:
            # 没有可用 init 参数且无回调接管（理论上不会发生：待恢复意味着此前已 init 并传输）
            self._resume_pending = False
            return
        self._recovering = True
        self._log.i(_TAG, f"autoReconnect: start data stream recovery for {self._device_mac}")
        thread = threading.Thread(
            target=self._recover_data_stream,
            daemon=True,
            name=f"AutoReconnect-{self._device_mac}",
        )
        thread.start()

    def _ask_auto_reconnect_handler(self, restore: bool):
        """调用 onAutoReconnect 回调并等待其异步应答。

        返回 True（应用接管）/ False（回落默认恢复），或 None（等待应答期间
        恢复被取消，如用户主动断连）。回调为异步应答式：应用须在任意线程恰好
        调用一次 answer(handled)；10s 无应答按 answer(False) 处理并告警。
        兼容旧的二参形式 callback(sensor, restore) -> bool：返回值即应答。
        """
        cb = self._on_auto_reconnect
        import inspect
        try:
            arity = len(inspect.signature(cb).parameters)
        except (TypeError, ValueError):
            arity = 3
        if arity <= 2:
            # 旧形式：同步返回值即应答
            try:
                return bool(cb(self, restore))
            except Exception as e:
                self._log.e(_TAG, f"autoReconnect: onAutoReconnect callback failed: {e}")
                return False

        answered = threading.Event()
        answer_lock = threading.Lock()
        box = {"handled": False}

        def answer(handled: bool):
            # exactly-once（对齐 C++ 的 atomic CAS）：重复应答忽略
            with answer_lock:
                if answered.is_set():
                    return
                box["handled"] = bool(handled)
                answered.set()

        try:
            cb(self, restore, answer)
        except Exception as e:
            self._log.e(_TAG, f"autoReconnect: onAutoReconnect callback failed: {e}")
            answer(False)
        # 专用恢复线程，可阻塞等待；等待期间恢复被取消（用户断连等）则放弃本轮
        deadline = time.monotonic() + _AUTO_RECONNECT_ANSWER_TIMEOUT_SECONDS
        while not answered.wait(0.1):
            if not self._resume_pending:
                self._log.i(_TAG, f"autoReconnect: recovery cancelled while waiting "
                                  f"onAutoReconnect answer for {self._device_mac}")
                return None
            if time.monotonic() >= deadline:
                self._log.w(_TAG, f"autoReconnect: onAutoReconnect answer timed out, "
                                  f"default recovery for {self._device_mac}")
                answer(False)
        return box["handled"]

    def _recover_data_stream(self):
        """恢复上次传输：先问 onAutoReconnect 回调（异步应答式，可自行为全新初始化/自定义恢复），
        未接管时走默认流程：init（上次参数）→ 按序回放 setParam → 开始数据通知。
        恢复阶段（_resume_pending 下）的 init / startDataNotification 失败会立即
        强制断链（在 _init / _startDataNotification 内部，init 已重试 3 轮，再失败
        大概率是设备应用层卡死，断链让链路层重连滚动重来）——默认流程与应用接管
        路径（onAutoReconnect 应答 True 后应用自己调 init）同样生效；
        其余失败保持 _resume_pending，由卡死看门狗兜底，等待下一次重连成功再试；
        应用接管时待恢复标记延迟到 startDataNotification 成功才清除，应用恢复失败
        同样保留标记等待下一次重连。"""
        try:
            if self._on_auto_reconnect is not None:
                restore = self._last_init_args is not None
                handled = self._ask_auto_reconnect_handler(restore)
                if handled is None:
                    # 等待应答期间恢复已被取消（用户断连等）
                    return
                if handled:
                    # 应用已接管恢复（全新初始化或自定义恢复）：待恢复标记不在此清除，
                    # 延迟到 startDataNotification 成功（流真正恢复）时清除；若应用的
                    # 恢复失败，标记保留，下一次重连 Ready 时再次触发本回调重试恢复
                    self._log.i(_TAG, f"autoReconnect: handled by onAutoReconnect callback for {self._device_mac}")
                    return
                # 回调未接管：回落到默认恢复流程
            if self._last_init_args is None:
                # 无上次会话参数，默认流程无法恢复（留给应用或放弃）
                self._log.w(_TAG, f"autoReconnect: no previous session to restore for {self._device_mac}")
                return
            package_count, power_interval = self._last_init_args
            if not self.init(package_count, power_interval):
                # init 内部已重试 3 轮，仍失败说明设备应用层很可能已卡死：
                # _init 内已在 _resume_pending 下强制断链，让链路层重连滚动重来
                self._log.w(_TAG, f"autoReconnect: init failed for {self._device_mac}")
                return
            for key, value in self._saved_params.items():
                try:
                    ret = self.setParam(key, value)
                    if str(ret).startswith("Error"):
                        self._log.w(_TAG, f"autoReconnect: restore setParam({key}, {value}) failed: {ret}")
                except Exception as e:
                    self._log.w(_TAG, f"autoReconnect: restore setParam({key}, {value}) error: {e}")
            if self.startDataNotification():
                self._resume_pending = False
                self._log.i(_TAG, f"autoReconnect: data stream recovered for {self._device_mac}")
            else:
                # _startDataNotification 内已在 _resume_pending 下强制断链
                self._log.w(_TAG, f"autoReconnect: startDataNotification failed for {self._device_mac}")
        except Exception as e:
            self._log.exception(_TAG, f"autoReconnect: recovery failed for {self._device_mac}: {e}")
        finally:
            self._recovering = False
            if self._resume_pending:
                # 恢复未成功：启动卡死看门狗——若链路一直健在但流迟迟恢复
                # 不了（设备应用层不应答），超时计数到上限后强制断链重连
                self._schedule_recovery_stuck_check()
            else:
                self._cancel_recovery_stuck_check()
                self._recovery_stuck_count = 0

    def _force_recovery_link_drop(self, reason: str):
        """恢复阶段关键步骤失败：立即强制断链（子进程按异常断开处理，
        链路层自动重连，新的 Ready 会再次触发恢复）。仅链路健在时发送；
        卡死看门狗不取消，作为后续恢复尝试的兜底。"""
        if self._device_state not in (DeviceStateEx.Connected, DeviceStateEx.Ready):
            return
        self._log.w(_TAG, f"autoReconnect: force link drop ({reason}) for {self._device_mac}")
        if self._bleak_host is not None:
            self._bleak_host.send_command({
                "type": "force_link_drop",
                "device_mac": self._device.Address,
            })

    def _schedule_recovery_stuck_check(self):
        """（重新）安排一次恢复卡死检查。"""
        self._cancel_recovery_stuck_check()
        timer = threading.Timer(_RECOVERY_STUCK_CHECK_SECONDS, self._on_recovery_stuck_check)
        timer.daemon = True
        self._recovery_stuck_timer = timer
        timer.start()

    def _cancel_recovery_stuck_check(self):
        timer = self._recovery_stuck_timer
        self._recovery_stuck_timer = None
        if timer is not None:
            timer.cancel()

    def _on_recovery_stuck_check(self):
        """恢复卡死检查（定时器线程上下文）。恢复未完成且链路仍在时计数，
        连续达到上限则强制断链（子进程按异常断开处理，链路层自动重连，
        新的 Ready 会再次触发恢复）。"""
        self._recovery_stuck_timer = None
        try:
            if sensor_utils._terminated or not self._resume_pending:
                self._recovery_stuck_count = 0
                return
            if self._recovering:
                # 恢复流程仍在执行（如多轮 init 重试），延后再查
                self._schedule_recovery_stuck_check()
                return
            if self._device_state not in (DeviceStateEx.Connected, DeviceStateEx.Ready):
                # 链路已断或正在断开：交给链路层重连，无需干预
                return
            self._recovery_stuck_count += 1
            self._log.w(_TAG, f"autoReconnect: recovery stuck ({self._recovery_stuck_count}/"
                           f"{_RECOVERY_STUCK_MAX}) for {self._device_mac}: "
                           f"link up but stream not restored")
            if self._recovery_stuck_count < _RECOVERY_STUCK_MAX:
                self._schedule_recovery_stuck_check()
                return
            self._recovery_stuck_count = 0
            self._force_recovery_link_drop("recovery stuck")
        except Exception as e:
            self._log.exception(_TAG, f"autoReconnect: recovery stuck check failed for {self._device_mac}: {e}")

    @property
    def hasInited(self) -> bool:

        return self._has_inited

    @property
    def isDataTransfering(self) -> bool:

        return self._is_data_transfering

    @property
    def BLEDevice(self) -> BLEDevice:

        return self._device

    def log(self, message: str, level: str = "I"):
        """记录一条应用日志到本设备的 profile 日志（profile 日志未开启时
        回落 controller log 公共通道）。

        便于应用把本设备相关的事件（用户操作、业务状态等）与 SDK 日志
        写进同一文件，统一时间线排查问题。

        Args:
            message: 日志内容。
            level: "D"/"I"/"W"/"E"（大小写不敏感），默认 "I"；
                "D" 受 SensorController.setDebugEnabled 开关控制，其余级别总是输出。
        """
        try:
            SdkLog.log(level, "App", str(message), mac=self._device_mac)
        except Exception:
            pass

    @property
    def onStateChanged(self) -> Callable[["SensorProfile", DeviceStateEx], None]:

        return self._on_state_changed

    @onStateChanged.setter
    def onStateChanged(self, callback: Callable[["SensorProfile", DeviceStateEx], None]):
        self._log.d(_TAG, "onStateChanged registered")
        self._on_state_changed = callback

    @property
    def onErrorCallback(self) -> Callable[["SensorProfile", str], None]:

        return self._on_error_callback

    @onErrorCallback.setter
    def onErrorCallback(self, callback: Callable[["SensorProfile", str], None]):
        self._log.d(_TAG, "onErrorCallback registered")
        self._on_error_callback = callback

    @property
    def onDataCallback(self) -> Callable[["SensorProfile", "list[SensorData]"], None]:

        return self._on_data_callback

    @onDataCallback.setter
    def onDataCallback(self, callback: Callable[["SensorProfile", "list[SensorData]"], None]):
        self._log.d(_TAG, "onDataCallback registered")
        self._on_data_callback = callback

    @property
    def onPowerChanged(self) -> Callable[["SensorProfile", int], None]:

        return self._on_power_changed

    @onPowerChanged.setter
    def onPowerChanged(self, callback: Callable[["SensorProfile", int], None]):
        self._log.d(_TAG, "onPowerChanged registered")
        self._on_power_changed = callback

    @property
    def onDeviceInfoUpdate(self) -> Callable[["SensorProfile", DeviceInfo], None]:

        return self._on_device_info_update

    @onDeviceInfoUpdate.setter
    def onDeviceInfoUpdate(self, callback: Callable[["SensorProfile", DeviceInfo], None]):
        self._log.d(_TAG, "onDeviceInfoUpdate registered")
        self._on_device_info_update = callback

    @property
    def onDataTransferStateChange(self) -> Callable[["SensorProfile", bool], None]:
        """数据流开关状态变化回调：callback(profile, isTransferring)。
        只在真实 起流/停流 变化时触发（起流成功、停流、断连、回放开始/结束）。"""
        return self._on_data_transfer_state_change

    @onDataTransferStateChange.setter
    def onDataTransferStateChange(self, callback: Callable[["SensorProfile", bool], None]):
        self._log.d(_TAG, "onDataTransferStateChange registered")
        self._on_data_transfer_state_change = callback

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
        # 调试日志开启时从连接起点就启用该 profile 的日志文件：主进程算好
        # 路径并随 connect 命令传给子进程，两个进程写同一文件，连接阶段的
        # 设备日志（bin recorder / gforce connect / init）不再回落 controller log
        if SdkLog.is_debug_enabled() and SdkLog.is_file_output_enabled():
            log_path = SdkLog.get_profile_log_path(self._device_mac) \
                or SdkLog.get_default_profile_log_path(prefix=self._device_name or "")
            if SdkLog.enable_profile_log(self._device_mac, log_path) is not None:
                cmd["profile_log_path"] = log_path
        # 此前会话已设置过 bin 导出路径（DEBUG_BLE_DATA_PATH，_saved_params
        # 中存的是展开后的具体路径）时随 connect 命令捎给子进程：bin 从连接
        # 起点直写导出文件，不再先写 temp 再在停止时整段拷贝
        bin_path = self._saved_params.get("DEBUG_BLE_DATA_PATH")
        if bin_path and bin_path != "False":
            cmd["bin_export_path"] = bin_path
        result = await self._send_cmd_async(cmd, timeout=self._connect_timeout())
        success = result.get("success", False) if result else False
        return success

    def connect(self) -> bool:
        self._log.d(_TAG, f"connect called: {self._device_mac}")
        # 用户主动连接：由用户驱动后续 init/起流，取消自动恢复
        self._resume_pending = False
        result = sync_call(self._connect(), _timeout=self._connect_timeout())
        return result

    async def asyncConnect(self) -> bool:
        self._log.d(_TAG, f"asyncConnect called: {self._device_mac}")
        self._resume_pending = False
        return await async_call(self._connect(), _timeout=self._connect_timeout())

    def _post_disconnect_intent(self):
        """把用户断连意图以 fire-and-forget 方式先行送达子进程。

        dongle 固件可能在推流中收到 ATT 写时挂死（实测 OB5200 + Actions
        dongle）：随后的停流/断开命令永远执行不完，链路按 supervision
        timeout 异常掉线。意图先到，子进程的异常清理才不会把它当作
        异常断开去自动重连。
        """
        if self._bleak_host is not None:
            self._bleak_host.send_command({
                "type": "cancel_reconnect",
                "device_mac": self._device.Address,
            })

    async def _disconnect(self) -> bool:
        # 用户主动断开：取消自动恢复；意图先行送达子进程（在任何等待之前）
        self._resume_pending = False
        self._post_disconnect_intent()
        if self.deviceState != DeviceStateEx.Connected and self.deviceState != DeviceStateEx.Ready:
            # 设备已断开：多为设备刚异常掉链、状态同步后用户才点到断开。
            # 仍以 fire-and-forget 转发 disconnect，让子进程取消可能已调度的
            # 链路层自动重连（此前不转发，用户明确断开后设备会被自动连回）
            self._log.i(_TAG, f"disconnect: already {self.deviceState}, forwarding intent only: {self._device_mac}")
            if self._bleak_host is not None:
                self._bleak_host.send_command({
                    "type": "disconnect",
                    "device_mac": self._device.Address,
                })
            return True

        # 与 Android SDK 对齐：断开前若正在推流，先停止数据通知
        if self._is_data_transfering:
            await self._stopDataNotification()

        cmd = {"type": "disconnect"}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        self._has_inited = False
        self._set_data_transfering(False)
        return True

    def disconnect(self) -> bool:
        self._log.d(_TAG, f"disconnect called: {self._device_mac}")
        return sync_call(self._disconnect())

    async def asyncDisconnect(self) -> bool:
        self._log.d(_TAG, f"asyncDisconnect called: {self._device_mac}")
        return await async_call(self._disconnect())

    # ------------------------------------------------------------------
    # Data notification
    # ------------------------------------------------------------------
    def _apply_start_notification_success(self):
        """起流成功后的本地状态更新（单设备起流与 controller 同步起流共用）。

        递增数据回调 epoch 丢弃旧流回调；数据流真正起来才算恢复完成：
        app 接管恢复（onAutoReconnect 返回 True）时待恢复标记不在回调返回时
        清除，而是延迟到这里；若应用的恢复失败，标记保留，下一次重连 Ready
        时会再次触发恢复。
        """
        self._set_data_transfering(True)
        # 开始新数据流时递增 epoch，丢弃旧流尚未执行的回调
        self._data_callback_epoch += 1
        self._resume_pending = False

    async def _startDataNotification(self) -> bool:
        if not self.isReady:
            return False
        if not self._has_inited:
            return False

        cmd = {"type": "start_notification"}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        success = result.get("success", False) if result else False
        if success:
            self._apply_start_notification_success()
        elif self._resume_pending:
            # 断链恢复阶段的起流失败：立即强制断链，让链路层重连滚动重来
            # （应用接管恢复时同样生效，与 _init 的恢复失败处理一致）
            self._force_recovery_link_drop("startDataNotification failed during recovery")
        return success

    def startDataNotification(self) -> bool:
        self._log.d(_TAG, f"startDataNotification called: {self._device_mac}")
        if self._is_starting:
            return False

        try:
            self._is_starting = True
            ret = sync_call(self._startDataNotification())
            self._is_starting = False
            return ret
        except Exception as e:
            self._is_starting = False
            self._log.exception(_TAG, "startDataNotification failed")
            raise

    async def asyncStartDataNotification(self) -> bool:
        self._log.d(_TAG, f"asyncStartDataNotification called: {self._device_mac}")
        if self._is_starting:
            return False

        try:
            self._is_starting = True
            ret = await async_call(self._startDataNotification())
            self._is_starting = False
            return ret
        except Exception as e:
            self._is_starting = False
            self._log.exception(_TAG, "asyncStartDataNotification failed")
            raise

    def _apply_stop_notification_success(self):
        """停流成功后的本地状态更新（单设备停流与 controller 同步停流共用）。"""
        self._set_data_transfering(False)
        # 停止数据流时递增 epoch，丢弃已提交但未执行的旧数据回调
        self._data_callback_epoch += 1

    async def _stopDataNotification(self) -> bool:
        # 用户主动停流：取消自动恢复
        self._resume_pending = False
        if not self.isReady:
            return False
        if not self._has_inited:
            return False

        cmd = {"type": "stop_notification"}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        success = result.get("success", False) if result else False
        if success:
            self._apply_stop_notification_success()
        return success

    def stopDataNotification(self) -> bool:
        self._log.d(_TAG, f"stopDataNotification called: {self._device_mac}")
        if self._is_starting:
            return False

        try:
            self._is_starting = True
            ret = sync_call(self._stopDataNotification())
            self._is_starting = False
            return ret
        except Exception as e:
            self._is_starting = False
            self._log.exception(_TAG, "stopDataNotification failed")
            raise

    async def asyncStopDataNotification(self) -> bool:
        self._log.d(_TAG, f"asyncStopDataNotification called: {self._device_mac}")
        if self._is_starting:
            return False

        try:
            self._is_starting = True
            ret = await async_call(self._stopDataNotification())
            self._is_starting = False
            return ret
        except Exception as e:
            self._is_starting = False
            self._log.exception(_TAG, "asyncStopDataNotification failed")
            raise

    # ------------------------------------------------------------------
    # Init
    # ------------------------------------------------------------------
    async def _init(self, packageSampleCount: int, powerRefreshInterval: int) -> bool:
        if not self.isReady:
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
            # 记录上次成功 init 的参数，供自动重连后恢复
            self._last_init_args = (packageSampleCount, powerRefreshInterval)
        elif self._resume_pending:
            # 断链恢复阶段的 init 失败（init 内部已重试 3 轮，再失败大概率是
            # 设备应用层卡死）：立即强制断链，让链路层重连滚动重来——应用接管
            # 恢复（onAutoReconnect 返回 True）时同样生效，不再只依赖卡死看门狗
            self._force_recovery_link_drop("init failed during recovery")
        return success

    def init(self, packageSampleCount: int, powerRefreshInterval: int) -> bool:
        self._log.d(_TAG, f"init called: {self._device_mac} packageSampleCount={packageSampleCount} powerRefreshInterval={powerRefreshInterval}")
        return sync_call(
            self._init(packageSampleCount, powerRefreshInterval),
            20,
        )

    async def asyncInit(self, packageSampleCount: int, powerRefreshInterval: int) -> bool:
        self._log.d(_TAG, f"asyncInit called: {self._device_mac} packageSampleCount={packageSampleCount} powerRefreshInterval={powerRefreshInterval}")
        return await async_call(
            self._init(packageSampleCount, powerRefreshInterval),
            20,
        )

    # ------------------------------------------------------------------
    # Battery
    # ------------------------------------------------------------------
    async def _asyncGetBatteryLevel(self) -> int:
        if not self.isReady:
            return -1
        if not self._has_inited:
            return -1

        cmd = {"type": "get_battery"}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        if result and result.get("success"):
            return result.get("result", -1)
        return -1

    async def asyncGetBatteryLevel(self) -> int:
        self._log.d(_TAG, f"asyncGetBatteryLevel called: {self._device_mac}")
        return await async_call(self._asyncGetBatteryLevel())

    def getBatteryLevel(self) -> int:
        self._log.d(_TAG, f"getBatteryLevel called: {self._device_mac}")
        return self._power

    def getDeviceInfo(self) -> Optional[DeviceInfo]:
        self._log.d(_TAG, f"getDeviceInfo called: {self._device_mac}")
        if self.hasInited:
            return self._device_info
        return None

    # ------------------------------------------------------------------
    # Neucir
    # ------------------------------------------------------------------
    async def _asyncSet_neucir_app_control(self, open: bool, close: bool, stop: bool) -> str:
        if not self.isReady:
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
        if not self.isReady:
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
        # 兼容 bool 形式（True/False），统一转成字符串再走后续比较
        if isinstance(value, bool):
            value = "True" if value else "False"
        if not self.isReady:
            return "Error: Please connect first"

        if key == "DEBUG_LOG_PATH":
            if value == "False" or value == "":
                SdkLog.disable_profile_log(self._device_mac)
            elif value == "True":
                # 默认路径：日志目录下 {DeviceName}_log_YYYYMMDD_HHMMSS.txt
                # （文件输出关闭时默认导出禁用，显式路径不受影响）；
                # 连接时已自动开启的沿用现有文件，避免一次会话拆成两个文件
                if not SdkLog.is_file_output_enabled():
                    return "Error: SDK file output disabled"
                value = SdkLog.get_profile_log_path(self._device_mac) \
                    or SdkLog.get_default_profile_log_path(prefix=self.BLEDevice.Name)
                SdkLog.enable_profile_log(self._device_mac, value)
            else:
                SdkLog.enable_profile_log(self._device_mac, value)

        if key == "DEBUG_BLE_DATA_PATH":
            if value == "True":
                # 默认导出路径：日志目录下 {DeviceName}_data_YYYYMMDD_HHMMSS.bin
                # （文件输出关闭时默认导出禁用，显式路径不受影响）
                if not SdkLog.is_file_output_enabled():
                    return "Error: SDK file output disabled"
                value = SdkLog.get_default_bin_path(prefix=self.BLEDevice.Name)
            # "False"/"" 保持原值，由子进程关闭

        cmd = {"type": "set_param", "key": key, "value": value}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        if result:
            ret = result.get("result", "Error: Unknown error")
            if not str(ret).startswith("Error"):
                # 记录成功设置的参数，供自动重连后按序恢复
                self._saved_params[key] = value
            return ret
        return "Error: Timeout"

    def setParam(self, key: str, value) -> str:
        self._log.d(_TAG, f"setParam called: {self._device_mac} key={key} value={value}")
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
            self._log.exception(_TAG, f"setParam({key}) failed")
            raise

    async def asyncSetParam(self, key: str, value) -> str:
        self._log.d(_TAG, f"asyncSetParam called: {self._device_mac} key={key} value={value}")
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
            self._log.exception(_TAG, f"asyncSetParam({key}) failed")
            raise

    # ------------------------------------------------------------------
    # GetParam
    # ------------------------------------------------------------------
    async def _getParam(self, key: str) -> str:
        if not self.isReady:
            return "Error: Please connect first"

        if key == "DEBUG_LOG_PATH":
            return SdkLog.get_profile_log_path(self._device_mac) or ""

        cmd = {"type": "get_param", "key": key}
        result = await self._send_cmd_async(cmd, timeout=10.0)
        if result:
            return result.get("result", "Error: Unknown error")
        return "Error: Timeout"

    def getParam(self, key: str) -> str:
        self._log.d(_TAG, f"getParam called: {self._device_mac} key={key}")
        return sync_call(
            self._getParam(key),
            10,
        )

    async def asyncGetParam(self, key: str) -> str:
        self._log.d(_TAG, f"asyncGetParam called: {self._device_mac} key={key}")
        return await async_call(
            self._getParam(key),
            10,
        )
