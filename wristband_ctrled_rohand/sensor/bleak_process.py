import asyncio
import multiprocessing
import os
import platform
import queue
import threading
import time

import bleak
from bleak import BleakScanner
from bleak import AdvertisementData

from sensor import bumble_dongle
from sensor import sensor_utils
from sensor.bin_recorder import BIN_RECORD_CMD_SEND, BIN_RECORD_CONFIG, BIN_RECORD_DATA, BIN_RECORD_EVENT, decode_bin_config, iter_bin_records
from sensor.sensor_device import BLEChipType
from sensor.sdk_log import SdkLog

_TAG = "BleakProcess"

SERVICE_GUID = "0000ffd0-0000-1000-8000-00805f9b34fb"
RFSTAR_SERVICE_GUID = "00001812-0000-1000-8000-00805f9b34fb"

OYM_CMD_NOTIFY_CHAR_UUID = "f000ffe1-0451-4000-b000-000000000000"
OYM_DATA_NOTIFY_CHAR_UUID = "f000ffe2-0451-4000-b000-000000000000"

RFSTAR_CMD_UUID = "00000002-0000-1000-8000-00805f9b34fb"
RFSTAR_DATA_UUID = "00000003-0000-1000-8000-00805f9b34fb"

# 异常断开后自动重连参数
_MAX_RECONNECT_ATTEMPTS = 5
_RECONNECT_DELAY_SECONDS = 1.0
# 单次连接等待上限（dongle 链路建立 + 服务发现 + MTU 交换比原生栈慢）
_CONNECT_TIMEOUT_SECONDS = 25.0
# 重连 attempt 独立结果看门狗（秒）：attempt 发起后超时仍无结果即强制判失败。
# 必须大于 _CONNECT_TIMEOUT_SECONDS，正常路径由 25s 超时先行收尾
_RECONNECT_WATCHDOG_SECONDS = 40.0
# 自动重连成功后、发布 Ready 前的稳定时间（秒）：设备刚重新上电，固件
# 可能尚未就绪应答 GATT 命令（实测重连 3ms 后即 init，GET_DEVICE_NAME
# 2s 无响应导致 init 失败），Ready 会触发上层流恢复，稍缓发布
_RECONNECT_SETTLE_SECONDS = 1.0
# 单轮扫描的外层超时余量（秒）：dongle 在扫描中被拔出时，discover 内部的
# stop() 会向死传输发 HCI 命令且永不超时，用外层 wait_for 强制结束本轮
_SCAN_ROUND_GRACE_SECONDS = 5.0
# 扫描空结果看门狗（轮）：缓存传输半死（HCI 命令有应答、扫描轮正常结束，
# 但收不到任何广播）时不会有超时/异常触发缓存清理；连续 N 轮空扫描后
# 弹出缓存传输，强制下一轮重开
_SCAN_EMPTY_ROUND_TRANSPORT_RESET = 3
# 主循环单轮迭代 stall 告警阈值（秒）：正常一轮 ≤50ms，超过即说明事件
# 循环被同步阻塞/饿死，命令会延迟被取出
_MAIN_LOOP_STALL_SECONDS = 1.0
# 用户断开/停流时，对可能已卡死的 gforce 事件循环的调用侧超时（秒）：
# dongle 挂死时投递到 gforce loop 的协程永远不会执行（其内部超时形同虚设），
# 必须在 device loop 调用侧兜底，让断开流程能走完
_STOP_STREAM_TIMEOUT_SECONDS = 6.0
_CLIENT_DISCONNECT_TIMEOUT_SECONDS = 6.0
# multiStart 首包 delay 离散度校验：全部设备起流后等待各自首包 delay
# （起流写发送时刻→首个原始数据包到达），max-min 超过
# _MULTI_START_DELAY_DISPERSION_MAX_MS 或有设备在
# _MULTI_START_FIRST_PACKET_TIMEOUT_SECONDS 内未出首包时整体停流重来，
# 最多 _MULTI_START_MAX_ATTEMPTS 次，仍不达标返回失败
_MULTI_START_MAX_ATTEMPTS = 3
_MULTI_START_DELAY_DISPERSION_MAX_MS = 5
_MULTI_START_FIRST_PACKET_TIMEOUT_SECONDS = 2.0


def _extract_mac(_device: bleak.BLEDevice, adv: AdvertisementData) -> str:

    mac = None
    if adv.service_data.get(SERVICE_GUID) is not None:
        bytes_val = adv.service_data[SERVICE_GUID]
        mac = ":".join(f"{byte:02X}" for byte in bytes_val)
    elif adv.service_data.get(RFSTAR_SERVICE_GUID) is not None:
        bytes_val = adv.service_data[RFSTAR_SERVICE_GUID]
        mac = ":".join(f"{byte:02X}" for byte in reversed(bytes_val))
    return mac


def _serialize_device(_device: bleak.BLEDevice, adv: AdvertisementData):
    """序列化扫描到的设备；MAC 无法解析时返回 None（返回值类型不唯一，按约定不加返回注解，
    避免 Cython 编译后运行时类型检查抛 TypeError）。"""

    mac = _extract_mac(_device, adv)
    if mac is None:
        return None
    return {
        "address": _device.address,
        "name": _device.name,
        "rssi": adv.rssi,
        "mac": mac,
        "service_uuids": list(adv.service_uuids),
        "service_data": {
            k: v.hex() if isinstance(v, bytes) else v
            for k, v in adv.service_data.items()
        },
    }


def _match_device(_device: bleak.BLEDevice, adv: AdvertisementData):

    if _device.name is None:
        return False
    if SERVICE_GUID in adv.service_uuids:
        return True
    return False


class BleakProcess(multiprocessing.Process):


    def __init__(
        self,
        cmd_queue: multiprocessing.Queue,
        result_queue: multiprocessing.Queue,
        data_queue: multiprocessing.Queue,
        log_dir: str = None,
        file_output_enabled: bool = True,
        debug_enabled: bool = True,
        controller_log_path: str = None,
    ):
        super().__init__(daemon=True)
        self.cmd_queue = cmd_queue
        self.result_queue = result_queue
        self.data_queue = data_queue
        self._log_dir = log_dir
        self._file_output_enabled = file_output_enabled
        self._debug_enabled = debug_enabled
        # 主进程已打开的 controller log 具体路径（None 时按默认规则自建）
        self._controller_log_path = controller_log_path
        self._scanner = None
        self._is_scanning = False
        self._should_exit = False

        # Per-device state (all in sub-process)
        self._devices: dict = {}          # mac -> bleak.BLEDevice
        self._gforces: dict = {}          # mac -> GForce
        self._data_ctxs: dict = {}        # mac -> SensorProfileDataCtx
        self._raw_bufs: dict = {}         # mac -> queue.Queue
        self._device_states: dict = {}    # mac -> str state name
        self._power_intervals: dict = {}  # mac -> int (ms)
        self._data_tasks: dict = {}       # mac -> asyncio.Task
        self._battery_tasks: dict = {}    # mac -> asyncio.Task
        self._reconnect_info: dict = {}   # mac -> {cmd, attempts, task, normal_disconnect}
        # 用户断连意图集合：disconnect/cancel_reconnect 命令一被取出即登记，
        # 即使设备/gforce 事件循环已卡死、_do_disconnect 迟迟执行不到，随后
        # 的异常掉链清理也能识别意图、不再触发自动重连（实测 dongle 固件在
        # 推流中收到 ATT 写时挂死，停流命令永远执行不完，链路 6.4s 后按
        # supervision timeout 异常掉线，若不识别意图会被当作异常断开自动重连）
        self._user_disconnect_pending: set = set()
        # 连接世代号：mac -> int。每次 _do_connect 递增；连接超时不再等待
        # inner 协程退出（它可能挂在对取消不响应的 bumble 等待上），
        # inner 迟到的结果按世代号失效处理
        self._connect_epochs: dict = {}
        self._main_event_loop = None      # BleakProcess 主事件循环

        # 进行中的 bin 回放控制标志：device_mac -> {"paused": bool, "stop": bool}
        self._replay_controls: dict = {}

        # Per-device event loops and threads
        self._event_loops: dict = {}      # mac -> asyncio.AbstractEventLoop
        self._event_threads: dict = {}    # mac -> threading.Thread
        self._data_event_loops: dict = {}    # mac -> asyncio.AbstractEventLoop
        self._data_event_threads: dict = {}  # mac -> threading.Thread
        self._cleanup_locks: dict = {}    # mac -> asyncio.Lock

        # Singleton gforce loop for all bleak object creation/access
        self._gforce_event_loop = None
        self._gforce_event_thread = None

        # 独立扫描 loop：扫描轮次（含 bumble 扫描的 HCI 处理）不再占用
        # gforce loop，避免扫描与其他设备的命令/连接互相排队；
        # bumble 扫描传输打开时绑定本 loop
        self._scan_event_loop = None
        self._scan_event_thread = None

        # BLE 后端选择（None=原生 bleak；BumbleBackend=bleak_bumble），run() 时确定
        self._ble_backend = None
        # dongle 分配表：device_mac -> transport spec（bumble 后端时，一 dongle 一设备）
        self._dongle_assignments = {}
        # dongle 拓扑身份表：transport spec -> "usb:vid:pid@bus:port.path"
        # （物理端口路径；硬件层无唯一身份——serial 全同、BD_ADDR 命令不支持，
        # 拓扑是唯一可用的持久标识，随每次枚举/热插拔刷新）
        self._spec_topology = {}
        # 设备连接偏好：device_mac -> 上次成功连接所用 dongle 的拓扑身份；
        # 断连/热插拔后重连时优先用原 dongle（偏好未命中立即退回任意空闲）
        self._dongle_affinity = {}
        # 最近一次上报的 USB dongle 枚举（热插拔对账基准）
        self._dongle_specs = []
        # USB dongle 热插拔监控（DongleHotplugMonitor，_run_main 中启动）
        self._dongle_monitor = None
        # 无空闲 dongle 被跳过的持续扫描周期；dongle 池有空闲后自动补扫
        self._pending_scan_period = None
        # 扫描当前是否借用了 dongle（扫描周期内临时占用，结束即归还）；
        # 连接分配遇到无空闲时据此等待在途扫描释放，而不是立即判失败
        self._scan_borrowed = False
        # 扫描空结果看门狗：transport spec -> 连续空扫描轮数（仅 bumble 后端）
        self._scan_empty_rounds = {}

    # Message types that can be dropped when the result queue is full.
    _DROPABLE_MSG_TYPES = ("sensor_data", "devices", "scan_once_result", "error")

    def _publish(self, msg_type: str, **kwargs):

        try:
            msg = {"type": msg_type, **kwargs}

            # Memory guard: prevent the internal message queue from growing without bound.
            if self._msg_queue.qsize() > sensor_utils.BLEAK_RESULT_QUEUE_MAXSIZE:
                if msg_type == "sensor_data":
                    # 解码结果不丢：等待发布循环消费（进程退出时放弃），
                    # 背压沿数据通路回传，最终由 bin 文件缓存吸收
                    while (not self._should_exit
                           and self._msg_queue.qsize() > sensor_utils.BLEAK_RESULT_QUEUE_MAXSIZE):
                        time.sleep(0.01)
                elif msg_type in self._DROPABLE_MSG_TYPES:
                    return
                else:
                    # For non-droppable messages, try to evict one old droppable message.
                    try:
                        old_msg = self._msg_queue.get_nowait()
                        old_type = old_msg.get("type")
                        if old_type not in self._DROPABLE_MSG_TYPES:
                            # If the oldest message is also non-droppable, put it back.
                            self._msg_queue.put_nowait(old_msg)
                    except queue.Empty:
                        pass
                    except Exception:
                        pass

            try:
                self._msg_queue.put_nowait(msg)
            except queue.Full:
                # If still full, use a short blocking put for important messages.
                if msg_type not in self._DROPABLE_MSG_TYPES:
                    try:
                        self._msg_queue.put(msg, timeout=1.0)
                    except queue.Full:
                        SdkLog.w(_TAG, f"Message queue still full, dropping important message: {msg_type}")
        except Exception as e:
            SdkLog.exception(_TAG, "Unexpected error")

    def _flush_msg_queue(self):

        def _route_and_put(msg):
            msg_type = msg.get("type")
            if msg_type == "sensor_data":
                try:
                    self.data_queue.put_nowait(msg)
                except queue.Full:
                    pass
            elif msg_type in self._DROPABLE_MSG_TYPES:
                self.result_queue.put_nowait(msg)
            else:
                # Use a short timeout during shutdown so the child process can exit cleanly.
                self.result_queue.put(msg, timeout=2.0)

        while True:
            try:
                msg = self._msg_queue.get_nowait()
                _route_and_put(msg)
            except queue.Empty:
                break
            except queue.Full:
                # Drop one old droppable message to make room for important messages.
                SdkLog.w(_TAG, "Result queue full, trying to drop one old droppable message")
                try:
                    old_msg = self._msg_queue.get_nowait()
                    old_type = old_msg.get("type")
                    if old_type not in self._DROPABLE_MSG_TYPES:
                        self._msg_queue.put_nowait(old_msg)
                except queue.Empty:
                    break
                except Exception:
                    break
            except Exception:
                break

    def _put_data_queue_blocking(self, msg):
        """等待 data_queue 腾出空间后放入解码结果；进程退出时放弃。"""
        while not self._should_exit:
            try:
                self.data_queue.put(msg, timeout=0.5)
                return
            except queue.Full:
                continue

    def _publisher_loop(self):
        # 独立线程消费内部消息队列（原为 asyncio 任务）：_publish 在事件循环线程上
        # 的同步背压等待（sensor_data 爆发使队列超过 MAXSIZE 时）不再饿死发布者——
        # 原先任务与等待同处一个事件循环，sync wait 冻结循环后任务永远无法运行，
        # 形成自死锁（全速回放时必现）。
        # Throttle queue-full logs to avoid blocking stdout on Windows.
        _last_queue_full_log = 0.0

        while True:
            msg = None
            msg_type = None
            try:
                msg = self._msg_queue.get_nowait()
                msg_type = msg.get("type")
                if msg_type == "sensor_data":
                    # High-frequency data: use a dedicated queue so it cannot
                    # block control/result messages in the main result queue.
                    # 解码结果不丢：队列满时阻塞等待主进程消费（进程退出时放弃）
                    try:
                        self.data_queue.put_nowait(msg)
                    except queue.Full:
                        self._put_data_queue_blocking(msg)
                elif msg_type in self._DROPABLE_MSG_TYPES:
                    self.result_queue.put_nowait(msg)
                else:
                    # Use a short timeout for important messages so the publisher does not stall.
                    self.result_queue.put(msg, timeout=2.0)
            except queue.Empty:
                if self._should_exit:
                    break
                time.sleep(0.001)
            except queue.Full as e:
                if msg is not None:
                    if msg_type not in self._DROPABLE_MSG_TYPES:
                        # Put important messages back so they can be retried later.
                        try:
                            self._msg_queue.put_nowait(msg)
                        except queue.Full:
                            pass
                    else:
                        # Droppable messages are discarded to avoid unbounded memory growth.
                        pass
                # Log at most once every 2 seconds to prevent stdout flooding.
                now = time.time()
                if now - _last_queue_full_log >= 2.0:
                    _last_queue_full_log = now
                    SdkLog.w(_TAG, f"Result queue is full, dropping message: {e}")
                time.sleep(0.01)
            except Exception as e:
                now = time.time()
                if now - _last_queue_full_log >= 2.0:
                    _last_queue_full_log = now
                    SdkLog.e(_TAG, f"Error in publisher_loop: {e}")
                time.sleep(0.001)

    def _scan_dongle_kwargs(self):
        """为一次扫描领取空闲 dongle，返回 (spec, kwargs)。

        原生后端返回 (None, {})；bumble 后端无空闲 dongle 时返回 (None, None)。
        """
        if self._ble_backend is None:
            return None, {}
        spec = self._ble_backend.allocate()
        if spec is None:
            return None, None
        return spec, self._ble_backend.scanner_kwargs(spec)

    def _assign_dongle(self, device_mac: str):
        """为连接分配一只空闲 dongle，返回 (spec, client_kwargs)；无空闲返回 (None, None)。

        设备上次成功连接所用的 dongle（按拓扑身份记录）当前空闲时优先分配它
        （断连/热插拔后优先用原 dongle）；偏好 dongle 占用/已拔出/换口时
        立即退回任意空闲 dongle，不为偏好等待。
        """
        if self._ble_backend is None:
            return None, {}
        preferred = None
        affinity = self._dongle_affinity.get(device_mac)
        if affinity is not None:
            # 拓扑身份（usb:vid:pid@bus:port.path）匹配当前枚举中的 spec：
            # 同一物理口复插的同一只 dongle 命中；换口/不在枚举则 miss
            for spec, topo in self._spec_topology.items():
                if topo == affinity:
                    preferred = spec
                    break
        spec = self._ble_backend.allocate(preferred)
        if spec is None:
            return None, None
        if affinity is not None:
            if spec == preferred:
                SdkLog.i(_TAG, f"Assign preferred dongle {spec} to {device_mac} (affinity {affinity})", mac=device_mac)
            else:
                SdkLog.i(_TAG, f"Preferred dongle ({affinity}) not available for {device_mac}, fall back to {spec}", mac=device_mac)
        old = self._dongle_assignments.get(device_mac)
        if old is not None and old != spec:
            # 防御：上次连接的旧分配尚未归还（其清理仍在进行，如传输 close
            # 秒级耗时）时被新分配覆盖，旧 spec 会永久泄漏——实测双 dongle
            # 双双泄漏后所有连接因无空闲 dongle 秒败。先归还旧 spec 再记录
            # 新分配；旧清理随后的 _release_dongle 弹出新条目属正常幂等路径
            SdkLog.w(_TAG, f"assign dongle {spec} to {device_mac} overwrites unreleased {old}, releasing old", mac=device_mac)
            self._ble_backend.release(old)
        self._dongle_assignments[device_mac] = spec
        SdkLog.i(_TAG, f"Assign dongle {spec} to {device_mac}", mac=device_mac)
        return spec, self._ble_backend.client_kwargs(spec)

    async def _wait_scan_dongle_release(self, device_mac: str, timeout: float = 10.0):
        """无空闲 dongle 且扫描正在借用时：等待在途扫描周期结束释放后重试分配。

        扫描借用是短暂的（一个扫描周期即归还），连接优先于扫描；仅扫描
        借用期间等待，真正全被连接占用时仍快速失败。返回同 _assign_dongle。
        """
        if not self._scan_borrowed:
            return None, None
        SdkLog.i(_TAG, f"No free dongle for {device_mac}, waiting for in-flight scan to release", mac=device_mac)
        deadline = time.monotonic() + timeout
        while self._scan_borrowed and time.monotonic() < deadline:
            await asyncio.sleep(0.2)
            spec, kwargs = self._assign_dongle(device_mac)
            if spec is not None:
                return spec, kwargs
        return None, None

    def _release_dongle(self, device_mac: str):
        """归还连接占用的 dongle（幂等）。释放后对账一次枚举，解冻可重建的空闲槽。"""
        spec = self._dongle_assignments.pop(device_mac, None)
        if spec is None or self._ble_backend is None:
            return
        self._ble_backend.release(spec)
        SdkLog.i(_TAG, f"Release dongle {spec} from {device_mac}", mac=device_mac)
        old_specs = list(self._ble_backend.transport_specs)
        departed = self._ble_backend.reconcile(self._dongle_specs)
        if departed:
            self._handle_dongle_departed(departed)
        # 消失的 spec（含冻结后本次解冻重建丢弃的）同步弹出传输缓存（幂等）
        now = set(self._ble_backend.transport_specs)
        for s in old_specs:
            if s not in now:
                asyncio.ensure_future(self._discard_dongle_transport(s))
        # 释放产生了新的空闲槽：补扫挂起的持续扫描（此前只挂在热插拔枚举变化上，
        # 单 dongle 被失败重连循环占用时，释放窗口无人触发补扫，扫描被饿死——
        # 补扫后下一轮重连会经 _wait_scan_dongle_release 等扫描轮结束，两者交替）
        self._retry_pending_scan()
        # 若有已耗尽预算的挂起重连（等其他 dongle 空出），给一次新机会
        self._retry_pending_reconnects()

    def _on_dongle_specs_changed(self, new_specs: list, topo_map: dict = None):
        """热插拔监控线程回调：记录最新枚举并序列化到主事件循环处理。"""
        self._dongle_specs = list(new_specs)
        loop = self._main_event_loop
        if loop is None or loop.is_closed():
            return
        loop.call_soon_threadsafe(self._apply_dongle_specs, list(new_specs), topo_map)

    def _apply_dongle_specs(self, new_specs: list, topo_map: dict = None):
        """在主事件循环中应用 dongle 枚举变化：运行中切换后端 / 对账 dongle 池。"""
        if topo_map is not None:
            # 每次枚举刷新 spec -> 拓扑身份映射（供连接偏好用）
            if topo_map != self._spec_topology:
                SdkLog.i(_TAG, "dongle topology: " + ", ".join(
                    f"{spec}={topo}" for spec, topo in sorted(topo_map.items())))
            self._spec_topology = topo_map
        if self._ble_backend is None:
            # 启动时无 dongle（原生 bleak 运行中），插入后切换到 bumble 后端
            if not new_specs:
                return
            backend = bumble_dongle.create_runtime_backend(new_specs)
            if backend is None:
                return
            self._ble_backend = backend
            SdkLog.i(_TAG, f"Hotplug: BLE backend switched to bumble ({new_specs})")
            self._publish("backend_info", backend="bumble",
                          transport=",".join(backend.transport_specs))
            self._retry_pending_scan()
            self._retry_pending_reconnects()
            return

        old_free = self._ble_backend.free_count()
        old_specs = list(self._ble_backend.transport_specs)
        departed = self._ble_backend.reconcile(new_specs)
        if departed:
            self._handle_dongle_departed(departed)
        if departed or self._ble_backend.transport_specs != old_specs:
            self._publish("backend_info", backend="bumble",
                          transport=",".join(self._ble_backend.transport_specs))
        # 空闲 dongle 离开（拔出/复位）：同样弹出其传输缓存。reconcile 只对
        # 占用中的 spec 报告 departed；空闲离开的 spec 若不及时清理，复插后
        # 同 spec 会复用到死传输（扫描/连接静默失败）。占用中离开的已由
        # _handle_dongle_departed 清理，这里的弹出是幂等 no-op
        now = set(self._ble_backend.transport_specs)
        for spec in old_specs:
            if spec not in now:
                asyncio.ensure_future(self._discard_dongle_transport(spec))
        self._retry_pending_scan()
        # 空闲槽增加（新插入/解冻）时，给已耗尽的重试预算一次新机会
        if self._ble_backend.free_count() > old_free:
            self._retry_pending_reconnects()

    def _retry_pending_reconnects(self):
        """dongle 池新增空闲槽后，重新激活已耗尽的自动重试。

        无空闲 dongle 期间的连接尝试会快速失败，很快就会耗尽
        _MAX_RECONNECT_ATTEMPTS 预算；新硬件到位时重置计数并重新调度，
        否则插入 dongle 后自动重连不会恢复。只处理已耗尽的条目，
        进行中的重连（含刚断开后正常调度的）不受影响。
        """
        for mac, info in list(self._reconnect_info.items()):
            if (info.get("normal_disconnect") or info.get("task") is not None
                    or mac in self._gforces
                    or info["attempts"] < _MAX_RECONNECT_ATTEMPTS):
                continue
            SdkLog.i(_TAG, f"Retrying auto reconnect for {mac} after dongle change", mac=mac)
            info["attempts"] = 0
            self._schedule_reconnect(mac)

    def _retry_pending_scan(self):
        """无空闲 dongle 期间被跳过的持续扫描：dongle 池有空闲后补扫，
        结果到达主进程后由其重扫循环接管（stopScan 已清除挂起）。"""
        period = self._pending_scan_period
        if period is None or self._ble_backend is None:
            return
        if not self._ble_backend.has_free():
            return
        self._pending_scan_period = None
        SdkLog.i(_TAG, "Retrying pending scan after dongle change")
        SdkLog.controller(_TAG, f"retrying pending scan (period={period}) after dongle change")
        asyncio.run_coroutine_threadsafe(
            self._do_start_scan(period), self._ensure_scan_loop())

    async def _discard_dongle_transport(self, spec: str):
        """dongle 离开（拔出/总线复位）或扫描轮异常后，清理 bleak_bumble 传输缓存。

        bleak_bumble 的 transports 按 spec 字符串缓存复用传输（patch 5 让连接
        传输在进程内常驻）；dongle 物理离开或扫描异常中止时，死传输会留在缓存
        里——dongle 重新插入（或 PowerShell 复位重枚举）后同 spec 会复用到死
        传输，重连/扫描静默失败。这里弹出并尽力关闭。
        """
        if spec is None:
            return
        try:
            from sensor import bleak_bumble as _bb

            transport = _bb.transports.pop(spec, None)
            if transport is not None:
                SdkLog.d(_TAG, f"discarding dongle transport: {spec}")
                t0 = time.monotonic()
                # patched_close 内部已有界（terminate 2×1s + join 2s，且超时
                # 也会走完 releaseInterface/context.close()），这里只留 6s
                # 兜底防病态卡死。绝不能用更短的超时从外面取消 close：
                # 取消会让 release 步骤执行不到，dongle 接口被僵尸传输占死，
                # 之后打开该 dongle 永远 LIBUSB_ERROR_ACCESS
                try:
                    await asyncio.wait_for(transport.close(), timeout=6.0)
                    SdkLog.d(_TAG, f"discarded dongle transport: {spec}, close took {time.monotonic() - t0:.3f}s")
                except asyncio.TimeoutError:
                    SdkLog.w(_TAG, f"discard dongle transport close timeout (6s): {spec}")
        except Exception:
            SdkLog.exception(_TAG, f"Failed to discard dongle transport for {spec}")

    async def _note_scan_round_result(self, spec: str, device_count: int):
        """扫描空结果看门狗：连续空轮达阈值时弹出缓存传输，强制下轮重开。

        异常断连后缓存传输可能半死——HCI 命令有应答（power_on/start_scanning
        成功、扫描轮按时结束）但不再投递广播，表现为扫描无限期空结果且无任何
        错误/超时，三条既有缓存清理路径都不会触发；只有弹出缓存强制重开才能
        恢复。扫到设备即清零；阈值内仅计数，避免正常空环境频繁重开传输。
        """
        if spec is None:
            return
        if device_count > 0:
            self._scan_empty_rounds.pop(spec, None)
            return
        n = self._scan_empty_rounds.get(spec, 0) + 1
        if n < _SCAN_EMPTY_ROUND_TRANSPORT_RESET:
            self._scan_empty_rounds[spec] = n
            return
        self._scan_empty_rounds.pop(spec, None)
        SdkLog.w(_TAG, f"scan watchdog: {n} consecutive empty rounds on {spec}, discarding cached transport")
        SdkLog.controller(_TAG, f"scan watchdog: {n} consecutive empty rounds on {spec}, discarding cached transport")
        await self._discard_dongle_transport(spec)

    def _handle_dongle_departed(self, departed_specs: list):
        """占用中的 dongle 被拔出：主动断开受影响连接（按异常断开处理，保留自动重连）。

        断开前向主进程发 error 消息（onErrorCallback），随后 _cleanup_device
        发布 state_changed(Disconnected)，清理完成后调度自动重连（有其他空闲
        dongle 时恢复连接；无空闲 dongle 时重连失败即止，需重新插入后手动连接）。
        扫描借用的 dongle 不在分配表中，仅清理 departed 记录，扫描自身经现有
        错误路径/下次扫描自愈。
        """
        for spec in departed_specs:
            macs = [mac for mac, s in self._dongle_assignments.items() if s == spec]
            for mac in macs:
                SdkLog.w(_TAG, f"Dongle {spec} unplugged, disconnecting {mac}", mac=mac)
                self._dongle_assignments.pop(mac, None)
                self._publish("error", device_mac=mac,
                              message=f"USB BLE dongle {spec} unplugged")
                loop = self._event_loops.get(mac)
                if loop is not None and not loop.is_closed():
                    try:
                        asyncio.run_coroutine_threadsafe(
                            self._cleanup_and_reconnect(mac), loop)
                    except Exception as e:
                        SdkLog.exception(_TAG, f"Failed to cleanup {mac} after dongle unplug: {e}", mac=mac)
            # 清理 departed 记录（幂等；连接清理里的 _release_dongle 找不到分配会安全跳过）
            if self._ble_backend is not None:
                self._ble_backend.release(spec)
            # 同步弹出该 dongle 的传输缓存：复位/重插后是全新设备，
            # 不能复用旧传输（否则重连永远打到死句柄上）
            asyncio.ensure_future(self._discard_dongle_transport(spec))

    async def _cleanup_and_reconnect(self, device_mac: str):
        """先完成异常断开清理，再调度自动重连（dongle 拔出等链路死亡场景）。

        清理完成后 _gforces 已弹出，_do_reconnect 的"已连接则跳过"检查不会
        误命中；重连由 _schedule_reconnect 的既有去重/次数上限约束。
        """
        await self._cleanup_device(device_mac, disconnect_client=False)
        self._schedule_reconnect(device_mac)

    def _init_scanner(self, kwargs: dict):
        if self._scanner is None:
            self._scanner = BleakScanner(
                detection_callback=_match_device,
                service_uuids=[RFSTAR_SERVICE_GUID, SERVICE_GUID],
                **kwargs,
            )

    def _ensure_gforce_loop(self):
        from sensor import sensor_utils
        if self._gforce_event_loop is None or self._gforce_event_loop.is_closed():
            self._gforce_event_loop = asyncio.new_event_loop()
            self._gforce_event_thread = threading.Thread(
                target=sensor_utils.start_loop, args=(self._gforce_event_loop,)
            )
            self._gforce_event_thread.daemon = True
            self._gforce_event_thread.name = "gforce_event"
            self._gforce_event_thread.start()
        return self._gforce_event_loop

    def _ensure_scan_loop(self):
        from sensor import sensor_utils
        if self._scan_event_loop is None or self._scan_event_loop.is_closed():
            self._scan_event_loop = asyncio.new_event_loop()
            self._scan_event_thread = threading.Thread(
                target=sensor_utils.start_loop, args=(self._scan_event_loop,)
            )
            self._scan_event_thread.daemon = True
            self._scan_event_thread.name = "scan_event"
            self._scan_event_thread.start()
        return self._scan_event_loop

    def _ensure_device_loops(self, device_mac: str):
        from sensor import sensor_utils

        event_loop = self._event_loops.get(device_mac)
        data_event_loop = self._data_event_loops.get(device_mac)
        if (event_loop is not None and not event_loop.is_closed() and
                data_event_loop is not None and not data_event_loop.is_closed()):
            self._cleanup_locks.setdefault(device_mac, asyncio.Lock())
            return

        event_loop = asyncio.new_event_loop()
        event_thread = threading.Thread(target=sensor_utils.start_loop, args=(event_loop,))
        event_thread.daemon = True
        event_thread.name = device_mac + "_event"
        event_thread.start()

        gforce_event_loop = self._ensure_gforce_loop()

        data_event_loop = asyncio.new_event_loop()
        data_event_thread = threading.Thread(target=sensor_utils.start_loop, args=(data_event_loop,))
        data_event_thread.daemon = True
        data_event_thread.name = device_mac + "_data_event"
        data_event_thread.start()

        self._event_loops[device_mac] = event_loop
        self._event_threads[device_mac] = event_thread
        self._data_event_loops[device_mac] = data_event_loop
        self._data_event_threads[device_mac] = data_event_thread
        self._cleanup_locks[device_mac] = asyncio.Lock()

    def _stop_device_loops(self, device_mac: str):

        # Cancel battery task if still running in event_loop
        if device_mac in self._battery_tasks:
            task = self._battery_tasks.pop(device_mac, None)
            if task:
                try:
                    loop = self._event_loops.get(device_mac)
                    if loop and not loop.is_closed():
                        loop.call_soon_threadsafe(task.cancel)
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error")

        # Cancel data task first if still running in data_event_loop
        if device_mac in self._data_tasks:
            task = self._data_tasks.pop(device_mac, None)
            if task:
                try:
                    loop = self._data_event_loops.get(device_mac)
                    if loop and not loop.is_closed():
                        loop.call_soon_threadsafe(task.cancel)
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error")

        # Stop loops and join threads (gforce loop 为单例、per-dongle loop 归
        # BumbleBackend 池管理，都不在此停止)
        for loop_dict, thread_dict in [
            (self._data_event_loops, self._data_event_threads),
            (self._event_loops, self._event_threads),
        ]:
            loop = loop_dict.pop(device_mac, None)
            thread = thread_dict.pop(device_mac, None)
            if loop is not None and not loop.is_closed():
                try:
                    loop.call_soon_threadsafe(loop.stop)
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error")
            if thread is not None and thread.is_alive():
                try:
                    thread.join(timeout=2)
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error")

        self._cleanup_locks.pop(device_mac, None)

    def run(self):

        # 先对齐 debug 标志，避免 set_log_dir 在默认目录误建 controller log
        SdkLog._debug_enabled = self._debug_enabled
        SdkLog.set_log_dir(self._log_dir or None,
                           enabled=self._file_output_enabled,
                           controller_log_path=self._controller_log_path)

        # bleak 原始日志中的扫描记录转发到 controller log（两种后端通用）
        bumble_dongle.patch_bleak_scan_logging()

        data_log_enabled = os.environ.get("SENSORSKD_DATA_LOG_ENABLED", "0") == "1"
        SdkLog.set_data_log_enabled(data_log_enabled)

        if platform.system() == "Windows":
            try:
                from bleak.backends.winrt.util import allow_sta
                allow_sta()
            except ImportError as e:
                SdkLog.exception(_TAG, "Unexpected error")


        # "cannot pickle '_thread.lock' object"
        self._msg_queue = queue.Queue()

        # 检测 USB 蓝牙 dongle，决定使用 bleak_bumble 还是原生 bleak 后端（启动时确定一次）
        self._ble_backend = bumble_dongle.resolve_bumble_backend()
        if self._ble_backend is not None:
            self._dongle_specs = list(self._ble_backend.transport_specs)
            # 启动时补一次带拓扑的枚举，建立 spec -> 拓扑身份映射（供连接偏好用）
            try:
                _specs, topo_map = bumble_dongle.detect_usb_dongle_specs(with_topology=True)
                self._spec_topology = topo_map
                if topo_map:
                    SdkLog.i(_TAG, "dongle topology: " + ", ".join(
                        f"{spec}={topo}" for spec, topo in sorted(topo_map.items())))
            except Exception:
                pass
            SdkLog.i(_TAG, f"BLE backend: bumble ({self._ble_backend.transport_specs})")
        else:
            SdkLog.i(_TAG, "BLE backend: bleak (native)")
        self._publish(
            "backend_info",
            backend="bumble" if self._ble_backend is not None else "bleak",
            transport=(",".join(self._ble_backend.transport_specs)
                       if self._ble_backend is not None else ""),
        )

        async def _run_main():
            self._main_event_loop = asyncio.get_running_loop()
            # 发布者用独立线程，避免被事件循环上的同步背压等待饿死（见 _publisher_loop）
            publisher_thread = threading.Thread(
                target=self._publisher_loop, name="BleakProcessPublisher", daemon=True
            )
            publisher_thread.start()
            # USB dongle 热插拔监控：插入/拔出事件经主事件循环对账 dongle 池
            self._dongle_monitor = bumble_dongle.create_hotplug_monitor(
                self._on_dongle_specs_changed, self._dongle_specs)
            if self._dongle_monitor is not None:
                self._dongle_monitor.start()
            try:
                await self._main_loop()
            finally:
                self._should_exit = True
                publisher_thread.join(timeout=2.0)
                self._flush_msg_queue()

        asyncio.run(_run_main())

        # Stop singleton gforce loop
        if self._gforce_event_loop is not None and not self._gforce_event_loop.is_closed():
            try:
                self._gforce_event_loop.call_soon_threadsafe(self._gforce_event_loop.stop)
            except Exception as e:
                SdkLog.exception(_TAG, "Unexpected error")
        if self._gforce_event_thread is not None and self._gforce_event_thread.is_alive():
            try:
                self._gforce_event_thread.join(timeout=2)
            except Exception as e:
                SdkLog.exception(_TAG, "Unexpected error")

        for device_mac in list(self._event_loops.keys()):
            self._stop_device_loops(device_mac)

        # bumble 后端：进程退出前显式关闭复用的 USB transport，
        # 避免解释器 GC 在 USB 事件线程存活时销毁 libusb 上下文（SIGABRT）
        if self._ble_backend is not None:
            try:
                from sensor.bleak_bumble import transports as bumble_transports

                async def _close_bumble_transports():
                    for transport in list(bumble_transports.values()):
                        try:
                            await asyncio.wait_for(transport.close(), timeout=3.0)
                        except Exception:
                            pass
                    bumble_transports.clear()

                asyncio.run(_close_bumble_transports())
            except Exception as e:
                SdkLog.exception(_TAG, "Unexpected error")

        # 进程退出前停止日志监听器，确保队列中的日志落盘
        SdkLog.stop()

        if self._ble_backend is not None or self._dongle_monitor is not None:
            # bumble 后端 / dongle 热插拔监控：跳过后续 GC/atexit。usb1/libusb
            # 的对象析构会在解释器收尾时再次触碰已销毁（或监控线程仍占用）的
            # 上下文，触发 usbi_mutex_destroy 断言（SIGABRT）。
            # 此处一切资源已清理完毕，直接退出由内核回收。
            os._exit(0)

    async def _main_loop(self):

        # 循环健康监控：单轮迭代超过 _MAIN_LOOP_STALL_SECONDS 即告警。
        # 实测出现过命令入队后 10~30s 才被取出的 stall（stop_notification /
        # terminate 迟迟未被处理），根因未定时靠此日志定位卡在哪一轮
        last_tick = time.monotonic()
        while not self._should_exit:
            try:
                cmd = self.cmd_queue.get_nowait()
                SdkLog.d(_TAG, f"main loop dequeued cmd: {cmd.get('type')}, "
                               f"mac={cmd.get('device_mac')}")
                asyncio.create_task(self._handle_command(cmd))
            except multiprocessing.queues.Empty:
                await asyncio.sleep(0.05)
            now = time.monotonic()
            if now - last_tick > _MAIN_LOOP_STALL_SECONDS:
                SdkLog.w(_TAG, f"main loop stalled {now - last_tick:.1f}s between iterations")
                SdkLog.controller(_TAG, f"main loop stalled {now - last_tick:.1f}s between iterations")
            last_tick = now

        await self._cleanup_all_devices()

    async def _cleanup_all_devices(self):
        for device_mac in list(self._gforces.keys()):
            SdkLog.i(_TAG, f"shutdown: cleaning up device {device_mac}", mac=device_mac)
            loop = self._event_loops.get(device_mac)
            if loop is not None and not loop.is_closed():
                try:
                    future = asyncio.run_coroutine_threadsafe(
                        self._cleanup_device(device_mac, disconnect_client=False), loop
                    )
                    await asyncio.wait_for(asyncio.wrap_future(future), timeout=3)
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error")
            self._stop_device_loops(device_mac)

    async def _handle_command(self, cmd: dict):
        cmd_type = cmd.get("type")

        # Scan commands run in the dedicated scan loop
        if cmd_type == "scan_once":
            SdkLog.controller(_TAG, f"scan_once: period={cmd.get('period', 5000)}")
            loop = self._ensure_scan_loop()
            asyncio.run_coroutine_threadsafe(
                self._do_scan_once(cmd.get("period", 5000)), loop
            )
            return
        if cmd_type == "start_scan":
            SdkLog.controller(_TAG, f"start_scan: period={cmd.get('period', 5000)}")
            loop = self._ensure_scan_loop()
            asyncio.run_coroutine_threadsafe(
                self._do_start_scan(cmd.get("period", 5000)), loop
            )
            return
        if cmd_type == "stop_scan":
            self._is_scanning = False
            self._pending_scan_period = None
            SdkLog.controller(_TAG, "stop_scan")
            return
        if cmd_type == "terminate":
            self._is_scanning = False
            self._should_exit = True
            return
        if cmd_type == "set_log_dir":
            # 主进程日志目录/文件输出开关变更时同步到子进程
            try:
                SdkLog.set_log_dir(cmd.get("dir") or None,
                                   enabled=cmd.get("enabled", True),
                                   controller_log_path=cmd.get("controller_log_path"))
            except Exception as e:
                SdkLog.exception(_TAG, f"set_log_dir failed: {e}")
            return
        if cmd_type == "set_debug_enabled":
            # 主进程 debug 开关变更时同步到子进程（子进程同步创建/关闭 controller log）
            try:
                SdkLog.set_debug_enabled(cmd.get("enabled", True),
                                         controller_log_path=cmd.get("controller_log_path"))
            except Exception as e:
                SdkLog.exception(_TAG, f"set_debug_enabled failed: {e}")
            return

        # Connect runs in main loop because it creates device loops
        if cmd_type == "connect":
            await self._do_connect(cmd)
            return

        # 离线回放 bin 文件：在主事件循环执行，不依赖真实设备连接
        if cmd_type == "replay_bin":
            await self._do_replay_bin(cmd)
            return

        # 多设备同步起流：主循环协调各设备 loop，起流写命令在 bumble 后端
        # 等待同一个 SyncWriteGate 放行后几乎同时下发
        if cmd_type == "multi_start_notification":
            await self._do_multi_start_notification(cmd)
            return

        # 多设备同步停流：与同步起流同一门闩机制
        if cmd_type == "multi_stop_notification":
            await self._do_multi_stop_notification(cmd)
            return

        # 暂停/恢复、停止回放：只设置控制标志，立即返回
        if cmd_type == "pause_replay_bin":
            device_mac = cmd.get("device_mac")
            ctrl = self._replay_controls.get(device_mac)
            paused = bool(cmd.get("paused", True))
            if ctrl is not None:
                ctrl["paused"] = paused
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=(ctrl is not None),
                result="OK" if ctrl is not None else "Error: no replay running",
            )
            return
        if cmd_type == "stop_replay_bin":
            device_mac = cmd.get("device_mac")
            ctrl = self._replay_controls.get(device_mac)
            if ctrl is not None:
                ctrl["stop"] = True
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=(ctrl is not None),
                result="OK" if ctrl is not None else "Error: no replay running",
            )
            return

        if cmd_type == "cancel_reconnect":
            # 轻量意图命令（主进程 disconnect 入口先行发出，fire-and-forget）：
            # 直接在主循环登记用户断连意图并取消挂起的自动重连，不经过可能
            # 已卡死的设备/gforce 事件循环
            device_mac = cmd.get("device_mac")
            SdkLog.i(_TAG, f"cancel_reconnect (user disconnect intent): {device_mac}", mac=device_mac)
            SdkLog.controller(_TAG, f"cancel_reconnect (user disconnect intent): {device_mac}")
            self._user_disconnect_pending.add(device_mac)
            self._cancel_pending_reconnect(device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=True,
                result=True,
            )
            return

        if cmd_type == "force_link_drop":
            # 主进程恢复流程发出（看门狗超时，或默认恢复流程 init/起流失败的
            # fail-fast）：重连成功后恢复反复失败，链路虽在但设备应用层不应答——
            # 按异常断开强制掉链（保留重连信息），让链路层重连滚动重来。直接在
            # 主循环调度到 gforce 事件循环，不经过可能已积压的设备命令队列
            device_mac = cmd.get("device_mac")
            ctx = self._data_ctxs.get(device_mac)
            dropped = False
            gforce = getattr(ctx, "gForce", None) if ctx is not None else None
            if gforce is not None:
                loop = getattr(gforce, "gforce_event_loop", None)
                client = getattr(gforce, "client", None)
                if (loop is not None and not loop.is_closed()
                        and getattr(client, "is_connected", False)):
                    SdkLog.w(_TAG, f"force_link_drop (recovery stuck): {device_mac}", mac=device_mac)
                    SdkLog.controller(_TAG, f"force_link_drop (recovery stuck): {device_mac}")
                    asyncio.run_coroutine_threadsafe(
                        ctx._force_half_dead_disconnect(
                            "recovery stuck after reconnect"), loop)
                    dropped = True
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=True,
                result=dropped,
            )
            return

        # All other device commands run in the device's event_loop
        device_cmd_handlers = {
            "disconnect": self._do_disconnect,
            "init": self._do_init,
            "start_notification": self._do_start_notification,
            "stop_notification": self._do_stop_notification,
            "get_battery": self._do_get_battery,
            "get_param": self._do_get_param,
            "set_neucir_app_control": self._do_set_neucir_app_control,
            "set_neucir_mode": self._do_set_neucir_mode,
            "set_param": self._do_set_param,
        }

        if cmd_type not in device_cmd_handlers:
            return

        device_mac = cmd.get("device_mac")
        if cmd_type == "disconnect":
            # 断连意图尽早登记（与 cancel_reconnect 相同）：即使 _do_disconnect
            # 因设备 loop 积压迟迟执行不到，随后的异常掉链清理也不会再触发
            # 自动重连
            self._user_disconnect_pending.add(device_mac)
            self._cancel_pending_reconnect(device_mac)
        loop = self._event_loops.get(device_mac)
        if loop is None or loop.is_closed():
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Device not connected",
            )
            return

        handler = device_cmd_handlers[cmd_type]
        future = asyncio.run_coroutine_threadsafe(handler(cmd), loop)
        SdkLog.d(_TAG, f"cmd {cmd_type} dispatched to device loop: {device_mac}", mac=device_mac)
        try:
            await asyncio.wait_for(asyncio.wrap_future(future), timeout=25)
        except asyncio.TimeoutError:
            SdkLog.e(_TAG, f"_handle_command timeout: {cmd_type}: {device_mac}", mac=device_mac)
            SdkLog.controller(_TAG, f"_handle_command timeout: {cmd_type}: {device_mac}")
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Timeout",
            )
        except Exception as e:
            SdkLog.exception(_TAG, f"_handle_command failed: {cmd_type}")
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )

    # ------------------------------------------------------------------
    # Scan commands (existing)
    # ------------------------------------------------------------------
    async def _do_scan_once(self, period: int):

        # bumble 后端：扫描占用一只空闲 dongle；全部已被连接占用时跳过本次扫描
        # （主进程不再收到 devices，周期重扫随之停止）
        spec, kwargs = self._scan_dongle_kwargs()
        if self._ble_backend is not None and spec is None:
            SdkLog.w(_TAG, "Scan ignored: no free dongle (all in use)")
            SdkLog.controller(_TAG, "scan_once skipped: no free dongle (all in use)")
            return

        await self._run_scan_round_on_dongle_loop(period, spec, kwargs, once=True)

    async def _do_start_scan(self, period: int):

        spec, kwargs = self._scan_dongle_kwargs()
        if self._ble_backend is not None and spec is None:
            # 主进程的重扫循环由扫描结果驱动：跳过即停摆。
            # 记录挂起扫描，dongle 池有空闲后由 _apply_dongle_specs 补扫恢复。
            SdkLog.w(_TAG, "Scan ignored: no free dongle (all in use), pending until one is available")
            SdkLog.controller(_TAG, "start_scan skipped: no free dongle, pending until one is available")
            self._pending_scan_period = period
            return
        self._pending_scan_period = None

        await self._run_scan_round_on_dongle_loop(period, spec, kwargs, once=False)

    async def _run_scan_round_on_dongle_loop(self, period: int, spec, kwargs: dict, once: bool):
        """执行一轮扫描，bumble 后端投递到被借 dongle 的专属 loop 上运行。

        bumble 传输打开时绑定事件循环（USB 事件线程把 HCI 包投回该 loop）：
        dongle 在扫描/连接间交接时传输终身绑在其专属 loop 上，双方都不需要
        重开传输。借用全程由本协程持有（含投递等待），轮次结束归还；
        原生 bleak 后端直接在当前（扫描）loop 上执行。
        """
        self._scan_borrowed = spec is not None
        try:
            loop = self._ble_backend.get_loop(spec) if spec is not None else None
            if loop is not None and asyncio.get_running_loop() is not loop:
                future = asyncio.run_coroutine_threadsafe(
                    self._run_scan_round(period, spec, kwargs, once), loop)
                # 轮次内部已有 period+grace 超时，外层只留兜底：spec loop
                # 卡死时释放借用，连接才不会永久等不到在途扫描释放
                await asyncio.wait_for(
                    asyncio.wrap_future(future),
                    timeout=period / 1000 + _SCAN_ROUND_GRACE_SECONDS + 30,
                )
            else:
                await self._run_scan_round(period, spec, kwargs, once)
        finally:
            if spec is not None:
                self._ble_backend.release(spec)
            self._scan_borrowed = False

    async def _run_scan_round(self, period: int, spec, kwargs: dict, once: bool):
        """一轮扫描：初始化 scanner、discover、处理并发布结果、喂看门狗。

        在被借 dongle 的专属 loop（bumble）或扫描 loop（原生后端）上执行；
        dongle 的借用/归还由 _run_scan_round_on_dongle_loop 负责。
        """
        tag = "scan_once" if once else "start_scan"
        try:
            self._init_scanner(kwargs)
            try:
                found_devices = await asyncio.wait_for(
                    self._scanner.discover(
                        timeout=period / 1000, return_adv=True, **kwargs
                    ),
                    timeout=period / 1000 + _SCAN_ROUND_GRACE_SECONDS,
                )
            except asyncio.TimeoutError:
                # 扫描中的 dongle 被拔出/死链路：stop() 内的 HCI 命令永远等不到
                # 应答，外层超时强制结束本轮，保证借用归还、不阻塞后续扫描/连接；
                # start_scan 按空结果上报以保活主进程重扫循环
                SdkLog.w(_TAG, f"{tag} round timed out (dongle lost?), finishing round")
                SdkLog.controller(_TAG, f"{tag} round timed out (dongle lost?), spec={spec}")
                await self._discard_dongle_transport(spec)
                found_devices = None
            if found_devices is None:
                found_devices = {}
            devices = self._process_ble_devices(found_devices)
            SdkLog.controller(_TAG, f"{tag} result: {len(devices)} device(s), spec={spec}")
            self._publish("scan_once_result" if once else "devices", devices=devices)
            await self._note_scan_round_result(spec, len(devices))
        except Exception as e:
            SdkLog.exception(_TAG, f"{tag} failed: {e}, spec={spec}")
            self._publish("error", message=f"{tag} failed: {e}")

    def _process_ble_devices(self, found_devices: dict) -> list:

        devices = []
        for uuid in found_devices:
            device = found_devices[uuid][0]
            if device.name is None:
                continue
            adv = found_devices[uuid][1]
            if SERVICE_GUID in adv.service_uuids:
                serialized = _serialize_device(device, adv)
                if serialized is not None:
                    mac = serialized.get("mac")
                    if mac:
                        self._devices[mac] = device
                    devices.append(serialized)
        return devices

    # ------------------------------------------------------------------
    # Device connection commands
    # ------------------------------------------------------------------
    async def _do_connect(self, cmd: dict):

        device_mac = cmd["device_mac"]
        # 注册 profile，使连接阶段日志可路由/缓存
        SdkLog.register_profile(device_mac)
        # 主进程在调试日志开启时随 connect 命令捎来 profile log 路径：
        # 从连接起点启用同一文件（两进程 append 同一文件），连接阶段日志
        # 直接进 profile log 而不回落 controller log；自动重连沿用原命令
        # 中的路径，追加写同一文件
        connect_log_path = cmd.get("profile_log_path")
        if connect_log_path:
            SdkLog.enable_profile_log(device_mac, connect_log_path)

        if device_mac in self._gforces:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=True,
                result=True,
            )
            return

        self._ensure_device_loops(device_mac)
        gforce_loop = self._ensure_gforce_loop()

        # 每次连接递增世代号。连接等待不能用 asyncio.wait_for：它的取消语义
        # 需要等 inner 协程处理完取消才真正返回，而 inner 可能挂在对取消不
        # 响应的 bumble 等待上（实测卡死 7 分钟直到 dongle 被拔掉），整个
        # 重连链随之停摆。改用 asyncio.wait：超时即放弃，不再等 inner 退出，
        # inner 迟到的结果按世代号失效处理
        epoch = self._connect_epochs.get(device_mac, 0) + 1
        self._connect_epochs[device_mac] = epoch

        future = asyncio.run_coroutine_threadsafe(
            self._do_connect_inner(cmd, epoch), gforce_loop)
        wrapped = asyncio.wrap_future(future)
        done, _pending = await asyncio.wait({wrapped}, timeout=_CONNECT_TIMEOUT_SECONDS)
        if not done:
            SdkLog.e(_TAG, f"_do_connect timeout: {device_mac}", mac=device_mac)
            SdkLog.controller(_TAG, f"connect timeout ({_CONNECT_TIMEOUT_SECONDS}s): {device_mac}")
            # 使本次卡死协程的迟到结果失效（已有更新的连接尝试时不动）
            if self._connect_epochs.get(device_mac) == epoch:
                self._connect_epochs[device_mac] = epoch + 1
            # best effort 取消；协程对取消不响应也不阻塞下面的清理。
            # 取回迟到结果/异常，避免 "exception was never retrieved" 噪音
            future.cancel()
            wrapped.add_done_callback(
                lambda f: None if f.cancelled() else f.exception())
            # 显式断开半开的 HCI 链路、清理设备状态并归还 dongle，
            # 否则分配泄漏且该设备会被误判为已连接
            gforce = self._gforces.get(device_mac)
            if gforce is not None:
                try:
                    await asyncio.wait_for(gforce.disconnect(), timeout=3.0)
                except Exception:
                    pass
            try:
                await self._cleanup_device(device_mac, disconnect_client=False)
            except Exception:
                SdkLog.exception(_TAG, f"cleanup after connect timeout failed: {device_mac}", mac=device_mac)
            self._release_dongle(device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Connect timeout",
            )
            return
        try:
            wrapped.result()
        except asyncio.CancelledError:
            # inner 协程被取消（区别于外层任务取消——那会在上面的
            # await asyncio.wait 处抛出，不在此臂）：实测为连接中途链路
            # 断开，bumble 取消 pending connect，CancelledError 穿透 inner
            # 各层 except Exception（它继承 BaseException）。按普通连接
            # 失败处理：清理、归还 dongle（否则分配泄漏）、发布失败结果，
            # 让 _do_reconnect 按失败续调度，重连链不中断
            SdkLog.w(_TAG, f"_do_connect inner cancelled (link lost mid-connect): {device_mac}", mac=device_mac)
            SdkLog.controller(_TAG, f"connect cancelled (link lost mid-connect): {device_mac}")
            try:
                await self._cleanup_device(device_mac, disconnect_client=False)
            except Exception:
                SdkLog.exception(_TAG, f"cleanup after connect cancel failed: {device_mac}", mac=device_mac)
            self._release_dongle(device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Connect cancelled (link lost mid-connect)",
            )
        except Exception as e:
            SdkLog.exception(_TAG, f"_do_connect failed: {device_mac}", mac=device_mac)
            # _do_connect_inner 自身 try 块之外抛错（分配/建对象阶段）时，
            # 同样需要清理并归还 dongle
            try:
                await self._cleanup_device(device_mac, disconnect_client=False)
            except Exception:
                SdkLog.exception(_TAG, f"cleanup after connect failure failed: {device_mac}", mac=device_mac)
            self._release_dongle(device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )

    async def _do_connect_inner(self, cmd: dict, epoch: int):

        from sensor.gforce import GForce
        from sensor.sensor_data_context import SensorProfileDataCtx

        device_mac = cmd["device_mac"]
        device_address = cmd["device_address"]
        name = cmd.get("name", "")
        service_data = cmd.get("service_data", {})

        # 保存连接信息，用于异常断开后自动重连。
        # 自动重连会再次走到这里：必须在原条目上就地更新并保留 attempts，
        # 否则计数每次被清零、_MAX_RECONNECT_ATTEMPTS 上限失效（无限重连）；
        # 用户主动发起的连接则重置计数，给自动重连全新的重试预算
        if not cmd.get("_is_reconnect"):
            # 用户显式重新连接：此前登记的断连意图已不适用，清除
            self._user_disconnect_pending.discard(device_mac)
        info = self._reconnect_info.get(device_mac)
        if info is not None:
            info["cmd"] = cmd
            info["normal_disconnect"] = False
            if not cmd.get("_is_reconnect"):
                info["attempts"] = 0
        else:
            self._reconnect_info[device_mac] = {
                "cmd": cmd,
                "attempts": 0,
                "task": None,
                "normal_disconnect": False,
            }

        event_loop = self._event_loops.get(device_mac)
        gforce_event_loop = self._gforce_event_loop
        data_event_loop = self._data_event_loops.get(device_mac)

        if event_loop is None or gforce_event_loop is None or data_event_loop is None:
            SdkLog.w(_TAG, f"connect failed fast: event loops not initialized: {device_mac}", mac=device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Event loops not initialized",
            )
            return

        # Determine service type and BLE chip type
        chip_type = BLEChipType.Unknown
        if service_data.get(SERVICE_GUID) is not None:
            cmd_char = OYM_CMD_NOTIFY_CHAR_UUID
            data_char = OYM_DATA_NOTIFY_CHAR_UUID
            is_universal = False
            chip_type = BLEChipType.OYM
        elif service_data.get(RFSTAR_SERVICE_GUID) is not None:
            cmd_char = RFSTAR_CMD_UUID
            data_char = RFSTAR_DATA_UUID
            is_universal = True
            chip_type = BLEChipType.RFSTAR
        else:
            SdkLog.w(_TAG, f"connect failed fast: invalid device service uuid: {device_mac}", mac=device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Invalid device service uuid",
            )
            return

        # Read bleak BLEDevice from scanned devices
        bleak_device = self._devices.get(device_mac)
        if bleak_device is None:
            SdkLog.w(_TAG, f"connect failed fast: device not found in scanned devices: {device_mac}", mac=device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Device not found in scanned devices",
            )
            return

        # 上次连接的清理可能仍在进行（异常清理丢弃传输的 close 有秒级耗时）：
        # 等其完成再分配 dongle，否则新分配覆盖旧条目后旧清理按新条目释放，
        # 旧 spec 永久泄漏（实测双 dongle 双双泄漏，之后所有连接因无空闲
        # dongle 秒败）。清理内部耗时已有上界（close 兜底 6s），轮询等待
        # 上限 10s，超时仍继续（防御，由 _assign_dongle 的覆盖兜底防泄漏）
        cleanup_lock = self._cleanup_locks.get(device_mac)
        if cleanup_lock is not None and cleanup_lock.locked():
            SdkLog.i(_TAG, f"connect waits for in-flight cleanup: {device_mac}", mac=device_mac)
            SdkLog.controller(_TAG, f"connect waits for in-flight cleanup: {device_mac}")
            for _ in range(100):
                if not cleanup_lock.locked():
                    break
                await asyncio.sleep(0.1)
            else:
                SdkLog.w(_TAG, f"connect cleanup wait timeout (10s), proceeding: {device_mac}", mac=device_mac)

        # bumble 后端：为本连接分配一只空闲 dongle（一 dongle 一设备，
        # 连接存续期间该 dongle 不再用于扫描或其他连接）；无空闲可能只是
        # 扫描临时借用，先等待在途扫描释放，真正全被占用才判失败
        dongle_spec, client_kwargs = self._assign_dongle(device_mac)
        if self._ble_backend is not None and dongle_spec is None:
            dongle_spec, client_kwargs = await self._wait_scan_dongle_release(device_mac)
        if self._ble_backend is not None and dongle_spec is None:
            SdkLog.w(_TAG, f"connect failed fast: no free BLE dongle: {device_mac}", mac=device_mac)
            SdkLog.controller(_TAG, f"connect failed fast: no free BLE dongle: {device_mac}")
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="No free BLE dongle",
            )
            return

        # bumble 后端：本连接的 bleak 操作（connect/GATT 命令/通知分发）在被分配
        # dongle 的专属 loop 上执行——传输打开时绑定该 loop，各 dongle 的 HCI
        # 处理互不排队；原生 bleak 后端沿用单例 gforce loop
        if self._ble_backend is not None:
            dongle_loop = self._ble_backend.get_loop(dongle_spec)
            if dongle_loop is None:
                # 防御：分配后 dongle 被拔出、对账已移除 spec（本协程在 gforce
                # 编排 loop 上，与主 loop 的 reconcile 可交错）；归还分配后
                # fail fast，拔出清理路径会处理连接状态（幂等）
                SdkLog.w(_TAG, f"connect failed fast: dongle loop unavailable: {device_mac}, "
                               f"spec={dongle_spec}", mac=device_mac)
                self._release_dongle(device_mac)
                self._publish(
                    "command_result",
                    cmd_id=cmd.get("cmd_id"),
                    device_mac=device_mac,
                    success=False,
                    result="Dongle loop unavailable",
                )
                return
            gforce_event_loop = dongle_loop

        # Create raw data buffer (local to sub-process)
        raw_buf = queue.Queue(maxsize=sensor_utils.BLEAK_RESULT_QUEUE_MAXSIZE)

        # Create GForce with per-device event loops（device_mac 为统一注册 MAC，
        # bleak_device.address 在 macOS 原生后端是 UUID，不能用于 profile 路由）
        gforce = GForce(bleak_device, cmd_char, data_char, is_universal, event_loop, gforce_event_loop, chip_type, client_kwargs=client_kwargs, device_mac=device_mac)

        # 主进程随 connect 命令捎来此前已设置的 bin 导出路径
        # （DEBUG_BLE_DATA_PATH，来自 _saved_params）：bin 从连接起点直接
        # 写在导出文件上，不再先写 temp 再拷贝；自动重连沿用原命令中的路径
        bin_export_path = cmd.get("bin_export_path") or None
        if bin_export_path:
            gforce._bin_export_path_hint = bin_export_path

        # Define disconnect callback: schedule cleanup in event_loop
        def handle_disconnect(_):
            # 远端断链/静默丢链的唯一入口日志：区分「设备自发掉链」与
            # 看门狗/用户路径触发的清理，排查重连问题时先找这一行
            SdkLog.w(_TAG, f"BLE link lost (disconnect callback): {device_mac}", mac=device_mac)
            SdkLog.controller(_TAG, f"BLE link lost (disconnect callback): {device_mac}")
            loop = self._event_loops.get(device_mac)
            if loop is not None and not loop.is_closed():
                try:
                    asyncio.run_coroutine_threadsafe(
                        self._cleanup_device(device_mac, disconnect_client=False), loop
                    )
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error", mac=device_mac)

        try:
            connect_t0 = time.monotonic()
            await gforce.connect(handle_disconnect, raw_buf)
            SdkLog.d(_TAG, f"gforce.connect returned: {device_mac} "
                           f"({(time.monotonic() - connect_t0) * 1000:.0f}ms)", mac=device_mac)
            if dongle_spec is not None:
                # 记录本次成功连接所用 dongle 的拓扑身份：断连/热插拔后的
                # 重连优先分配同一只（同一物理口复插即命中；换口/占用则
                # 由 _assign_dongle 立即退回任意空闲 dongle）
                topo = self._spec_topology.get(dongle_spec)
                if topo is not None:
                    self._dongle_affinity[device_mac] = topo
        except Exception as e:
            SdkLog.exception(_TAG, f"gforce.connect failed: {device_mac}", mac=device_mac)
            if epoch != self._connect_epochs.get(device_mac):
                # 迟到失败：该世代已被外层超时分支清理（dongle 已归还），不再处理
                return
            await self._cleanup_device(device_mac, disconnect_client=False)
            self._release_dongle(device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )
            return

        if not gforce.client or not gforce.client.is_connected:
            if epoch != self._connect_epochs.get(device_mac):
                return
            await self._cleanup_device(device_mac, disconnect_client=False)
            self._release_dongle(device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Connection failed",
            )
            return

        # 连接耗时超过外层等待上限、已被宣告超时（或已被更新的连接尝试取代）
        # 时，迟到的成功不能提交：dongle 已归还/重分配，提交会复活半开状态
        if epoch != self._connect_epochs.get(device_mac):
            SdkLog.w(_TAG, f"Stale connect completed for {device_mac}, tearing down", mac=device_mac)
            try:
                await asyncio.wait_for(gforce.disconnect(), timeout=3.0)
            except Exception:
                pass
            return

        # Store state
        self._gforces[device_mac] = gforce
        self._raw_bufs[device_mac] = raw_buf
        self._device_states[device_mac] = "Connected"

        # 重连成功，重置重连计数
        reconnect_info = self._reconnect_info.get(device_mac)
        if reconnect_info is not None:
            reconnect_info["attempts"] = 0

        # Create data context
        data_ctx = SensorProfileDataCtx(
            gforce,
            device_mac,
            raw_buf,
            on_reconnect_request=lambda: self._schedule_reconnect(device_mac),
        )
        self._data_ctxs[device_mac] = data_ctx
        if bin_export_path:
            # 与 gforce 直写提示一致：finalize 不再拷贝（已在导出文件上），
            # getParam("DEBUG_BLE_DATA_PATH") 也能返回该路径
            data_ctx._data_log_enabled = True
            data_ctx._data_log_path = bin_export_path
        # DeviceInfo 字段变化（如 setParam 改 EEG/ECG 采样率）上报主进程
        data_ctx.on_device_info_changed = lambda fields: self._publish(
            "device_info_update", device_mac=device_mac, fields=fields)

        # bumble 后端：订阅连接参数更新事件（外设常在连接后经 L2CAP 更新一次
        # 参数），变化时记录日志并上报主进程刷新 DeviceInfo
        self._attach_link_param_listener(device_mac, gforce)

        # Start data processing loop in data_event_loop
        asyncio.run_coroutine_threadsafe(self._device_data_loop(device_mac), data_event_loop)

        # Publish states
        self._publish("state_changed", device_mac=device_mac, state="Connected")
        if cmd.get("_is_reconnect"):
            # 自动重连：设备刚重新上电，发布 Ready（触发上层流恢复）前给
            # 固件 1s 稳定时间；期间被断开/再次掉链则不再发布 Ready
            SdkLog.d(_TAG, f"reconnect settle {_RECONNECT_SETTLE_SECONDS}s before Ready: {device_mac}", mac=device_mac)
            await asyncio.sleep(_RECONNECT_SETTLE_SECONDS)
            if (device_mac not in self._gforces
                    or self._device_states.get(device_mac) != "Connected"):
                SdkLog.i(_TAG, f"reconnect settle aborted, device gone: {device_mac}", mac=device_mac)
                SdkLog.controller(_TAG, f"reconnect settle aborted, device gone: {device_mac}")
                return
        self._publish("state_changed", device_mac=device_mac, state="Ready")
        SdkLog.d(_TAG, f"connect success, publishing result: {device_mac}", mac=device_mac)
        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=True,
            chip_type=chip_type.value,
        )

    def _attach_link_param_listener(self, device_mac: str, gforce):
        """订阅 bumble 连接的参数更新事件：事件在 dongle 事件循环线程触发，
        读取最新 parameters 记录日志并上报主进程（_publish 基于队列，线程安全）。
        原生 bleak 后端无 _connection/parameters，直接返回。"""
        try:
            client = getattr(gforce, "client", None)
            # gforce.client 是 bleak.BleakClient 调度壳，bumble 实例在 _backend 上
            backend = getattr(client, "_backend", client)
            connection = getattr(backend, "_connection", None)
            if connection is None or not hasattr(connection, "parameters"):
                return

            def _on_params_updated():
                try:
                    p = connection.parameters
                    SdkLog.i(_TAG, f"connection parameters updated: "
                                   f"interval={p.connection_interval}ms, "
                                   f"latency={p.peripheral_latency}, "
                                   f"timeout={p.supervision_timeout}ms",
                             mac=device_mac)
                    fields = {
                        "ConnectionIntervalMs": float(p.connection_interval),
                        "PeripheralLatency": int(p.peripheral_latency),
                        "SupervisionTimeoutMs": int(p.supervision_timeout),
                    }
                    try:
                        mtu = int(client.mtu_size or 0)
                    except Exception:
                        mtu = 0
                    if mtu > 0:
                        fields["MTUSize"] = mtu
                    self._publish(
                        "device_info_update",
                        device_mac=device_mac,
                        fields=fields,
                    )
                except Exception:
                    pass

            connection.on("connection_parameters_update", _on_params_updated)
        except Exception:
            pass

    async def _cleanup_device(self, device_mac: str, disconnect_client: bool = False):

        lock = self._cleanup_locks.get(device_mac)
        if lock is None:
            return

        async with lock:
            if device_mac not in self._gforces and not disconnect_client:
                # 连接尚未提交（_gforces 登记前）就收到断开回调时也走这里，
                # 用于区分「重复清理」与「连接中途的断开事件」
                SdkLog.d(_TAG, f"cleanup skipped (already cleaned or not yet connected): {device_mac}", mac=device_mac)
                return  # Already cleaned

            SdkLog.d(_TAG, f"cleanup device {device_mac} (disconnect_client={disconnect_client})", mac=device_mac)

            # 用户主动断开时标记为正常断开，并取消正在进行的自动重连
            if disconnect_client:
                self._cancel_pending_reconnect(device_mac)

            # Cancel data task (running in data_event_loop)
            if device_mac in self._data_tasks:
                task = self._data_tasks.pop(device_mac, None)
                if task is not None:
                    try:
                        task.cancel()
                    except Exception as e:
                        SdkLog.exception(_TAG, "Unexpected error", mac=device_mac)

            # Cancel battery task
            if device_mac in self._battery_tasks:
                task = self._battery_tasks.pop(device_mac, None)
                if task is not None:
                    try:
                        task.cancel()
                    except Exception as e:
                        SdkLog.exception(_TAG, "Unexpected error", mac=device_mac)

            # Stop streaming and optionally disconnect client
            if disconnect_client and device_mac in self._gforces:
                if device_mac in self._data_ctxs:
                    ctx = self._data_ctxs[device_mac]
                    if ctx.isDataTransfering:
                        try:
                            # 调用侧超时兜底：dongle 挂死时投递到 gforce loop 的
                            # 停流协程永远不会执行，其内部超时不会生效
                            await asyncio.wait_for(
                                ctx.stop_streaming(),
                                timeout=_STOP_STREAM_TIMEOUT_SECONDS)
                        except asyncio.TimeoutError:
                            SdkLog.w(_TAG, f"cleanup stop_streaming timed out "
                                           f"({_STOP_STREAM_TIMEOUT_SECONDS}s): {device_mac}", mac=device_mac)
                            SdkLog.controller(_TAG, f"cleanup stop_streaming timed out: {device_mac}")
                        except Exception as e:
                            SdkLog.exception(_TAG, "Unexpected error", mac=device_mac)
                try:
                    await asyncio.wait_for(
                        self._gforces[device_mac].disconnect(),
                        timeout=_CLIENT_DISCONNECT_TIMEOUT_SECONDS)
                except asyncio.TimeoutError:
                    SdkLog.w(_TAG, f"cleanup client disconnect timed out "
                                   f"({_CLIENT_DISCONNECT_TIMEOUT_SECONDS}s): {device_mac}", mac=device_mac)
                    SdkLog.controller(_TAG, f"cleanup client disconnect timed out: {device_mac}")
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error", mac=device_mac)

            # Close data context
            if device_mac in self._data_ctxs:
                # 异常断开（远端断开/dongle 拔出/进程收尾）补写 disconnect 事件；
                # 用户主动断开已由 GForce._do_disconnect 记录
                if not disconnect_client:
                    gforce = self._gforces.get(device_mac)
                    if gforce is not None:
                        try:
                            gforce.log_bin_event("disconnect")
                        except Exception:
                            pass
                try:
                    self._data_ctxs[device_mac].close()
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error", mac=device_mac)
                self._data_ctxs.pop(device_mac, None)

            # Pop state
            self._gforces.pop(device_mac, None)
            self._raw_bufs.pop(device_mac, None)
            self._device_states[device_mac] = "Disconnected"
            # 异常断连（远端断链/半死链/看门狗/连接失败）：缓存传输可能已
            # 半死（HCI 命令有应答但不再投递事件/广播），弹出缓存强制后续
            # 扫描/重连重开传输；dongle 拔出路径已由 _handle_dongle_departed
            # 弹出，此处幂等。用户主动断开保留缓存以便快速重连。在归还 dongle
            # 之前弹出：占用中的 spec 不会被扫描借用，无竞争
            if not disconnect_client:
                spec = self._dongle_assignments.get(device_mac)
                if spec is not None:
                    await self._discard_dongle_transport(spec)
            # 归还连接占用的 dongle（异常断开同样归还，重连时重新分配）
            self._release_dongle(device_mac)

            # 正常断开后清理重连信息；异常断开保留以继续自动重连
            if disconnect_client:
                self._reconnect_info.pop(device_mac, None)

        self._publish("state_changed", device_mac=device_mac, state="Disconnected")

        # 异常断开（远端断链/半死链/dongle 拔出）：调度自动重连。
        # 与既有触发者（watchdog/活性探测/dongle 拔出）由 _schedule_reconnect
        # 内部 task 去重；用户主动断开走 disconnect_client=True 不在此调度
        if not disconnect_client and not self._should_exit:
            if device_mac in self._user_disconnect_pending:
                # 用户在掉链前已表达断连意图（dongle 挂死时停流/断开命令
                # 可能永远执行不完，链路按异常掉线收场）：按用户意图处理，
                # 清掉重连信息，不再自动重连
                self._user_disconnect_pending.discard(device_mac)
                self._reconnect_info.pop(device_mac, None)
                SdkLog.i(_TAG, f"abnormal cleanup honors pending user disconnect, "
                               f"no auto-reconnect: {device_mac}", mac=device_mac)
                SdkLog.controller(_TAG, f"abnormal cleanup honors pending user disconnect, "
                                        f"no auto-reconnect: {device_mac}")
                return
            self._schedule_reconnect(device_mac)

    def _cancel_pending_reconnect(self, device_mac: str):
        """取消挂起/进行中的自动重连（用户主动断开时调用）。

        标记 normal_disconnect、解除结果看门狗并取消已调度的重连任务；
        已进入 _do_reconnect 的任务由其入口的 normal_disconnect 复查拦截。
        幂等；读写字段与 _schedule_reconnect 相同，可从任意线程调用。
        """
        info = self._reconnect_info.get(device_mac)
        if info is None:
            return
        info["normal_disconnect"] = True
        self._disarm_reconnect_watchdog(device_mac)
        task = info.get("task")
        if task is not None:
            if not task.done():
                SdkLog.i(_TAG, f"Cancelling pending reconnect for {device_mac} (user disconnect)", mac=device_mac)
                SdkLog.controller(_TAG, f"cancel pending reconnect for {device_mac} (user disconnect)")
            try:
                task.cancel()
            except Exception:
                pass
            info["task"] = None

    def _schedule_reconnect(self, device_mac: str):
        """从 watchdog 线程调用，调度一次自动重连。"""
        if device_mac in self._user_disconnect_pending:
            # 防御：意图登记与异常清理竞态时（意图先到、清理尚未消费）同样
            # 不重连；清理路径会消费该标记
            SdkLog.i(_TAG, f"skip reconnect schedule for {device_mac}: user disconnect pending", mac=device_mac)
            SdkLog.controller(_TAG, f"skip reconnect schedule for {device_mac}: user disconnect pending")
            return
        info = self._reconnect_info.get(device_mac)
        if info is None:
            SdkLog.d(_TAG, f"skip reconnect schedule for {device_mac}: no reconnect info", mac=device_mac)
            return
        if info.get("normal_disconnect"):
            SdkLog.i(_TAG, f"skip reconnect schedule for {device_mac}: normal disconnect", mac=device_mac)
            SdkLog.controller(_TAG, f"skip reconnect schedule for {device_mac}: normal disconnect")
            return
        task = info.get("task")
        if task is not None:
            if not task.done():
                return  # 已有重连任务在进行
            # 任务已结束（异常/取消）但句柄未清：不挡后续调度
            info["task"] = None
        if info["attempts"] >= _MAX_RECONNECT_ATTEMPTS:
            SdkLog.e(_TAG, f"Max reconnect attempts reached for {device_mac}", mac=device_mac)
            SdkLog.controller(_TAG, f"max reconnect attempts ({_MAX_RECONNECT_ATTEMPTS}) reached for {device_mac}")
            # 通知应用层自动重连已放弃（onErrorCallback）；有新 dongle 到位时
            # 由 _retry_pending_reconnects 重置计数并重新调度
            self._publish("error", device_mac=device_mac,
                          message=f"Auto reconnect failed after {_MAX_RECONNECT_ATTEMPTS} attempts")
            return

        info["attempts"] += 1
        SdkLog.i(_TAG, f"Scheduling reconnect attempt {info['attempts']} for {device_mac}", mac=device_mac)
        SdkLog.controller(_TAG, f"schedule reconnect attempt {info['attempts']} for {device_mac}")

        loop = self._main_event_loop
        if loop is None or loop.is_closed():
            SdkLog.e(_TAG, f"Main event loop not available, cannot reconnect {device_mac}", mac=device_mac)
            return

        async def _delayed_reconnect():
            await asyncio.sleep(_RECONNECT_DELAY_SECONDS)
            await self._do_reconnect(device_mac)

        try:
            info["task"] = asyncio.run_coroutine_threadsafe(_delayed_reconnect(), loop)
        except Exception as e:
            SdkLog.exception(_TAG, f"Failed to schedule reconnect for {device_mac}: {e}", mac=device_mac)
            info["task"] = None

    async def _do_reconnect(self, device_mac: str):
        """执行一次自动重连。"""
        info = self._reconnect_info.get(device_mac)
        if info is None or info.get("normal_disconnect"):
            SdkLog.i(_TAG, f"reconnect aborted for {device_mac}: normal disconnect or info cleared", mac=device_mac)
            SdkLog.controller(_TAG, f"reconnect aborted for {device_mac}: normal disconnect or info cleared")
            return

        # 已经连接则无需重连
        if device_mac in self._gforces:
            info["task"] = None
            return

        attempt = info["attempts"]
        # 记录本任务句柄：finally 里只在句柄仍属于自己时清除——看门狗强制
        # 失败后会取消本任务并调度新 attempt，迟到的 finally 不能清掉新句柄
        own_task = info.get("task")
        SdkLog.i(_TAG, f"Reconnecting {device_mac}, attempt {attempt}", mac=device_mac)
        SdkLog.controller(_TAG, f"reconnecting {device_mac}, attempt {attempt}")
        # 独立结果看门狗：attempt 发起后长时间无结果（含 25s 超时未生效的
        # 异常情况——实测 attempt 发起后整条链静默消失 73 分钟）时强制判失败
        self._arm_reconnect_watchdog(device_mac, attempt)
        try:
            # _is_reconnect 标记：_do_connect_inner 据此保留 attempts 计数，
            # 而不是像用户主动连接那样重置重试预算
            await self._do_connect({**info["cmd"], "_is_reconnect": True})
        except asyncio.CancelledError:
            # 合法取消（用户断开已标 normal_disconnect / 进程退出）直接终止；
            # 其余来源（防御：连接中途链路断开等穿透性取消）按 attempt 失败
            # 处理，落到 finally 之后的续调度逻辑，重连链不因此中断
            SdkLog.w(_TAG, f"Reconnect attempt {attempt} cancelled for {device_mac}", mac=device_mac)
            SdkLog.controller(_TAG, f"reconnect attempt {attempt} cancelled for {device_mac}")
            if self._should_exit or (info is not None and info.get("normal_disconnect")):
                raise
        except BaseException as e:
            SdkLog.exception(_TAG, f"Reconnect attempt failed for {device_mac}: {e}", mac=device_mac)
        finally:
            self._disarm_reconnect_watchdog(device_mac, attempt)
            if info is not None and info.get("task") is own_task:
                info["task"] = None

        # _do_connect 失败（如 25s 超时）只发布失败结果、不抛异常：
        # 未连上且重连信息仍在（未被用户正常断开/进程未退出）则继续调度下一次
        if (device_mac not in self._gforces
                and self._reconnect_info.get(device_mac) is not None
                and not self._should_exit):
            self._schedule_reconnect(device_mac)

    def _arm_reconnect_watchdog(self, device_mac: str, attempt: int):
        """为一次重连 attempt 武装独立结果看门狗（threading.Timer）。

        用独立线程而不是主事件循环定时器：主循环自身异常时（定时器不触发）
        看门狗仍能到达日志与强制失败路径。
        """
        self._disarm_reconnect_watchdog(device_mac)
        info = self._reconnect_info.get(device_mac)
        if info is None:
            return
        timer = threading.Timer(
            _RECONNECT_WATCHDOG_SECONDS,
            self._reconnect_watchdog_fired,
            args=(device_mac, attempt),
        )
        timer.daemon = True
        timer._reconnect_attempt = attempt
        info["watchdog"] = timer
        timer.start()

    def _disarm_reconnect_watchdog(self, device_mac: str, attempt: int = None):
        """解除看门狗；attempt 不匹配时不解除（旧任务的 finally 不误杀新看门狗）。"""
        info = self._reconnect_info.get(device_mac)
        if info is None:
            return
        timer = info.get("watchdog")
        if timer is None:
            return
        if attempt is not None and getattr(timer, "_reconnect_attempt", None) != attempt:
            return
        info.pop("watchdog", None)
        try:
            timer.cancel()
        except Exception:
            pass

    def _reconnect_watchdog_fired(self, device_mac: str, attempt: int):
        """看门狗线程回调：attempt 超时仍无结果，强制判失败并续调度。"""
        info = self._reconnect_info.get(device_mac)
        if info is None or info.get("normal_disconnect") or self._should_exit:
            return
        if device_mac in self._gforces:
            return  # 已连上
        if info["attempts"] != attempt:
            return  # 已有更新的 attempt 接管
        SdkLog.e(_TAG, f"Reconnect attempt {attempt} for {device_mac} produced no result in "
                       f"{_RECONNECT_WATCHDOG_SECONDS}s, forcing failure", mac=device_mac)
        SdkLog.controller(_TAG, f"reconnect attempt {attempt} for {device_mac} timed out "
                                f"(watchdog {_RECONNECT_WATCHDOG_SECONDS}s), forcing failure")
        # 使卡死协程的迟到结果失效（同 _do_connect 超时臂的 epoch 手法）
        epoch = self._connect_epochs.get(device_mac)
        if epoch is not None:
            self._connect_epochs[device_mac] = epoch + 1
        # 清掉卡死/静默消失的任务句柄，允许续调度
        task = info.get("task")
        if task is not None:
            try:
                task.cancel()
            except Exception:
                pass
            info["task"] = None
        # 占用中的 dongle 传输疑似已死：归还并逐出缓存，卡死的 inner 随之报错
        # 退出（其结果由上面的 epoch 失效守护丢弃）
        spec = self._dongle_assignments.get(device_mac)
        loop = self._main_event_loop
        if loop is not None and not loop.is_closed():
            asyncio.run_coroutine_threadsafe(
                self._reconnect_watchdog_cleanup(device_mac, spec), loop)
        else:
            self._release_dongle(device_mac)
            self._schedule_reconnect(device_mac)

    async def _reconnect_watchdog_cleanup(self, device_mac: str, spec):
        """看门狗强制失败的清理臂（主事件循环）：清理、归还 dongle、逐出传输、续调度。"""
        try:
            await self._cleanup_device(device_mac, disconnect_client=False)
        except Exception:
            SdkLog.exception(_TAG, f"watchdog cleanup failed: {device_mac}", mac=device_mac)
        self._release_dongle(device_mac)
        if spec is not None:
            asyncio.ensure_future(self._discard_dongle_transport(spec))
        self._schedule_reconnect(device_mac)

    async def _do_disconnect(self, cmd: dict):

        device_mac = cmd["device_mac"]
        # 兜底登记断连意图（正常路径 _handle_command 已登记，幂等）
        self._user_disconnect_pending.add(device_mac)
        self._cancel_pending_reconnect(device_mac)
        state = self._device_states.get(device_mac, "Disconnected")

        if state not in ("Connected", "Ready"):
            # 设备已断开仍收到用户断开命令：多为设备刚异常掉链、主进程状态
            # 同步后用户才点到断开。必须取消可能已调度的链路层自动重连并
            # 清掉重连信息，否则用户明确断开后设备又会被自动连回
            SdkLog.i(_TAG, f"disconnect cmd while already {state}: {device_mac}", mac=device_mac)
            SdkLog.controller(_TAG, f"disconnect cmd while already {state}: {device_mac}")
            self._cancel_pending_reconnect(device_mac)
            self._reconnect_info.pop(device_mac, None)
            self._user_disconnect_pending.discard(device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=True,
                result=True,
            )
            return

        SdkLog.d(_TAG, f"user disconnect: {device_mac}", mac=device_mac)
        self._device_states[device_mac] = "Disconnecting"
        self._publish("state_changed", device_mac=device_mac, state="Disconnecting")

        await self._cleanup_device(device_mac, disconnect_client=True)
        self._user_disconnect_pending.discard(device_mac)
        SdkLog.d(_TAG, f"user disconnect done: {device_mac}", mac=device_mac)

        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=True,
        )

    async def _do_init(self, cmd: dict):

        device_mac = cmd["device_mac"]
        package_sample_count = cmd.get("package_sample_count", 16)
        power_refresh_interval = cmd.get("power_refresh_interval", 0)

        if device_mac not in self._data_ctxs:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Not connected",
            )
            return

        ctx = self._data_ctxs[device_mac]

        try:
            success = await ctx.init(package_sample_count)
        except Exception as e:
            SdkLog.exception(_TAG, f"_do_init failed: {device_mac}", mac=device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )
            return

        if success:
            self._power_intervals[device_mac] = power_refresh_interval

            # Get initial battery
            if device_mac in self._gforces:
                try:
                    power = await self._gforces[device_mac].get_battery_level()
                    self._publish("power_changed", device_mac=device_mac, power=power)
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error", mac=device_mac)

            # Start battery polling task (runs in event_loop)
            if power_refresh_interval > 0:
                self._battery_tasks[device_mac] = asyncio.create_task(
                    self._battery_loop(device_mac)
                )

        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=success,
            result=success,
            device_info=ctx._device_info if success else None,
        )

    async def _battery_loop(self, device_mac: str):

        while True:
            interval = self._power_intervals.get(device_mac, 0)
            if interval <= 0 or device_mac not in self._gforces:
                break
            await asyncio.sleep(interval / 1000)
            if getattr(self, "_multi_notification_active", 0) > 0:
                # multi start/stop 期间挂起本轮电量轮询，下一轮恢复正常
                continue
            ctx = self._data_ctxs.get(device_mac)
            try:
                power = await self._gforces[device_mac].get_battery_level()
                self._publish("power_changed", device_mac=device_mac, power=power)
                if ctx is not None:
                    # 读取失败（-1）计入命令通道失败，连续失败判定命令通道半死
                    ctx.note_command_channel_result(power >= 0)
            except Exception:
                # 单次刷新失败不中断循环，否则一次超时/丢包后就再也无法刷新电量
                SdkLog.exception(_TAG, f"Battery refresh failed: {device_mac}", mac=device_mac)
                if ctx is not None:
                    ctx.note_command_channel_result(False)

    async def _device_data_loop(self, device_mac: str):

        from sensor.sensor_data import SensorData

        ctx = self._data_ctxs.get(device_mac)
        if ctx is None:
            return

        local_buf = queue.Queue(maxsize=sensor_utils.BLEAK_RESULT_QUEUE_MAXSIZE)

        def on_data(sensor_data_list: list):
            # 统计已发布的 sensor_data 消息数：全速回放时主进程消费滞后，
            # replayBinFile 据此等待本次回放的消息全部到达后再恢复传输标志
            if ctx is not None:
                ctx._published_sensor_data_msgs += 1
            self._publish("sensor_data", device_mac=device_mac, data=sensor_data_list)

        def on_error(message: str):
            self._publish("error", device_mac=device_mac, message=message)

        # 持续运行，异常后自动重启，避免单包错误导致整个解析线程退出
        while not self._should_exit and device_mac in self._data_ctxs:
            try:
                if ctx.isUniversalStream:
                    await ctx._processUniversalData(local_buf, on_data, on_error)
                else:
                    await ctx._process_data(local_buf, on_data, on_error)
                # 正常返回说明 ctx 已关闭或停止
                break
            except asyncio.CancelledError:
                SdkLog.i(_TAG, f"Data loop cancelled: {device_mac}", mac=device_mac)
                break
            except Exception as e:
                SdkLog.exception(_TAG, f"Error in data loop, restarting: {device_mac}", mac=device_mac)
                try:
                    await asyncio.sleep(0.5)
                except asyncio.CancelledError:
                    break

    async def _replay_pace_sleep(self, seconds: float, ctrl: dict):
        """按回放节奏等待；暂停期间冻结剩余时间，停止/退出时立即返回。"""
        remaining = seconds
        while remaining > 0:
            if self._should_exit or ctrl["stop"]:
                return
            if ctrl["paused"]:
                await asyncio.sleep(0.1)
                continue
            step = min(remaining, 0.1)
            await asyncio.sleep(step)
            remaining -= step

    async def _do_replay_bin(self, cmd: dict):
        """回放 bin 文件：把原始数据包重新注入解析管线，结果走正常 sensor_data 通路。

        - 若 device_mac 已有解析上下文（profile 已连接/初始化），复用它；
          否则按 bin 文件中的配置记录创建离线上下文。
        - 配置记录会在回放过程中按顺序应用（与录制时的 init 时序一致）。
        """
        device_mac = cmd["device_mac"]
        path = cmd.get("path", "")
        realtime = bool(cmd.get("realtime", True))
        cmd_id = cmd.get("cmd_id")

        ctx = self._data_ctxs.get(device_mac)
        owns_ctx = False
        prev_transfering = False

        if ctx is not None and ctx.gForce is not None and ctx.isDataTransfering:
            self._publish(
                "command_result",
                cmd_id=cmd_id,
                device_mac=device_mac,
                success=False,
                result="Error: device is streaming, stop data notification before replay",
            )
            return

        if ctx is None:
            try:
                from sensor.sensor_data_context import SensorProfileDataCtx

                ctx = SensorProfileDataCtx(
                    None,
                    device_mac,
                    queue.Queue(maxsize=sensor_utils.BLEAK_DATA_QUEUE_MAXSIZE),
                )
            except Exception as e:
                SdkLog.exception(_TAG, f"Create replay context failed: {device_mac}", mac=device_mac)
                self._publish(
                    "command_result",
                    cmd_id=cmd_id,
                    device_mac=device_mac,
                    success=False,
                    result="Error: " + str(e),
                )
                return
            self._data_ctxs[device_mac] = ctx
            owns_ctx = True
            # 回放中配置记录切换可能改变采样率（load_replay_config 检测并回调），
            # 与连接上下文一样上报主进程刷新 DeviceInfo
            ctx.on_device_info_changed = lambda fields: self._publish(
                "device_info_update", device_mac=device_mac, fields=fields)
        else:
            prev_transfering = ctx._is_data_transfering
            ctx._is_data_transfering = True

        processed = 0
        skipped = 0
        config_seen = ctx.hasInit()
        data_loop_started = not owns_ctx  # 已有 ctx 的解析循环已在运行
        published_baseline = ctx._published_sensor_data_msgs
        last_ts = None
        error = None
        stopped = False
        pending_first_packet_ts = False  # 起流标记后等待首个数据包以还原 delay
        ctrl = {"paused": False, "stop": False}
        self._replay_controls[device_mac] = ctrl
        try:
            SdkLog.i(_TAG, f"Replay bin start: {device_mac} file={path} realtime={realtime}", mac=device_mac)
            for record_type, ts, payload in iter_bin_records(path):
                if self._should_exit or ctrl["stop"]:
                    stopped = ctrl["stop"]
                    break
                # 暂停：保持当前位置等待恢复或停止
                while ctrl["paused"] and not self._should_exit and not ctrl["stop"]:
                    await asyncio.sleep(0.1)
                if self._should_exit or ctrl["stop"]:
                    stopped = ctrl["stop"]
                    break
                if record_type == BIN_RECORD_CONFIG:
                    try:
                        config = decode_bin_config(payload)
                        if config is None:
                            raise ValueError("invalid bin config record")
                        if data_loop_started:
                            # 先等数据循环把配置记录之前的原始包消费完：
                            # 配置记录意味着流重启（设备包序号归零），旧包若在
                            # 新状态下解析会被误判成巨额丢包
                            drain_deadline = time.monotonic() + 10.0
                            while (not ctx._rawDataBuffer.empty()
                                   and time.monotonic() < drain_deadline
                                   and not self._should_exit and not ctrl["stop"]):
                                await asyncio.sleep(0.01)
                            await asyncio.sleep(0.05)
                        ctx.load_replay_config(config)
                        config_seen = True
                        # 每条 CONFIG 都用其记录 ts 预置近似锚点（startTimeSec
                        # 与 startTimeStamp 同源，后者取低 32 位）：回放里流对象
                        # 按段重建，预置随之按段生效；该段随后的起流事件/非零
                        # 0x4F 记录到来时两者被精确重盖
                        ctx._stream_start_ts_sec = ts / 1000.0
                        ctx._stream_start_ts_ms = ts & 0xFFFFFFFF
                        SdkLog.i(_TAG, f"Replay config applied: {device_mac}", mac=device_mac)
                    except Exception:
                        SdkLog.exception(_TAG, f"Replay config record failed: {device_mac}", mac=device_mac)
                    if owns_ctx and not data_loop_started and config_seen:
                        # 等到配置就绪后再启动解析循环，确保流模式（普通/通用）正确
                        data_loop_started = True
                        asyncio.create_task(self._device_data_loop(device_mac))
                    continue
                if record_type != BIN_RECORD_DATA:
                    # 起流时刻还原：OYM 起流（CCCD 写）后记录 stream_start 事件；
                    # RFSTAR 起流是 SET_DATA_NOTIF_SWITCH CMD 写（非零订阅）。
                    # 记录 ts 即起流写发送时刻（bumble 层打点，见 gforce
                    # _backend_write_wall_ms），首个数据包 ts 之差还原 delay
                    start_ts = None
                    if record_type == BIN_RECORD_EVENT:
                        if payload == b"stream_start":
                            start_ts = ts
                    elif record_type == BIN_RECORD_CMD_SEND:
                        from sensor.gforce import Command
                        if (len(payload) >= 5
                                and payload[0] == Command.SET_DATA_NOTIF_SWITCH
                                and int.from_bytes(payload[1:5], "little") != 0):
                            start_ts = ts
                    if start_ts is not None:
                        ctx._stream_start_ts_ms = start_ts & 0xFFFFFFFF
                        # LSL 绝对时间戳锚点：bin 记录 ts 是完整墙钟毫秒
                        ctx._stream_start_ts_sec = start_ts / 1000.0
                        ctx._stream_first_delay_ms = 0
                        pending_first_packet_ts = True
                    # 头部等其他记录类型不参与回放
                    continue
                if not config_seen:
                    # 配置记录之前的数据无法解析，跳过
                    skipped += 1
                    continue
                if pending_first_packet_ts:
                    # 起流后首个数据包：还原首包 delay（32 位回绕安全）
                    pending_first_packet_ts = False
                    ctx._stream_first_delay_ms = (
                        (ts & 0xFFFFFFFF) - ctx._stream_start_ts_ms) & 0xFFFFFFFF
                if realtime and last_ts is not None and ts > last_ts:
                    await self._replay_pace_sleep(min((ts - last_ts) / 1000.0, 5.0), ctrl)
                    if self._should_exit or ctrl["stop"]:
                        stopped = ctrl["stop"]
                        break
                last_ts = ts
                while not self._should_exit and not ctrl["stop"]:
                    try:
                        ctx._rawDataBuffer.put_nowait(payload)
                        break
                    except queue.Full:
                        await asyncio.sleep(0.01)
                if self._should_exit or ctrl["stop"]:
                    stopped = ctrl["stop"]
                    break
                processed += 1

            # 等待解析与发布完成：先等原始队列清空，再留一段余量给 FlatBuffers 队列与发布
            drain_deadline = time.time() + 30
            while not ctx._rawDataBuffer.empty() and time.time() < drain_deadline and not ctrl["stop"]:
                await asyncio.sleep(0.05)
            if not ctrl["stop"]:
                await asyncio.sleep(0.5)

            if not config_seen:
                error = "Error: no config record found in bin file"
                SdkLog.e(_TAG, f"Replay failed: {device_mac} {error}", mac=device_mac)
            if error:
                result = error
            elif stopped:
                result = f"OK: replay stopped after {processed} packets (skipped {skipped})"
            else:
                result = f"OK: replayed {processed} packets (skipped {skipped})"
        except Exception as e:
            SdkLog.exception(_TAG, f"Replay failed: {device_mac}", mac=device_mac)
            error = "Error: " + str(e)
            result = error
        finally:
            self._replay_controls.pop(device_mac, None)
            if owns_ctx:
                try:
                    ctx.close()
                except Exception:
                    pass
                if self._data_ctxs.get(device_mac) is ctx:
                    del self._data_ctxs[device_mac]
            else:
                ctx._is_data_transfering = prev_transfering

        SdkLog.i(_TAG, f"Replay bin finished: {device_mac} {result}", mac=device_mac)
        self._publish(
            "command_result",
            cmd_id=cmd_id,
            device_mac=device_mac,
            success=(error is None),
            result=result,
            published_data_msgs=ctx._published_sensor_data_msgs - published_baseline,
        )

    async def _do_multi_start_notification(self, cmd: dict):
        # multi start/stop 期间挂起电量轮询（计数器，起/停可嵌套）：
        # 避免 GET_BATTERY 写挤进门闩窗口或与起/停流写争抢命令通道
        self._multi_notification_active = getattr(self, "_multi_notification_active", 0) + 1
        try:
            await self._do_multi_start_notification_inner(cmd)
        finally:
            self._multi_notification_active -= 1

    async def _do_multi_start_notification_inner(self, cmd: dict):
        """多设备同步起流：并发投到各设备 loop，写命令经 SyncWriteGate 统一下发。

        restart 语义：已在流的设备先经停流门闩同步停流，再与空闲设备一起
        经起流门闩同步起流——消除用户先前单独 start 造成的起流时刻不一致。
        """
        cmd_id = cmd.get("cmd_id")
        device_macs = cmd.get("device_macs") or []
        SdkLog.controller(_TAG, f"multi_start_notification: {device_macs}")

        results = {}
        targets = []
        streaming = []
        for device_mac in device_macs:
            ctx = self._data_ctxs.get(device_mac)
            loop = self._event_loops.get(device_mac)
            if ctx is None or not ctx.hasInit():
                results[device_mac] = "Not initialized"
            elif loop is None or loop.is_closed():
                results[device_mac] = "Device not connected"
            elif ctx._is_data_transfering:
                # 已在流：先停后起（restart），与空闲设备一起入起流门闩
                streaming.append((device_mac, ctx, loop))
            else:
                targets.append((device_mac, ctx, loop))

        if streaming:
            SdkLog.controller(_TAG, f"multi_start_notification restart (stop first): "
                                    f"{[m for m, _, _ in streaming]}")
            stop_gate = bumble_dongle.SyncWriteGate(parties=len(streaming))
            stop_futures = {}
            for device_mac, ctx, loop in streaming:
                stop_futures[device_mac] = asyncio.run_coroutine_threadsafe(
                    ctx.stop_streaming(sync_gate=stop_gate), loop
                )
            for device_mac, future in stop_futures.items():
                ctx = self._data_ctxs.get(device_mac)
                loop = self._event_loops.get(device_mac)
                try:
                    await asyncio.wait_for(asyncio.wrap_future(future), timeout=25)
                    targets.append((device_mac, ctx, loop))
                except asyncio.TimeoutError:
                    SdkLog.e(_TAG, f"multi_start_notification restart stop timeout: {device_mac}",
                             mac=device_mac)
                    results[device_mac] = "Timeout"
                except Exception as e:
                    SdkLog.exception(_TAG, f"multi_start_notification restart stop failed: "
                                           f"{device_mac}", mac=device_mac)
                    results[device_mac] = str(e)

        if targets:
            max_dispersion_ms = cmd.get("max_delay_dispersion_ms",
                                        _MULTI_START_DELAY_DISPERSION_MAX_MS)
            max_attempts = cmd.get("max_attempts", _MULTI_START_MAX_ATTEMPTS)
            try:
                max_attempts = max(1, int(max_attempts))
            except (TypeError, ValueError):
                max_attempts = _MULTI_START_MAX_ATTEMPTS
            results.update(await self._multi_start_aligned(targets, max_dispersion_ms, max_attempts))

        success = all(v is True for v in results.values())
        SdkLog.controller(_TAG, f"multi_start_notification done: {results}")
        self._publish(
            "command_result",
            cmd_id=cmd_id,
            success=success,
            result=results,
        )

    async def _multi_start_once(self, targets):
        """一轮同步起流：并发投到各设备 loop，写命令经 SyncWriteGate 统一下发。
        返回 {mac: True 或错误串}。"""
        results = {}
        gate = bumble_dongle.SyncWriteGate(parties=len(targets))
        futures = {}
        for device_mac, ctx, loop in targets:
            futures[device_mac] = asyncio.run_coroutine_threadsafe(
                ctx.start_streaming(sync_gate=gate), loop
            )
            SdkLog.d(_TAG, f"multi_start_once dispatched to device loop: {device_mac}",
                     mac=device_mac)
        for device_mac, future in futures.items():
            try:
                await asyncio.wait_for(asyncio.wrap_future(future), timeout=25)
                results[device_mac] = True
            except asyncio.TimeoutError:
                SdkLog.e(_TAG, f"multi_start_once timeout: {device_mac}", mac=device_mac)
                results[device_mac] = "Timeout"
            except Exception as e:
                SdkLog.exception(_TAG, f"multi_start_once failed: {device_mac}",
                                 mac=device_mac)
                results[device_mac] = str(e)
        return results

    async def _multi_stop_once(self, targets):
        """一轮同步停流（结构同 _multi_start_once），返回 {mac: True 或错误串}。"""
        results = {}
        gate = bumble_dongle.SyncWriteGate(parties=len(targets))
        futures = {}
        for device_mac, ctx, loop in targets:
            futures[device_mac] = asyncio.run_coroutine_threadsafe(
                ctx.stop_streaming(sync_gate=gate), loop
            )
            SdkLog.d(_TAG, f"multi_stop_once dispatched to device loop: {device_mac}",
                     mac=device_mac)
        for device_mac, future in futures.items():
            try:
                await asyncio.wait_for(asyncio.wrap_future(future), timeout=25)
                results[device_mac] = True
            except asyncio.TimeoutError:
                SdkLog.e(_TAG, f"multi_stop_once timeout: {device_mac}", mac=device_mac)
                results[device_mac] = "Timeout"
            except Exception as e:
                SdkLog.exception(_TAG, f"multi_stop_once failed: {device_mac}",
                                 mac=device_mac)
                results[device_mac] = str(e)
        return results

    async def _await_first_packet_delays(self, macs, timeout):
        """等待各设备起流后的首包 delay 上报（ctx._stream_first_delay_ms，
        起流写发送时刻→首个原始数据包到达，32 位毫秒）；返回
        {mac: delay_ms}，超时未出首包的设备不在结果中。"""
        delays = {}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and len(delays) < len(macs):
            for device_mac in macs:
                if device_mac in delays:
                    continue
                ctx = self._data_ctxs.get(device_mac)
                delay = getattr(ctx, "_stream_first_delay_ms", 0) if ctx is not None else 0
                if delay:
                    delays[device_mac] = delay
            if len(delays) < len(macs):
                await asyncio.sleep(0.01)
        return delays

    async def _multi_start_aligned(self, targets, max_dispersion_ms=_MULTI_START_DELAY_DISPERSION_MAX_MS,
                                   max_attempts=_MULTI_START_MAX_ATTEMPTS):
        """带首包 delay 离散度校验的多设备同步起流。

        每轮起流后等待全部设备的首包 delay，离散值（max-min）超过
        max_dispersion_ms（默认 _MULTI_START_DELAY_DISPERSION_MAX_MS；
        传入 <0 时不做离散度校验，只要求各设备起流成功并出首包）、
        有设备超时未出首包或起流
        失败时，整体停流后重来，最多 max_attempts 次（默认
        _MULTI_START_MAX_ATTEMPTS）；仍不
        达标则停掉最后一轮的数据流并返回失败（保持"失败=不在流"的一致
        状态，避免设备静默在流空耗电量）。
        """
        last_results = {}
        last_reason = "unknown"
        excluded = {}
        for attempt in range(1, max_attempts + 1):
            start_results = await self._multi_start_once(targets)
            last_results = start_results
            fail_note = {m: r for m, r in start_results.items() if r is not True}
            if fail_note:
                last_reason = f"start failed: {fail_note}"
            else:
                ok_macs = [m for m, r in start_results.items() if r is True]
                delays = await self._await_first_packet_delays(
                    ok_macs, _MULTI_START_FIRST_PACKET_TIMEOUT_SECONDS)
                missing = [m for m in ok_macs if m not in delays]
                if missing:
                    last_reason = (f"no first packet within "
                                   f"{_MULTI_START_FIRST_PACKET_TIMEOUT_SECONDS}s: {missing}")
                else:
                    values = list(delays.values())
                    dispersion = (max(values) - min(values)) if len(values) > 1 else 0
                    SdkLog.controller(
                        _TAG,
                        f"multi_start attempt {attempt}: delays={delays}, "
                        f"dispersion={dispersion}ms")
                    if max_dispersion_ms < 0 or dispersion <= max_dispersion_ms:
                        return {m: True for m in ok_macs}
                    last_reason = (f"first packet delay dispersion {dispersion}ms "
                                   f"> {max_dispersion_ms}ms")
            if attempt < max_attempts:
                SdkLog.controller(_TAG,
                                  f"multi_start attempt {attempt} failed, restarting: {last_reason}")
                stop_results = await self._multi_stop_once(targets)
                # 停流失败的设备无法加入下一轮门闩（其 ctx 认为仍在流会直通），
                # 从后续尝试中排除并记为失败
                for m, r in stop_results.items():
                    if r is not True:
                        excluded[m] = f"stop failed during retry: {r}"
                        SdkLog.e(_TAG, f"multi_start retry stop failed: {m}: {r}", mac=m)
                targets = [t for t in targets if t[0] not in excluded]
                if not targets:
                    break

        # 最终失败：停掉最后一轮的数据流，保持"失败=不在流"的一致状态
        if targets:
            await self._multi_stop_once(targets)
        results = dict(excluded)
        for m, r in last_results.items():
            if m in excluded:
                continue
            results[m] = last_reason if r is True else r
        SdkLog.controller(
            _TAG, f"multi_start failed after {max_attempts} attempts: {last_reason}")
        return results

    async def _do_multi_stop_notification(self, cmd: dict):
        # 同 _do_multi_start_notification：multi 期间挂起电量轮询
        self._multi_notification_active = getattr(self, "_multi_notification_active", 0) + 1
        try:
            await self._do_multi_stop_notification_inner(cmd)
        finally:
            self._multi_notification_active -= 1

    async def _do_multi_stop_notification_inner(self, cmd: dict):
        """多设备同步停流：并发投到各设备 loop，停流写命令经 SyncWriteGate 统一下发。"""
        cmd_id = cmd.get("cmd_id")
        device_macs = cmd.get("device_macs") or []
        SdkLog.controller(_TAG, f"multi_stop_notification: {device_macs}")

        results = {}
        targets = []
        for device_mac in device_macs:
            ctx = self._data_ctxs.get(device_mac)
            loop = self._event_loops.get(device_mac)
            if ctx is None or not ctx.hasInit():
                results[device_mac] = "Not initialized"
            elif loop is None or loop.is_closed():
                results[device_mac] = "Device not connected"
            elif not ctx._is_data_transfering:
                # 未在流的设备无需再停流，也不计入门闩参与数
                results[device_mac] = True
            else:
                targets.append((device_mac, ctx, loop))

        if targets:
            results.update(await self._multi_stop_once(targets))

        success = all(v is True for v in results.values())
        SdkLog.controller(_TAG, f"multi_stop_notification done: {results}")
        self._publish(
            "command_result",
            cmd_id=cmd_id,
            success=success,
            result=results,
        )

    async def _do_start_notification(self, cmd: dict):

        device_mac = cmd["device_mac"]
        if device_mac not in self._data_ctxs:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Not initialized",
            )
            return

        ctx = self._data_ctxs[device_mac]
        if not ctx.hasInit():
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Not initialized",
            )
            return

        try:
            result = await ctx.start_streaming()
        except Exception as e:
            SdkLog.exception(_TAG, f"_do_start_notification failed: {device_mac}", mac=device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )
            return

        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=result,
        )

    async def _do_stop_notification(self, cmd: dict):

        device_mac = cmd["device_mac"]
        if device_mac not in self._data_ctxs:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Not initialized",
            )
            return

        ctx = self._data_ctxs[device_mac]
        try:
            SdkLog.d(_TAG, f"stop_notification: stop_streaming start: {device_mac}", mac=device_mac)
            # 调用侧超时兜底：dongle 挂死时停流协程投递到 gforce loop 后
            # 永远不会执行，其内部超时不会生效（实测 OB5200 + Actions dongle）
            result = await asyncio.wait_for(
                ctx.stop_streaming(), timeout=_STOP_STREAM_TIMEOUT_SECONDS + 2)
            SdkLog.d(_TAG, f"stop_notification: stop_streaming done: {device_mac}", mac=device_mac)
        except asyncio.TimeoutError:
            SdkLog.w(_TAG, f"stop_notification: stop_streaming timed out: {device_mac}", mac=device_mac)
            SdkLog.controller(_TAG, f"stop_notification: stop_streaming timed out: {device_mac}")
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Timeout",
            )
            return
        except Exception as e:
            SdkLog.exception(_TAG, f"_do_stop_notification failed: {device_mac}", mac=device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )
            return

        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=result,
        )

    async def _do_get_battery(self, cmd: dict):

        device_mac = cmd["device_mac"]
        if device_mac not in self._gforces:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=-1,
            )
            return

        try:
            power = await self._gforces[device_mac].get_battery_level()
        except Exception:
            SdkLog.exception(_TAG, f"_do_get_battery failed: {device_mac}", mac=device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=-1,
            )
            return

        self._publish("power_changed", device_mac=device_mac, power=power)
        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=power,
        )

    async def _do_set_neucir_app_control(self, cmd: dict):

        device_mac = cmd["device_mac"]
        if device_mac not in self._gforces:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Not connected",
            )
            return

        try:
            ret = await self._gforces[device_mac].set_neucir_app_control(
                cmd.get("open", False),
                cmd.get("close", False),
                cmd.get("stop", False),
            )
        except Exception as e:
            SdkLog.exception(_TAG, f"_do_set_neucir_app_control failed: {device_mac}", mac=device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )
            return

        result = "OK" if ret else "Error: Unknown error"
        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=result,
        )

    async def _do_set_neucir_mode(self, cmd: dict):

        device_mac = cmd["device_mac"]
        if device_mac not in self._gforces:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Not connected",
            )
            return

        try:
            ret = await self._gforces[device_mac].set_neucir_mode(cmd.get("mode", 0))
        except Exception as e:
            SdkLog.exception(_TAG, f"_do_set_neucir_mode failed: {device_mac}", mac=device_mac)
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )
            return

        result = "OK" if ret else "Error: Unknown error"
        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=result,
        )

    async def _do_get_param(self, cmd: dict):

        device_mac = cmd["device_mac"]
        key = cmd.get("key", "")

        if device_mac not in self._data_ctxs:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Error: Please connect first",
            )
            return

        ctx = self._data_ctxs[device_mac]
        if not ctx.hasInit():
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Error: Not initialized",
            )
            return

        result = "Error: Not supported"

        if key == "FILTER":
            sorted_keys = sorted(ctx.filter_map.keys())
            result = "|".join(f"{k}|{ctx.filter_map[k]}" for k in sorted_keys)

        if key == "NTF":
            sorted_keys = sorted(ctx.notify_map.keys())
            result = "|".join(f"{k}|{ctx.notify_map[k]}" for k in sorted_keys)

        if key == "NTF_IMU":
            imu_sub_keys = ["NTF_GFORCE_ACC", "NTF_GFORCE_GYRO", "NTF_GFORCE_QUAT", "NTF_GFORCE_EULER"]
            result = "ON" if all(ctx.notify_map.get(k) == "ON" for k in imu_sub_keys) else "OFF"
        elif key in ctx.notify_map:
            result = ctx.notify_map[key]

        if key == "DEBUG_LOG_PATH":
            result = SdkLog.get_profile_log_path(device_mac) or ""

        if key == "DEBUG_BLE_DATA_PATH":
            result = ctx._data_log_path if ctx._data_log_enabled else ""

        if key == "EEG_SAMPLE_RATE":
            result = str(ctx.get_eeg_sample_rate())

        if key == "EEG_SAMPLE_RATE_LIST":
            options = ctx.get_eeg_sample_rate_options()
            if options:
                result = "|".join(str(r) for r in options)

        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=(not result.startswith("Error")),
            result=result,
        )

    async def _do_set_param(self, cmd: dict):

        device_mac = cmd["device_mac"]
        key = cmd.get("key", "")
        value = cmd.get("value", "")

        if device_mac not in self._data_ctxs:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Error: Please connect first",
            )
            return

        ctx = self._data_ctxs[device_mac]
        if not ctx.hasInit():
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Error: Not initialized",
            )
            return

        result = "Error: Not supported"
        needs_restart = False
        oym_needs_subscribe = False
        was_streaming = ctx.isDataTransfering

        ntf_keys = [
            "NTF_GEST", "NTF_EMG", "NTF_EEG", "NTF_ECG", "NTF_IMU", "NTF_BRTH", "NTF_IMPEDANCE",
            "NTF_MAG_ANGLE", "NTF_PPG", "NTF_PPG_RAW", "NTF_SPO2",
            "NTF_GFORCE_EULER", "NTF_GFORCE_QUAT",
            "NTF_GFORCE_ACC", "NTF_GFORCE_GYRO",
        ]
        if key in ntf_keys:
            if value in ["ON", "OFF"]:
                # 统一 PPG 开关别名
                map_key = key
                if key == "NTF_PPG_RAW":
                    map_key = "NTF_PPG"

                # IMU 总开关同时控制 ACC/GYRO/QUAT/EULER
                imu_sub_keys = ["NTF_GFORCE_ACC", "NTF_GFORCE_GYRO", "NTF_GFORCE_QUAT", "NTF_GFORCE_EULER"]
                if map_key == "NTF_IMU":
                    for sub in imu_sub_keys:
                        ctx.notify_map[sub] = value

                # 老版本 EMG 设备上 Gesture 与 EMG 互斥，自动切换
                if not ctx.isNewEMG:
                    if map_key == "NTF_GEST" and value == "ON" and ctx.notify_map.get("NTF_EMG") == "ON":
                        ctx.notify_map["NTF_EMG"] = "OFF"
                        ctx.notify_map[map_key] = value
                        result = "OK"
                    elif map_key == "NTF_EMG" and value == "ON" and ctx.notify_map.get("NTF_GEST") == "ON":
                        ctx.notify_map["NTF_GEST"] = "OFF"
                        ctx.notify_map[map_key] = value
                        result = "OK"
                    else:
                        ctx.notify_map[map_key] = value
                        result = "OK"
                else:
                    # 新 EMG：Gesture 依赖 EMG 数据，关闭 EMG 时同步关闭 Gesture；打开 Gesture 时自动打开 EMG
                    if map_key == "NTF_EMG" and value == "OFF":
                        ctx.notify_map["NTF_GEST"] = "OFF"
                    elif map_key == "NTF_GEST" and value == "ON" and ctx.notify_map.get("NTF_EMG") != "ON":
                        ctx.notify_map["NTF_EMG"] = "ON"
                    ctx.notify_map[map_key] = value
                    result = "OK"

                # 单个 IMU 子开关变化时，同步更新 NTF_IMU 总开关的聚合状态
                if map_key in imu_sub_keys:
                    ctx.notify_map["NTF_IMU"] = "ON" if all(ctx.notify_map.get(k) == "ON" for k in imu_sub_keys) else "OFF"

                if result == "OK" and ctx.hasInit():
                    ctx._buildNotifyDataFlag()

                    # 新 EMG 设备通过 function switch 控制 EMG/Gesture 输出，bit0=gesture, bit1=emg
                    if ctx.isNewEMG and map_key in ("NTF_EMG", "NTF_GEST"):
                        emg_bit = 1 if ctx.notify_map.get("NTF_EMG") == "ON" else 0
                        gest_bit = 1 if (emg_bit and ctx.notify_map.get("NTF_GEST") == "ON") else 0
                        func_switch = (emg_bit << 1) | gest_bit
                        try:
                            await ctx.gForce.set_function_switch(func_switch)
                            await asyncio.sleep(0.5)
                        except Exception as e:
                            SdkLog.exception(_TAG, f"_do_set_param set_function_switch failed: {device_mac}", mac=device_mac)
                            result = "ERROR: set_function_switch fail: " + str(e)
                        if ctx.getChipType() == BLEChipType.OYM:
                            oym_needs_subscribe = True
                    elif was_streaming:
                        needs_restart = True
                        if ctx.getChipType() == BLEChipType.OYM:
                            oym_needs_subscribe = True
                    elif ctx.getChipType() == BLEChipType.OYM:
                        # 未在传输时，直接更新 OYM 订阅掩码
                        try:
                            await ctx.gForce.set_subscription(ctx.notifyDataFlag)
                        except Exception as e:
                            SdkLog.exception(_TAG, f"_do_set_param set_subscription failed: {device_mac}", mac=device_mac)
                            result = "ERROR: set_subscription fail: " + str(e)

        if key in ["FILTER_50HZ", "FILTER_60HZ", "FILTER_HPF", "FILTER_LPF"]:
            if value in ["ON", "OFF"]:
                try:
                    result = await ctx.setFilter(key, value)
                    # 滤波开关由固件命令直接生效，不重启数据流
                except Exception as e:
                    SdkLog.exception(_TAG, f"_do_set_param setFilter failed: {device_mac} {key}={value}", mac=device_mac)
                    result = "ERROR: " + str(e)

        if key == "EEG_SAMPLE_RATE":
            # EEG/ECG 绑定采样率：先按 cap 解码出的可选列表校验，再两者一起写
            try:
                rate = int(str(value).strip())
            except (TypeError, ValueError):
                result = "Error: invalid sample rate: " + str(value)
            else:
                options = ctx.get_eeg_sample_rate_options()
                if not options:
                    result = "Error: sample rate capability unknown"
                elif rate not in options:
                    result = ("Error: unsupported sample rate %d, valid: %s"
                              % (rate, "|".join(str(r) for r in options)))
                else:
                    result = await ctx.set_eeg_sample_rate(rate)
                    if result == "OK" and ctx.hasInit() and ctx.isDataTransfering:
                        needs_restart = True

        if needs_restart or oym_needs_subscribe:
            # 停流失败不中止整个重启序列：stop_streaming 已把本地置为未传输，
            # 而设备端可能已执行（响应帧丢失——RFSTAR 命令响应走数据流 0xAA
            # 帧，关流时固件可能先停流后发响应）。中止会把 SDK 留在半状态：
            # ctx 未传输 + 设备订阅已清零，后续 setParam 在 RFSTAR 未传输分支
            # 不再下发任何命令，数据永远无法恢复。继续重新下发订阅并起流，
            # 强制设备与 SDK 状态对齐；起流也失败才报 ERROR
            if was_streaming:
                try:
                    await ctx.stop_streaming()
                except Exception as e:
                    SdkLog.w(_TAG, f"_do_set_param stop stream failed, resyncing: {device_mac}: {e}", mac=device_mac)
            try:
                if oym_needs_subscribe and ctx.getChipType() == BLEChipType.OYM:
                    await ctx.gForce.set_subscription(ctx.notifyDataFlag)
                if was_streaming:
                    await ctx.start_streaming()
            except Exception as e:
                SdkLog.exception(_TAG, f"_do_set_param restart stream failed: {device_mac}", mac=device_mac)
                result = "ERROR: restart stream fail: " + str(e)

        if key == "DEBUG_LOG_PATH":
            try:
                if value == "False" or value == "":
                    SdkLog.disable_profile_log(device_mac)
                    result = "OK"
                else:
                    # value 为主进程算好的具体路径（"True" 已在主进程展开为默认路径）
                    path = value if value != "True" else None
                    if SdkLog.enable_profile_log(device_mac, path) is not None:
                        result = "OK"
                    else:
                        result = "Error: cannot create profile log file"
            except Exception as e:
                SdkLog.exception(_TAG, f"_do_set_param DEBUG_LOG_PATH failed: {e}", mac=device_mac)
                result = "ERROR: " + str(e)

        if key == "DEBUG_BLE_DATA_PATH":
            try:
                if value == "False" or value == "":
                    ctx._data_log_enabled = False
                    ctx._data_log_path = None
                    result = "OK"
                elif value == "True" and not SdkLog.is_file_output_enabled():
                    # 默认导出（SDK 日志目录）在文件输出关闭时禁用；显式路径不受影响
                    result = "Error: SDK file output disabled"
                else:
                    if value == "True":
                        # 默认导出到 SDK 日志目录，.bin 后缀
                        name = ""
                        if ctx._device_info is not None and ctx._device_info.DeviceName:
                            name = ctx._device_info.DeviceName
                        safe = "".join(c if (c.isalnum() or c in "-_()") else "_" for c in (name or device_mac))
                        value = SdkLog.get_default_bin_path(safe)
                    # 记录导出位置，并把 bin 记录切到直写：当前 temp 段并入
                    # 导出文件后直接在导出文件上继续记录，停止推流/断连时
                    # 不再整段拷贝
                    ctx._data_log_enabled = True
                    ctx._data_log_path = value
                    result = "OK"
                if result == "OK":
                    gforce = self._gforces.get(device_mac)
                    if gforce is not None:
                        gforce.switch_bin_export(
                            ctx._data_log_path if ctx._data_log_enabled else None)
            except Exception as e:
                SdkLog.exception(_TAG, f"_do_set_param DEBUG_BLE_DATA_PATH failed: {device_mac} path={value}", mac=device_mac)
                result = "ERROR: " + str(e)

        if key == "NEUCIR_SET_MODE":
            if value in ["APP_REMOTE"]:
                try:
                    ret = await self._gforces[device_mac].set_neucir_mode(1)
                    result = "OK" if ret else "Error: Unknown error"
                except Exception as e:
                    SdkLog.exception(_TAG, f"_do_set_param NEUCIR_SET_MODE failed: {device_mac}", mac=device_mac)
                    result = "ERROR: " + str(e)

        if key == "NEUCIR_APP_CONTROL":
            if value in ["OPEN", "CLOSE", "STOP"]:
                try:
                    if value == "OPEN":
                        ret = await self._gforces[device_mac].set_neucir_app_control(True, False, False)
                    elif value == "CLOSE":
                        ret = await self._gforces[device_mac].set_neucir_app_control(False, True, False)
                    elif value == "STOP":
                        ret = await self._gforces[device_mac].set_neucir_app_control(False, False, True)
                    result = "OK" if ret else "Error: Unknown error"
                except Exception as e:
                    SdkLog.exception(_TAG, f"_do_set_param NEUCIR_APP_CONTROL failed: {device_mac} {value}", mac=device_mac)
                    result = "ERROR: " + str(e)

        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=(not result.startswith("Error")),
            result=result,
        )
