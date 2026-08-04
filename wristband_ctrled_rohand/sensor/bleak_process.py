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
from sensor.bin_recorder import BIN_RECORD_CONFIG, BIN_RECORD_DATA, decode_bin_config, iter_bin_records
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


def _extract_mac(_device: bleak.BLEDevice, adv: AdvertisementData) -> str:

    mac = None
    if adv.service_data.get(SERVICE_GUID) is not None:
        bytes_val = adv.service_data[SERVICE_GUID]
        mac = ":".join(f"{byte:02X}" for byte in bytes_val)
    elif adv.service_data.get(RFSTAR_SERVICE_GUID) is not None:
        bytes_val = adv.service_data[RFSTAR_SERVICE_GUID]
        mac = ":".join(f"{byte:02X}" for byte in reversed(bytes_val))
    return mac


def _serialize_device(_device: bleak.BLEDevice, adv: AdvertisementData) -> dict:

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
        log_path: str = None,
    ):
        super().__init__(daemon=True)
        self.cmd_queue = cmd_queue
        self.result_queue = result_queue
        self.data_queue = data_queue
        self._log_path = log_path
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

        # BLE 后端选择（None=原生 bleak；BumbleBackend=bleak_bumble），run() 时确定
        self._ble_backend = None
        # dongle 分配表：device_mac -> transport spec（bumble 后端时，一 dongle 一设备）
        self._dongle_assignments = {}
        # 最近一次上报的 USB dongle 枚举（热插拔对账基准）
        self._dongle_specs = []
        # USB dongle 热插拔监控（DongleHotplugMonitor，_run_main 中启动）
        self._dongle_monitor = None

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

    async def _publisher_task(self):
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
                    # 解码结果不丢：队列满时等待主进程消费（进程退出时放弃），
                    # 通过执行器等待避免阻塞事件循环
                    try:
                        self.data_queue.put_nowait(msg)
                    except queue.Full:
                        await asyncio.get_running_loop().run_in_executor(
                            None, self._put_data_queue_blocking, msg
                        )
                elif msg_type in self._DROPABLE_MSG_TYPES:
                    self.result_queue.put_nowait(msg)
                else:
                    # Use a short timeout for important messages so the publisher does not stall.
                    self.result_queue.put(msg, timeout=2.0)
            except queue.Empty:
                if self._should_exit:
                    break
                await asyncio.sleep(0.001)
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
                await asyncio.sleep(0.01)
            except Exception as e:
                now = time.time()
                if now - _last_queue_full_log >= 2.0:
                    _last_queue_full_log = now
                    SdkLog.e(_TAG, f"Error in publisher_task: {e}")
                await asyncio.sleep(0.001)

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
        """为连接分配一只空闲 dongle，返回 (spec, client_kwargs)；无空闲返回 (None, None)。"""
        if self._ble_backend is None:
            return None, {}
        spec = self._ble_backend.allocate()
        if spec is None:
            return None, None
        self._dongle_assignments[device_mac] = spec
        SdkLog.i(_TAG, f"Assign dongle {spec} to {device_mac}")
        return spec, self._ble_backend.client_kwargs(spec)

    def _release_dongle(self, device_mac: str):
        """归还连接占用的 dongle（幂等）。释放后对账一次枚举，解冻可重建的空闲槽。"""
        spec = self._dongle_assignments.pop(device_mac, None)
        if spec is None or self._ble_backend is None:
            return
        self._ble_backend.release(spec)
        SdkLog.i(_TAG, f"Release dongle {spec} from {device_mac}")
        departed = self._ble_backend.reconcile(self._dongle_specs)
        if departed:
            self._handle_dongle_departed(departed)

    def _on_dongle_specs_changed(self, new_specs: list):
        """热插拔监控线程回调：记录最新枚举并序列化到主事件循环处理。"""
        self._dongle_specs = list(new_specs)
        loop = self._main_event_loop
        if loop is None or loop.is_closed():
            return
        loop.call_soon_threadsafe(self._apply_dongle_specs, list(new_specs))

    def _apply_dongle_specs(self, new_specs: list):
        """在主事件循环中应用 dongle 枚举变化：运行中切换后端 / 对账 dongle 池。"""
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
            return

        old_specs = list(self._ble_backend.transport_specs)
        departed = self._ble_backend.reconcile(new_specs)
        if departed:
            self._handle_dongle_departed(departed)
        if departed or self._ble_backend.transport_specs != old_specs:
            self._publish("backend_info", backend="bumble",
                          transport=",".join(self._ble_backend.transport_specs))

    def _handle_dongle_departed(self, departed_specs: list):
        """占用中的 dongle 被拔出：主动断开受影响连接（按异常断开处理，保留自动重连）。

        断开前向主进程发 error 消息（onErrorCallback），随后 _cleanup_device
        发布 state_changed(Disconnected)。扫描借用的 dongle 不在分配表中，
        仅清理 departed 记录，扫描自身经现有错误路径/下次扫描自愈。
        """
        for spec in departed_specs:
            macs = [mac for mac, s in self._dongle_assignments.items() if s == spec]
            for mac in macs:
                SdkLog.w(_TAG, f"Dongle {spec} unplugged, disconnecting {mac}")
                self._dongle_assignments.pop(mac, None)
                self._publish("error", device_mac=mac,
                              message=f"USB BLE dongle {spec} unplugged")
                loop = self._event_loops.get(mac)
                if loop is not None and not loop.is_closed():
                    try:
                        asyncio.run_coroutine_threadsafe(
                            self._cleanup_device(mac, disconnect_client=False), loop)
                    except Exception as e:
                        SdkLog.exception(_TAG, f"Failed to cleanup {mac} after dongle unplug: {e}")
            # 清理 departed 记录（幂等；连接清理里的 _release_dongle 找不到分配会安全跳过）
            if self._ble_backend is not None:
                self._ble_backend.release(spec)

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

        # Stop loops and join threads (gforce loop is singleton, do not stop here)
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

        if self._log_path:
            SdkLog.set_log_path(self._log_path)

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
            publisher_task = asyncio.create_task(self._publisher_task())
            # USB dongle 热插拔监控：插入/拔出事件经主事件循环对账 dongle 池
            self._dongle_monitor = bumble_dongle.create_hotplug_monitor(
                self._on_dongle_specs_changed, self._dongle_specs)
            if self._dongle_monitor is not None:
                self._dongle_monitor.start()
            try:
                await self._main_loop()
            finally:
                self._should_exit = True

                try:
                    await asyncio.wait_for(publisher_task, timeout=2.0)
                except asyncio.TimeoutError:
                    publisher_task.cancel()
                    try:
                        await publisher_task
                    except asyncio.CancelledError as e:
                        SdkLog.exception(_TAG, "Unexpected error")

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

        while not self._should_exit:
            try:
                cmd = self.cmd_queue.get_nowait()
                asyncio.create_task(self._handle_command(cmd))
            except multiprocessing.queues.Empty:
                await asyncio.sleep(0.05)


        await self._cleanup_all_devices()

    async def _cleanup_all_devices(self):
        for device_mac in list(self._gforces.keys()):
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

        # Scan commands run in singleton gforce loop
        if cmd_type == "scan_once":
            loop = self._ensure_gforce_loop()
            asyncio.run_coroutine_threadsafe(
                self._do_scan_once(cmd.get("period", 5000)), loop
            )
            return
        if cmd_type == "start_scan":
            loop = self._ensure_gforce_loop()
            asyncio.run_coroutine_threadsafe(
                self._do_start_scan(cmd.get("period", 5000)), loop
            )
            return
        if cmd_type == "stop_scan":
            self._is_scanning = False
            return
        if cmd_type == "terminate":
            self._is_scanning = False
            self._should_exit = True
            return
        if cmd_type == "set_log_path":
            # 主进程日志路径变更时同步到子进程，"" 表示关闭文件日志
            try:
                SdkLog.set_log_path(cmd.get("path") or "")
            except Exception as e:
                SdkLog.exception(_TAG, f"set_log_path failed: {e}")
            return

        # Connect runs in main loop because it creates device loops
        if cmd_type == "connect":
            await self._do_connect(cmd)
            return

        # 离线回放 bin 文件：在主事件循环执行，不依赖真实设备连接
        if cmd_type == "replay_bin":
            await self._do_replay_bin(cmd)
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
        try:
            await asyncio.wait_for(asyncio.wrap_future(future), timeout=25)
        except asyncio.TimeoutError:
            SdkLog.e(_TAG, f"_handle_command timeout: {cmd_type}")
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
            return

        try:
            self._init_scanner(kwargs)
            found_devices = await self._scanner.discover(
                timeout=period / 1000, return_adv=True, **kwargs
            )
            devices = self._process_ble_devices(found_devices)
            self._publish("scan_once_result", devices=devices)
        except Exception as e:
            SdkLog.exception(_TAG, f"scan_once failed: {e}")
            self._publish("error", message=f"scan_once failed: {e}")
        finally:
            if spec is not None:
                self._ble_backend.release(spec)

    async def _do_start_scan(self, period: int):

        spec, kwargs = self._scan_dongle_kwargs()
        if self._ble_backend is not None and spec is None:
            SdkLog.w(_TAG, "Scan ignored: no free dongle (all in use)")
            return

        try:
            self._init_scanner(kwargs)
            found_devices = await self._scanner.discover(
                timeout=period / 1000, return_adv=True, **kwargs
            )
            devices = self._process_ble_devices(found_devices)
            self._publish("devices", devices=devices)
        except Exception as e:
            SdkLog.exception(_TAG, f"start_scan failed: {e}")
            self._publish("error", message=f"start_scan failed: {e}")
        finally:
            if spec is not None:
                self._ble_backend.release(spec)

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

        future = asyncio.run_coroutine_threadsafe(self._do_connect_inner(cmd), gforce_loop)
        try:
            await asyncio.wait_for(asyncio.wrap_future(future), timeout=25)
        except asyncio.TimeoutError:
            SdkLog.e(_TAG, f"_do_connect timeout: {device_mac}")
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Connect timeout",
            )
        except Exception as e:
            SdkLog.exception(_TAG, f"_do_connect failed: {device_mac}")
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result=str(e),
            )

    async def _do_connect_inner(self, cmd: dict):

        from sensor.gforce import GForce
        from sensor.sensor_data_context import SensorProfileDataCtx

        device_mac = cmd["device_mac"]
        device_address = cmd["device_address"]
        name = cmd.get("name", "")
        service_data = cmd.get("service_data", {})

        # 保存连接信息，用于异常断开后自动重连
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
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="Device not found in scanned devices",
            )
            return

        # bumble 后端：为本连接分配一只空闲 dongle（一 dongle 一设备，
        # 连接存续期间该 dongle 不再用于扫描或其他连接）
        dongle_spec, client_kwargs = self._assign_dongle(device_mac)
        if self._ble_backend is not None and dongle_spec is None:
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=False,
                result="No free BLE dongle",
            )
            return

        # Create raw data buffer (local to sub-process)
        raw_buf = queue.Queue(maxsize=sensor_utils.BLEAK_RESULT_QUEUE_MAXSIZE)

        # Create GForce with per-device event loops
        gforce = GForce(bleak_device, cmd_char, data_char, is_universal, event_loop, gforce_event_loop, chip_type, client_kwargs=client_kwargs)

        # Define disconnect callback: schedule cleanup in event_loop
        def handle_disconnect(_):
            loop = self._event_loops.get(device_mac)
            if loop is not None and not loop.is_closed():
                try:
                    asyncio.run_coroutine_threadsafe(
                        self._cleanup_device(device_mac, disconnect_client=False), loop
                    )
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error")

        try:
            await gforce.connect(handle_disconnect, raw_buf)
            SdkLog.d(_TAG, f"gforce.connect returned: {device_mac}")
        except Exception as e:
            SdkLog.exception(_TAG, f"gforce.connect failed: {device_mac}")
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

        # Start data processing loop in data_event_loop
        asyncio.run_coroutine_threadsafe(self._device_data_loop(device_mac), data_event_loop)

        # Publish states
        self._publish("state_changed", device_mac=device_mac, state="Connected")
        self._publish("state_changed", device_mac=device_mac, state="Ready")
        SdkLog.d(_TAG, f"connect success, publishing result: {device_mac}")
        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=True,
            result=True,
            chip_type=chip_type.value,
        )

    async def _cleanup_device(self, device_mac: str, disconnect_client: bool = False):

        lock = self._cleanup_locks.get(device_mac)
        if lock is None:
            return

        async with lock:
            if device_mac not in self._gforces and not disconnect_client:
                return  # Already cleaned

            # 用户主动断开时标记为正常断开，并取消正在进行的自动重连
            if disconnect_client:
                info = self._reconnect_info.get(device_mac)
                if info is not None:
                    info["normal_disconnect"] = True
                    task = info.get("task")
                    if task is not None:
                        try:
                            task.cancel()
                        except Exception:
                            pass
                        info["task"] = None

            # Cancel data task (running in data_event_loop)
            if device_mac in self._data_tasks:
                task = self._data_tasks.pop(device_mac, None)
                if task is not None:
                    try:
                        task.cancel()
                    except Exception as e:
                        SdkLog.exception(_TAG, "Unexpected error")

            # Cancel battery task
            if device_mac in self._battery_tasks:
                task = self._battery_tasks.pop(device_mac, None)
                if task is not None:
                    try:
                        task.cancel()
                    except Exception as e:
                        SdkLog.exception(_TAG, "Unexpected error")

            # Stop streaming and optionally disconnect client
            if disconnect_client and device_mac in self._gforces:
                if device_mac in self._data_ctxs:
                    ctx = self._data_ctxs[device_mac]
                    if ctx.isDataTransfering:
                        try:
                            await ctx.stop_streaming()
                        except Exception as e:
                            SdkLog.exception(_TAG, "Unexpected error")
                try:
                    await self._gforces[device_mac].disconnect()
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error")

            # Close data context
            if device_mac in self._data_ctxs:
                try:
                    self._data_ctxs[device_mac].close()
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error")
                self._data_ctxs.pop(device_mac, None)

            # Pop state
            self._gforces.pop(device_mac, None)
            self._raw_bufs.pop(device_mac, None)
            self._device_states[device_mac] = "Disconnected"
            # 归还连接占用的 dongle（异常断开同样归还，重连时重新分配）
            self._release_dongle(device_mac)

            # 正常断开后清理重连信息；异常断开保留以继续自动重连
            if disconnect_client:
                self._reconnect_info.pop(device_mac, None)

        self._publish("state_changed", device_mac=device_mac, state="Disconnected")

    def _schedule_reconnect(self, device_mac: str):
        """从 watchdog 线程调用，调度一次自动重连。"""
        info = self._reconnect_info.get(device_mac)
        if info is None or info.get("normal_disconnect"):
            return
        if info.get("task") is not None:
            return  # 已有重连任务在进行
        if info["attempts"] >= _MAX_RECONNECT_ATTEMPTS:
            SdkLog.e(_TAG, f"Max reconnect attempts reached for {device_mac}")
            return

        info["attempts"] += 1
        SdkLog.i(_TAG, f"Scheduling reconnect attempt {info['attempts']} for {device_mac}")

        loop = self._main_event_loop
        if loop is None or loop.is_closed():
            SdkLog.e(_TAG, f"Main event loop not available, cannot reconnect {device_mac}")
            return

        async def _delayed_reconnect():
            await asyncio.sleep(_RECONNECT_DELAY_SECONDS)
            await self._do_reconnect(device_mac)

        try:
            info["task"] = asyncio.run_coroutine_threadsafe(_delayed_reconnect(), loop)
        except Exception as e:
            SdkLog.exception(_TAG, f"Failed to schedule reconnect for {device_mac}: {e}")
            info["task"] = None

    async def _do_reconnect(self, device_mac: str):
        """执行一次自动重连。"""
        info = self._reconnect_info.get(device_mac)
        if info is None or info.get("normal_disconnect"):
            return

        # 已经连接则无需重连
        if device_mac in self._gforces:
            info["task"] = None
            return

        SdkLog.i(_TAG, f"Reconnecting {device_mac}, attempt {info['attempts']}")
        try:
            await self._do_connect(info["cmd"])
        except Exception as e:
            SdkLog.exception(_TAG, f"Reconnect attempt failed for {device_mac}: {e}")
            # 失败后会再次由 _schedule_reconnect 决定是否继续
            self._schedule_reconnect(device_mac)
        finally:
            if info is not None:
                info["task"] = None

    async def _do_disconnect(self, cmd: dict):

        device_mac = cmd["device_mac"]
        state = self._device_states.get(device_mac, "Disconnected")

        if state not in ("Connected", "Ready"):
            self._publish(
                "command_result",
                cmd_id=cmd.get("cmd_id"),
                device_mac=device_mac,
                success=True,
                result=True,
            )
            return

        self._device_states[device_mac] = "Disconnecting"
        self._publish("state_changed", device_mac=device_mac, state="Disconnecting")

        await self._cleanup_device(device_mac, disconnect_client=True)

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
            SdkLog.exception(_TAG, f"_do_init failed: {device_mac}")
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
                    SdkLog.exception(_TAG, "Unexpected error")

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
            try:
                power = await self._gforces[device_mac].get_battery_level()
                self._publish("power_changed", device_mac=device_mac, power=power)
            except Exception:
                # 单次刷新失败不中断循环，否则一次超时/丢包后就再也无法刷新电量
                SdkLog.exception(_TAG, f"Battery refresh failed: {device_mac}")

    async def _device_data_loop(self, device_mac: str):

        from sensor.sensor_data import SensorData

        ctx = self._data_ctxs.get(device_mac)
        if ctx is None:
            return

        local_buf = queue.Queue(maxsize=sensor_utils.BLEAK_RESULT_QUEUE_MAXSIZE)

        def on_data(sensor_data: SensorData):
            self._publish("sensor_data", device_mac=device_mac, data=sensor_data)

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
                SdkLog.i(_TAG, f"Data loop cancelled: {device_mac}")
                break
            except Exception as e:
                SdkLog.exception(_TAG, f"Error in data loop, restarting: {device_mac}")
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
                SdkLog.exception(_TAG, f"Create replay context failed: {device_mac}")
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
        else:
            prev_transfering = ctx._is_data_transfering
            ctx._is_data_transfering = True

        processed = 0
        skipped = 0
        config_seen = ctx.hasInit()
        data_loop_started = not owns_ctx  # 已有 ctx 的解析循环已在运行
        last_ts = None
        error = None
        stopped = False
        ctrl = {"paused": False, "stop": False}
        self._replay_controls[device_mac] = ctrl
        try:
            SdkLog.i(_TAG, f"Replay bin start: {device_mac} file={path} realtime={realtime}")
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
                        ctx.load_replay_config(config)
                        config_seen = True
                        SdkLog.i(_TAG, f"Replay config applied: {device_mac}")
                    except Exception:
                        SdkLog.exception(_TAG, f"Replay config record failed: {device_mac}")
                    if owns_ctx and not data_loop_started and config_seen:
                        # 等到配置就绪后再启动解析循环，确保流模式（普通/通用）正确
                        data_loop_started = True
                        asyncio.create_task(self._device_data_loop(device_mac))
                    continue
                if record_type != BIN_RECORD_DATA:
                    # 头部等其他记录类型不参与回放
                    continue
                if not config_seen:
                    # 配置记录之前的数据无法解析，跳过
                    skipped += 1
                    continue
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
                SdkLog.e(_TAG, f"Replay failed: {device_mac} {error}")
            if error:
                result = error
            elif stopped:
                result = f"OK: replay stopped after {processed} packets (skipped {skipped})"
            else:
                result = f"OK: replayed {processed} packets (skipped {skipped})"
        except Exception as e:
            SdkLog.exception(_TAG, f"Replay failed: {device_mac}")
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

        SdkLog.i(_TAG, f"Replay bin finished: {device_mac} {result}")
        self._publish(
            "command_result",
            cmd_id=cmd_id,
            device_mac=device_mac,
            success=(error is None),
            result=result,
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
            SdkLog.exception(_TAG, f"_do_start_notification failed: {device_mac}")
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
            result = await ctx.stop_streaming()
        except Exception as e:
            SdkLog.exception(_TAG, f"_do_stop_notification failed: {device_mac}")
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
            SdkLog.exception(_TAG, f"_do_get_battery failed: {device_mac}")
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
            SdkLog.exception(_TAG, f"_do_set_neucir_app_control failed: {device_mac}")
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
            SdkLog.exception(_TAG, f"_do_set_neucir_mode failed: {device_mac}")
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
            result = SdkLog.get_log_path() or ""

        if key == "DEBUG_BLE_DATA_PATH":
            result = ctx._data_log_path if ctx._data_log_enabled else ""

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
                            SdkLog.exception(_TAG, f"_do_set_param set_function_switch failed: {device_mac}")
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
                            SdkLog.exception(_TAG, f"_do_set_param set_subscription failed: {device_mac}")
                            result = "ERROR: set_subscription fail: " + str(e)

        if key in ["FILTER_50HZ", "FILTER_60HZ", "FILTER_HPF", "FILTER_LPF"]:
            if value in ["ON", "OFF"]:
                try:
                    result = await ctx.setFilter(key, value)
                    if result == "OK" and ctx.hasInit() and ctx.isDataTransfering:
                        needs_restart = True
                except Exception as e:
                    SdkLog.exception(_TAG, f"_do_set_param setFilter failed: {device_mac} {key}={value}")
                    result = "ERROR: " + str(e)

        if needs_restart or oym_needs_subscribe:
            try:
                if was_streaming:
                    await ctx.stop_streaming()
                if oym_needs_subscribe and ctx.getChipType() == BLEChipType.OYM:
                    await ctx.gForce.set_subscription(ctx.notifyDataFlag)
                if was_streaming:
                    await ctx.start_streaming()
            except Exception as e:
                SdkLog.exception(_TAG, f"_do_set_param restart stream failed: {device_mac}")
                result = "ERROR: restart stream fail: " + str(e)

        if key == "DEBUG_LOG_PATH":
            try:
                if value == "False" or value == "":
                    SdkLog.set_log_path("")
                elif value == "True":
                    path = SdkLog.get_default_log_path()
                    SdkLog.set_log_path(path)
                else:
                    SdkLog.set_log_path(value)
                result = "OK"
            except Exception as e:
                SdkLog.exception(_TAG, f"_do_set_param DEBUG_LOG_PATH failed: {e}")
                result = "ERROR: " + str(e)

        if key == "DEBUG_BLE_DATA_PATH":
            try:
                if value == "False" or value == "":
                    ctx._data_log_enabled = False
                    ctx._data_log_path = None
                else:
                    if value == "True":
                        # 默认导出到 SDK 日志目录，.bin 后缀
                        name = ""
                        if ctx._device_info is not None and ctx._device_info.DeviceName:
                            name = ctx._device_info.DeviceName
                        safe = "".join(c if (c.isalnum() or c in "-_()") else "_" for c in (name or device_mac))
                        value = os.path.join(
                            SdkLog.get_log_dir(),
                            f"{safe}_data_{time.strftime('%Y%m%d_%H%M%S')}.bin",
                        )
                    # 仅记录导出位置；停止推流/断连时把 temp bin 拷贝过去
                    ctx._data_log_enabled = True
                    ctx._data_log_path = value
                result = "OK"
            except Exception as e:
                SdkLog.exception(_TAG, f"_do_set_param DEBUG_BLE_DATA_PATH failed: {device_mac} path={value}")
                result = "ERROR: " + str(e)

        if key == "NEUCIR_SET_MODE":
            if value in ["APP_REMOTE"]:
                try:
                    ret = await self._gforces[device_mac].set_neucir_mode(1)
                    result = "OK" if ret else "Error: Unknown error"
                except Exception as e:
                    SdkLog.exception(_TAG, f"_do_set_param NEUCIR_SET_MODE failed: {device_mac}")
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
                    SdkLog.exception(_TAG, f"_do_set_param NEUCIR_APP_CONTROL failed: {device_mac} {value}")
                    result = "ERROR: " + str(e)

        self._publish(
            "command_result",
            cmd_id=cmd.get("cmd_id"),
            device_mac=device_mac,
            success=(not result.startswith("Error")),
            result=result,
        )
