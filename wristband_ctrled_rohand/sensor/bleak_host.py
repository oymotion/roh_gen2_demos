import asyncio
import multiprocessing
import queue
import threading
import uuid
from typing import Optional

from sensor.bleak_process import BleakProcess
from sensor import sensor_utils
from sensor.sdk_log import SdkLog

_TAG = "BleakHost"


class BleakHost:


    def __init__(self):
        self._cmd_queue = multiprocessing.Queue(maxsize=50)
        self._result_queue = multiprocessing.Queue(maxsize=sensor_utils.BLEAK_RESULT_QUEUE_MAXSIZE)
        self._data_queue = multiprocessing.Queue(maxsize=sensor_utils.BLEAK_DATA_QUEUE_MAXSIZE)
        # BleakProcess 延迟到 start() 时创建，以便把当时最新的日志路径传给子进程
        self._bleak_process: Optional[BleakProcess] = None
        self._started = False


        self._result_loop = None
        self._result_thread = None


        self._cmd_loop = None
        self._cmd_thread = None

        # Pending commands: cmd_id -> (waiter_type, waiter, container, loop)
        self._pending_lock = threading.Lock()
        self._pending = {}


        self.on_scan_once_result = None  # callable(msg)
        self.on_scan_result = None       # callable(msg)
        self.on_device_message = None    # callable(device_mac, msg)
        self.on_backend_info = None      # callable(msg)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self):
        if self._started:
            return
        # 启动时才创建子进程，并传入当前日志路径，保证子进程尽早写文件日志
        self._bleak_process = BleakProcess(
            self._cmd_queue, self._result_queue, self._data_queue, SdkLog.get_log_path()
        )
        self._bleak_process.start()
        self._started = True
        self._start_result_loop()
        self._start_cmd_loop()

    def stop(self):
        if not self._started:
            return
        self._should_exit = True
        try:
            self._cmd_queue.put({"type": "terminate"})
        except Exception as e:
            SdkLog.exception(_TAG, "Unexpected error")

        if self._bleak_process is not None:
            if self._bleak_process.is_alive():
                self._bleak_process.join(timeout=5)
            if self._bleak_process.is_alive():
                self._bleak_process.terminate()
                self._bleak_process.join(timeout=2)

        self._stop_result_loop()
        self._stop_cmd_loop()

        try:
            self._cmd_queue.close()
            self._result_queue.close()
            self._data_queue.close()
        except Exception as e:
            SdkLog.exception(_TAG, "Error closing queues")

        try:
            if self._bleak_process is not None:
                self._bleak_process.close()
        except Exception as e:
            SdkLog.exception(_TAG, "Error closing bleak process")
        self._bleak_process = None

        self._started = False

    # ------------------------------------------------------------------
    # Result loop
    # ------------------------------------------------------------------
    def _start_result_loop(self):
        from sensor import sensor_utils
        result_loop = asyncio.new_event_loop()
        result_thread = threading.Thread(target=sensor_utils.start_loop, args=(result_loop,))
        result_thread.daemon = True
        result_thread.name = "BleakHost result"
        result_thread.start()
        self._result_loop = result_loop
        self._result_thread = result_thread

        def _create_consume_task():
            self._consume_task = result_loop.create_task(self._consume_results())

        result_loop.call_soon_threadsafe(_create_consume_task)

    def _stop_result_loop(self):
        task = getattr(self, '_consume_task', None)
        if task is not None:
            try:
                loop = task.get_loop()
                if loop is not None and not loop.is_closed():
                    loop.call_soon_threadsafe(task.cancel)
            except Exception as e:
                SdkLog.exception(_TAG, "Unexpected error")
        self._consume_task = None

        loop = getattr(self, '_result_loop', None)
        thread = getattr(self, '_result_thread', None)
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
        self._result_loop = None
        self._result_thread = None

    async def _consume_results(self):

        def _try_get(q, timeout):
            try:
                return q.get(True, timeout)
            except queue.Empty:
                return None

        while not getattr(self, '_should_exit', False):
            try:
                # Prioritize control/result messages; data messages are drained in batch below.
                msg = await asyncio.get_event_loop().run_in_executor(
                    None, _try_get, self._result_queue, 0.02
                )
            except Exception:
                msg = None

            if msg is None:
                try:
                    msg = await asyncio.get_event_loop().run_in_executor(
                        None, _try_get, self._data_queue, 0.02
                    )
                except Exception:
                    msg = None

            if msg is None:
                await asyncio.sleep(0.001)
                continue

            self._dispatch_message(msg)

            # Batch drain both queues to catch up quickly and reduce per-get overhead.
            for q in (self._result_queue, self._data_queue):
                while True:
                    try:
                        self._dispatch_message(q.get_nowait())
                    except queue.Empty:
                        break
                    except Exception:
                        break

    def _dispatch_message(self, msg):
        msg_type = msg.get("type")
        if msg_type == "scan_once_result":
            if self.on_scan_once_result is not None:
                try:
                    self.on_scan_once_result(msg)
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error")
        elif msg_type == "devices":
            if self.on_scan_result is not None:
                try:
                    self.on_scan_result(msg)
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error")
        elif msg_type == "backend_info":
            if self.on_backend_info is not None:
                try:
                    self.on_backend_info(msg)
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error")
        elif msg_type == "command_result":
            self._handle_command_result(msg)
        elif msg_type in ("state_changed", "power_changed", "sensor_data", "error"):
            if self.on_device_message is not None:
                device_mac = msg.get("device_mac")
                try:
                    self.on_device_message(device_mac, msg)
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error")

    def _handle_command_result(self, msg):

        cmd_id = msg.get("cmd_id")
        with self._pending_lock:
            pending = self._pending.pop(cmd_id, None)
        if pending is None:
            return
        waiter_type, waiter, container, loop = pending
        if waiter_type == "async_event":
            container["result"] = msg
            if loop is not None and not loop.is_closed():
                loop.call_soon_threadsafe(waiter.set)
        elif waiter_type == "future":
            if loop is not None and not loop.is_closed() and not waiter.done():
                loop.call_soon_threadsafe(waiter.set_result, msg)

        # 将 connect 返回的 chip_type 转发给对应 SensorProfile
        chip_type = msg.get("chip_type")
        device_mac = msg.get("device_mac")
        if chip_type is not None and self.on_device_message is not None and device_mac is not None:
            try:
                self.on_device_message(device_mac, {"type": "chip_type", "chip_type": chip_type})
            except Exception as e:
                SdkLog.exception(_TAG, "Error forwarding chip_type")

    # ------------------------------------------------------------------
    # Command loop
    # ------------------------------------------------------------------
    def _start_cmd_loop(self):
        from sensor import sensor_utils
        cmd_loop = asyncio.new_event_loop()
        cmd_thread = threading.Thread(target=sensor_utils.start_loop, args=(cmd_loop,))
        cmd_thread.daemon = True
        cmd_thread.name = "BleakHost cmd"
        cmd_thread.start()
        self._cmd_loop = cmd_loop
        self._cmd_thread = cmd_thread

    def _stop_cmd_loop(self):
        loop = getattr(self, '_cmd_loop', None)
        thread = getattr(self, '_cmd_thread', None)
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
        self._cmd_loop = None
        self._cmd_thread = None

    def _ensure_cmd_loop(self):
        if self._cmd_loop is None or self._cmd_loop.is_closed():
            self._start_cmd_loop()

    # ------------------------------------------------------------------
    # Public APIs
    # ------------------------------------------------------------------
    def scan_once(self, period):

        try:
            self._cmd_queue.put_nowait({"type": "scan_once", "period": period})
        except queue.Full as e:
            SdkLog.exception(_TAG, "Unexpected error")

    def start_scan(self, period):

        try:
            self._cmd_queue.put_nowait({"type": "start_scan", "period": period})
        except queue.Full as e:
            SdkLog.exception(_TAG, "Unexpected error")

    def stop_scan(self):

        try:
            self._cmd_queue.put_nowait({"type": "stop_scan"})
        except queue.Full as e:
            SdkLog.exception(_TAG, "Unexpected error")

    def send_command(self, cmd: dict):

        try:
            self._cmd_queue.put_nowait(cmd)
        except queue.Full as e:
            SdkLog.exception(_TAG, "Command queue full")

    def send_command_sync(self, cmd: dict, timeout: float = 10.0) -> dict:

        if not self._started:
            return {}
        self._ensure_cmd_loop()

        async def _do_send():
            cmd_id = str(uuid.uuid4())
            cmd["cmd_id"] = cmd_id
            event = asyncio.Event()
            container = {}
            with self._pending_lock:
                self._pending[cmd_id] = ("async_event", event, container, asyncio.get_event_loop())
            try:
                self._cmd_queue.put(cmd, timeout=1.0)
            except queue.Full:
                SdkLog.e(_TAG, f"send_command_sync queue full: {cmd.get('type')}")
                with self._pending_lock:
                    self._pending.pop(cmd_id, None)
                return {}
            try:
                await asyncio.wait_for(event.wait(), timeout=timeout)
            except asyncio.TimeoutError:
                SdkLog.e(_TAG, f"send_command_sync timeout: {cmd.get('type')}")
                with self._pending_lock:
                    self._pending.pop(cmd_id, None)
                return {}
            with self._pending_lock:
                self._pending.pop(cmd_id, None)
            return container.get("result", {})

        future = asyncio.run_coroutine_threadsafe(_do_send(), self._cmd_loop)
        try:
            return future.result(timeout=timeout)
        except Exception:
            SdkLog.exception(_TAG, "send_command_sync future failed")
            return {}

    async def send_command_async(self, cmd: dict, timeout: float = 10.0) -> dict:

        if not self._started:
            return {}
        self._ensure_cmd_loop()

        async def _do_send():
            cmd_id = str(uuid.uuid4())
            cmd["cmd_id"] = cmd_id
            future = asyncio.get_running_loop().create_future()
            with self._pending_lock:
                self._pending[cmd_id] = ("future", future, None, asyncio.get_running_loop())
            try:
                self._cmd_queue.put(cmd, timeout=1.0)
            except queue.Full:
                SdkLog.e(_TAG, f"send_command_async queue full: {cmd.get('type')}")
                with self._pending_lock:
                    self._pending.pop(cmd_id, None)
                return {}
            try:
                return await asyncio.wait_for(future, timeout=timeout)
            except asyncio.TimeoutError:
                SdkLog.e(_TAG, f"send_command_async timeout: {cmd.get('type')}")
                with self._pending_lock:
                    self._pending.pop(cmd_id, None)
                return {}

        try:
            caller_loop = asyncio.get_running_loop()
        except RuntimeError:
            caller_loop = None

        future = asyncio.run_coroutine_threadsafe(_do_send(), self._cmd_loop)
        if caller_loop is not None:
            return await asyncio.wrap_future(future, loop=caller_loop)
        else:
            return future.result()
