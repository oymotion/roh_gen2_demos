"""Synchroni SDK 统一日志开关。

用法与 Android 端的 com.sensor.SdkLog 保持一致，并额外支持将日志写入文件：
    from sensor.sdk_log import SdkLog

    SdkLog.set_debug_enabled(False)          # 关闭调试日志
    SdkLog.set_log_path()                    # 启用文件日志，使用默认路径
    SdkLog.set_log_path("/tmp/my.log")       # 启用文件日志，自定义路径
    SdkLog.set_log_path("")                  # 禁用文件日志
"""

import atexit
import collections
import logging
import logging.handlers
import os
import queue
from datetime import datetime
from pathlib import Path
from typing import Optional

# 自定义 data 日志级别，位于 DEBUG 与 INFO 之间
DATA_LOG_LEVEL = 15
logging.addLevelName(DATA_LOG_LEVEL, "DATA")


class _StartupBufferHandler(logging.Handler):
    """文件日志启用前的启动日志缓存。

    在文件日志首次启用之前，日志记录会缓存在内存中；首次调用
    ``set_log_path`` / ``set_controller_log_path`` 启用文件日志时，
    缓存的记录会被回放到文件里，避免开头的日志（扫描、连接等阶段）丢失。
    """

    def __init__(self, capacity: int):
        super().__init__()
        self.buffer = collections.deque(maxlen=capacity)
        self.active = True

    def emit(self, record: logging.LogRecord):
        if self.active:
            self.buffer.append(record)


class SdkLog:
    """SDK 全局日志开关。

    所有 SDK 内部调试日志都应通过本类输出，以便调用方统一控制日志噪音。
    支持将日志写入文件，默认路径为 ``~/Documents/sensorsdklog/sensor_sdk.log``。

    额外提供独立的 data 日志通道，用于记录原始蓝牙数据包与解析后的数据，
    默认关闭，通过 ``set_data_log_enabled`` 控制。
    """

    _debug_enabled = True
    _data_log_enabled = False
    _logger = logging.getLogger("sensor_sdk")
    _logger.setLevel(logging.DEBUG)
    _file_handler: Optional[logging.FileHandler] = None
    _log_path: Optional[str] = None

    # SensorController 专用日志（默认不开启）
    _controller_logger = logging.getLogger("sensor_sdk_controller")
    _controller_logger.setLevel(logging.DEBUG)
    _controller_handler: Optional[logging.FileHandler] = None

    # 异步日志队列与监听器，避免写文件阻塞业务线程
    _LOG_QUEUE_MAXSIZE = 10000
    _log_queue = queue.Queue(maxsize=_LOG_QUEUE_MAXSIZE)
    _log_listener: Optional[logging.handlers.QueueListener] = None
    _log_queue_handler: Optional[logging.handlers.QueueHandler] = None

    _controller_log_queue = queue.Queue(maxsize=_LOG_QUEUE_MAXSIZE)
    _controller_log_listener: Optional[logging.handlers.QueueListener] = None
    _controller_log_queue_handler: Optional[logging.handlers.QueueHandler] = None

    # 启动日志缓存：文件日志首次启用前缓存记录，启用时回放，避免开头日志丢失
    _STARTUP_BUFFER_CAPACITY = _LOG_QUEUE_MAXSIZE
    _startup_buffer_handler = _StartupBufferHandler(_STARTUP_BUFFER_CAPACITY)
    _logger.addHandler(_startup_buffer_handler)
    _startup_buffer_replayed = False

    _controller_startup_buffer_handler = _StartupBufferHandler(_STARTUP_BUFFER_CAPACITY)
    _controller_logger.addHandler(_controller_startup_buffer_handler)
    _controller_startup_buffer_replayed = False

    @classmethod
    def _formatter(cls) -> logging.Formatter:
        return logging.Formatter(
            "%(asctime)s [%(levelname)s] [%(name)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )

    @classmethod
    def get_default_log_path(cls, prefix: str = "") -> str:
        """返回默认 SDK 日志文件路径。

        若提供 ``prefix``，则文件名为 ``{prefix}_log_YYYYMMDD_HHMMSS.txt``，
        否则为 ``log_YYYYMMDD_HHMMSS.txt``。
        """
        docs = Path.home() / "Documents"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix_part = f"{prefix}_" if prefix else ""
        return str(docs / "sensorsdklog" / f"{prefix_part}log_{timestamp}.txt")

    @classmethod
    def get_log_dir(cls) -> str:
        """返回当前日志文件所在目录；未启用文件日志时返回默认日志目录。"""
        if cls._log_path:
            directory = os.path.dirname(cls._log_path)
            if directory:
                return directory
        return str(Path.home() / "Documents" / "sensorsdklog")

    # 保持向后兼容的旧名称
    _get_default_log_path = get_default_log_path

    @classmethod
    def get_default_data_log_path(cls, prefix: str = "") -> str:
        """返回默认 data 日志 CSV 文件路径。

        若提供 ``prefix``，则文件名为 ``{prefix}_data_YYYYMMDD_HHMMSS.csv``，
        否则为 ``data_YYYYMMDD_HHMMSS.csv``。
        """
        docs = Path.home() / "Documents"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix_part = f"{prefix}_" if prefix else ""
        return str(docs / "sensorsdklog" / f"{prefix_part}data_{timestamp}.csv")

    @classmethod
    def _ensure_dir(cls, path: str):
        if path:
            try:
                os.makedirs(path, exist_ok=True)
            except Exception:
                pass

    @classmethod
    def _remove_file_handler(cls):
        if cls._file_handler is not None:
            try:
                cls._file_handler.close()
            except Exception:
                pass
            cls._file_handler = None

    @classmethod
    def _start_log_listener(cls):
        """启动主日志监听器，让文件写操作在独立线程执行。"""
        if cls._file_handler is None:
            return
        if cls._log_listener is not None:
            return
        cls._log_listener = logging.handlers.QueueListener(
            cls._log_queue, cls._file_handler, respect_handler_level=True
        )
        cls._log_listener.start()
        if cls._log_queue_handler is None:
            cls._log_queue_handler = logging.handlers.QueueHandler(cls._log_queue)
            cls._logger.addHandler(cls._log_queue_handler)

    @classmethod
    def _stop_log_listener(cls):
        """停止主日志监听器并移除队列处理器。"""
        if cls._log_listener is not None:
            try:
                cls._log_listener.stop()
            except Exception:
                pass
            cls._log_listener = None
        if cls._log_queue_handler is not None:
            try:
                cls._logger.removeHandler(cls._log_queue_handler)
            except Exception:
                pass
            cls._log_queue_handler = None

    @classmethod
    def _replay_startup_buffer(cls):
        """把文件日志启用前缓存的记录写入日志文件（仅首次启用时回放一次）。"""
        if cls._startup_buffer_replayed:
            return
        cls._startup_buffer_replayed = True
        buf = cls._startup_buffer_handler
        buf.active = False
        if len(buf.buffer) >= cls._STARTUP_BUFFER_CAPACITY:
            cls._logger.warning("SDK startup log buffer overflowed, earliest logs were dropped")
        while buf.buffer:
            try:
                cls._log_queue.put(buf.buffer.popleft(), timeout=1.0)
            except queue.Full:
                break
        try:
            cls._logger.removeHandler(buf)
        except Exception:
            pass

    @classmethod
    def set_debug_enabled(cls, enabled: bool):
        """开启或关闭 SDK 调试日志。"""
        cls._debug_enabled = bool(enabled)

    @classmethod
    def is_debug_enabled(cls) -> bool:
        """返回当前是否开启调试日志。"""
        return cls._debug_enabled

    @classmethod
    def set_data_log_enabled(cls, enabled: bool):
        """开启或关闭 data 日志（记录蓝牙数据包与解析结果），默认关闭。"""
        cls._data_log_enabled = bool(enabled)
        os.environ["SENSORSKD_DATA_LOG_ENABLED"] = "1" if cls._data_log_enabled else "0"

    @classmethod
    def is_data_log_enabled(cls) -> bool:
        """返回当前是否开启 data 日志。"""
        return cls._data_log_enabled

    @classmethod
    def set_log_path(cls, path: Optional[str] = None):
        """设置 SDK 日志文件路径。

        Args:
            path: 日志文件路径。
                - ``None``：使用默认路径 ``~/Documents/sensorsdklog/sensor_sdk.log``。
                - ``""``：禁用文件日志。
                - 其他：使用指定路径。
        """
        cls._stop_log_listener()
        cls._remove_file_handler()
        if path == "":
            cls._log_path = None
            return

        if path is None:
            path = cls.get_default_log_path()

        cls._log_path = path
        cls._ensure_dir(os.path.dirname(path))
        try:
            handler = logging.FileHandler(path, encoding="utf-8")
            handler.setFormatter(cls._formatter())
            cls._file_handler = handler
            cls._start_log_listener()
            cls._replay_startup_buffer()
        except Exception as e:
            cls._logger.warning(f"Failed to create log file {path}: {e}")

    @classmethod
    def get_log_path(cls) -> Optional[str]:
        """返回当前日志文件路径，若未启用文件日志则返回 None。"""
        return cls._log_path

    @classmethod
    def enable_file_log(cls, enabled: bool = True):
        """开启或关闭文件日志。开启时使用默认日志路径。"""
        if enabled:
            if cls._file_handler is None:
                cls.set_log_path()
        else:
            cls.set_log_path("")

    @classmethod
    def _controller_formatter(cls) -> logging.Formatter:
        return logging.Formatter(
            "%(asctime)s [%(name)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )

    @classmethod
    def _get_default_controller_log_path(cls) -> str:
        """返回 SensorController 默认日志文件路径：~/Documents/sensorsdklog/sensor_controller_log_YYYYMMDD_HHMMSS.txt。"""
        docs = Path.home() / "Documents"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return str(docs / "sensorsdklog" / f"sensor_controller_log_{timestamp}.txt")

    @classmethod
    def _start_controller_log_listener(cls):
        """启动 Controller 日志监听器。"""
        if cls._controller_handler is None:
            return
        if cls._controller_log_listener is not None:
            return
        cls._controller_log_listener = logging.handlers.QueueListener(
            cls._controller_log_queue, cls._controller_handler, respect_handler_level=True
        )
        cls._controller_log_listener.start()
        if cls._controller_log_queue_handler is None:
            cls._controller_log_queue_handler = logging.handlers.QueueHandler(cls._controller_log_queue)
            cls._controller_logger.addHandler(cls._controller_log_queue_handler)

    @classmethod
    def _stop_controller_log_listener(cls):
        """停止 Controller 日志监听器。"""
        if cls._controller_log_listener is not None:
            try:
                cls._controller_log_listener.stop()
            except Exception:
                pass
            cls._controller_log_listener = None
        if cls._controller_log_queue_handler is not None:
            try:
                cls._controller_logger.removeHandler(cls._controller_log_queue_handler)
            except Exception:
                pass
            cls._controller_log_queue_handler = None

    @classmethod
    def _replay_controller_startup_buffer(cls):
        """把 Controller 日志启用前缓存的记录写入日志文件（仅首次启用时回放一次）。"""
        if cls._controller_startup_buffer_replayed:
            return
        cls._controller_startup_buffer_replayed = True
        buf = cls._controller_startup_buffer_handler
        buf.active = False
        if len(buf.buffer) >= cls._STARTUP_BUFFER_CAPACITY:
            cls._controller_logger.warning("SDK controller startup log buffer overflowed, earliest logs were dropped")
        while buf.buffer:
            try:
                cls._controller_log_queue.put(buf.buffer.popleft(), timeout=1.0)
            except queue.Full:
                break
        try:
            cls._controller_logger.removeHandler(buf)
        except Exception:
            pass

    @classmethod
    def set_controller_log_path(cls, path: Optional[str] = None):
        """设置 SensorController 专用日志文件路径。

        Args:
            path: 日志文件路径。
                - ``None``：使用默认路径 ``~/Documents/sensorsdklog/sensor_controller_log_YYYYMMDD_HHMMSS.txt``。
                - ``""``：禁用 Controller 日志。
                - 其他：使用指定路径。
        """
        cls._stop_controller_log_listener()
        if cls._controller_handler is not None:
            try:
                cls._controller_handler.close()
            except Exception:
                pass
            cls._controller_handler = None

        if path == "":
            return

        if path is None:
            path = cls._get_default_controller_log_path()

        cls._ensure_dir(os.path.dirname(path))
        try:
            handler = logging.FileHandler(path, encoding="utf-8")
            handler.setFormatter(cls._controller_formatter())
            cls._controller_handler = handler
            cls._start_controller_log_listener()
            cls._replay_controller_startup_buffer()
        except Exception as e:
            cls._controller_logger.warning(f"Failed to create controller log file {path}: {e}")

    @classmethod
    def get_controller_log_path(cls) -> Optional[str]:
        """返回当前 SensorController 日志文件路径，若未启用则返回 None。"""
        return cls._controller_handler.baseFilename if cls._controller_handler is not None else None

    @classmethod
    def enable_controller_log(cls, enabled: bool = True):
        """开启或关闭 SensorController 专用日志。开启时使用默认路径。"""
        if enabled:
            if cls._controller_handler is None:
                cls.set_controller_log_path()
        else:
            cls.set_controller_log_path("")

    @classmethod
    def stop(cls):
        """停止所有异步日志监听器，确保日志 flush 到文件。

        程序退出时建议调用一次，避免队列中的日志丢失。
        """
        cls._stop_log_listener()
        cls._stop_controller_log_listener()

    @classmethod
    def controller(cls, tag: str, msg: str):
        """输出 SensorController 专用日志。"""
        cls._controller_logger.info(f"[{tag}] {msg}")

    @classmethod
    def d(cls, tag: str, msg: str):
        """输出 debug 日志（可被 set_debug_enabled 关闭）。"""
        if cls._debug_enabled:
            cls._logger.debug(f"[{tag}] {msg}")

    @classmethod
    def data(cls, tag: str, msg: str):
        """输出 data 日志（记录蓝牙数据包与解析结果）。

        仅当 ``set_data_log_enabled(True)`` 时输出，默认关闭。
        """
        if cls._data_log_enabled:
            cls._logger.log(DATA_LOG_LEVEL, f"[{tag}] {msg}")

    @classmethod
    def i(cls, tag: str, msg: str):
        """输出 info 日志。"""
        cls._logger.info(f"[{tag}] {msg}")

    @classmethod
    def w(cls, tag: str, msg: str):
        """输出 warning 日志。"""
        cls._logger.warning(f"[{tag}] {msg}")

    @classmethod
    def e(cls, tag: str, msg: str):
        """输出 error 日志。"""
        cls._logger.error(f"[{tag}] {msg}")

    @classmethod
    def exception(cls, tag: str, msg: str = ""):
        """输出 exception 日志，包含当前异常堆栈。"""
        cls._logger.exception(f"[{tag}] {msg}")


# 程序正常退出时自动停止日志监听器，尽量保证队列中的日志落盘
atexit.register(SdkLog.stop)
