"""Synchroni SDK 统一日志开关（目录化 + profile/controller 分流）。

日志输出目录由 ``set_log_dir`` 设置（必须是目录），SDK 的所有默认文件输出
——controller log、默认 profile log、默认 bin 导出——都写在该目录下：

- controller log（``sensor_controller_log_YYYYMMDD_HHMMSS.txt``）：公共日志，
  包括扫描、连接管理、dongle/后端等全局日志，以及尚未开启 profile log 的
  设备日志；``set_debug_enabled(True)`` 时自动在日志目录创建。
- profile log（``{DeviceName}_log_YYYYMMDD_HHMMSS.txt``）：只包含单个
  profile 相关的日志，以及与该 profile 当前连接相关的 bleak/bumble 日志；
  通过 ``enable_profile_log(mac, path)`` 开启（对应 DEBUG_LOG_PATH 参数）。

用法：
    from sensor.sdk_log import SdkLog

    SdkLog.set_debug_enabled(True)       # 开启调试日志，并在日志目录自动创建 controller log
    SdkLog.set_log_dir("/tmp/sdklogs")   # 设置日志目录（必须是目录）
    SdkLog.set_log_dir(enabled=False)    # 关闭文件输出
"""

import atexit
import collections
import logging
import logging.handlers
import os
import queue
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

# 自定义 data 日志级别，位于 DEBUG 与 INFO 之间
DATA_LOG_LEVEL = 15
logging.addLevelName(DATA_LOG_LEVEL, "DATA")


class _StartupBufferHandler(logging.Handler):
    """文件日志启用前的启动日志缓存。

    在 controller log 首次创建之前，公共通道的日志记录会缓存在内存中；
    首次创建时缓存的记录会被回放到文件里，避免开头的日志
    （扫描、连接等阶段）丢失。
    """

    def __init__(self, capacity: int):
        super().__init__()
        self.buffer = collections.deque(maxlen=capacity)
        self.active = True

    def emit(self, record: logging.LogRecord):
        if self.active:
            self.buffer.append(record)


class _ProfileLogState:
    """单个 profile 的日志文件状态（未开启时仅保留缓存）。"""

    def __init__(self, mac: str, buffer_capacity: int):
        self.mac = mac
        self.logger = logging.getLogger(f"sensor_sdk.profile.{mac}")
        self.logger.setLevel(logging.DEBUG)
        # 不向 sensor_sdk 父 logger 传播，避免 profile 记录被公共 handler 重复写
        self.logger.propagate = False
        self.handler: Optional[logging.FileHandler] = None
        self.listener: Optional[logging.handlers.QueueListener] = None
        self.queue: Optional[queue.Queue] = None
        self.queue_handler: Optional[logging.handlers.QueueHandler] = None
        # 开启前的该 profile 记录缓存，开启时回放进文件（connect 阶段日志不丢）
        self.buffer = collections.deque(maxlen=buffer_capacity)
        self.path: Optional[str] = None


class _BoundSdkLog:
    """绑定 profile mac 的 SdkLog 转发器（per-device 类持有，接口与 SdkLog 一致）。"""

    def __init__(self, mac: str):
        self._mac = mac

    def d(self, tag: str, msg: str):
        SdkLog.d(tag, msg, mac=self._mac)

    def data(self, tag: str, msg: str):
        SdkLog.data(tag, msg, mac=self._mac)

    def i(self, tag: str, msg: str):
        SdkLog.i(tag, msg, mac=self._mac)

    def w(self, tag: str, msg: str):
        SdkLog.w(tag, msg, mac=self._mac)

    def e(self, tag: str, msg: str):
        SdkLog.e(tag, msg, mac=self._mac)

    def exception(self, tag: str, msg: str = ""):
        SdkLog.exception(tag, msg, mac=self._mac)


class SdkLog:
    """SDK 全局日志开关。

    所有 SDK 内部调试日志都应通过本类输出，以便调用方统一控制日志噪音。
    公共日志写入日志目录下的 controller log 文件；带 ``mac`` 的日志在对应
    profile log 开启后只写入该 profile 文件，未开启时回落公共通道并按
    profile 缓存（开启时回放）。

    额外提供独立的 data 日志通道，用于记录原始蓝牙数据包与解析后的数据，
    默认关闭，通过 ``set_data_log_enabled`` 控制。
    """

    _debug_enabled = True
    _data_log_enabled = False
    _file_output_enabled = True
    _log_dir: str = str(Path.home() / "Documents" / "sensorsdklog")

    # 公共日志通道（写入 controller log 文件）
    _logger = logging.getLogger("sensor_sdk")
    _logger.setLevel(logging.DEBUG)
    _controller_logger = logging.getLogger("sensor_sdk_controller")
    _controller_logger.setLevel(logging.DEBUG)

    # 异步日志队列与监听器，避免写文件阻塞业务线程（两个公共 logger 共用）
    _LOG_QUEUE_MAXSIZE = 10000
    _common_queue = queue.Queue(maxsize=_LOG_QUEUE_MAXSIZE)
    _common_listener: Optional[logging.handlers.QueueListener] = None
    _common_handler: Optional[logging.FileHandler] = None
    _common_queue_handler: Optional[logging.handlers.QueueHandler] = None
    # 当前打开的 controller log 文件路径（主进程据此把同一路径传给子进程，
    # 避免两个进程各自按当前秒生成不同文件名、同一会话拆成两个文件）
    _common_log_path: Optional[str] = None

    # 启动日志缓存：controller log 首次创建前缓存公共记录，创建时回放
    _STARTUP_BUFFER_CAPACITY = _LOG_QUEUE_MAXSIZE
    _PROFILE_BUFFER_CAPACITY = 2000
    _startup_buffer_handler = _StartupBufferHandler(_STARTUP_BUFFER_CAPACITY)
    _logger.addHandler(_startup_buffer_handler)
    _controller_logger.addHandler(_startup_buffer_handler)
    _startup_buffer_replayed = False

    # profile 日志注册表：mac -> 状态
    _profiles: Dict[str, _ProfileLogState] = {}

    @classmethod
    def _formatter(cls) -> logging.Formatter:
        # 毫秒精度：主/子进程写同一日志文件时按秒排序不可靠（同秒多事件
        # 先后无法判断），带毫秒才能还原跨进程事件真实时序
        return logging.Formatter(
            "%(asctime)s.%(msecs)03d [%(levelname)s] [%(name)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )

    # ------------------------------------------------------------------
    # 日志目录与文件输出开关
    # ------------------------------------------------------------------
    @classmethod
    def set_log_dir(cls, path: Optional[str] = None, enabled: bool = True,
                    controller_log_path: Optional[str] = None):
        """设置日志目录（必须是目录）。

        Args:
            path: 日志目录。
                - ``None``/``""``：恢复默认目录 ``~/Documents/sensorsdklog``。
                - 已存在的文件：拒绝（记 error 日志并忽略本次设置）。
                - 不存在：自动创建。
            enabled: ``False`` 关闭文件输出（关闭 controller log 且不再自动
                创建，默认 profile log / 默认 bin 导出也随之禁用；显式绝对
                路径的 profile log 仍可用）。
            controller_log_path: 打开 controller log 时使用的具体文件路径
                （主进程传给子进程，保证两个进程写同一文件）；``None`` 时按
                默认命名新建。
        """
        cls._file_output_enabled = bool(enabled)
        if not cls._file_output_enabled:
            cls._close_common_log()
            return

        if path:
            if os.path.exists(path) and not os.path.isdir(path):
                cls._logger.error(f"[SdkLog] set_log_dir rejected, not a directory: {path}")
                return
            cls._ensure_dir(path)
            cls._log_dir = os.path.abspath(path)
        else:
            cls._log_dir = str(Path.home() / "Documents" / "sensorsdklog")

        if cls._debug_enabled:
            cls._open_common_log(controller_log_path)

    @classmethod
    def get_log_dir(cls) -> str:
        """返回当前日志目录。"""
        return cls._log_dir

    @classmethod
    def is_file_output_enabled(cls) -> bool:
        """返回文件输出是否开启（set_log_dir enabled 参数）。"""
        return cls._file_output_enabled

    @classmethod
    def _timestamp(cls) -> str:
        return datetime.now().strftime("%Y%m%d_%H%M%S")

    @classmethod
    def get_default_controller_log_path(cls) -> str:
        """返回日志目录下的默认 controller log 文件路径。"""
        return os.path.join(cls._log_dir, f"sensor_controller_log_{cls._timestamp()}.txt")

    @classmethod
    def get_controller_log_path(cls) -> Optional[str]:
        """返回当前打开的 controller log 文件路径（未打开返回 None）。"""
        return cls._common_log_path

    @classmethod
    def get_default_profile_log_path(cls, prefix: str = "") -> str:
        """返回日志目录下的默认 profile log 文件路径。

        若提供 ``prefix``，则文件名为 ``{prefix}_log_YYYYMMDD_HHMMSS.txt``，
        否则为 ``log_YYYYMMDD_HHMMSS.txt``。
        """
        prefix_part = f"{prefix}_" if prefix else ""
        return os.path.join(cls._log_dir, f"{prefix_part}log_{cls._timestamp()}.txt")

    @classmethod
    def get_default_bin_path(cls, prefix: str = "") -> str:
        """返回日志目录下的默认 bin 导出文件路径。

        若提供 ``prefix``，则文件名为 ``{prefix}_data_YYYYMMDD_HHMMSS.bin``，
        否则为 ``data_YYYYMMDD_HHMMSS.bin``。
        """
        prefix_part = f"{prefix}_" if prefix else ""
        return os.path.join(cls._log_dir, f"{prefix_part}data_{cls._timestamp()}.bin")

    @classmethod
    def _ensure_dir(cls, path: str):
        if path:
            try:
                os.makedirs(path, exist_ok=True)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # 公共通道（controller log 文件）
    # ------------------------------------------------------------------
    @classmethod
    def _close_common_log(cls):
        if cls._common_listener is not None:
            try:
                cls._common_listener.stop()
            except Exception:
                pass
            cls._common_listener = None
        if cls._common_queue_handler is not None:
            for logger in (cls._logger, cls._controller_logger):
                try:
                    logger.removeHandler(cls._common_queue_handler)
                except Exception:
                    pass
            cls._common_queue_handler = None
        if cls._common_handler is not None:
            try:
                cls._common_handler.close()
            except Exception:
                pass
            cls._common_handler = None
        cls._common_log_path = None

    @classmethod
    def _open_common_log(cls, path: Optional[str] = None):
        """在日志目录创建 controller log 文件（已打开则先关闭再重建）。

        ``path`` 指定时打开该具体文件：主进程创建后把路径传给 BLE 子进程，
        两个进程写同一文件，而不是各自按当前秒生成不同文件名。
        """
        cls._close_common_log()
        if not cls._file_output_enabled:
            return
        path = path or cls.get_default_controller_log_path()
        cls._ensure_dir(os.path.dirname(path))
        try:
            handler = logging.FileHandler(path, encoding="utf-8")
            handler.setFormatter(cls._formatter())
            cls._common_handler = handler
            cls._common_log_path = path
            cls._common_listener = logging.handlers.QueueListener(
                cls._common_queue, handler, respect_handler_level=True
            )
            cls._common_listener.start()
            if cls._common_queue_handler is None:
                cls._common_queue_handler = logging.handlers.QueueHandler(cls._common_queue)
                cls._logger.addHandler(cls._common_queue_handler)
                cls._controller_logger.addHandler(cls._common_queue_handler)
            cls._replay_startup_buffer()
            cls._logger.info(f"sensor-sdk version: {cls._sdk_version()}")
        except Exception as e:
            cls._logger.warning(f"Failed to create controller log file {path}: {e}")

    @classmethod
    def _replay_startup_buffer(cls):
        """把 controller log 创建前缓存的公共记录写入文件（仅首次创建时回放一次）。"""
        if cls._startup_buffer_replayed:
            return
        cls._startup_buffer_replayed = True
        buf = cls._startup_buffer_handler
        buf.active = False
        if len(buf.buffer) >= cls._STARTUP_BUFFER_CAPACITY:
            cls._logger.warning("SDK startup log buffer overflowed, earliest logs were dropped")
        while buf.buffer:
            try:
                cls._common_queue.put(buf.buffer.popleft(), timeout=1.0)
            except queue.Full:
                break
        for logger in (cls._logger, cls._controller_logger):
            try:
                logger.removeHandler(buf)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # 调试 / data 日志开关
    # ------------------------------------------------------------------
    @classmethod
    def set_debug_enabled(cls, enabled: bool,
                          controller_log_path: Optional[str] = None):
        """开启或关闭 SDK 调试日志。

        开启时若文件输出已启用，自动在日志目录创建 controller log
        （``controller_log_path`` 指定时打开该具体文件，用于子进程与主进程
        写同一文件）；关闭时同时关闭 controller log。
        """
        cls._debug_enabled = bool(enabled)
        if cls._debug_enabled and cls._file_output_enabled:
            cls._open_common_log(controller_log_path)
        elif not cls._debug_enabled:
            cls._close_common_log()

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
    def _sdk_version(cls) -> str:
        """读取 sensor 包版本号（sensor/__init__.py 为单一数据源）。"""
        try:
            from sensor import __version__
            return __version__
        except Exception:
            return "unknown"

    # ------------------------------------------------------------------
    # profile 日志注册表
    # ------------------------------------------------------------------
    @classmethod
    def register_profile(cls, mac: str):
        """注册一个 profile（幂等），使其日志可被路由/缓存。"""
        if mac and mac not in cls._profiles:
            cls._profiles[mac] = _ProfileLogState(mac, cls._PROFILE_BUFFER_CAPACITY)

    @classmethod
    def _norm_mac(cls, mac: str) -> str:
        return mac.replace(":", "").replace("-", "").upper()

    @classmethod
    def _close_profile_handler(cls, state: _ProfileLogState):
        if state.listener is not None:
            try:
                state.listener.stop()
            except Exception:
                pass
            state.listener = None
        if state.queue_handler is not None:
            try:
                state.logger.removeHandler(state.queue_handler)
            except Exception:
                pass
            state.queue_handler = None
        if state.handler is not None:
            try:
                state.handler.close()
            except Exception:
                pass
            state.handler = None
        state.queue = None
        state.path = None

    @classmethod
    def enable_profile_log(cls, mac: str, path: Optional[str] = None) -> Optional[str]:
        """开启指定 profile 的日志文件，返回实际文件路径（失败返回 None）。

        Args:
            mac: profile 的设备 MAC。
            path: 日志文件路径；``None`` 时使用日志目录下的默认命名
                （文件输出关闭时默认导出被禁用，返回 None）。
        """
        if not mac:
            return None
        cls.register_profile(mac)
        state = cls._profiles[mac]
        cls._close_profile_handler(state)

        if path is None:
            if not cls._file_output_enabled:
                cls._logger.warning(f"[SdkLog] default profile log disabled (file output off): {mac}")
                return None
            path = cls.get_default_profile_log_path(cls._norm_mac(mac))

        cls._ensure_dir(os.path.dirname(path))
        try:
            handler = logging.FileHandler(path, encoding="utf-8")
            handler.setFormatter(cls._formatter())
            log_queue: queue.Queue = queue.Queue(maxsize=cls._LOG_QUEUE_MAXSIZE)
            listener = logging.handlers.QueueListener(
                log_queue, handler, respect_handler_level=True
            )
            listener.start()
            queue_handler = logging.handlers.QueueHandler(log_queue)
            state.logger.addHandler(queue_handler)
            state.handler = handler
            state.listener = listener
            state.queue = log_queue
            state.queue_handler = queue_handler
            state.path = path
            # 回放开启前缓存的该 profile 记录（connect 阶段日志不丢）
            while state.buffer:
                try:
                    log_queue.put(state.buffer.popleft(), timeout=1.0)
                except queue.Full:
                    break
            state.logger.info(f"sensor-sdk version: {cls._sdk_version()}")
            return path
        except Exception as e:
            cls._logger.warning(f"Failed to create profile log file {path}: {e}")
            return None

    @classmethod
    def disable_profile_log(cls, mac: str):
        """关闭指定 profile 的日志文件，后续记录回落公共通道。"""
        state = cls._profiles.get(mac)
        if state is not None:
            cls._close_profile_handler(state)

    @classmethod
    def get_profile_log_path(cls, mac: str) -> Optional[str]:
        """返回指定 profile 当前日志文件路径，未开启返回 None。"""
        state = cls._profiles.get(mac)
        return state.path if state is not None else None

    # ------------------------------------------------------------------
    # 记录路由
    # ------------------------------------------------------------------
    @classmethod
    def _emit(cls, level: int, tag: str, msg: str, mac: Optional[str] = None,
              exc_info: bool = False):
        text = f"[{tag}] {msg}"
        if mac:
            state = cls._profiles.get(mac)
            if state is not None:
                if state.handler is not None:
                    # profile log 已开启：只进 profile 文件
                    state.logger.log(level, text, exc_info=exc_info)
                    return
                # 未开启：缓存等待回放，同时回落公共通道
                try:
                    record = state.logger.makeRecord(
                        state.logger.name, level, "", 0, text, None,
                        sys.exc_info() if exc_info else None,
                    )
                    state.buffer.append(record)
                except Exception:
                    pass
        cls._logger.log(level, text, exc_info=exc_info)

    @classmethod
    def route_external(cls, tag: str, msg: str, level: int = logging.INFO):
        """第三方（bleak/bumble）日志路由：文本含已注册 profile 的 mac 时
        路由到该 profile log（与该 profile 当前连接相关），否则进公共通道。"""
        norm = msg.replace(":", "").replace("-", "").upper()
        for mac in cls._profiles:
            norm_mac = cls._norm_mac(mac)
            if norm_mac and norm_mac in norm:
                cls._emit(level, tag, msg, mac=mac)
                return
        cls._emit(level, tag, msg)

    @classmethod
    def bind(cls, mac: str) -> _BoundSdkLog:
        """返回绑定指定 profile mac 的日志转发器（并注册该 profile）。"""
        cls.register_profile(mac)
        return _BoundSdkLog(mac)

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    @classmethod
    def stop(cls):
        """停止所有异步日志监听器，确保日志 flush 到文件。

        程序退出时建议调用一次，避免队列中的日志丢失。
        """
        cls._close_common_log()
        for state in cls._profiles.values():
            cls._close_profile_handler(state)

    # ------------------------------------------------------------------
    # 输出方法（mac 为空 -> 公共通道；mac 非空 -> profile 路由）
    # ------------------------------------------------------------------
    @classmethod
    def controller(cls, tag: str, msg: str):
        """输出 SensorController 公共日志。"""
        cls._controller_logger.info(f"[{tag}] {msg}")

    @classmethod
    def d(cls, tag: str, msg: str, mac: Optional[str] = None):
        """输出 debug 日志（可被 set_debug_enabled 关闭）。"""
        if cls._debug_enabled:
            cls._emit(logging.DEBUG, tag, msg, mac)

    @classmethod
    def data(cls, tag: str, msg: str, mac: Optional[str] = None):
        """输出 data 日志（记录蓝牙数据包与解析结果）。

        仅当 ``set_data_log_enabled(True)`` 时输出，默认关闭。
        """
        if cls._data_log_enabled:
            cls._emit(DATA_LOG_LEVEL, tag, msg, mac)

    @classmethod
    def i(cls, tag: str, msg: str, mac: Optional[str] = None):
        """输出 info 日志。"""
        cls._emit(logging.INFO, tag, msg, mac)

    @classmethod
    def w(cls, tag: str, msg: str, mac: Optional[str] = None):
        """输出 warning 日志。"""
        cls._emit(logging.WARNING, tag, msg, mac)

    @classmethod
    def e(cls, tag: str, msg: str, mac: Optional[str] = None):
        """输出 error 日志。"""
        cls._emit(logging.ERROR, tag, msg, mac)

    @classmethod
    def exception(cls, tag: str, msg: str = "", mac: Optional[str] = None):
        """输出 exception 日志，包含当前异常堆栈。"""
        cls._emit(logging.ERROR, tag, msg, mac, exc_info=True)

    @classmethod
    def log(cls, level: str, tag: str, msg: str, mac: Optional[str] = None):
        """按级别字符串输出日志：level 取 "D"/"I"/"W"/"E"（大小写不敏感，
        其他值按 info 处理）。"""
        lv = (level or "I").strip().upper()[:1]
        if lv == "D":
            cls.d(tag, msg, mac)
        elif lv == "W":
            cls.w(tag, msg, mac)
        elif lv == "E":
            cls.e(tag, msg, mac)
        else:
            cls.i(tag, msg, mac)


# 程序正常退出时自动停止日志监听器，尽量保证队列中的日志落盘
atexit.register(SdkLog.stop)
