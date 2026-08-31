import asyncio
import os
import queue
import shutil
import struct
import platform
import tempfile
import threading
import time
from contextlib import suppress
from datetime import datetime
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Dict, List
import logging

import numpy as np
from bleak import (
    BleakScanner,
    BLEDevice,
    AdvertisementData,
    BleakClient,
    BleakGATTCharacteristic,
)

from sensor import sensor_utils
from sensor.bin_recorder import BIN_RECORD_CMD_RECV, BIN_RECORD_CMD_SEND, BIN_RECORD_DATA, BIN_RECORD_EVENT, BinRecordWriter, format_mac, read_bin_record_at
from sensor.sensor_device import BLEChipType

from sensor.sdk_log import SdkLog
_TAG = "GForce"

# 队列满时等待入队的超时时间（秒）；超时仍未入队的包由恢复线程从 bin 文件读回补入
_QUEUE_FULL_WAIT_TIMEOUT = 1.0
# 待恢复的 bin 记录偏移队列上限，防止长期积压撑爆内存
_BIN_RECOVERY_OFFSET_MAX = 10000
# 打开 bin 记录所需的最低磁盘剩余空间，低于该值跳过记录
_BIN_MIN_FREE_BYTES = 100 * 1024 * 1024  # 100MB
# bin 文件写入不可用（打开失败/磁盘错误）时，退化为内存缓存的最大包数，超出丢弃最旧包
_MEM_FALLBACK_QUEUE_MAX = 500


@dataclass
class Characteristic:
    uuid: str
    service_uuid: str
    descriptor_uuids: List[str]


class Command(IntEnum):
    GET_PROTOCOL_VERSION = (0x00,)
    GET_FEATURE_MAP = (0x01,)
    GET_DEVICE_NAME = (0x02,)
    GET_MODEL_NUMBER = (0x03,)
    GET_SERIAL_NUMBER = (0x04,)
    GET_HW_REVISION = (0x05,)
    GET_FW_REVISION = (0x06,)
    GET_MANUFACTURER_NAME = (0x07,)
    GET_BOOTLOADER_VERSION = (0x0A,)

    GET_BATTERY_LEVEL = (0x08,)
    GET_TEMPERATURE = (0x09,)

    POWEROFF = (0x1D,)
    SWITCH_TO_OAD = (0x1E,)
    SYSTEM_RESET = (0x1F,)
    SWITCH_SERVICE = (0x20,)

    SET_LOG_LEVEL = (0x21,)
    SET_LOG_MODULE = (0x22,)
    PRINT_KERNEL_MSG = (0x23,)
    MOTOR_CONTROL = (0x24,)
    LED_CONTROL_TEST = (0x25,)
    PACKAGE_ID_CONTROL = (0x26,)
    SEND_TRAINING_PACKAGE = (0x27,)

    GET_ACCELERATE_CAP = (0x30,)
    SET_ACCELERATE_CONFIG = (0x31,)

    GET_GYROSCOPE_CAP = (0x32,)
    SET_GYROSCOPE_CONFIG = (0x33,)

    GET_MAGNETOMETER_CAP = (0x34,)
    SET_MAGNETOMETER_CONFIG = (0x35,)

    GET_EULER_ANGLE_CAP = (0x36,)
    SET_EULER_ANGLE_CONFIG = (0x37,)

    QUATERNION_CAP = (0x38,)
    QUATERNION_CONFIG = (0x39,)

    GET_ROTATION_MATRIX_CAP = (0x3A,)
    SET_ROTATION_MATRIX_CONFIG = (0x3B,)

    GET_GESTURE_CAP = (0x3C,)
    SET_GESTURE_CONFIG = (0x3D,)
    GET_GESTURE_THRESHOLD = (0x47,)
    SET_GESTURE_THRESHOLD = (0x48,)

    GET_EMG_RAWDATA_CAP = (0x3E,)
    SET_EMG_RAWDATA_CONFIG = (0x3F,)

    GET_MOUSE_DATA_CAP = (0x40,)
    SET_MOUSE_DATA_CONFIG = (0x41,)

    GET_JOYSTICK_DATA_CAP = (0x42,)
    SET_JOYSTICK_DATA_CONFIG = (0x43,)

    GET_DEVICE_STATUS_CAP = (0x44,)
    SET_DEVICE_STATUS_CONFIG = (0x45,)

    GET_EMG_RAWDATA_CONFIG = (0x46,)

    SET_DATA_NOTIF_SWITCH = (0x4F,)
    SET_FUNCTION_SWITCH = (0x85,)
    CMD_SET_NEUCIR_STATUS = (0x87,)
    CMD_SET_APP_REMOTE_CMD = (0x89,)

    CMD_GET_EEG_CONFIG = (0xA0,)
    CMD_SET_EEG_CONFIG = (0xA1,)
    CMD_GET_ECG_CONFIG = (0xA2,)
    CMD_SET_ECG_CONFIG = (0xA3,)
    CMD_GET_IMPEDANCE_CONFIG = (0xA4,)
    CMD_SET_IMPEDANCE_CONFIG = (0xA5,)
    CMD_GET_EEG_CAP = (0xA6,)
    CMD_GET_ECG_CAP = (0xA7,)
    CMD_GET_IMPEDANCE_CAP = (0xA8,)
    CMD_GET_IMU_CONFIG = (0xAC,)
    CMD_GET_IMU_CAP = (0xAB,)
    CMD_SET_IMU_CONFIG = (0xAD,)
    CMD_GET_BLE_MTU_INFO = (0xAE,)
    CMD_GET_BRT_CONFIG = (0xB3,)
    CMD_GET_PPG_CAP = (0xB5,)
    CMD_GET_PPG_CONFIG = (0xB6,)
    CMD_SET_PPG_CONFIG = (0xB7,)

    CMD_SET_FRIMWARE_FILTER_SWITCH = (0xAA,)
    CMD_GET_FRIMWARE_FILTER_SWITCH = (0xA9,)
    # Partial command packet, format: [CMD_PARTIAL_DATA, packet number in reverse order, packet content]
    MD_PARTIAL_DATA = 0xFF


class DataSubscription(IntEnum):
    # Data Notify All Off
    OFF = (0x00000000,)

    # Accelerate On(C.7)
    ACCELERATE = (0x00000001,)

    # Gyroscope On(C.8)
    GYROSCOPE = (0x00000002,)

    # Magnetometer On(C.9)
    MAGNETOMETER = (0x00000004,)

    # Euler Angle On(C.10)
    EULERANGLE = (0x00000008,)

    # Quaternion On(C.11)
    QUATERNION = (0x00000010,)

    # Rotation Matrix On(C.12)
    ROTATIONMATRIX = (0x00000020,)

    # EMG Gesture On(C.13)
    EMG_GESTURE = (0x00000040,)

    # EMG Raw Data On(C.14)
    EMG_RAW = (0x00000080,)

    # HID Mouse On(C.15)
    HID_MOUSE = (0x00000100,)

    # HID Joystick On(C.16)
    HID_JOYSTICK = (0x00000200,)

    # Device Status On(C.17)
    DEVICE_STATUS = (0x00000400,)

    # Device Log On
    LOG = (0x00000800,)

    DNF_TYPE_GEST_EXT = (0x00001000,)

    DNF_MAG_ANGLE_EXT = (0x00002000,)
    
    DNF_EEG = (0x00010000,)

    DNF_ECG = (0x00020000,)

    DNF_IMPEDANCE = (0x00040000,)

    DNF_IMU = (0x00080000,)

    DNF_ADS = (0x00100000,)

    DNF_BRTH = (0x00200000,)

    DNF_CONCAT_BLE = (0x80000000,)
    DNF_PPG = (0x00400000,)
    # Data Notify All On
    ALL = 0xFFFFFFFF


class DataType(IntEnum):
    ACC = (0x01,)
    GYO = (0x02,)
    MAG = (0x03,)
    EULER = (0x04,)
    QUAT = (0x05,)
    ROTA = (0x06,)
    EMG_GEST = (0x07,)
    EMG_ADC = (0x08,)
    HID_MOUSE = (0x09,)
    HID_JOYSTICK = (0x0A,)
    DEV_STATUS = (0x0B,)
    LOG = (0x0C,)

    PARTIAL = 0xFF


class SampleResolution(IntEnum):
    BITS_8 = (8,)
    BITS_12 = (12,)
    BITS_16 = (16,)
    BITS_24 = 24


class SamplingRate(IntEnum):
    HZ_50 = (50,)
    HZ_100 = (100,)
    Hz_200 = (200,)
    HZ_250 = (250,)
    HZ_400 = (400,)
    HZ_500 = (500,)
    HZ_650 = (650,)
    HZ_1000 = (1000,)
    HZ_2000 = (2000,)


# EEG/ECG cap 响应 fs 字节的可选采样率位掩码：bit0=250Hz，bit1=500Hz，
# bit2=1000Hz，bit3=2000Hz（OB6000 固件约定，Cerelax-Ultra 实测上报 0x0F；
# EEG 与 ECG 采样率绑定，共用同一张表）
CAP_FS_BITMASK_RATES = (250, 500, 1000, 2000)


def decode_cap_fs_bitmask(fs_mask: int):
    """把 EEG/ECG cap 响应的 fs 位掩码解码为支持的采样率列表（Hz，升序）。"""
    try:
        mask = int(fs_mask)
    except (TypeError, ValueError):
        return []
    return [rate for bit, rate in enumerate(CAP_FS_BITMASK_RATES) if mask & (1 << bit)]


@dataclass
class EmgRawDataConfig:
    fs: SamplingRate = SamplingRate.HZ_500
    channel_mask: int = 0xFF
    batch_len: int = 16
    resolution: SampleResolution = SampleResolution.BITS_8

    def to_bytes(self) -> bytes:
        body = b""
        body += struct.pack("<H", self.fs)
        body += struct.pack("<H", self.channel_mask)
        body += struct.pack("<B", self.batch_len)
        body += struct.pack("<B", self.resolution)
        return body

    @classmethod
    def from_bytes(cls, data: bytes):
        fs, channel_mask, batch_len, resolution = struct.unpack(
            "<HHBB",
            data,
        )
        return cls(fs, channel_mask, batch_len, resolution)


@dataclass
class EmgRawDataCap:
    fs: SamplingRate = 0
    channel_mask: int = 0
    batch_len: int = 0
    resolution: SampleResolution = 0

    def to_bytes(self) -> bytes:
        body = b""
        body += struct.pack("<H", self.fs)
        body += struct.pack("<H", self.channel_mask)
        body += struct.pack("<B", self.batch_len)
        body += struct.pack("<B", self.resolution)
        return body

    @classmethod
    def from_bytes(cls, data: bytes):
        fs, channel_mask, batch_len, resolution = struct.unpack(
            "<HHBB",
            data,
        )
        return cls(fs, channel_mask, batch_len, resolution)


@dataclass
class EegRawDataConfig:
    fs: SamplingRate = 0
    channel_mask: int = 0
    batch_len: int = 0
    resolution: SampleResolution = 0
    K: float = 0

    def to_bytes(self) -> bytes:
        body = b""
        body += struct.pack("<H", self.fs)
        body += struct.pack("<Q", self.channel_mask)
        body += struct.pack("<B", self.batch_len)
        body += struct.pack("<B", self.resolution)
        body += struct.pack("<d", self.K)
        return body

    @classmethod
    def from_bytes(cls, data: bytes):
        fs, channel_mask, batch_len, resolution, K = struct.unpack(
            "<HQBBd",
            data,
        )
        return cls(fs, channel_mask, batch_len, resolution, K)


@dataclass
class EegRawDataCap:
    fs: SamplingRate = 0
    channel_count: int = 0
    batch_len: int = 0
    resolution: SampleResolution = 0

    def to_bytes(self) -> bytes:
        body = b""
        body += struct.pack("<B", self.fs)
        body += struct.pack("<B", self.channel_count)
        body += struct.pack("<B", self.batch_len)
        body += struct.pack("<B", self.resolution)
        return body

    @classmethod
    def from_bytes(cls, data: bytes):
        fs, channel_count, batch_len, resolution = struct.unpack(
            "<BBBB",
            data,
        )
        return cls(fs, channel_count, batch_len, resolution)


@dataclass
class EcgRawDataConfig:
    fs: SamplingRate = SamplingRate.HZ_250
    channel_mask: int = 0
    batch_len: int = 16
    resolution: SampleResolution = SampleResolution.BITS_24
    K: float = 0

    def to_bytes(self) -> bytes:
        body = b""
        body += struct.pack("<H", self.fs)
        body += struct.pack("<H", self.channel_mask)
        body += struct.pack("<B", self.batch_len)
        body += struct.pack("<B", self.resolution)
        body += struct.pack("<d", self.K)
        return body

    @classmethod
    def from_bytes(cls, data: bytes):
        fs, channel_mask, batch_len, resolution, K = struct.unpack(
            "<HHBBd",
            data,
        )
        return cls(fs, channel_mask, batch_len, resolution, K)


@dataclass
class EcgRawDataCap:
    fs: SamplingRate = 0
    channel_count: int = 0
    batch_len: int = 0
    resolution: SampleResolution = 0

    def to_bytes(self) -> bytes:
        body = b""
        body += struct.pack("<B", self.fs)
        body += struct.pack("<B", self.channel_count)
        body += struct.pack("<B", self.batch_len)
        body += struct.pack("<B", self.resolution)
        return body

    @classmethod
    def from_bytes(cls, data: bytes):
        fs, channel_count, batch_len, resolution = struct.unpack(
            "<BBBB",
            data,
        )
        return cls(fs, channel_count, batch_len, resolution)


@dataclass
class ImuRawDataConfig:
    channel_count: int = 0
    fs: SamplingRate = 0
    batch_len: int = 0
    accK: float = 0
    gyroK: float = 0

    def to_bytes(self) -> bytes:
        """生成 CMD_SET_IMU_CONFIG 命令体，与 Android setImuDataConfig 协议一致。

        协议格式：channel_count(int32) + sample_rate(uint16) + sample_count(uint8)
        """
        body = b""
        body += struct.pack("<i", self.channel_count)
        body += struct.pack("<H", self.fs)
        body += struct.pack("<B", self.batch_len)
        return body

    @classmethod
    def from_bytes(cls, data: bytes):
        channel_count, fs, batch_len, accK, gyroK = struct.unpack(
            "<iHBdd",
            data,
        )
        return cls(channel_count, fs, batch_len, accK, gyroK)


@dataclass
class BrthRawDataConfig:
    fs: SamplingRate = 0
    channel_mask: int = 0
    batch_len: int = 0
    resolution: SampleResolution = 0
    K: float = 0

    def to_bytes(self) -> bytes:
        body = b""
        body += struct.pack("<H", self.fs)
        body += struct.pack("<H", self.channel_mask)
        body += struct.pack("<B", self.batch_len)
        body += struct.pack("<B", self.resolution)
        body += struct.pack("<d", self.K)
        return body

    @classmethod
    def from_bytes(cls, data: bytes):
        fs, channel_mask, batch_len, resolution, K = struct.unpack(
            "<HHBBd",
            data,
        )
        return cls(fs, channel_mask, batch_len, resolution, K)


@dataclass
class PpgRawDataConfig:
    mode: int = 0
    period: int = 0
    fs: int = 0  # rawSampleRate
    batch_len: int = 0  # rawSampleCount
    reserved: List[int] = None  # 5 bytes reserved

    def __post_init__(self):
        if self.reserved is None:
            self.reserved = [0] * 5

    def to_bytes(self) -> bytes:
        body = b""
        body += struct.pack("<B", self.mode)
        body += struct.pack("<H", self.period)
        body += struct.pack("<H", self.fs)
        body += struct.pack("<B", self.batch_len)
        # Add 5 reserved bytes
        for i in range(5):
            body += struct.pack("<B", self.reserved[i] if i < len(self.reserved) else 0)
        # Add 1 padding byte to make it 12 bytes total
        body += struct.pack("<B", 0)
        return body

    @classmethod
    def from_bytes(cls, data: bytes):
        if len(data) < 12:
            raise ValueError(f"PPG config data too short: {len(data)} bytes, expected 12")
        mode = data[0]
        period = struct.unpack("<H", data[1:3])[0]
        fs = struct.unpack("<H", data[3:5])[0]
        batch_len = data[5]
        reserved = list(data[6:11])
        return cls(mode, period, fs, batch_len, reserved)


@dataclass
class Request:
    cmd: Command
    has_res: bool
    body: Optional[bytes] = None


class ResponseCode(IntEnum):
    SUCCESS = (0x00,)
    NOT_SUPPORT = (0x01,)
    BAD_PARAM = (0x02,)
    FAILED = (0x03,)
    TIMEOUT = (0x04,)
    PARTIAL_PACKET = 0xFF


@dataclass
class Response:
    code: ResponseCode
    cmd: Command
    data: bytes


class GForce:
    def __init__(
        self,
        device: BLEDevice,
        cmd_char: str,
        data_char: str,
        isUniversalStream: bool,
        event_loop: asyncio.AbstractEventLoop,
        gforce_event_loop: asyncio.AbstractEventLoop,
        chip_type: BLEChipType = BLEChipType.Unknown,
        client_kwargs: dict = None,
        device_mac: Optional[str] = None,
    ):
        # 绑定 profile 日志（优先用调用方给的统一注册 MAC；缺省回退 bleak
        # device.address——macOS 原生后端下是 UUID，仅作兜底）
        self._log = SdkLog.bind(device_mac or device.address)
        self.device_name = ""
        self.client = None
        self.event_loop = event_loop
        self.gforce_event_loop = gforce_event_loop
        self.cmd_char = cmd_char
        self.data_char = data_char
        self.responses: Dict[Command, queue.Queue] = {}
        # 每命令一把 asyncio 串行锁（仅在 gforce 事件循环内使用，不阻塞任何线程）：
        # 协议无事务 ID，同 cmd 并发请求无法区分响应归属，并发时响应队列竞态
        # 会使等待者连锁饿死；用 threading.Lock 则会在单线程事件循环内死锁
        self._cmd_locks: Dict[Command, asyncio.Lock] = {}
        self.last_command_failure_time: Optional[float] = None
        self.last_command_failure_cmd: Optional[int] = None
        self.resolution = SampleResolution.BITS_8
        self._num_channels = 8
        self._device = device
        self._is_universal_stream = isUniversalStream
        self._chip_type = chip_type
        # bleak_bumble 后端时携带 backend/cfg/host_mode，原生后端为空 dict
        self._client_kwargs = client_kwargs or {}
        self._raw_data_buf: queue.Queue[bytes] = None
        self.packet_id = 0
        self.data_packet = []

        # 起流时刻（32 位毫秒）与首包 delay 上报：
        # on_stream_start_ts(ts_ms, wall_ms) 在起流写完成后触发，取 bin 起流记录
        # （OYM stream_start 事件 / RFSTAR cmd_send）的时间戳，与回放还原值一致；
        # ts_ms 为低 32 位（delay 计算用），wall_ms 为完整墙钟毫秒（LSL 锚点用）；
        # on_first_packet_delay(delay_ms) 在起流后首个原始数据包到达时触发
        self.on_stream_start_ts = None
        self.on_first_packet_delay = None
        self._stream_start_ts_ms = 0
        self._stream_start_wall_ms = 0
        self._await_first_packet = False

        # 原始数据 bin 记录器：连接成功后打开，保存目录与日志目录一致
        self._bin_writer: Optional[BinRecordWriter] = None
        # 已知的 bin 导出路径（DEBUG_BLE_DATA_PATH，connect 命令捎来或
        # setParam 时由 switch_bin_export 更新）：非空时 _open_bin_recorder
        # 直接在导出文件上追加记录，不再先写 temp 再拷贝
        self._bin_export_path_hint: Optional[str] = None
        # 当前 writer 是否为直写导出文件（finalize 时只关闭，不拷贝不删除）
        self._bin_writer_direct = False
        # 队列满等待超时后仍未入队的记录偏移，由恢复线程从 bin 文件读回补入队列
        self._bin_dropped_offsets: "queue.Queue[int]" = queue.Queue(maxsize=_BIN_RECOVERY_OFFSET_MAX)
        # bin 文件写入不可用（打开失败/磁盘错误）时的降级缓存：
        # 队列满时未入队的包暂存于此（最多 500 条，超出丢弃最旧包），由恢复线程补入
        self._mem_dropped: "queue.Queue[bytes]" = queue.Queue(maxsize=_MEM_FALLBACK_QUEUE_MAX)
        self._mem_drop_last_log = 0.0
        self._bin_recovery_stop = threading.Event()
        self._bin_recovery_thread: Optional[threading.Thread] = None

    def get_chip_type(self) -> BLEChipType:
        return self._chip_type

    # ------------------------------------------------------------------
    # 原始数据 bin 记录
    # ------------------------------------------------------------------
    def _open_bin_recorder(self):
        """打开 bin 记录文件，失败不影响正常数据流。

        已知导出路径（_bin_export_path_hint，来自 connect 命令或此前的
        DEBUG_BLE_DATA_PATH setParam）时直接在导出文件上追加记录；否则写在
        系统 temp 目录，finalize 时再拷贝导出。无论 bin 记录是否开启成功，
        都会启动恢复线程：bin 不可用时的 500 条内存降级缓存同样依赖它补回
        数据包。
        """
        self.finalize_bin_recorder()
        self._start_bin_recovery()
        hint = self._bin_export_path_hint
        if hint:
            try:
                directory = os.path.dirname(hint) or "."
                try:
                    free_bytes = shutil.disk_usage(directory).free
                    if free_bytes < _BIN_MIN_FREE_BYTES:
                        raise OSError(f"磁盘剩余空间不足 ({free_bytes // 1024 // 1024}MB)")
                except OSError:
                    raise
                except Exception:
                    pass  # 检查磁盘空间失败时仍尝试打开
                self._bin_writer = BinRecordWriter(hint, append=True)
                self._bin_writer_direct = True
                self._log.i(_TAG, f"Bin recorder opened directly at export path: {hint}")
                return
            except Exception as e:
                self._log.w(_TAG, f"直写导出路径失败，回退 temp bin: {hint}: {e}")
        try:
            log_dir = tempfile.gettempdir()
            # 磁盘剩余空间不足时不开启 bin 记录
            try:
                free_bytes = shutil.disk_usage(log_dir).free
                if free_bytes < _BIN_MIN_FREE_BYTES:
                    self._log.w(_TAG, f"磁盘剩余空间不足 ({free_bytes // 1024 // 1024}MB)，跳过 bin 记录")
                    return
            except Exception as e:
                self._log.w(_TAG, f"检查磁盘空间失败，仍尝试开启 bin 记录: {e}")
            name = self.device_name or getattr(self._device, "name", "") or "unknown"
            safe_name = "".join(c if (c.isalnum() or c in "-_") else "_" for c in name)
            # Windows 文件名不允许 ':'，MAC 用 '-' 分隔
            mac = format_mac(getattr(self._device, "address", ""), sep="-") or "unknown"
            safe_mac = "".join(c if (c.isalnum() or c in "-") else "_" for c in mac)
            path = os.path.join(log_dir, f"{safe_name}_{safe_mac}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.bin")
            self._bin_writer = BinRecordWriter(path)
            self._bin_writer_direct = False
            self._log.i(_TAG, f"Bin recorder opened: {path}")
        except Exception as e:
            self._log.exception(_TAG, f"Failed to open bin recorder: {e}")
            self._bin_writer = None
            self._bin_writer_direct = False

    def close_bin_recorder(self):
        """停止 bin 恢复线程，关闭 bin 记录文件并 flush。"""
        self._bin_recovery_stop.set()
        thread = self._bin_recovery_thread
        if thread is not None and thread.is_alive():
            try:
                thread.join(timeout=2.0)
            except Exception:
                pass
        self._bin_recovery_thread = None
        self._bin_recovery_stop.clear()
        if self._bin_writer is not None:
            try:
                self._bin_writer.close()
            except Exception:
                pass
            self._bin_writer = None

    def finalize_bin_recorder(self, export_path: Optional[str] = None):
        """结束本次 bin 记录：flush 关闭后按需导出并删除 temp 原文件（幂等）。

        bin 默认写在系统 temp 目录；export_path 非空（DEBUG_BLE_DATA_PATH
        设置的导出位置）时先把 bin 拷贝过去（文件已存在则追加，续上上次
        会话的 bin——bin 记录格式可顺序拼接，且每段起流时都会重写配置记录，
        各段自描述），再删除 temp 原文件。writer 为直写导出文件
        （_bin_writer_direct，路径提前已知时 _open_bin_recorder 直接打开
        导出文件）时只关闭，不拷贝不删除。
        """
        writer = self._bin_writer
        path = writer.path if writer is not None else None
        direct = self._bin_writer_direct
        self._log.d(_TAG, f"finalize bin recorder: temp={path}, export={export_path or 'disabled'}"
                          f"{', direct' if direct else ''}")
        self.close_bin_recorder()
        self._bin_writer_direct = False
        if not path:
            return
        if direct:
            # 数据已直接写在导出文件内，无需拷贝，也不能删除
            return
        if export_path:
            try:
                directory = os.path.dirname(export_path)
                if directory:
                    os.makedirs(directory, exist_ok=True)
                if os.path.exists(export_path) and os.path.getsize(export_path) > 0:
                    # 目标文件已存在：追加，续上上次会话的 bin
                    with open(path, "rb") as src, open(export_path, "ab") as dst:
                        shutil.copyfileobj(src, dst)
                    self._log.i(_TAG, f"Bin appended to: {export_path}")
                else:
                    shutil.copyfile(path, export_path)
                    self._log.i(_TAG, f"Bin exported to: {export_path}")
            except Exception as e:
                self._log.w(_TAG, f"Bin export failed: {export_path}: {e}")
        try:
            os.remove(path)
        except Exception:
            pass

    def switch_bin_export(self, export_path: Optional[str] = None):
        """更新 bin 导出路径（DEBUG_BLE_DATA_PATH setParam 时调用）。

        export_path 非空：之后直接在导出文件上追加记录——当前还有 temp
        writer 开着时，先把 temp 段内容并入导出文件再切换，避免整段
        「tmp 写入后移动」；为 None：恢复 temp 记录（直写段已落导出文件，
        直接保留）。当前没有打开的 writer 时只记路径，下次
        _open_bin_recorder 生效。
        """
        export_path = export_path or None
        self._bin_export_path_hint = export_path
        writer = self._bin_writer
        if writer is None:
            return
        current = writer.path
        was_direct = self._bin_writer_direct
        if export_path is None and not was_direct:
            return  # temp 记录中且导出关闭：无需切换
        if export_path is not None and was_direct \
                and os.path.abspath(export_path) == os.path.abspath(current):
            return  # 已直写同一文件
        # 关闭当前段（回填头部记录、停恢复线程）
        self.close_bin_recorder()
        self._bin_writer_direct = False
        if not was_direct:
            if export_path is not None:
                # temp 段并入导出文件（通常为连接阶段的一小段）
                try:
                    directory = os.path.dirname(export_path)
                    if directory:
                        os.makedirs(directory, exist_ok=True)
                    with open(current, "rb") as src, open(export_path, "ab") as dst:
                        shutil.copyfileobj(src, dst)
                    self._log.i(_TAG, f"Bin appended to: {export_path}")
                except Exception as e:
                    self._log.w(_TAG, f"Bin export failed: {export_path}: {e}")
            try:
                os.remove(current)
            except Exception:
                pass
        # 恢复队列中待补的偏移指向旧文件/旧位置，已失效，丢弃
        while not self._bin_dropped_offsets.empty():
            try:
                self._bin_dropped_offsets.get_nowait()
            except Exception:
                break
        # 开启新一段：hint 非空直写导出文件，否则回 temp
        self._open_bin_recorder()

    def write_bin_config(self, config: dict):
        """把解析配置写入 bin 文件（init 成功后调用），供离线回放恢复上下文。"""
        writer = self._bin_writer
        if writer is not None:
            writer.write_config(config)

    def _write_bin_data(self, data: bytes, ts_ms: Optional[int] = None,
                        perf_ns: Optional[int] = None) -> Optional[int]:
        writer = self._bin_writer
        if writer is not None:
            return writer.write_record(BIN_RECORD_DATA, data, ts_ms=ts_ms, perf_ns=perf_ns)
        return None

    def _write_bin_record(self, record_type: int, payload: bytes,
                          perf_ns: Optional[int] = None,
                          ts_ms: Optional[int] = None) -> int:
        """写一条指定类型的 bin 记录（命令收发等），返回记录使用的时间戳（ms）。

        ``perf_ns``（bumble 层 perf_counter_ns 打点）不为 None 时同步写入
        一条紧邻前置的 0x07 高精度时间戳记录。``ts_ms`` 不为 None 时作为
        记录时间戳（bumble 层发送时刻的 wall ms），否则现场打点。
        """
        if ts_ms is None:
            ts_ms = int(time.time() * 1000)
        writer = self._bin_writer
        if writer is not None:
            writer.write_record(record_type, payload, ts_ms=ts_ms, perf_ns=perf_ns)
        return ts_ms

    def log_bin_event(self, name: str, perf_ns: Optional[int] = None,
                      ts_ms: Optional[int] = None) -> int:
        """把蓝牙事件（connect/disconnect/stream_start/stream_stop 等）写入 bin，返回记录时间戳（ms）。"""
        return self._write_bin_record(BIN_RECORD_EVENT, name.encode("utf-8"), perf_ns, ts_ms)

    # ------------------------------------------------------------------
    # bin 恢复：队列满时未入队的数据包，之后从 bin 文件读回补入队列
    # ------------------------------------------------------------------
    def _start_bin_recovery(self):
        if self._bin_recovery_thread is not None:
            return
        self._bin_recovery_stop.clear()
        self._bin_recovery_thread = threading.Thread(
            target=self._bin_recovery_loop,
            name=f"BinRecovery-{self.device_name or 'unknown'}",
            daemon=True,
        )
        self._bin_recovery_thread.start()

    def _queue_bin_recovery(self, offset: Optional[int]):
        if offset is None:
            return
        try:
            self._bin_dropped_offsets.put_nowait(offset)
        except queue.Full:
            self._log.w(_TAG, "Bin recovery offset queue full, skip recovery for one packet")

    def _queue_mem_recovery(self, data: bytes):
        """bin 不可用时的降级缓存：最多 500 条，满时丢弃最旧的一条。"""
        try:
            self._mem_dropped.put_nowait(data)
        except queue.Full:
            try:
                self._mem_dropped.get_nowait()
            except queue.Empty:
                pass
            try:
                self._mem_dropped.put_nowait(data)
            except queue.Full:
                pass
            now = time.time()
            if now - self._mem_drop_last_log >= 2.0:
                self._mem_drop_last_log = now
                self._log.w(_TAG, "Memory fallback cache full, dropping oldest packet")

    def _recover_payload(self, q: queue.Queue, payload: bytes):
        """把补回的数据包等待放入原始队列；停止时放弃。"""
        while not self._bin_recovery_stop.is_set():
            try:
                q.put(payload, timeout=0.5)
                self._log.i(_TAG, "Recovered one packet into raw queue")
                return
            except queue.Full:
                continue

    def _bin_recovery_loop(self):
        """把队列满时未能入队的数据包补回原始队列。

        优先处理 bin 不可用时的内存降级缓存；否则按偏移从 bin 文件读回补入。
        """
        reader = None
        try:
            while not self._bin_recovery_stop.is_set():
                # bin 写入不可用时的降级缓存，直接补入
                try:
                    payload = self._mem_dropped.get_nowait()
                except queue.Empty:
                    payload = None
                if payload is not None:
                    if self._raw_data_buf is not None:
                        self._recover_payload(self._raw_data_buf, payload)
                    continue

                try:
                    offset = self._bin_dropped_offsets.get(timeout=0.5)
                except queue.Empty:
                    continue
                writer = self._bin_writer
                q = self._raw_data_buf
                if writer is None or writer.failed or q is None:
                    # 写入器已停用（磁盘错误）时放弃恢复，避免反复读失败
                    continue
                try:
                    if reader is None:
                        reader = open(writer.path, "rb")
                    # 确保待读记录已落盘
                    writer.flush()
                    record = read_bin_record_at(reader, offset)
                except Exception:
                    self._log.exception(_TAG, "Bin recovery read failed")
                    continue
                if record is None or record[0] != BIN_RECORD_DATA:
                    self._log.w(_TAG, f"Bin recovery got invalid record at offset {offset}")
                    continue
                self._recover_payload(q, record[2])
        finally:
            if reader is not None:
                try:
                    reader.close()
                except Exception:
                    pass

    async def _run_in_gforce_loop(self, coro, timeout=None):

        if asyncio.get_running_loop() == self.gforce_event_loop:
            if timeout is not None:
                return await asyncio.wait_for(coro, timeout=timeout)
            return await coro
        future = asyncio.run_coroutine_threadsafe(coro, self.gforce_event_loop)
        if timeout is not None:
            return await asyncio.wait_for(asyncio.wrap_future(future), timeout=timeout)
        return await asyncio.wrap_future(future)

    async def connect(self, disconnect_cb, buf: queue.Queue[bytes]):
        return await self._run_in_gforce_loop(self._do_connect(disconnect_cb, buf))

    async def _do_connect(self, disconnect_cb, buf: queue.Queue[bytes]):

        client = BleakClient(self._device, disconnected_callback=disconnect_cb, **self._client_kwargs)
        self.client = client
        # bumble 后端按芯片类型指定期望 ATT MTU（_patch_bumble_client_connect
        # 在 connect 内据此交换）：RFSTAR(BLE 5.3)尝试 511，其余 247；
        # 原生 bleak 后端的 _backend 无补丁读取该属性，无副作用
        try:
            desired_mtu = 511 if self._chip_type == BLEChipType.RFSTAR else 247
            setattr(getattr(client, "_backend", client), "_sdk_att_mtu", desired_mtu)
        except Exception:
            pass
        self.device_name = self._device.name
        self._raw_data_buf = buf
        
        max_retries = 3
        for attempt in range(max_retries):
            try:
                self._log.d(_TAG, f"bleak connect attempt {attempt + 1}/{max_retries}: {self._device.name}")
                connect_t0 = time.monotonic()
                await asyncio.wait_for(client.connect(), timeout=sensor_utils._TIMEOUT)
                self._log.d(_TAG, f"bleak connect attempt {attempt + 1}/{max_retries} returned "
                                  f"in {(time.monotonic() - connect_t0) * 1000:.0f}ms, "
                                  f"is_connected={client.is_connected}: {self._device.name}")
                await asyncio.sleep(0.5)

                if client.is_connected:
                    break
            except Exception as e:
                self._log.w(_TAG, f"bleak connect attempt {attempt + 1}/{max_retries} failed: "
                                  f"{type(e).__name__}: {e}: {self._device.name}")
                if attempt < max_retries - 1:
                    await asyncio.sleep(1.0)
                else:
                    raise ConnectionError("Connect %s fail: %s" % (self._device.name , e))

        if not client.is_connected:
            raise TimeoutError("Connect timeout: " + self._device.name)

        try:
            if not self._is_universal_stream:
                await asyncio.wait_for(
                    client.start_notify(self.cmd_char, self._on_cmd_response),
                    timeout=sensor_utils._TIMEOUT
                )
            else:
                await asyncio.wait_for(
                    client.start_notify(self.data_char, self._on_universal_response),
                    timeout=sensor_utils._TIMEOUT
                )
            self._log.d(_TAG, f"notify enabled ({'universal data' if self._is_universal_stream else 'cmd'}), "
                              f"connect sequence done: {self._device.name}")
        except Exception as e:
            self._log.w(_TAG, f"start_notify failed, disconnecting: "
                              f"{type(e).__name__}: {e}: {self._device.name}")
            await client.disconnect()
            raise ConnectionError("Connect %s fail: %s" % (self._device.name , e))

        # 连接成功后打开 bin 记录文件，完整保存收到的原始数据
        self._open_bin_recorder()
        self.log_bin_event("connect")

    def _backend_perf_ns(self, attr: str) -> int:
        """读取 bumble 后端实例上的高精度打点（_last_write_perf_ns /
        _last_notify_perf_ns，bumble 层 perf_counter_ns）；原生 bleak 后端
        或补丁未生效时退化为现场打点。"""
        client = self.client
        backend = getattr(client, "_backend", client) if client is not None else None
        ns = getattr(backend, attr, None) if backend is not None else None
        return ns if ns is not None else time.perf_counter_ns()

    def _backend_write_wall_ms(self) -> int:
        """读取 bumble 后端实例上的发送时刻 wall ms 打点（_last_write_wall_ms，
        与 _last_write_perf_ns 同一时点）；原生 bleak 后端或补丁未生效时
        退化为现场打点。"""
        client = self.client
        backend = getattr(client, "_backend", client) if client is not None else None
        ms = getattr(backend, "_last_write_wall_ms", None) if backend is not None else None
        return ms if ms is not None else int(time.time() * 1000)

    def _note_stream_start_ts(self, ts_ms: int):
        """记录起流时刻（与 bin 起流记录同一时间戳）并上报；同时武装首包 delay 测量。
        ts_ms 为完整墙钟毫秒；32 位截断只用于 delay 的毫秒值。"""
        self._stream_start_ts_ms = ts_ms & 0xFFFFFFFF
        self._stream_start_wall_ms = ts_ms
        self._await_first_packet = True
        if self.on_stream_start_ts is not None:
            self.on_stream_start_ts(ts_ms & 0xFFFFFFFF, ts_ms)

    def _note_raw_packet_arrival(self, now_ms: Optional[int] = None):
        """原始数据包到达回调的公共入口：起流后首包计算 delay 并上报。"""
        if not self._await_first_packet:
            return
        self._await_first_packet = False
        if now_ms is None:
            now_ms = int(time.time() * 1000) & 0xFFFFFFFF
        delay = (now_ms - self._stream_start_ts_ms) & 0xFFFFFFFF
        self._log.d(_TAG, f"first packet delay: {delay}ms (start ts {self._stream_start_ts_ms})")
        if self.on_first_packet_delay is not None:
            self.on_first_packet_delay(delay)

    def _on_data_response(self, q: queue.Queue[bytes], bs):
        # 首包 delay 与 bin 数据记录取同一时间戳，保证回放还原值与 live 一致；
        # perf_ns 取 bumble 层通知分发入口打点（原生后端退化为现场打点）
        ts_ms = int(time.time() * 1000)
        perf_ns = self._backend_perf_ns("_last_notify_perf_ns")
        self._note_raw_packet_arrival(ts_ms & 0xFFFFFFFF)
        # 先写 bin 文件再放入队列；队列满时等待入队，等待超时仍未入队的
        # 交给恢复线程：bin 可用时按偏移从 bin 读回补入，bin 不可用时
        # 退化为 500 条内存缓存（超出丢弃最旧包）
        data = bytes(bs)
        offset = self._write_bin_data(data, ts_ms, perf_ns)
        try:
            q.put(data, timeout=_QUEUE_FULL_WAIT_TIMEOUT)
        except queue.Full:
            if offset is not None:
                self._log.w(_TAG, "Raw data queue full after wait, scheduling bin recovery")
                self._queue_bin_recovery(offset)
            else:
                self._log.w(_TAG, "Raw data queue full after wait, using in-memory fallback cache")
                self._queue_mem_recovery(data)

    @staticmethod
    def _convert_acceleration_to_g(data: bytes) -> np.ndarray[np.float32]:
        normalizing_factor = 65536.0

        acceleration_data = np.frombuffer(data, dtype=np.int32).astype(np.float32) / normalizing_factor
        num_channels = 3

        return acceleration_data.reshape(-1, num_channels)

    @staticmethod
    def _convert_gyro_to_dps(data: bytes) -> np.ndarray[np.float32]:
        normalizing_factor = 65536.0

        gyro_data = np.frombuffer(data, dtype=np.int32).astype(np.float32) / normalizing_factor
        num_channels = 3

        return gyro_data.reshape(-1, num_channels)

    @staticmethod
    def _convert_magnetometer_to_ut(data: bytes) -> np.ndarray[np.float32]:
        normalizing_factor = 65536.0

        magnetometer_data = np.frombuffer(data, dtype=np.int32).astype(np.float32) / normalizing_factor
        num_channels = 3

        return magnetometer_data.reshape(-1, num_channels)

    @staticmethod
    def _convert_euler(data: bytes) -> np.ndarray[np.float32]:

        euler_data = np.frombuffer(data, dtype=np.float32).astype(np.float32)
        num_channels = 3

        return euler_data.reshape(-1, num_channels)

    @staticmethod
    def _convert_quaternion(data: bytes) -> np.ndarray[np.float32]:

        quaternion_data = np.frombuffer(data, dtype=np.float32).astype(np.float32)
        num_channels = 4

        return quaternion_data.reshape(-1, num_channels)

    @staticmethod
    def _convert_rotation_matrix(data: bytes) -> np.ndarray[np.float32]:

        rotation_matrix_data = np.frombuffer(data, dtype=np.int32).astype(np.float32)
        num_channels = 9

        return rotation_matrix_data.reshape(-1, num_channels)

    @staticmethod
    def _convert_emg_gesture(data: bytes) -> np.ndarray[np.float16]:

        emg_gesture_data = np.frombuffer(data, dtype=np.int16).astype(np.float16)
        num_channels = 6

        return emg_gesture_data.reshape(-1, num_channels)

    def _on_universal_response(self, _: BleakGATTCharacteristic, bs):
        # 首包 delay 与 bin 数据记录取同一时间戳，保证回放还原值与 live 一致；
        # perf_ns 取 bumble 层通知分发入口打点（原生后端退化为现场打点）
        ts_ms = int(time.time() * 1000)
        perf_ns = self._backend_perf_ns("_last_notify_perf_ns")
        self._note_raw_packet_arrival(ts_ms & 0xFFFFFFFF)
        # 先写 bin 文件再放入队列；队列满时等待入队，等待超时仍未入队的
        # 交给恢复线程：bin 可用时按偏移从 bin 读回补入，bin 不可用时
        # 退化为 500 条内存缓存（超出丢弃最旧包）
        data = bytes(bs)
        offset = self._write_bin_data(data, ts_ms, perf_ns)
        q = self._raw_data_buf
        try:
            q.put(data, timeout=_QUEUE_FULL_WAIT_TIMEOUT)
        except queue.Full:
            if offset is not None:
                self._log.w(_TAG, "Universal raw data queue full after wait, scheduling bin recovery")
                self._queue_bin_recovery(offset)
            else:
                self._log.w(_TAG, "Universal raw data queue full after wait, using in-memory fallback cache")
                self._queue_mem_recovery(data)

    def _on_cmd_response(self, _: BleakGATTCharacteristic, bs):
        # 命令响应先写 bin（CMD 特征上报），再交给响应解析；
        # perf_ns 取 bumble 层通知分发入口打点
        self._write_bin_record(BIN_RECORD_CMD_RECV, bytes(bs),
                               self._backend_perf_ns("_last_notify_perf_ns"))
        sensor_utils.async_exec(self.async_on_cmd_response(bs), self.event_loop)

    async def async_on_cmd_response(self, bs):
        try:
            # print(bytes(bs))
            response = self._parse_response(bytes(bs))
            if self.responses.get(response.cmd) != None:
                self.responses[response.cmd].put_nowait(
                    response.data,
                )
        except Exception as e:
            raise Exception("Failed to parse response: %s" % e)

    @staticmethod
    def _parse_response(res: bytes) -> Response:
        code = int.from_bytes(res[:1], byteorder="big")
        code = ResponseCode(code)

        cmd = int.from_bytes(res[1:2], byteorder="big")
        cmd = Command(cmd)

        data = res[2:]

        return Response(
            code=code,
            cmd=cmd,
            data=data,
        )

    async def get_protocol_version(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_PROTOCOL_VERSION,
                has_res=True,
            )
        )
        return buf.decode("utf-8")

    async def get_feature_map(self) -> int:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_FEATURE_MAP,
                has_res=True,
            )
        )
        return int.from_bytes(buf, byteorder="little")  # TODO: check if this is correct

    async def get_device_name(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_DEVICE_NAME,
                has_res=True,
            )
        )
        return buf.decode("utf-8")

    async def get_firmware_revision(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_FW_REVISION,
                has_res=True,
            )
        )
        return buf.decode("utf-8")

    async def get_hardware_revision(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_HW_REVISION,
                has_res=True,
            )
        )
        # 硬件版本定义：temp[2]=SYS_HARDWARE_REV_ADDR, temp[3]=SYS_HARDWARE_TYPE_ADDR
        # 响应数据中前两个字节分别为硬件版本号和硬件类型
        if len(buf) >= 2:
            return f"{buf[0]}.{buf[1]}"
        if len(buf) == 1:
            return str(buf[0])
        return "0"

    async def get_model_number(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_MODEL_NUMBER,
                has_res=True,
            )
        )
        return buf.decode("utf-8")

    async def get_serial_number(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_SERIAL_NUMBER,
                has_res=True,
            )
        )
        return buf.decode("utf-8")

    async def get_manufacturer_name(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_MANUFACTURER_NAME,
                has_res=True,
            )
        )

        return buf.decode("utf-8")

    async def get_bootloader_version(self) -> str:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_BOOTLOADER_VERSION,
                has_res=True,
            )
        )

        return buf.decode("utf-8")

    async def get_battery_level(self) -> int:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_BATTERY_LEVEL,
                has_res=True,
            )
        )
        if buf is None or len(buf) == 0:
            return -1
        return int.from_bytes(buf, byteorder="big")

    async def get_temperature(self) -> int:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_TEMPERATURE,
                has_res=True,
            )
        )
        if buf is None or len(buf) == 0:
            return -1
        return int.from_bytes(buf, byteorder="big")

    async def power_off(self) -> None:
        await self._send_request(
            Request(
                cmd=Command.POWEROFF,
                has_res=False,
            )
        )

    async def system_reset(self):
        await self._send_request(
            Request(
                cmd=Command.SYSTEM_RESET,
                has_res=False,
            )
        )

    async def set_motor(self, switchStatus):
        body = [switchStatus == True]
        body = bytes(body)
        ret = await self._send_request(
            Request(
                cmd=Command.MOTOR_CONTROL,
                body=body,
                has_res=True,
            )
        )
        return self._check_set_response(ret, "set_motor")

    async def set_led(self, switchStatus):
        body = [switchStatus == True]
        body = bytes(body)
        ret = await self._send_request(
            Request(
                cmd=Command.LED_CONTROL_TEST,
                body=body,
                has_res=True,
            )
        )
        return self._check_set_response(ret, "set_led")

    async def set_package_id(self, switchStatus):
        body = [switchStatus == True]
        body = bytes(body)
        ret = await self._send_request(
            Request(
                cmd=Command.PACKAGE_ID_CONTROL,
                body=body,
                has_res=True,
            )
        )
        return self._check_set_response(ret, "set_package_id")

    async def set_log_level(self, logLevel):
        body = [0xFF & logLevel]
        body = bytes(body)
        ret = await self._send_request(
            Request(
                cmd=Command.SET_LOG_LEVEL,
                body=body,
                has_res=True,
            )
        )
        return self._check_set_response(ret, "set_log_level")

    async def set_function_switch(self, funcSwitch):
        body = [0xFF & funcSwitch]
        body = bytes(body)
        ret = await self._send_request(
            Request(
                cmd=Command.SET_FUNCTION_SWITCH,
                body=body,
                has_res=True,
            )
        )
        return self._check_set_response(ret, "set_function_switch")

    async def set_neucir_app_control(self, open, close, stop):
        if stop:
            body = [4]
        elif open:
            body = [6]
        elif close:
            body = [5]

        body = bytes(body)
        ret = await self._send_request(
            Request(
                cmd=Command.CMD_SET_APP_REMOTE_CMD,
                body=body,
                has_res=True,
            )
        )
        return self._check_set_response(ret, "set_neucir_app_control")

    async def set_neucir_mode(self, mode):
        body = [0x90]

        body = bytes(body)
        ret = await self._send_request(
            Request(
                cmd=Command.CMD_SET_NEUCIR_STATUS,
                body=body,
                has_res=True,
            )
        )
        return self._check_set_response(ret, "set_neucir_mode")
    
    async def set_firmware_filter_switch(self, switchStatus: int):
        body = [0xFF & switchStatus]
        body = bytes(body)
        ret = await self._send_request(Request(cmd=Command.CMD_SET_FRIMWARE_FILTER_SWITCH, body=body, has_res=True))
        return self._check_set_response(ret, "set_firmware_filter_switch")

    async def get_firmware_filter_switch(self):
        buf = await self._send_request(Request(cmd=Command.CMD_GET_FRIMWARE_FILTER_SWITCH, has_res=True))
        return buf[0]

    async def set_emg_raw_data_config(self, cfg=EmgRawDataConfig()):
        body = cfg.to_bytes()
        ret = await self._send_request(
            Request(
                cmd=Command.SET_EMG_RAWDATA_CONFIG,
                body=body,
                has_res=True,
            )
        )

        # print('set_emg_raw_data_config returned:', ret)
        return self._check_set_response(ret, "set_emg_raw_data_config")

        self.resolution = cfg.resolution

        num_channels = 0
        ch_mask = cfg.channel_mask

        while ch_mask != 0:
            if ch_mask & 0x01 != 0:
                num_channels += 1
            ch_mask >>= 1

        self.__num_channels = num_channels

    async def get_emg_raw_data_config(self) -> EmgRawDataConfig:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_EMG_RAWDATA_CONFIG,
                has_res=True,
            )
        )
        return EmgRawDataConfig.from_bytes(buf)

    async def get_emg_raw_data_cap(self) -> EmgRawDataCap:
        buf = await self._send_request(
            Request(
                cmd=Command.GET_EMG_RAWDATA_CAP,
                has_res=True,
            )
        )
        return EmgRawDataCap.from_bytes(buf)

    async def get_eeg_raw_data_config(self) -> EegRawDataConfig:
        buf = await self._send_request(
            Request(
                cmd=Command.CMD_GET_EEG_CONFIG,
                has_res=True,
            )
        )
        return EegRawDataConfig.from_bytes(buf)

    async def get_eeg_raw_data_cap(self) -> EegRawDataCap:
        buf = await self._send_request(
            Request(
                cmd=Command.CMD_GET_EEG_CAP,
                has_res=True,
            )
        )
        return EegRawDataCap.from_bytes(buf)

    async def set_eeg_raw_data_config(self, cfg: EegRawDataConfig):
        body = cfg.to_bytes()
        ret = await self._send_request(
            Request(
                cmd=Command.CMD_SET_EEG_CONFIG,
                body=body,
                has_res=True,
            )
        )
        return self._check_set_response(ret, "set_eeg_raw_data_config")

    async def get_ecg_raw_data_config(self) -> EcgRawDataConfig:
        buf = await self._send_request(
            Request(
                cmd=Command.CMD_GET_ECG_CONFIG,
                has_res=True,
            )
        )
        return EcgRawDataConfig.from_bytes(buf)

    async def get_ecg_raw_data_cap(self) -> EcgRawDataCap:
        buf = await self._send_request(
            Request(
                cmd=Command.CMD_GET_ECG_CAP,
                has_res=True,
            )
        )
        return EcgRawDataCap.from_bytes(buf)

    async def set_ecg_raw_data_config(self, cfg: EcgRawDataConfig):
        body = cfg.to_bytes()
        ret = await self._send_request(
            Request(
                cmd=Command.CMD_SET_ECG_CONFIG,
                body=body,
                has_res=True,
            )
        )
        return self._check_set_response(ret, "set_ecg_raw_data_config")

    async def get_ppg_raw_data_config(self) -> PpgRawDataConfig:
        buf = await self._send_request(
            Request(
                cmd=Command.CMD_GET_PPG_CONFIG,
                has_res=True,
            )
        )
        return PpgRawDataConfig.from_bytes(buf)

    async def set_ppg_raw_data_config(self, cfg: PpgRawDataConfig):
        body = cfg.to_bytes()
        ret = await self._send_request(
            Request(
                cmd=Command.CMD_SET_PPG_CONFIG,
                body=body,
                has_res=True,
            )
        )
        return self._check_set_response(ret, "set_ppg_raw_data_config")

    async def get_imu_raw_data_config(self) -> ImuRawDataConfig:
        buf = await self._send_request(
            Request(
                cmd=Command.CMD_GET_IMU_CONFIG,
                has_res=True,
            )
        )
        return ImuRawDataConfig.from_bytes(buf)

    async def set_imu_raw_data_config(self, cfg: ImuRawDataConfig):
        body = cfg.to_bytes()
        ret = await self._send_request(
            Request(
                cmd=Command.CMD_SET_IMU_CONFIG,
                body=body,
                has_res=True,
            )
        )
        return self._check_set_response(ret, "set_imu_raw_data_config")

    async def get_imu_cap_data_config(self) -> Optional[tuple]:
        """
        Get IMU capability configuration.
        
        Returns:
            Optional[tuple]: (channel_mask, samp_rate, sample_count) if successful, None otherwise
        """
        buf = await self._send_request(
            Request(
                cmd=Command.CMD_GET_IMU_CAP,
                has_res=True,
            )
        )
        
        if buf is None or len(buf) < 7:
            return None
        
        # Parse the response: 4 bytes channel_mask + 2 bytes samp_rate + 1 byte sample_count
        channel_mask = struct.unpack("<I", buf[0:4])[0]  # unsigned int (4 bytes)
        samp_rate = struct.unpack("<H", buf[4:6])[0]     # unsigned short (2 bytes)
        sample_count = struct.unpack("<B", buf[6:7])[0]  # unsigned byte (1 byte)
        
        # Check if IMU_TYPE_QAT6 is supported

        return (channel_mask, samp_rate, sample_count)

    async def get_brth_raw_data_config(self) -> BrthRawDataConfig:
        buf = await self._send_request(
            Request(
                cmd=Command.CMD_GET_BRT_CONFIG,
                has_res=True,
            )
        )
        return BrthRawDataConfig.from_bytes(buf)

    async def set_subscription(self, subscription: DataSubscription, sync_gate=None, mark_stream_start=False):
        body = [
            0xFF & subscription,
            0xFF & (subscription >> 8),
            0xFF & (subscription >> 16),
            0xFF & (subscription >> 24),
        ]
        body = bytes(body)
        ret = await self._send_request(
            Request(
                cmd=Command.SET_DATA_NOTIF_SWITCH,
                body=body,
                has_res=True,
            ),
            sync_gate=sync_gate,
            mark_stream_start=mark_stream_start,
        )
        return self._check_set_response(ret, "set_subscription")

    def _check_set_response(self, ret: Optional[bytes], name: str) -> Optional[bytes]:
        # """检查 set_xxxx 命令的响应，失败时抛出 RuntimeError。"""
        # if ret is None:
        #     raise RuntimeError(f"{name} failed: no response")
        # if len(ret) == 0:
        #     raise RuntimeError(f"{name} failed: empty response")
        # if ret[0] != 0:
        #     raise RuntimeError(f"{name} failed: error code {ret[0]}")
        return ret

    async def start_streaming(self, q: queue.Queue, sync_gate=None):
        return await self._run_in_gforce_loop(self._do_start_streaming(q, sync_gate))

    async def _do_start_streaming(self, q: queue.Queue, sync_gate=None):
        # stop 后 bin 已 finalize 删除；重新起流时在 temp 开启新一轮捕获
        if self._bin_writer is None:
            self._open_bin_recorder()
        if sync_gate is not None:
            # 多设备同步起流：CCCD 起流写在 bumble 后端等待统一放行
            from sensor.bumble_dongle import arm_sync_write_gate

            arm_sync_write_gate(self.client, sync_gate, ("start_notify",))
        await asyncio.wait_for(
            self.client.start_notify(
                self.data_char,
                lambda _, data: self._on_data_response(q, data),
            ),
            timeout=sensor_utils._TIMEOUT
        )
        # 起流时刻与 bin 的 stream_start 事件记录取同一时间戳，
        # 保证回放还原的 startTimeStamp 与 live 一致；
        # perf_ns / ts_ms 取 bumble 层 CCCD 写下发前打点（门闩放行后）
        ts_ms = self.log_bin_event("stream_start", self._backend_perf_ns("_last_write_perf_ns"),
                                   self._backend_write_wall_ms())
        self._note_stream_start_ts(ts_ms)

    async def stop_streaming(self, sync_gate=None):
        return await self._run_in_gforce_loop(self._do_stop_streaming(sync_gate))

    async def _do_stop_streaming(self, sync_gate=None):
        if sync_gate is not None:
            # 多设备同步停流：CCCD 停流写在 bumble 后端等待统一放行
            from sensor.bumble_dongle import arm_sync_write_gate

            arm_sync_write_gate(self.client, sync_gate, ("stop_notify",))
        try:
            await asyncio.wait_for(self.client.stop_notify(self.data_char), timeout=sensor_utils._TIMEOUT)
        except Exception as e:
            raise RuntimeError("Stop streaming %s fail: %s" % (self._device.name , e))
        # perf_ns / ts_ms 取 bumble 层 CCCD 停流写下发前打点（门闩放行后）
        self.log_bin_event("stream_stop", self._backend_perf_ns("_last_write_perf_ns"),
                           self._backend_write_wall_ms())

    async def disconnect(self):
        return await self._run_in_gforce_loop(self._do_disconnect())

    async def _do_disconnect(self):
        # bin 的 finalize（导出+删除）由 ctx 的 stop_streaming/close 统一处理
        with suppress(asyncio.CancelledError):
            try:
                if self.client:
                    await asyncio.wait_for(self.client.disconnect(), timeout=sensor_utils._TIMEOUT)
            except Exception as e:
                raise RuntimeError("Disconnect %s fail: %s" % (self._device.name , e))
        self.log_bin_event("disconnect")

    def _get_response_channel(self, cmd: Command) -> queue.Queue:
        if self.responses.get(cmd) != None:
            return self.responses[cmd]
        else:
            q = queue.Queue()
            self.responses[cmd] = q
            return q

    async def _send_request(self, req: Request, sync_gate=None, mark_stream_start=False) -> Optional[bytes]:
        # 整个发送+等待统一在 gforce 事件循环内串行（asyncio.Lock 不阻塞线程），
        # 消除同 cmd 并发请求的响应归属竞态。
        # 日志成对出现（dispatched/started）：只有前者没有后者即 gforce 事件
        # 循环已卡死（投递的协程永远排不上），其内部超时也随之失效
        self._log.d(_TAG, f"_send_request dispatched to gforce loop: {req.cmd.name}")
        return await self._run_in_gforce_loop(self._send_request_on_gforce_loop(req, sync_gate, mark_stream_start))

    async def _send_request_on_gforce_loop(self, req: Request, sync_gate=None, mark_stream_start=False) -> Optional[bytes]:
        self._log.d(_TAG, f"_send_request started on gforce loop: {req.cmd.name}")
        lock = self._cmd_locks.setdefault(req.cmd, asyncio.Lock())
        async with lock:
            return await self._send_request_locked(req, sync_gate, mark_stream_start)

    async def _send_request_locked(self, req: Request, sync_gate=None, mark_stream_start=False) -> Optional[bytes]:
        q = None
        if req.has_res:
            q = self._get_response_channel(req.cmd)
            # 清空可能残留的旧响应，避免拿到上次超时的数据
            while not q.empty():
                q.get_nowait()

        bs = bytes([req.cmd])
        if req.body is not None:
            bs += req.body

        if sync_gate is not None:
            # 多设备同步起流：CMD 写在 bumble 后端等待统一放行；
            # 期望标识限定只有本命令写消费门闩，电量轮询等其它写直通
            from sensor.bumble_dongle import arm_sync_write_gate

            arm_sync_write_gate(self.client, sync_gate, ("write", req.cmd))

        # print(str(req.cmd) + str(req.body))
        response = False if self._chip_type == BLEChipType.RFSTAR else None
        try:
            await self._run_in_gforce_loop(
                self.client.write_gatt_char(self.cmd_char, bs, response=response),
                timeout=2
            )
        except Exception as e:
            self.last_command_failure_time = time.time()
            self.last_command_failure_cmd = req.cmd
            self._log.exception(_TAG, f"_send_request write_gatt_char failed: {req.cmd}")
            if req.has_res:
                self.responses[req.cmd] = None
            return None

        # 命令写入成功后记录到 bin（[cmd][body...]）；
        # perf_ns / ts_ms 取 bumble 层写下发前打点（门闩放行后），
        # 使起流时刻反映 ATT 命令的实际发出时刻而非写完成时刻
        ts_ms = self._write_bin_record(BIN_RECORD_CMD_SEND, bs,
                                       self._backend_perf_ns("_last_write_perf_ns"),
                                       self._backend_write_wall_ms())
        if mark_stream_start:
            # RFSTAR 起流时刻与 bin 的 cmd_send 记录取同一时间戳，
            # 保证回放还原的 startTimeStamp 与 live 一致
            self._note_stream_start_ts(ts_ms)

        if not req.has_res:
            return None

        try:
            # 队列等待放到执行器线程，避免阻塞事件循环
            ret = await asyncio.get_running_loop().run_in_executor(None, q.get, True, 2)
            return ret
        except Exception as e:
            self._log.exception(_TAG, f"_send_request wait response failed: {req.cmd}")
            self.responses[req.cmd] = None
            raise RuntimeError(f"_send_request wait response failed: {req.cmd}") from e
