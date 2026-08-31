"""原始蓝牙数据 bin 记录格式与读写工具。

bin 文件用于完整保存设备连接期间收到的原始 BLE 数据包，供离线回放分析。
文件保存在 SDK 日志目录下（见 ``SdkLog.get_log_dir``）。

记录格式（小端）：
    [type: u8][timestamp_ms: u64][length: u32][payload: length bytes]

type:
    ``BIN_RECORD_HEADER`` (0x03) — 头部记录，固定为文件第一条。
        payload 固定 16 字节：``<QQ``（首条/末条数据记录的 timestamp_ms）。
        文件打开时先写占位值，关闭时回填，使 ``getBinFileInfo`` 不必扫描
        整个文件即可获得录制时长。
    ``BIN_RECORD_CONFIG`` (0x01) — 配置记录，init 成功后写入，
        回放时用于恢复解析上下文（采样率、通道数、包结构等）。
        payload 固定 1024 字节，为字节对齐的 C 结构体（小端、自然对齐），
        C/C++ 端可直接 fread 到结构体读取，见下方 ``BinReplayConfig`` 定义。
    ``BIN_RECORD_DATA`` (0x02) — 原始数据包（BLE 通知收到的原始字节）。
    ``BIN_RECORD_CMD_SEND`` (0x04) — 主机发往设备的命令（CMD 特征写入，
        payload 为 ``[cmd: u8][body...]``）。
    ``BIN_RECORD_CMD_RECV`` (0x05) — 设备经 CMD 特征上报的命令响应
        （payload 为 ``[code: u8][cmd: u8][data...]``；RFSTAR 设备的命令
        响应以 0xAA 帧混在数据流中，已随 0x02 记录，不再重复）。
    ``BIN_RECORD_EVENT`` (0x06) — 蓝牙事件，payload 为 UTF-8 事件名：
        ``connect``（连接成功）、``disconnect``（断开，含异常断开）、
        ``stream_start``（开始数据通知）、``stream_stop``（停止数据通知）。
    ``BIN_RECORD_PRECISE_TS`` (0x07) — 高精度时间戳记录，紧邻写在它描述的
        记录（0x02 收包 / 0x04 发命令 / 0x06 起停流事件）之前，payload 为
        ``<Q``（8 字节 ``time.perf_counter_ns()``，在 bumble 层打点：
        发送取门闩放行后真正下发前，接收取通知分发入口）。与主记录在
        同一锁内原子写入；旧读取端按未知类型跳过，向后兼容。
        ``iter_bin_records_precise`` 读取时把它配对到紧随的主记录上。

配置记录结构（1024 字节，字段已按自然对齐排布，无隐式填充）::

    #define BIN_CONFIG_MAX_SENSOR_DATAS 14  // SensorDataType.DATA_TYPE_COUNT

    typedef struct {             // 48 字节
        double   sample_rate;
        double   k;
        uint64_t channel_mask;
        int32_t  data_type;
        uint8_t  type_index;
        uint8_t  channel_count;
        uint8_t  package_index_length;
        uint8_t  resolution_bits;
        uint8_t  resolution_signed;
        uint8_t  reserved[3];
        uint16_t package_sample_count;
        uint16_t min_package_sample_count;
        uint8_t  reserved2[8];
    } BinSensorDataConfig;

    typedef struct {             // 200 字节
        char     deviceName[32];
        char     modelName[32];
        char     hardwareVersion[32];
        char     firmwareVersion[32];
        uint16_t MTUSize;
        uint8_t  channelCounts[13];  // Ppg,Spo2,Impe,Emg,Eeg,Ecg,Acc,Gyro,Brth,MagAngle,Euler,Quat,Imu
        uint16_t sampleRates[13];    // 与 channelCounts 同序
        uint8_t  reserved[31];
    } BinDeviceInfo;

    typedef struct {             // 1024 字节
        uint32_t magic;          // BIN_CONFIG_MAGIC ('BCFG')
        uint32_t version;        // BIN_CONFIG_VERSION
        char     device_mac[18];
        uint8_t  is_universal_stream;
        uint8_t  is_new_emg;
        uint8_t  is_contain_qat6;
        uint8_t  ppg_model;
        uint8_t  reserved[2];
        int32_t  chip_type;
        uint8_t  reserved2[2];
        int64_t  feature_map;
        int64_t  notify_data_flag;
        char     device_name[32];
        BinDeviceInfo device_info;
        BinSensorDataConfig sensor_datas[BIN_CONFIG_MAX_SENSOR_DATAS];
        uint8_t  tail_reserved[64];
    } BinReplayConfig;

版本约定：``magic`` + ``version`` 固定为 payload 前 8 字节，后续所有版本的
布局都必须保持该前缀不变。升级布局时递增 ``BIN_CONFIG_VERSION`` 并在
``_CONFIG_DECODERS`` 注册新版本的解码器（旧版本解码器保留，以兼容旧 bin
文件）；版本不受支持时 ``decode_bin_config`` 返回 None 而不是解出错误数据。
"""

import errno
import os
import struct
import threading
import time
from typing import Iterator, Optional, Tuple

from sensor.sdk_log import SdkLog

_TAG = "BinRecorder"

BIN_RECORD_CONFIG = 0x01
BIN_RECORD_DATA = 0x02
BIN_RECORD_HEADER = 0x03
BIN_RECORD_CMD_SEND = 0x04
BIN_RECORD_CMD_RECV = 0x05
BIN_RECORD_EVENT = 0x06
# 高精度时间戳记录：紧邻写在它描述的记录（0x02 收包 / 0x04 发命令 /
# 0x06 起停流事件）之前，payload 为 <Q（time.perf_counter_ns()）。
# 与主记录在同一锁内原子写入；旧读取端按未知类型跳过，向后兼容。
BIN_RECORD_PRECISE_TS = 0x07

_HEADER_STRUCT = struct.Struct("<BQI")
_HEADER_SIZE = _HEADER_STRUCT.size

# 高精度时间戳记录 payload：<perf_counter_ns u64>
_PRECISE_TS_PAYLOAD_STRUCT = struct.Struct("<Q")

# 头部记录 payload：<首条数据时间戳 u64, 末条数据时间戳 u64>
_HEADER_PAYLOAD_STRUCT = struct.Struct("<QQ")
_HEADER_PAYLOAD_SIZE = _HEADER_PAYLOAD_STRUCT.size

# 防御：单条记录长度上限，超过则认为文件已损坏
_MAX_RECORD_LENGTH = 64 * 1024 * 1024

_DEFAULT_FLUSH_INTERVAL = 64

# ------------------------------------------------------------------
# 配置记录：固定 1024 字节 C 结构（小端、自然对齐，布局见模块 docstring）
# ------------------------------------------------------------------
BIN_CONFIG_STRUCT_SIZE = 1024
BIN_CONFIG_MAGIC = 0x47464342  # 'BCFG'
# version 3：BinDeviceInfo 增加 Imu 通道数/采样率（12→13 槽，reserved 34→31）
BIN_CONFIG_VERSION = 3

# magic + version 固定为 payload 前 8 字节，所有版本的布局都必须保持这一前缀不变，
# 使解码端可以先读版本号再按版本分发（未来升级时递增 BIN_CONFIG_VERSION，
# 并在 _CONFIG_DECODERS 中注册对应版本的解码函数）。
_CONFIG_MAGIC_VERSION_STRUCT = struct.Struct("<II")

# SensorDataType.DATA_TYPE_COUNT（此处硬编码以避免反向依赖）
_CONFIG_MAX_SENSOR_DATAS = 14

_CONFIG_HEADER_STRUCT = struct.Struct("<II18s4B2xi4xqq32s")
_CONFIG_DEVICE_INFO_STRUCT = struct.Struct("<32s32s32s32sH12B12H34x")       # v2
_CONFIG_DEVICE_INFO_STRUCT_V3 = struct.Struct("<32s32s32s32sH13B13H31x")    # v3
_CONFIG_SENSOR_DATA_STRUCT = struct.Struct("<ddQi5B3x2H8x")

# DeviceInfo 通道数/采样率字段顺序（与 BinDeviceInfo 一致）；
# v3 在末尾追加 Imu（NTF_IMU 聚合流，仅新 EMG 设备）
_CONFIG_DEVICE_INFO_PREFIXES = (
    "Ppg", "Spo2", "Impe", "Emg", "Eeg", "Ecg",
    "Acc", "Gyro", "Brth", "MagAngle", "Euler", "Quat",
)
_CONFIG_DEVICE_INFO_PREFIXES_V3 = _CONFIG_DEVICE_INFO_PREFIXES + ("Imu",)


def _pack_cstr(value, size: int) -> bytes:
    """按 C char[N] 语义打包字符串：UTF-8 编码、截断到 N-1、NUL 填充。"""
    if value is None:
        raw = b""
    else:
        raw = str(value).encode("utf-8")
    return raw[: size - 1]


def _unpack_cstr(raw: bytes) -> str:
    """按 C char[N] 语义解包字符串：取首个 NUL 之前的内容。"""
    end = raw.find(b"\x00")
    if end >= 0:
        raw = raw[:end]
    return raw.decode("utf-8", errors="replace")


def _clamp_int(value, lo: int, hi: int) -> int:
    try:
        v = int(value)
    except Exception:
        v = 0
    return max(lo, min(hi, v))


def format_mac(address, sep: str = ":") -> str:
    """把 MAC 地址规范化为分隔符分隔的大写形式（默认 ``AA:BB:CC:DD:EE:FF``）。

    输入已含分隔符或为纯 12 位十六进制时统一成 ``sep`` 分隔；
    无法识别为 MAC（如 macOS 的 UUID 地址）时原样返回。
    """
    text = str(address or "")
    raw = "".join(c for c in text if c in "0123456789abcdefABCDEF")
    if len(raw) == 12:
        return sep.join(raw[i : i + 2] for i in range(0, 12, 2)).upper()
    return text


def encode_bin_config(config: dict) -> bytes:
    """把 ``SensorProfileDataCtx.dump_replay_config()`` 导出的 dict 编码为
    固定 1024 字节的 C 结构 payload（布局见模块 docstring）。"""
    device_info = config.get("device_info") or {}
    header = _CONFIG_HEADER_STRUCT.pack(
        BIN_CONFIG_MAGIC,
        BIN_CONFIG_VERSION,
        _pack_cstr(format_mac(config.get("device_mac")), 18),
        1 if config.get("is_universal_stream") else 0,
        1 if config.get("is_new_emg") else 0,
        1 if config.get("is_contain_qat6") else 0,
        _clamp_int(config.get("ppg_model", 0), 0, 0xFF),
        _clamp_int(config.get("chip_type", -1), -0x80000000, 0x7FFFFFFF),
        _clamp_int(config.get("feature_map", 0), -0x8000000000000000, 0x7FFFFFFFFFFFFFFF),
        _clamp_int(config.get("notify_data_flag", 0), -0x8000000000000000, 0x7FFFFFFFFFFFFFFF),
        _pack_cstr(config.get("device_name"), 32),
    )
    info = _CONFIG_DEVICE_INFO_STRUCT_V3.pack(
        _pack_cstr(device_info.get("DeviceName"), 32),
        _pack_cstr(device_info.get("ModelName"), 32),
        _pack_cstr(device_info.get("HardwareVersion"), 32),
        _pack_cstr(device_info.get("FirmwareVersion"), 32),
        _clamp_int(device_info.get("MTUSize", 0), 0, 0xFFFF),
        *(_clamp_int(device_info.get(p + "ChannelCount", 0), 0, 0xFF) for p in _CONFIG_DEVICE_INFO_PREFIXES_V3),
        *(_clamp_int(device_info.get(p + "SampleRate", 0), 0, 0xFFFF) for p in _CONFIG_DEVICE_INFO_PREFIXES_V3),
    )
    slots = [b"\x00" * _CONFIG_SENSOR_DATA_STRUCT.size] * _CONFIG_MAX_SENSOR_DATAS
    for item in config.get("sensor_datas") or []:
        try:
            idx = int(item.get("type_index"))
        except Exception:
            continue
        if idx < 0 or idx >= _CONFIG_MAX_SENSOR_DATAS:
            continue
        slots[idx] = _CONFIG_SENSOR_DATA_STRUCT.pack(
            float(item.get("sample_rate", 0.0)),
            float(item.get("k", 0.0)),
            _clamp_int(item.get("channel_mask", 0), 0, 0xFFFFFFFFFFFFFFFF),
            _clamp_int(item.get("data_type", 0), -0x80000000, 0x7FFFFFFF),
            idx & 0xFF,
            _clamp_int(item.get("channel_count", 0), 0, 0xFF),
            _clamp_int(item.get("package_index_length", 0), 0, 0xFF),
            _clamp_int(item.get("resolution_bits", 0), 0, 0xFF),
            _clamp_int(item.get("resolution_signed", 0), 0, 0xFF),
            _clamp_int(item.get("package_sample_count", 0), 0, 0xFFFF),
            _clamp_int(item.get("min_package_sample_count", 0), 0, 0xFFFF),
        )
    payload = header + info + b"".join(slots)
    return payload.ljust(BIN_CONFIG_STRUCT_SIZE, b"\x00")


def decode_bin_config(payload: bytes) -> Optional[dict]:
    """把配置记录 payload 解码回 dict（与 ``dump_replay_config`` 导出结构一致）。

    先读固定前 8 字节的 magic + version，再按版本分发到对应解码器；
    magic 不符、长度不足或版本不受支持（如更高版本的新文件）时返回 None。
    """
    if payload is None or len(payload) < BIN_CONFIG_STRUCT_SIZE:
        return None
    magic, version = _CONFIG_MAGIC_VERSION_STRUCT.unpack(payload[: _CONFIG_MAGIC_VERSION_STRUCT.size])
    if magic != BIN_CONFIG_MAGIC:
        return None
    decoder = _CONFIG_DECODERS.get(version)
    if decoder is None:
        SdkLog.w(_TAG, f"不支持的 bin 配置版本: {version}（当前支持: {sorted(_CONFIG_DECODERS)}）")
        return None
    return decoder(payload)


def _decode_bin_config_impl(payload: bytes, info_struct: struct.Struct, prefixes) -> Optional[dict]:
    """按给定 BinDeviceInfo 布局/前缀表解码（1024 字节，见模块 docstring）。"""
    offset = 0

    def _take(fmt: struct.Struct):
        nonlocal offset
        values = fmt.unpack(payload[offset : offset + fmt.size])
        offset += fmt.size
        return values

    (
        magic,
        version,
        device_mac,
        is_universal_stream,
        is_new_emg,
        is_contain_qat6,
        ppg_model,
        chip_type,
        feature_map,
        notify_data_flag,
        device_name,
    ) = _take(_CONFIG_HEADER_STRUCT)

    info_values = _take(info_struct)
    device_info = {
        "DeviceName": _unpack_cstr(info_values[0]),
        "ModelName": _unpack_cstr(info_values[1]),
        "HardwareVersion": _unpack_cstr(info_values[2]),
        "FirmwareVersion": _unpack_cstr(info_values[3]),
        "MTUSize": info_values[4],
    }
    n = len(prefixes)
    counts = info_values[5:5 + n]
    rates = info_values[5 + n:5 + 2 * n]
    for i, prefix in enumerate(prefixes):
        device_info[prefix + "ChannelCount"] = counts[i]
        device_info[prefix + "SampleRate"] = rates[i]

    sensor_datas = []
    for idx in range(_CONFIG_MAX_SENSOR_DATAS):
        (
            sample_rate,
            k,
            channel_mask,
            data_type,
            type_index,
            channel_count,
            package_index_length,
            resolution_bits,
            resolution_signed,
            package_sample_count,
            min_package_sample_count,
        ) = _take(_CONFIG_SENSOR_DATA_STRUCT)
        # 与 dump 侧相同的有效性判断：空槽（全零）跳过
        if package_sample_count <= 0 or channel_count <= 0:
            continue
        sensor_datas.append({
            "type_index": type_index,
            "data_type": data_type,
            "sample_rate": sample_rate,
            "channel_count": channel_count,
            "package_sample_count": package_sample_count,
            "min_package_sample_count": min_package_sample_count,
            "package_index_length": package_index_length,
            "resolution_bits": resolution_bits,
            "resolution_signed": resolution_signed,
            "channel_mask": channel_mask,
            "k": k,
        })

    return {
        "version": version,
        "device_mac": _unpack_cstr(device_mac),
        "device_name": _unpack_cstr(device_name),
        "chip_type": chip_type,
        "is_universal_stream": bool(is_universal_stream),
        "feature_map": feature_map,
        "notify_data_flag": notify_data_flag,
        "is_new_emg": bool(is_new_emg),
        "is_contain_qat6": bool(is_contain_qat6),
        "ppg_model": ppg_model,
        "device_info": device_info,
        "sensor_datas": sensor_datas,
    }


# 按配置版本注册的解码器；升级布局时保留旧版本解码器以维持向后兼容
def _decode_bin_config_v2(payload: bytes) -> Optional[dict]:
    """version 2 布局：BinDeviceInfo 12 槽（无 Imu）。"""
    return _decode_bin_config_impl(payload, _CONFIG_DEVICE_INFO_STRUCT, _CONFIG_DEVICE_INFO_PREFIXES)


def _decode_bin_config_v3(payload: bytes) -> Optional[dict]:
    """version 3 布局：BinDeviceInfo 13 槽（末尾追加 Imu）。"""
    return _decode_bin_config_impl(payload, _CONFIG_DEVICE_INFO_STRUCT_V3, _CONFIG_DEVICE_INFO_PREFIXES_V3)


_CONFIG_DECODERS = {
    2: _decode_bin_config_v2,
    3: _decode_bin_config_v3,
}


class BinRecordWriter:
    """bin 记录写入器（线程安全）。

    数据先写 bin 文件再放入解析队列，保证队列满丢包时原始数据仍完整保存。
    记录追加（缓冲写，微秒级）留在调用线程同步完成，保证偏移/0x07 配对的
    同步语义；真正阻塞的磁盘 flush 由后台线程执行——按 ``flush_interval``
    条记录触发一次非阻塞请求，``flush()``（恢复线程读回前）与 ``close``
    走同步等待。这样文件 I/O 不再阻塞通知回调所在的 dongle 事件循环。
    """

    def __init__(self, path: str, flush_interval: int = _DEFAULT_FLUSH_INTERVAL,
                 append: bool = False):
        self._path = path
        self._flush_interval = max(1, int(flush_interval))
        self._lock = threading.Lock()
        # 后台 flush：请求/完成计数配对，flush() 可同步等待一轮完成
        self._flush_cond = threading.Condition(self._lock)
        self._flush_requested = 0
        self._flush_completed = 0
        self._closing = False
        self._count = 0
        self._offset = 0
        # 写盘出错后停用写入器，避免每个数据包都重复触发异常
        self._failed = False
        # 首/末条数据记录时间戳，close 时回填到头部记录
        self._first_data_ts: Optional[int] = None
        self._last_data_ts: Optional[int] = None
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        if append and os.path.exists(path) and os.path.getsize(path) > 0:
            # 追加模式（直写导出文件）：续在上次会话内容之后，偏移从文件尾起算，
            # 头部占位记录写在本段起点，回填时定位到本段头部而非文件开头；
            # close 时还会把本段时间跨度折入文件第一条头部记录
            self._appended = True
            self._file = open(path, "r+b")
            self._file.seek(0, os.SEEK_END)
            self._offset = self._file.tell()
        else:
            self._appended = False
            self._file = open(path, "wb")
        self._flusher = threading.Thread(
            target=self._flush_loop,
            name=f"BinFlush-{os.path.basename(path)}",
            daemon=True,
        )
        self._flusher.start()
        # 头部记录（占位），close 时回填真实首末时间戳
        self._header_payload_offset = self._offset + _HEADER_SIZE
        self.write_record(BIN_RECORD_HEADER, _HEADER_PAYLOAD_STRUCT.pack(0, 0))

    @property
    def path(self) -> str:
        return self._path

    @property
    def failed(self) -> bool:
        """写入器是否已因磁盘错误（空间不足/权限/占用等）停用。"""
        return self._failed

    def _handle_write_error(self, e: Exception):
        """磁盘写出错处理：按错误类型记录原因，并停用写入器（只报一次）。"""
        if self._failed:
            return
        self._failed = True
        err_no = getattr(e, "errno", None)
        if err_no == errno.ENOSPC:
            reason = "磁盘空间不足"
        elif err_no in (errno.EACCES, errno.EPERM):
            reason = "没有写入权限或文件被占用"
        else:
            reason = str(e)
        SdkLog.e(_TAG, f"bin 文件写入失败，已停止记录: {self._path} ({reason})")
        try:
            if self._file is not None:
                self._file.close()
        except Exception:
            pass
        self._file = None

    def write_record(self, record_type: int, payload: bytes, ts_ms: Optional[int] = None,
                     perf_ns: Optional[int] = None) -> Optional[int]:
        """写入一条记录，返回记录在文件中的起始偏移；写入失败/已停用返回 None。

        ``perf_ns`` 不为 None 时，先在主记录前紧邻写入一条
        ``BIN_RECORD_PRECISE_TS`` (0x07) 高精度时间戳记录（payload 为
        ``<Q`` perf_counter_ns），两条记录在同一锁内原子写入。
        """
        if ts_ms is None:
            ts_ms = int(time.time() * 1000)
        with self._lock:
            if self._file is None:
                return None
            try:
                if perf_ns is not None:
                    self._write_one(BIN_RECORD_PRECISE_TS, ts_ms,
                                    _PRECISE_TS_PAYLOAD_STRUCT.pack(perf_ns))
                offset = self._offset
                self._write_one(record_type, ts_ms, payload)
                return offset
            except Exception as e:
                self._handle_write_error(e)
                return None

    def _write_one(self, record_type: int, ts_ms: int, payload: bytes):
        """锁内写单条记录并做偏移/计数/首尾数据时间戳记账。"""
        self._file.write(_HEADER_STRUCT.pack(record_type, ts_ms, len(payload)))
        self._file.write(payload)
        self._offset += _HEADER_SIZE + len(payload)
        self._count += 1
        if record_type == BIN_RECORD_DATA:
            if self._first_data_ts is None:
                self._first_data_ts = ts_ms
            self._last_data_ts = ts_ms
        if self._count % self._flush_interval == 0:
            # 非阻塞后台 flush 请求：落盘 I/O 在 flusher 线程执行
            self._flush_requested += 1
            self._flush_cond.notify()

    def _flush_loop(self):
        """后台 flush 线程：把磁盘 flush 移出通知/dongle 线程。"""
        with self._flush_cond:
            while not self._closing:
                while self._flush_completed >= self._flush_requested and not self._closing:
                    self._flush_cond.wait()
                if self._closing:
                    break
                target = self._flush_requested
                if self._file is not None:
                    try:
                        self._file.flush()
                    except Exception as e:
                        self._handle_write_error(e)
                self._flush_completed = target
                self._flush_cond.notify_all()

    def write_data(self, data: bytes) -> Optional[int]:
        return self.write_record(BIN_RECORD_DATA, data)

    def write_config(self, config: dict) -> Optional[int]:
        return self.write_record(BIN_RECORD_CONFIG, encode_bin_config(config))

    def flush(self):
        """把缓冲中的记录强制落盘并等待完成（供恢复线程读取前调用）。"""
        with self._flush_cond:
            if self._file is None or self._closing:
                return
            self._flush_requested += 1
            self._flush_cond.notify()
            target = self._flush_requested
            while self._flush_completed < target and not self._closing:
                self._flush_cond.wait()

    def close(self):
        """关闭文件：先把首末数据时间戳回填到头部记录，再 flush 关闭。"""
        with self._flush_cond:
            self._closing = True
            self._flush_cond.notify_all()
            if self._file is not None:
                try:
                    self._file.flush()
                    # 回填头部记录（本段第一条记录的 payload 位置；追加模式下
                    # 在文件中部，不能固定在文件开头）
                    first_ts = self._first_data_ts or 0
                    last_ts = self._last_data_ts or 0
                    self._file.seek(self._header_payload_offset)
                    self._file.write(_HEADER_PAYLOAD_STRUCT.pack(first_ts, last_ts))
                    if self._appended:
                        # 追加段：把本段时间跨度折入文件第一条头部记录，
                        # 使 getBinFileInfo 读到的时长覆盖整个文件而非仅首段
                        self._file.seek(0)
                        if self._file.read(1) == bytes([BIN_RECORD_HEADER]):
                            self._file.seek(_HEADER_SIZE)
                            prev = self._file.read(_HEADER_PAYLOAD_SIZE)
                            if len(prev) == _HEADER_PAYLOAD_SIZE:
                                prev_first, prev_last = _HEADER_PAYLOAD_STRUCT.unpack(prev)
                                positive = [t for t in (prev_first, first_ts) if t > 0]
                                combined_first = min(positive) if positive else 0
                                combined_last = max(prev_last, last_ts)
                                self._file.seek(_HEADER_SIZE)
                                self._file.write(_HEADER_PAYLOAD_STRUCT.pack(
                                    combined_first, combined_last))
                    self._file.flush()
                except Exception as e:
                    self._handle_write_error(e)
                try:
                    self._file.close()
                except Exception:
                    pass
                self._file = None
        # flusher 线程在 _closing 置位后自行退出；锁外 join 避免持锁等待
        self._flusher.join(timeout=2)


def iter_bin_records(path: str) -> Iterator[Tuple[int, int, bytes]]:
    """顺序读取 bin 文件记录，产出 ``(record_type, ts_ms, payload)``。

    读到文件尾或损坏（截断）记录时停止。
    """
    with open(path, "rb") as f:
        while True:
            header = f.read(_HEADER_SIZE)
            if len(header) < _HEADER_SIZE:
                return
            record_type, ts_ms, length = _HEADER_STRUCT.unpack(header)
            if length > _MAX_RECORD_LENGTH:
                return
            payload = f.read(length)
            if len(payload) < length:
                return
            yield record_type, ts_ms, payload


def iter_bin_records_precise(path: str) -> Iterator[Tuple[int, int, bytes, Optional[int]]]:
    """顺序读取 bin 文件记录，产出 ``(record_type, ts_ms, payload, perf_ns)``。

    ``perf_ns`` 取自紧邻前置的 ``BIN_RECORD_PRECISE_TS`` (0x07) 记录
    （``time.perf_counter_ns()`` 打点，bumble 层发送/接收时刻），
    没有前置 0x07 记录（旧 bin）时为 None；0x07 记录本身不产出。
    """
    pending_perf_ns = None
    for record_type, ts_ms, payload in iter_bin_records(path):
        if record_type == BIN_RECORD_PRECISE_TS:
            if len(payload) == _PRECISE_TS_PAYLOAD_STRUCT.size:
                (pending_perf_ns,) = _PRECISE_TS_PAYLOAD_STRUCT.unpack(payload)
            continue
        yield record_type, ts_ms, payload, pending_perf_ns
        pending_perf_ns = None


def read_bin_record_at(f, offset: int) -> Optional[Tuple[int, int, bytes]]:
    """在已打开的文件对象 ``f`` 中读取指定偏移的记录；记录不存在或不完整返回 None。

    与写入并发使用前，先调用 ``BinRecordWriter.flush`` 确保记录已落盘。
    """
    try:
        f.seek(offset)
        header = f.read(_HEADER_SIZE)
        if len(header) < _HEADER_SIZE:
            return None
        record_type, ts_ms, length = _HEADER_STRUCT.unpack(header)
        if length > _MAX_RECORD_LENGTH:
            return None
        payload = f.read(length)
        if len(payload) < length:
            return None
        return record_type, ts_ms, payload
    except Exception:
        return None


def read_bin_config(path: str) -> Optional[dict]:
    """读取 bin 文件中的第一条配置记录；没有则返回 None。"""
    try:
        for record_type, _, payload in iter_bin_records(path):
            if record_type == BIN_RECORD_CONFIG:
                return decode_bin_config(payload)
    except Exception:
        pass
    return None


