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
        uint8_t  channelCounts[12];  // Ppg,Spo2,Impe,Emg,Eeg,Ecg,Acc,Gyro,Brth,MagAngle,Euler,Quat
        uint16_t sampleRates[12];    // 与 channelCounts 同序
        uint8_t  reserved[34];
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

_HEADER_STRUCT = struct.Struct("<BQI")
_HEADER_SIZE = _HEADER_STRUCT.size

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
BIN_CONFIG_VERSION = 2

# magic + version 固定为 payload 前 8 字节，所有版本的布局都必须保持这一前缀不变，
# 使解码端可以先读版本号再按版本分发（未来升级时递增 BIN_CONFIG_VERSION，
# 并在 _CONFIG_DECODERS 中注册对应版本的解码函数）。
_CONFIG_MAGIC_VERSION_STRUCT = struct.Struct("<II")

# SensorDataType.DATA_TYPE_COUNT（此处硬编码以避免反向依赖）
_CONFIG_MAX_SENSOR_DATAS = 14

_CONFIG_HEADER_STRUCT = struct.Struct("<II18s4B2xi4xqq32s")
_CONFIG_DEVICE_INFO_STRUCT = struct.Struct("<32s32s32s32sH12B12H34x")
_CONFIG_SENSOR_DATA_STRUCT = struct.Struct("<ddQi5B3x2H8x")

# DeviceInfo 通道数/采样率字段顺序（与 BinDeviceInfo 一致）
_CONFIG_DEVICE_INFO_PREFIXES = (
    "Ppg", "Spo2", "Impe", "Emg", "Eeg", "Ecg",
    "Acc", "Gyro", "Brth", "MagAngle", "Euler", "Quat",
)


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
    info = _CONFIG_DEVICE_INFO_STRUCT.pack(
        _pack_cstr(device_info.get("DeviceName"), 32),
        _pack_cstr(device_info.get("ModelName"), 32),
        _pack_cstr(device_info.get("HardwareVersion"), 32),
        _pack_cstr(device_info.get("FirmwareVersion"), 32),
        _clamp_int(device_info.get("MTUSize", 0), 0, 0xFFFF),
        *(_clamp_int(device_info.get(p + "ChannelCount", 0), 0, 0xFF) for p in _CONFIG_DEVICE_INFO_PREFIXES),
        *(_clamp_int(device_info.get(p + "SampleRate", 0), 0, 0xFFFF) for p in _CONFIG_DEVICE_INFO_PREFIXES),
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


def _decode_bin_config_v2(payload: bytes) -> Optional[dict]:
    """version 2 布局解码（1024 字节，见模块 docstring 的 BinReplayConfig）。"""
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

    info_values = _take(_CONFIG_DEVICE_INFO_STRUCT)
    device_info = {
        "DeviceName": _unpack_cstr(info_values[0]),
        "ModelName": _unpack_cstr(info_values[1]),
        "HardwareVersion": _unpack_cstr(info_values[2]),
        "FirmwareVersion": _unpack_cstr(info_values[3]),
        "MTUSize": info_values[4],
    }
    counts = info_values[5:17]
    rates = info_values[17:29]
    for i, prefix in enumerate(_CONFIG_DEVICE_INFO_PREFIXES):
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
_CONFIG_DECODERS = {
    2: _decode_bin_config_v2,
}


class BinRecordWriter:
    """bin 记录写入器（线程安全）。

    数据先写 bin 文件再放入解析队列，保证队列满丢包时原始数据仍完整保存。
    为避免每条记录都触发磁盘 flush 阻塞通知线程，按 ``flush_interval`` 条
    记录周期 flush，``close`` 时兜底 flush。
    """

    def __init__(self, path: str, flush_interval: int = _DEFAULT_FLUSH_INTERVAL):
        self._path = path
        self._flush_interval = max(1, int(flush_interval))
        self._lock = threading.Lock()
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
        self._file = open(path, "wb")
        # 头部记录（占位），close 时回填真实首末时间戳
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

    def write_record(self, record_type: int, payload: bytes, ts_ms: Optional[int] = None) -> Optional[int]:
        """写入一条记录，返回记录在文件中的起始偏移；写入失败/已停用返回 None。"""
        if ts_ms is None:
            ts_ms = int(time.time() * 1000)
        with self._lock:
            if self._file is None:
                return None
            try:
                offset = self._offset
                self._file.write(_HEADER_STRUCT.pack(record_type, ts_ms, len(payload)))
                self._file.write(payload)
                self._offset += _HEADER_SIZE + len(payload)
                self._count += 1
                if record_type == BIN_RECORD_DATA:
                    if self._first_data_ts is None:
                        self._first_data_ts = ts_ms
                    self._last_data_ts = ts_ms
                if self._count % self._flush_interval == 0:
                    self._file.flush()
                return offset
            except Exception as e:
                self._handle_write_error(e)
                return None

    def write_data(self, data: bytes) -> Optional[int]:
        return self.write_record(BIN_RECORD_DATA, data)

    def write_config(self, config: dict) -> Optional[int]:
        return self.write_record(BIN_RECORD_CONFIG, encode_bin_config(config))

    def flush(self):
        """把缓冲中的记录强制落盘（供恢复线程读取前调用）。"""
        with self._lock:
            if self._file is not None:
                try:
                    self._file.flush()
                except Exception as e:
                    self._handle_write_error(e)

    def close(self):
        """关闭文件：先把首末数据时间戳回填到头部记录，再 flush 关闭。"""
        with self._lock:
            if self._file is not None:
                try:
                    self._file.flush()
                    # 回填头部记录（固定为文件第一条记录，payload 偏移 = 记录头长度）
                    first_ts = self._first_data_ts or 0
                    last_ts = self._last_data_ts or 0
                    self._file.seek(_HEADER_SIZE)
                    self._file.write(_HEADER_PAYLOAD_STRUCT.pack(first_ts, last_ts))
                    self._file.flush()
                except Exception as e:
                    self._handle_write_error(e)
                try:
                    self._file.close()
                except Exception:
                    pass
                self._file = None


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


