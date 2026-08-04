"""bin 文件离线解析为 CSV。

把 bin 记录（init 配置记录 + 原始 BLE 数据包）重新送入真实解析管线，
输出与历史实时 DEBUG_BLE_DATA CSV 相同格式的文件。
由 SensorController.parseBinToCsv 对外暴露。
"""

import asyncio
import csv
import os
import time
from datetime import datetime
from queue import Full, Queue

from sensor.bin_recorder import BIN_RECORD_CONFIG, decode_bin_config, iter_bin_records
from sensor.sdk_log import SdkLog

_TAG = "BinToCsv"

# 与历史实时 data 日志一致的列头
_CSV_HEADER = [
    "timestamp", "mac", "type", "raw_hex", "data_type",
    "sample_rate", "channel_count", "lost_count", "samples_info", "first_sample",
]


def _ts_str(ts_ms: int) -> str:
    """bin 记录时间戳（毫秒）转 ISO 字符串。"""
    return datetime.fromtimestamp(ts_ms / 1000.0).isoformat()


def _parsed_row(ts_ms: int, sensor_data) -> list:
    """按历史 _write_data_log_parsed 的字段构造 parsed 行。"""
    sample_counts = [len(ch) for ch in sensor_data.channelSamples]
    first_sample = None
    for ch in sensor_data.channelSamples:
        if ch:
            first_sample = ch[0]
            break
    first_sample_str = ""
    if first_sample is not None:
        first_sample_str = (
            f"data={first_sample.data}|raw={first_sample.rawData}|"
            f"imp={first_sample.impedance}|sat={first_sample.saturation}|"
            f"idx={first_sample.sampleIndex}|ts={first_sample.timeStampInMs}|"
            f"ch={first_sample.channelIndex}|lost={first_sample.isLost}"
        )
    data_type = sensor_data.dataType
    type_name = data_type.name if hasattr(data_type, "name") else data_type
    return [
        _ts_str(ts_ms),
        sensor_data.deviceMac or "",
        "parsed",
        "",
        type_name,
        sensor_data.sampleRate,
        sensor_data.channelCount,
        sensor_data.lostPackageCount,
        str(sample_counts),
        first_sample_str,
    ]


def bin_to_csv(bin_path: str, csv_path: str = None) -> str:
    """离线解析 bin 文件并输出 CSV，返回 CSV 文件路径。

    无配置记录时返回已写出的 raw 行（无法解析为 parsed 行，但不视为错误）。
    """
    if not bin_path or not os.path.isfile(bin_path):
        raise FileNotFoundError(f"bin file not found: {bin_path}")
    if not csv_path:
        root, _ = os.path.splitext(bin_path)
        csv_path = root + ".csv"

    from sensor.sensor_data_context import SensorProfileDataCtx
    from sensor.sensor_utils import BLEAK_DATA_QUEUE_MAXSIZE

    ctx = None
    device_mac = ""
    rows_written = 0

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, delimiter=",")
        writer.writerow(_CSV_HEADER)

        loop = asyncio.new_event_loop()
        try:
            asyncio.set_event_loop(loop)

            async def _feed_and_parse():
                nonlocal ctx, device_mac, rows_written
                local_buf = Queue()
                parse_task = None
                last_ts = 0

                def on_data(sensor_data):
                    nonlocal rows_written
                    writer.writerow(_parsed_row(last_ts, sensor_data))
                    rows_written += 1

                def on_error(message):
                    SdkLog.w(_TAG, f"parse error: {message}")

                async def _ensure_ctx(config: dict):
                    nonlocal ctx, device_mac, parse_task
                    device_mac = config.get("device_mac") or ""
                    ctx = SensorProfileDataCtx(
                        None, device_mac, Queue(maxsize=BLEAK_DATA_QUEUE_MAXSIZE)
                    )
                    ctx.load_replay_config(config)
                    ctx._is_data_transfering = True
                    if ctx.isUniversalStream:
                        parse_task = loop.create_task(
                            ctx._processUniversalData(local_buf, on_data, on_error)
                        )
                    else:
                        parse_task = loop.create_task(
                            ctx._process_data(local_buf, on_data, on_error)
                        )

                for record_type, ts, payload in iter_bin_records(bin_path):
                    last_ts = ts
                    if record_type == BIN_RECORD_CONFIG:
                        config = decode_bin_config(payload)
                        if config is not None:
                            if ctx is not None:
                                # 与录制时序一致：按顺序应用配置记录
                                ctx.load_replay_config(config)
                            else:
                                await _ensure_ctx(config)
                        continue
                    # raw 行无条件写出（配置记录前的包 mac 为空）
                    writer.writerow(
                        [_ts_str(ts), device_mac, "raw", payload.hex(), "", "", "", "", ""]
                    )
                    rows_written += 1
                    if ctx is None:
                        continue
                    # 喂包，原始队列满时让解析先追上来
                    while True:
                        try:
                            ctx._rawDataBuffer.put_nowait(payload)
                            break
                        except Full:
                            await asyncio.sleep(0.01)

                if ctx is not None:
                    # drain：先等原始队列清空，再留余量给 FlatBuffers 结果队列
                    deadline = time.time() + 30
                    while not ctx._rawDataBuffer.empty() and time.time() < deadline:
                        await asyncio.sleep(0.05)
                    await asyncio.sleep(0.5)
                    if parse_task is not None:
                        parse_task.cancel()
                        try:
                            await parse_task
                        except asyncio.CancelledError:
                            pass
                        except Exception:
                            pass

            loop.run_until_complete(_feed_and_parse())
        finally:
            if ctx is not None:
                try:
                    ctx.close()
                except Exception:
                    pass
            loop.close()

    SdkLog.i(_TAG, f"bin_to_csv: {bin_path} -> {csv_path} ({rows_written} rows)")
    return csv_path
