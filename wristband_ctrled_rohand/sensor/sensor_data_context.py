import asyncio
from collections import deque
from datetime import datetime
import os
import platform
import queue
from queue import Queue, Full, Empty
import struct
import math
import threading
import time
from typing import Dict, List, Optional, Tuple
import csv
from sensor import sensor_utils
from sensor.gforce import DataSubscription, GForce, ImuRawDataConfig, SamplingRate
from sensor.sensor_data import DataType, Sample, SensorData
from sensor.sensor_data_pool import SensorDataPool

from enum import Enum, IntEnum

from sensor.sensor_device import BLEChipType, DeviceInfo
from sensor.sdk_log import SdkLog

_TAG = "SensorProfileDataCtx"

QUAT_SCALE = 1 / 1073741824.0  # 2^30

# 拼接缓冲区最大长度，防止 corrupted/不同步数据无限增长
_MAX_CONCAT_BUFFER_SIZE = 64 * 1024  # 64KB
# 包序号最大允许跳变值，超过视为非法跳变/丢包
_MAX_ALLOWED_PACKAGE_INDEX_DELTA = 100
# 小前跳阈值：差值不超过该值时缺少的包可能随后到达（乱序），暂存 pending queue 等待恢复
_SMALL_JUMP_MAX = 3
# 不符合连续/翻转规则的包在 pending queue 中的最大暂存数量
_PENDING_PACKAGE_MAX = 16

class SensorDataType(IntEnum):
    DATA_TYPE_EEG = 0
    DATA_TYPE_ECG = 1
    DATA_TYPE_ACC = 2
    DATA_TYPE_GYRO = 3
    DATA_TYPE_BRTH = 4
    DATA_TYPE_EMG = 5
    DATA_TYPE_MAG_ANGLE = 6
    DATA_TYPE_QUATERNION = 7
    DATA_TYPE_PPG = 8
    DATA_TYPE_SPO2 = 9
    DATA_TYPE_EULER = 10
    DATA_TYPE_GFORCE_QUAT = 11
    DATA_TYPE_IMPEDANCE = 12
    DATA_TYPE_GEST = 13
    DATA_TYPE_COUNT = 14



class FeatureMaps(Enum):
    GFD_FEAT_GEST = 0x000001000
    GFD_FEAT_EMG = 0x000002000
    GFD_FEAT_MAGANG = 0x00080000
    GFD_FEAT_EEG = 0x000400000
    GFD_FEAT_ECG = 0x000800000
    GFD_FEAT_IMPEDANCE = 0x001000000
    GFD_FEAT_IMU = 0x002000000
    GFD_FEAT_ADS = 0x004000000
    GFD_FEAT_BRTH = 0x008000000
    GFD_FEAT_CONCAT_BLE = 0x80000000
    GFD_FEAT_PPG = 0x10000000
    GFD_FEAT_EULER = 0x000000200
    GFD_FEAT_QUAT = 0x000000400
    GFD_FEAT_ACC = 0x000000040
    GFD_FEAT_GYRO = 0x000000080


class PPGDataMode(IntEnum):

    SPO2_AND_HR = 0
    PPG_RAW = 1
    PPG_AND_SPO2 = 2


class ReadSamplesResult(IntEnum):
    """checkReadSamples 的返回值，与 C++ 三态枚举保持一致"""
    OK = 0
    Repeated = 1
    Error = 2


class SensorProfileDataCtx:
    def __init__(
        self,
        gForce: GForce,
        deviceMac: str,
        buf: Queue[bytes],
        on_reconnect_request: Optional[callable] = None,
    ):
        self.featureMap = 0
        self.notifyDataFlag: DataSubscription = 0

        self.gForce = gForce
        self._chip_type = gForce.get_chip_type() if gForce is not None else BLEChipType.Unknown
        self.deviceMac = deviceMac
        self._device_info: DeviceInfo = None

        self._is_initing = False
        self._is_running = True
        self._is_data_transfering = False
        # 回放场景下 gForce 可能为 None，此时由 load_replay_config 恢复该标志
        self.isUniversalStream: bool = gForce._is_universal_stream if gForce is not None else False
        self._rawDataBuffer: Queue[bytes] = buf
        self._concatDataBuffer = bytearray()

        # 对象池：静态预分配 SensorData / Sample / FlatBuffers 输出槽
        self._object_pool = SensorDataPool(
            sensor_data_slots=32,
            samples_per_slot=1024,
            flatbuffer_slots=32,
            flatbuffer_size=8192,
        )
        # EMG support
        self.isNewEMG = False
        # Quaternion support
        self.isContainQAT6 = False
        # PPG configuration
        self.ppgModel = PPGDataMode.PPG_AND_SPO2
        # self.ppgModel = PPGDataMode.PPG_RAW

        self.sensorDatas: List[SensorData] = list()
        for idx in range(0, SensorDataType.DATA_TYPE_COUNT):
            self.sensorDatas.append(SensorData())
        self.impedanceData: List[float] = list()
        self.saturationData: List[float] = list()
        # 解析结果回调（_process_data/_processUniversalData 入口处设置），
        # 结果直接同步分发，保证背压沿数据通路回传而不是积压在中间线程池
        self._on_data_callback = None
        self.notify_map = {"NTF_GEST": "ON", "NTF_EMG": "ON", "NTF_EEG": "ON", "NTF_ECG": "ON", "NTF_IMU": "ON", "NTF_BRTH": "ON",
                         "NTF_MAG_ANGLE": "ON", "NTF_IMPEDANCE": "ON", "NTF_PPG": "ON", "NTF_SPO2": "ON",
                         "NTF_GFORCE_EULER": "ON",
                         "NTF_GFORCE_QUAT": "ON",
                         "NTF_GFORCE_ACC": "ON",
                         "NTF_GFORCE_GYRO": "ON",
                         }
        self.filter_map = {"FILTER_50HZ": "ON", "FILTER_60HZ": "ON", "FILTER_HPF": "ON", "FILTER_LPF": "ON"}
        self.debugCSVWriter = None
        self.debugCSVPath = None

        # 每个 SensorProfile 独立的 bin 导出开关与目标路径（DEBUG_BLE_DATA_PATH）
        self._data_log_enabled = False
        self._data_log_path = None

        # 守护线程：监控解析线程是否卡住，防止内存无限增长
        self._last_progress_time = time.time()
        self._watchdog_stop_event = threading.Event()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name=f"DataWatchdog-{self.deviceMac}",
        )
        self._watchdog_thread.daemon = True
        self._watchdog_thread.start()

        # 非法跳变监控：持续超过阈值则重启数据通知
        self._illegal_jump_start_time: Optional[float] = None
        self._is_restarting_stream = False

        # 不符合连续/翻转规则的包暂存的 pending queue（按 id(sensorData) 区分数据流），
        # 等后续包到来时再判断接续或补包解码（见 checkReadSamples）
        self._pending_packages: Dict[int, List[Tuple[int, bytes]]] = {}

        # 命令失败导致异常断开后自动重连
        self._on_reconnect_request = on_reconnect_request
        self._disconnected_start_time: Optional[float] = None
        self._reconnect_requested = False

    def close(self):
        self._is_running = False
        if self.debugCSVWriter != None:
            self.debugCSVWriter = None
        # bin finalize（按 DEBUG_BLE_DATA_PATH 导出后删除 temp 原文件），幂等
        self._finalize_bin_export()
        self._stop_watchdog_thread()

    def _finalize_bin_export(self):
        """结束本次 bin 捕获：设置了 DEBUG_BLE_DATA_PATH 时先把 bin 拷贝到
        该位置，再删除 temp 原文件；gForce 为 None（离线回放）时跳过。"""
        gforce = self.gForce
        if gforce is None:
            return
        export_path = self._data_log_path if self._data_log_enabled else None
        try:
            gforce.finalize_bin_recorder(export_path)
        except Exception as e:
            SdkLog.exception(_TAG, f"finalize bin failed: {e}")

    def _stop_watchdog_thread(self):
        """停止数据解析看门狗线程。"""
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            self._watchdog_stop_event.set()
            try:
                self._watchdog_thread.join(timeout=1.0)
            except Exception:
                pass
            self._watchdog_thread = None

    async def _restart_streaming(self):
        """非法跳变持续时重启数据通知。"""
        if self._is_restarting_stream:
            return
        self._is_restarting_stream = True
        try:
            SdkLog.w(_TAG, f"Illegal package index jump persisted > 5s, restarting data notification for {self.deviceMac}")
            await self.stop_streaming()
            await self.start_streaming(self._rawDataBuffer)
            self._illegal_jump_start_time = None
        except Exception as e:
            SdkLog.exception(_TAG, f"Restart streaming failed for {self.deviceMac}: {e}")
        finally:
            self._is_restarting_stream = False

    def _watchdog_loop(self):
        """守护线程：监控解析线程进度，防止卡死后内存无限增长。"""
        WATCHDOG_INTERVAL = 1.0  # 检查间隔
        WATCHDOG_TIMEOUT = 5.0   # 无进度报警阈值
        ILLEGAL_JUMP_TIMEOUT = 5.0  # 非法跳变持续阈值
        RECONNECT_AFTER_DISCONNECT = 3.0  # 断开后等待多久尝试重连
        RECENT_CMD_FAILURE_WINDOW = 30.0  # 命令失败多久内视为相关
        RAW_BUF_CLEAR_THRESHOLD = 1500  # 超过此值且无进度时清空

        while not self._watchdog_stop_event.is_set():
            self._watchdog_stop_event.wait(WATCHDOG_INTERVAL)
            if self._watchdog_stop_event.is_set():
                break

            if not self._is_running:
                continue

            try:
                raw_qsize = self._rawDataBuffer.qsize()
                concat_len = len(self._concatDataBuffer)
                elapsed = time.time() - self._last_progress_time

                # 只在数据传输期间且队列有数据时判断
                if not self.isDataTransfering or raw_qsize == 0:
                    continue

                if elapsed > WATCHDOG_TIMEOUT:
                    SdkLog.w(
                        _TAG,
                        f"Data parsing appears stalled: no progress for {elapsed:.1f}s, "
                        f"raw_buf={raw_qsize}, concat_buf={concat_len}"
                    )
                    # 如果 raw buffer 已经积压到危险水位，直接清空防止内存爆炸
                    if raw_qsize > RAW_BUF_CLEAR_THRESHOLD:
                        SdkLog.w(_TAG, f"Raw buffer too large ({raw_qsize}), clearing to protect memory")
                        try:
                            while not self._rawDataBuffer.empty():
                                self._rawDataBuffer.get_nowait()
                                self._rawDataBuffer.task_done()
                        except Exception:
                            pass
                        self._concatDataBuffer.clear()
                        # 清空后更新时间戳，避免持续报警
                        self._last_progress_time = time.time()

                # 非法跳变持续超过阈值则重启数据通知（回放上下文没有 gForce，跳过）
                if (self.gForce is not None
                        and self._illegal_jump_start_time is not None
                        and not self._is_restarting_stream
                        and self.isDataTransfering):
                    jump_elapsed = time.time() - self._illegal_jump_start_time
                    if jump_elapsed > ILLEGAL_JUMP_TIMEOUT:
                        SdkLog.w(_TAG, f"Illegal jump persisted for {jump_elapsed:.1f}s, restarting stream for {self.deviceMac}")
                        try:
                            asyncio.run_coroutine_threadsafe(
                                self._restart_streaming(), self.gForce.gforce_event_loop
                            )
                        except Exception as e:
                            SdkLog.exception(_TAG, f"Failed to schedule stream restart for {self.deviceMac}: {e}")

                # 命令失败后设备异常断开，触发自动重连
                if self._on_reconnect_request is not None:
                    client = getattr(self.gForce, "client", None)
                    is_connected = getattr(client, "is_connected", False) if client is not None else False
                    if is_connected:
                        self._disconnected_start_time = None
                        self._reconnect_requested = False
                    else:
                        if self._disconnected_start_time is None:
                            self._disconnected_start_time = time.time()
                        disconnected_elapsed = time.time() - self._disconnected_start_time
                        cmd_fail_time = getattr(self.gForce, "last_command_failure_time", None)
                        recent_cmd_failure = (
                            cmd_fail_time is not None
                            and (time.time() - cmd_fail_time) < RECENT_CMD_FAILURE_WINDOW
                        )
                        if (disconnected_elapsed > RECONNECT_AFTER_DISCONNECT
                                and recent_cmd_failure
                                and not self._reconnect_requested):
                            SdkLog.w(_TAG, f"Device disconnected after command failure, scheduling reconnect for {self.deviceMac}")
                            self._reconnect_requested = True
                            try:
                                self._on_reconnect_request()
                            except Exception as e:
                                SdkLog.exception(_TAG, f"Reconnect request failed for {self.deviceMac}: {e}")
            except Exception as e:
                SdkLog.exception(_TAG, f"Watchdog error: {e}")

    def clear(self):
        for sensorData in self.sensorDatas:
            sensorData.clear()
        self.impedanceData.clear()
        self.saturationData.clear()
        self._concatDataBuffer.clear()
        self._rawDataBuffer.queue.clear()
        self._pending_packages.clear()

    def reset(self):
        self.notifyDataFlag = 0
        self.clear()

    @property
    def isDataTransfering(self) -> bool:

        return self._is_data_transfering

    def hasInit(self):
        return not self._is_initing and self.featureMap != 0 and self.notifyDataFlag != 0

    def getChipType(self) -> BLEChipType:
        return self._chip_type

    def dump_replay_config(self) -> dict:
        """导出当前解析配置，用于写入 bin 文件（离线回放时恢复上下文）。"""
        sensor_datas = []
        for idx, data in enumerate(self.sensorDatas):
            if data is None or data.packageSampleCount <= 0 or data.channelCount <= 0:
                continue
            sensor_datas.append({
                "type_index": int(idx),
                "data_type": int(data.dataType),
                "sample_rate": data.sampleRate,
                "channel_count": data.channelCount,
                "package_sample_count": data.packageSampleCount,
                "min_package_sample_count": data.minPackageSampleCount,
                "package_index_length": data.packageIndexLength,
                "resolution_bits": data.resolutionBits,
                "resolution_signed": data.resolutionSigned,
                "channel_mask": data.channelMask,
                "k": data.K,
            })
        device_info = {}
        if self._device_info is not None:
            try:
                device_info = dict(vars(self._device_info))
            except Exception:
                device_info = {}
        return {
            "version": 1,
            "device_mac": self.deviceMac,
            "device_name": self._device_info.DeviceName if self._device_info else "",
            "chip_type": int(getattr(self._chip_type, "value", -1)),
            "is_universal_stream": bool(self.isUniversalStream),
            "feature_map": int(self.featureMap),
            "notify_data_flag": int(self.notifyDataFlag),
            "is_new_emg": bool(self.isNewEMG),
            "is_contain_qat6": bool(self.isContainQAT6),
            "ppg_model": int(self.ppgModel),
            "device_info": device_info,
            "sensor_datas": sensor_datas,
        }

    def load_replay_config(self, config: dict):
        """从 bin 文件的配置记录恢复解析上下文（离线回放用）。

        会重置各数据类型的包序号跟踪状态，使回放从干净状态开始。
        """
        self.featureMap = int(config.get("feature_map", 0))
        # notifyDataFlag 是组合位掩码，直接存 int（DataSubscription 是 IntEnum，
        # 组合值无法构造枚举实例）
        self.notifyDataFlag = int(config.get("notify_data_flag", 0))
        self.isNewEMG = bool(config.get("is_new_emg", False))
        self.isContainQAT6 = bool(config.get("is_contain_qat6", False))
        self.ppgModel = PPGDataMode(int(config.get("ppg_model", int(PPGDataMode.PPG_AND_SPO2))))
        self.isUniversalStream = bool(config.get("is_universal_stream", self.isUniversalStream))
        try:
            self._chip_type = BLEChipType(int(config.get("chip_type", -1)))
        except Exception:
            self._chip_type = BLEChipType.Unknown

        info = DeviceInfo()
        for key, value in (config.get("device_info") or {}).items():
            if hasattr(info, key):
                try:
                    setattr(info, key, value)
                except Exception:
                    pass
        self._device_info = info

        # sensorDatas 即将被替换：先清空 pending queue，避免 id 复用后挂错暂存包
        self._pending_packages.clear()
        for item in config.get("sensor_datas") or []:
            try:
                idx = int(item.get("type_index"))
                if idx < 0 or idx >= int(SensorDataType.DATA_TYPE_COUNT):
                    continue
                data = SensorData()
                data.deviceMac = self.deviceMac
                data.dataType = DataType(int(item.get("data_type")))
                data.sampleRate = float(item.get("sample_rate", 0))
                data.channelCount = int(item.get("channel_count", 0))
                data.packageSampleCount = int(item.get("package_sample_count", 0))
                data.minPackageSampleCount = int(item.get("min_package_sample_count", 0))
                data.packageIndexLength = int(item.get("package_index_length", 0))
                data.resolutionBits = int(item.get("resolution_bits", 0))
                data.resolutionSigned = int(item.get("resolution_signed", 0))
                data.channelMask = int(item.get("channel_mask", 0))
                data.K = float(item.get("k", 0.0))
                data.clear()
                self.sensorDatas[idx] = data
            except Exception:
                SdkLog.exception(_TAG, "Unexpected error loading replay sensor data config")

        self._is_initing = False
        self._is_data_transfering = True

    def _buildNotifyDataFlag(self):
        """根据当前 notify_map 和能力位重建 notifyDataFlag 订阅掩码。

        此方法在 init() 结束以及 setParam 动态切换数据流时调用。
        """
        flag = DataSubscription(0)
        if self.hasConcatBLE():
            flag |= DataSubscription.DNF_CONCAT_BLE

        if self.hasEMG() and self.notify_map.get("NTF_EMG") == "ON":
            flag |= DataSubscription.EMG_RAW
        if self.hasGEST() and self.notify_map.get("NTF_GEST") == "ON":
            flag |= DataSubscription.DNF_TYPE_GEST_EXT
        if self.hasEEG() and self.notify_map.get("NTF_EEG") == "ON":
            flag |= DataSubscription.DNF_EEG
        if self.hasECG() and self.notify_map.get("NTF_ECG") == "ON":
            flag |= DataSubscription.DNF_ECG
        if self.hasImpedance() and self.notify_map.get("NTF_IMPEDANCE") == "ON":
            flag |= DataSubscription.DNF_IMPEDANCE
        if self.hasBrth() and self.notify_map.get("NTF_BRTH") == "ON":
            flag |= DataSubscription.DNF_BRTH
        if self.hasIMU() and self.notify_map.get("NTF_IMU") == "ON":
            flag |= DataSubscription.DNF_IMU
        if self.hasEuler() and self.notify_map.get("NTF_GFORCE_EULER") == "ON":
            flag |= DataSubscription.EULERANGLE
        if self.hasQuat() and self.notify_map.get("NTF_GFORCE_QUAT") == "ON":
            flag |= DataSubscription.QUATERNION
        if self.hasAcc() and self.notify_map.get("NTF_GFORCE_ACC") == "ON":
            flag |= DataSubscription.ACCELERATE
        if self.hasGyro() and self.notify_map.get("NTF_GFORCE_GYRO") == "ON":
            flag |= DataSubscription.GYROSCOPE
        if self.hasPPG() and (self.notify_map.get("NTF_PPG") == "ON" or self.notify_map.get("NTF_SPO2") == "ON"):
            flag |= DataSubscription.DNF_PPG
        if self.hasMagAngle() and self.notify_map.get("NTF_MAG_ANGLE") == "ON":
            flag |= DataSubscription.DNF_MAG_ANGLE_EXT

        self.notifyDataFlag = flag

    def hasGEST(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_GEST.value) != 0
    
    def hasEMG(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_EMG.value) != 0

    def hasEEG(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_EEG.value) != 0

    def hasECG(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_ECG.value) != 0

    def hasImpedance(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_IMPEDANCE.value) != 0

    def hasIMU(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_IMU.value) != 0

    def hasBrth(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_BRTH.value) != 0

    def hasMagAngle(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_MAGANG.value) != 0

    def hasConcatBLE(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_CONCAT_BLE.value) != 0

    def hasPPG(self):
        return (self.featureMap & FeatureMaps.GFD_FEAT_PPG.value) != 0

    def hasEuler(self):

        return (self.featureMap & FeatureMaps.GFD_FEAT_EULER.value) != 0

    def hasQuat(self):

        return (self.featureMap & FeatureMaps.GFD_FEAT_QUAT.value) != 0

    def hasAcc(self):

        return (self.featureMap & FeatureMaps.GFD_FEAT_ACC.value) != 0

    def hasGyro(self):

        return (self.featureMap & FeatureMaps.GFD_FEAT_GYRO.value) != 0


    async def initEMG(self, packageCount: int) -> int:
        config = await self.gForce.get_emg_raw_data_config()
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_EMG
        data.sampleRate = 500
        data.resolutionBits = 0
        data.resolutionSigned = 0
        data.channelCount = 8
        data.channelMask = config.channel_mask
        data.minPackageSampleCount = packageCount
        data.packageSampleCount = config.batch_len

        data.clear()

        if (self.isNewEMG):
            # new emg
            data.packageIndexLength = 2
            data.resolutionBits = 0
            data.resolutionSigned = 1
            gain = 6
            data.K = 4000000.0 / 8388607.0 / gain
            config.resolution = 8
        else:
            # old emg
            data.packageIndexLength = 1
            data.resolutionBits = 7
            data.resolutionSigned = 1
            gain = 1200
            min_voltage = -1.25 * 1000000
            max_voltage = 1.25 * 100000
            div = 127.0
            conversion_factor = (max_voltage - min_voltage) / gain / div
            data.K = conversion_factor
            config.resolution = 8

        config.fs = SamplingRate.HZ_500
        config.channel_mask = 255
        config.batch_len = 128

        if self.isNewEMG:
            # 新版 EMG：bit0=gesture，bit1=emg；Gesture 依赖 EMG，EMG 关闭时 Gesture 同步关闭
            emg_bit = 1 if self.notify_map.get("NTF_EMG") == "ON" else 0
            gest_bit = 1 if (emg_bit and self.notify_map.get("NTF_GEST") == "ON") else 0
            await self.gForce.set_function_switch((emg_bit << 1) | gest_bit)
            await asyncio.sleep(0.5)

        await self.gForce.set_emg_raw_data_config(config)
        await self.gForce.set_package_id(True)

        if self.isNewEMG:
            if self.hasConcatBLE():
                data.packageSampleCount = 15
            else:
                data.packageSampleCount = 8

        self.sensorDatas[SensorDataType.DATA_TYPE_EMG] = data

        # 新 EMG + OYM 芯片：默认只开 EMG，关闭 Gesture 和 IMU
        if self.isNewEMG and self._chip_type == BLEChipType.OYM:
            self.notify_map["NTF_GEST"] = "OFF"
            self.notify_map["NTF_IMU"] = "OFF"
            self.notify_map["NTF_GFORCE_ACC"] = "OFF"
            self.notify_map["NTF_GFORCE_GYRO"] = "OFF"
            self.notify_map["NTF_GFORCE_QUAT"] = "OFF"
            self.notify_map["NTF_GFORCE_EULER"] = "OFF"


        return data.channelCount

    
    async def initGesture(self, packageCount: int) -> int:
        emgSampleRate = self._device_info.EmgSampleRate
        if emgSampleRate <= 0:
            if self.isNewEMG:
                return 0
            emgSampleRate = 500#for old emg device, default 500Hz
        
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_GEST
        data.sampleRate = emgSampleRate / 32.0
        data.resolutionBits = 0
        data.resolutionSigned = 0
        data.channelCount = 1
        data.channelMask = 1
        data.minPackageSampleCount = 1
        data.packageSampleCount = 1
        data.K = 1
        if not self.isNewEMG:
            data.packageIndexLength = 1
            # 开启数据包序号：gesture-only 设备不经过 initEMG，需要在此补开，
            # 否则设备上报无序号数据包导致无法解析
            await self.gForce.set_package_id(True)
            # 老设备 Gesture 与 EMG 互斥；仅当设备真有 EMG 能力时才互斥，
            # 否则纯手势设备（如 gForce200）会被误关 Gesture
            if self.hasEMG() and self.notify_map["NTF_EMG"] == "ON":
                self.notify_map["NTF_GEST"] = "OFF"

        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_GEST] = data

        return data.channelCount
    
    async def initEEG(self, packageCount: int) -> int:
        config = await self.gForce.get_eeg_raw_data_config()
        cap = await self.gForce.get_eeg_raw_data_cap()
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_EEG
        data.sampleRate = config.fs
        data.resolutionBits = config.resolution
        data.resolutionSigned = 1
        data.channelCount = cap.channel_count
        data.channelMask = config.channel_mask
        data.minPackageSampleCount = packageCount
        data.packageSampleCount = config.batch_len
        data.K = config.K
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_EEG] = data
        return data.channelCount

    async def initECG(self, packageCount: int) -> int:
        config = await self.gForce.get_ecg_raw_data_config()
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_ECG
        data.sampleRate = config.fs
        data.resolutionBits = config.resolution
        data.resolutionSigned = 1   
        data.channelCount = 1
        data.channelMask = config.channel_mask
        data.minPackageSampleCount = packageCount
        data.packageSampleCount = config.batch_len
        data.K = config.K
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_ECG] = data
        return data.channelCount
    
    async def initImpedance(self, packageCount: int) -> int:

        channelCount = self.sensorDatas[SensorDataType.DATA_TYPE_EEG].channelCount + self.sensorDatas[SensorDataType.DATA_TYPE_ECG].channelCount + self.sensorDatas[SensorDataType.DATA_TYPE_EMG].channelCount
        if channelCount <= 0:
            return 0
        
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_IMPEDANCE
        if self.sensorDatas[SensorDataType.DATA_TYPE_EEG].sampleRate > 0:
            data.sampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_EEG].sampleRate
        elif self.sensorDatas[SensorDataType.DATA_TYPE_ECG].sampleRate > 0:
            data.sampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_ECG].sampleRate
        elif self.sensorDatas[SensorDataType.DATA_TYPE_EMG].sampleRate > 0:
            data.sampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_EMG].sampleRate
        else:
            data.sampleRate = 0
            
        data.resolutionBits = 0
        data.resolutionSigned = 0
        data.channelCount = channelCount
        data.channelMask = 1 << channelCount - 1
        data.minPackageSampleCount = 1
        data.packageSampleCount = 1
        data.K = 1
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_IMPEDANCE] = data

        return data.channelCount
    
    async def initPPG(self, packageCount: int) -> int:

        config = await self.gForce.get_ppg_raw_data_config()
        config.mode = self.ppgModel
        config.period = 1
        config.fs = 50
        await self.gForce.set_ppg_raw_data_config(config)

        data = SensorData()
        data.dataType = DataType.NTF_PPG
        data.deviceMac = self.deviceMac
        data.sampleRate = config.fs
        data.channelMask = 255
        data.minPackageSampleCount = packageCount
        data.packageSampleCount = config.batch_len
        data.K = 1.0
        data.resolutionBits = 24
        data.resolutionSigned = 0
        data.channelCount = 2
        self.sensorDatas[SensorDataType.DATA_TYPE_PPG] = data
        data.clear()

        data = SensorData()
        data.dataType = DataType.NTF_SPO2
        data.deviceMac = self.deviceMac
        data.sampleRate = config.period
        data.channelMask = 255
        data.minPackageSampleCount = 1
        data.packageSampleCount = 1
        data.K = 1.0
        data.resolutionBits = 17
        data.resolutionSigned = 1
        data.channelCount = 2
        self.sensorDatas[SensorDataType.DATA_TYPE_SPO2] = data
        data.clear()

        return data.channelCount


    async def initIMU(self, packageCount: int) -> int:
        SdkLog.d(_TAG, "initIMU(...)")
        IMU_TYPE_QAT6 = 0x0004
        min_package_sample_count = 2 if self._chip_type == BLEChipType.OYM else 1
        self.isContainQAT6 = False

        if not self.hasIMU():
            SdkLog.w(_TAG, "IMU not supported")
            return -1

        imu_cap = await self.gForce.get_imu_cap_data_config()
        if imu_cap is not None:
            channel_mask, samp_rate, sample_count = imu_cap
            if (channel_mask & IMU_TYPE_QAT6) == IMU_TYPE_QAT6:
                self.isContainQAT6 = True

            cfg = ImuRawDataConfig()
            cfg.channel_count = channel_mask
            cfg.fs = samp_rate
            cfg.batch_len = min_package_sample_count
            await self.gForce.set_imu_raw_data_config(cfg)

        config = await self.gForce.get_imu_raw_data_config()
        if config is None:
            return -1

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_ACC
        data.sampleRate = config.fs
        data.resolutionBits = 16
        data.resolutionSigned = 1
        data.channelCount = 3
        data.channelMask = 255
        data.minPackageSampleCount = min_package_sample_count
        data.packageSampleCount = config.batch_len
        data.K = config.accK
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_ACC] = data

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_GYRO
        data.sampleRate = config.fs
        data.resolutionBits = 16
        data.resolutionSigned = 1
        data.channelCount = 3
        data.channelMask = 255
        data.minPackageSampleCount = min_package_sample_count
        data.packageSampleCount = config.batch_len
        data.K = config.gyroK
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_GYRO] = data

        # Initialize quaternion data if supported
        if self.isContainQAT6:
            data = SensorData()
            data.deviceMac = self.deviceMac
            data.dataType = DataType.NTF_QUATERNION
            data.sampleRate = config.fs
            data.resolutionBits = 31 # 32-bit signed integer
            data.resolutionSigned = 1
            data.channelCount = 4  # w, x, y, z
            data.channelMask = 0b1110  # we don't read the first channel for quaternion data
            data.minPackageSampleCount = min_package_sample_count
            data.packageSampleCount = config.batch_len
            data.K = 1.0 / 1073741824.0  # 1 / 2^30
            data.clear()
            self.sensorDatas[SensorDataType.DATA_TYPE_QUATERNION] = data

            data = SensorData()
            data.deviceMac = self.deviceMac
            data.dataType = DataType.NTF_EULER_DATA
            data.sampleRate = config.fs
            data.resolutionBits = 0
            data.resolutionSigned = 1
            data.channelCount = 3           # pitch, roll, yaw
            data.channelMask = 0b0111
            data.packageIndexLength = 0
            data.minPackageSampleCount = min_package_sample_count
            data.packageSampleCount = config.batch_len
            data.K = 1.0
            data.clear()
            self.sensorDatas[SensorDataType.DATA_TYPE_EULER] = data

        # EEG + OYM 设备默认不订阅 IMU；带 PPG 的设备保留
        if not (self._chip_type == BLEChipType.OYM
                and self.hasEEG()
                and not self.hasPPG()):
            self.notifyDataFlag |= DataSubscription.DNF_IMU
        if self._device_info is not None:
            self._device_info.AccChannelCount = 3
            self._device_info.GyroChannelCount = 3
            self._device_info.AccSampleRate = config.fs
            self._device_info.GyroSampleRate = config.fs
            if self.isContainQAT6:
                self._device_info.QuatChannelCount = 4
                self._device_info.EulerChannelCount = 3
                self._device_info.QuatSampleRate = config.fs
                self._device_info.EulerSampleRate = config.fs

        return config.channel_count

    async def initEuler(self, packageCount: int) -> int:

        # config = await self.gForce.get_imu_raw_data_config()

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_EULER_DATA
        data.sampleRate = 40
        data.resolutionBits = 32        # float32
        data.channelCount = 3           # roll, pitch, yaw
        data.channelMask = 0b0111
        data.packageIndexLength = 1
        data.minPackageSampleCount = 1
        data.packageSampleCount = 1
        data.K = 1.0
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_EULER] = data
        return data.channelCount

    async def initGForceQuat(self, packageCount: int) -> int:

        # config = await self.gForce.get_imu_raw_data_config()

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_QUATERNION
        data.sampleRate = 40
        data.resolutionBits = 32        # float32
        data.channelCount = 4           # w, x, y, z
        data.channelMask = 0b1111
        data.packageIndexLength = 1
        data.minPackageSampleCount = 1
        data.packageSampleCount = 1
        data.K = 1.0
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_GFORCE_QUAT] = data
        return data.channelCount

    async def initGForceAcc(self, packageCount: int) -> int:

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_ACC
        data.sampleRate = 40
        data.resolutionBits = 31        # int32
        data.resolutionSigned = 1
        data.channelCount = 3           # x, y, z
        data.channelMask = 0b0111
        data.packageIndexLength = 1
        data.minPackageSampleCount = 1
        data.packageSampleCount = 1
        data.K = 1.0 / 65536.0
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_ACC] = data
        return data.channelCount

    async def initGForceGyro(self, packageCount: int) -> int:

        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_GYRO
        data.sampleRate = 40
        data.resolutionBits = 31        # int32
        data.resolutionSigned = 1
        data.channelCount = 3           # x, y, z
        data.channelMask = 0b0111
        data.packageIndexLength = 1
        data.minPackageSampleCount = 1
        data.packageSampleCount = 1
        data.K = 1.0 / 65536.0
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_GYRO] = data
        return data.channelCount


    async def initBrth(self, packageCount: int) -> int:
        config = await self.gForce.get_brth_raw_data_config()
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_BRTH
        data.sampleRate = config.fs
        data.resolutionBits = config.resolution
        data.resolutionSigned = 1
        data.channelCount = 1
        data.channelMask = config.channel_mask
        data.minPackageSampleCount = 1
        data.packageSampleCount = config.batch_len
        data.K = config.K
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_BRTH] = data
        return data.channelCount

    async def initMagAngle(self, packageCount: int) -> int:
        data = SensorData()
        data.deviceMac = self.deviceMac
        data.dataType = DataType.NTF_MAG_ANGLE_DATA
        data.sampleRate = 40
        data.resolutionBits = 8
        data.resolutionSigned = 0
        data.channelCount = 1
        data.channelMask = 1
        data.minPackageSampleCount = 1
        data.packageSampleCount = 1
        data.K = 1
        data.packageIndexLength = 2
        data.clear()
        self.sensorDatas[SensorDataType.DATA_TYPE_MAG_ANGLE] = data
        return data.channelCount

    async def initDataTransfer(self, isGetFeature: bool) -> int:
        if isGetFeature:
            self.featureMap = await self.gForce.get_feature_map()
            return self.featureMap
        else:
            await self.gForce.set_subscription(self.notifyDataFlag)
            return self.notifyDataFlag

    async def fetchDeviceInfo(self) -> DeviceInfo:
        info = DeviceInfo()
        if platform.system() != "Linux":
            info.MTUSize = self.gForce.client.mtu_size
        else:
            info.MTUSize = 0
        # print("get_device_name")
        info.DeviceName = await self.gForce.get_device_name()
        # print("get_model_number")
        info.ModelName = await self.gForce.get_model_number()
        # print("get_hardware_revision")
        info.HardwareVersion = await self.gForce.get_hardware_revision()
        # print("get_firmware_revision")
        info.FirmwareVersion = await self.gForce.get_firmware_revision()
        return info

    async def init(self, packageCount: int) -> bool:
        if self._is_initing:
            return False
        try:
            self._is_initing = True
            info = await self.fetchDeviceInfo()
            self._device_info = info
            await self.initDataTransfer(True)

            isNewEMG = True
            try:
                if not self.hasConcatBLE() and (self._device_info.DeviceName.startswith("gForce") or self._device_info.DeviceName.startswith(
                        "OHand") or self._device_info.DeviceName.startswith(
                        "ORE-") or self._device_info.DeviceName.startswith(
                        "OYEM-") or self._device_info.DeviceName.startswith("ORehab")):
                    isNewEMG = False
            except Exception as e:
                SdkLog.exception(_TAG, "Unexpected error")

            self.isNewEMG = isNewEMG
            if (not self.isNewEMG):
                self.filter_map.clear()

            # 非 RFSTAR 芯片设备默认关闭 IMU（RFSTAR 保留默认开启）；
            # 纯手势+IMU 设备（无 EMG，如 gForce200）保留 IMU 默认开启
            gesture_only = self.hasGEST() and not self.hasEMG()
            if self._chip_type != BLEChipType.RFSTAR and not gesture_only:
                self.notify_map["NTF_IMU"] = "OFF"
                self.notify_map["NTF_GFORCE_ACC"] = "OFF"
                self.notify_map["NTF_GFORCE_GYRO"] = "OFF"
                self.notify_map["NTF_GFORCE_QUAT"] = "OFF"
                self.notify_map["NTF_GFORCE_EULER"] = "OFF"

            if self.hasConcatBLE():
                self.notifyDataFlag |= DataSubscription.DNF_CONCAT_BLE

            if self.hasEMG():
                info.EmgChannelCount = await self.initEMG(packageCount)
                info.EmgSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_EMG].sampleRate

            if self.hasGEST():
                await self.initGesture(packageCount)

            if self.hasEEG():
                info.EegChannelCount = await self.initEEG(packageCount)
                info.EegSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_EEG].sampleRate

            if self.hasECG():
                info.EcgChannelCount = await self.initECG(packageCount)
                info.EcgSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_ECG].sampleRate

            if self.hasImpedance():
                info.ImpeChannelCount = await self.initImpedance(packageCount)
                info.ImpeSampleRate = 1

            if self.hasBrth():
                info.BrthChannelCount = await self.initBrth(packageCount)
                info.BrthSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_BRTH].sampleRate

            if self.hasIMU():
                await self.initIMU(packageCount)
                info.AccChannelCount = 3
                info.GyroChannelCount = 3
                info.AccSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_ACC].sampleRate
                info.GyroSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_GYRO].sampleRate
                if self.isContainQAT6:
                    info.QuatChannelCount = 4
                    info.EulerChannelCount = 3
                    info.QuatSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_QUATERNION].sampleRate
                    info.EulerSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_EULER].sampleRate

            if self.hasEuler():
                info.EulerChannelCount = await self.initEuler(packageCount)
                info.EulerSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_EULER].sampleRate

            if self.hasQuat():
                info.QuatChannelCount = await self.initGForceQuat(packageCount)
                info.QuatSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_GFORCE_QUAT].sampleRate

            if self.hasAcc():
                info.AccChannelCount = await self.initGForceAcc(packageCount)
                info.AccSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_ACC].sampleRate

            if self.hasGyro():
                info.GyroChannelCount = await self.initGForceGyro(packageCount)
                info.GyroSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_GYRO].sampleRate

            if self.hasPPG():
                info.PpgChannelCount = await self.initPPG(packageCount)
                info.Spo2ChannelCount = 2

                if self.ppgModel == PPGDataMode.PPG_RAW:

                    info.Spo2SampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_SPO2].sampleRate
                if self.ppgModel == PPGDataMode.SPO2_AND_HR:

                    info.PpgSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_SPO2].sampleRate
                else:
                    info.PpgSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_PPG].sampleRate
                    info.Spo2SampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_SPO2].sampleRate

            if self.hasMagAngle():
                magAngleChannelCount = await self.initMagAngle(packageCount)
                info.MagAngleChannelCount = magAngleChannelCount
                info.MagAngleSampleRate = self.sensorDatas[SensorDataType.DATA_TYPE_MAG_ANGLE].sampleRate

            self._device_info = info

            self._buildNotifyDataFlag()

            if not self.isUniversalStream:
                await self.initDataTransfer(False)

            # 把解析配置写入 bin 文件，供离线回放恢复上下文
            try:
                if self.gForce is not None:
                    self.gForce.write_bin_config(self.dump_replay_config())
            except Exception as e:
                SdkLog.exception(_TAG, f"Write bin replay config failed: {e}")

            self._is_initing = False
            return True
        except Exception as e:
            self._is_initing = False
            device_name = self._device_info.DeviceName if self._device_info else "Unknown"
            raise RuntimeError("Init %s fail: %s" % (device_name, e))
            return False

    async def start_streaming(self) -> bool:
        if self._is_data_transfering:
            return True
        self._is_data_transfering = True
        self._rawDataBuffer.queue.clear()
        self._concatDataBuffer.clear()
        self.clear()

        if not self.isUniversalStream:
            await self.gForce.start_streaming(self._rawDataBuffer)
        else:
            await self.gForce.set_subscription(self.notifyDataFlag)

        # 每次起流都补写配置记录：stop 后 bin 会在 temp 重新生成，
        # 保证任何一段 bin 都自描述、可离线回放/解析
        if self.hasInit() and self.gForce is not None:
            try:
                self.gForce.write_bin_config(self.dump_replay_config())
            except Exception as e:
                SdkLog.exception(_TAG, f"Write bin replay config failed: {e}")

        return True

    async def stop_streaming(self) -> bool:
        if not self._is_data_transfering:
            return True

        self._is_data_transfering = False

        try:

            if not self.isUniversalStream:
                await self.gForce.stop_streaming()
            else:
                await self.gForce.set_subscription(0)

            while self._is_running and not self._rawDataBuffer.empty():
                await asyncio.sleep(0.1)

        except Exception as e:
            raise RuntimeError("Stop stream %s fail: %s" % (self._device_info.DeviceName, e))
            return False

        # 结束本次捕获：按 DEBUG_BLE_DATA_PATH 导出 bin 后删除 temp 原文件
        self._finalize_bin_export()
        return True

    async def setFilter(self, filter: str, value: str) -> str:
        if not self.filter_map:
            return "ERROR: Filter not supported on this device"
        self.filter_map[filter] = value
        switch = 0
        for filter in self.filter_map.keys():
            value = self.filter_map[filter]
            if filter == "FILTER_50HZ":
                if value == "ON":
                    switch |= 1
            elif filter == "FILTER_60HZ":
                if value == "ON":
                    switch |= 2
            elif filter == "FILTER_HPF":
                if value == "ON":
                    switch |= 4
            elif filter == "FILTER_LPF":
                if value == "ON":
                    switch |= 8
        try:
            ret = await self.gForce.set_firmware_filter_switch(switch)
            if ret == None:
                return "ERROR: not success"
            return "OK"
        except Exception as e:
            SdkLog.exception(_TAG, f"setFilter failed: {e}")
            return "ERROR: " + str(e)

    ####################################################################################

    def _drain_parsed_one(self, buf: Queue[bytes]) -> bool:
        """从解析结果队列取出一条结果并同步交给数据回调；无结果可取返回 False。

        同步分发（不经中间线程池），保证结果队列满时背压能沿数据通路回传。
        """
        try:
            fb_bytes = buf.get_nowait()
        except Empty:
            return False
        sensorData: SensorData = None
        try:
            sensorData = self._object_pool.acquire_sensor_data()
            sensorData = SensorData.from_flatbuffers_pooled(fb_bytes, sensorData, self._object_pool)
        except Exception:
            # 反序列化失败：丢弃该条，避免阻塞后续结果
            SdkLog.exception(_TAG, "Unexpected error deserializing parsed data")
            buf.task_done()
            return True
        if not sensor_utils._terminated and sensorData is not None and self._on_data_callback is not None:
            try:
                self._on_data_callback(sensorData)
            except Exception as e:
                SdkLog.exception(_TAG, "Unexpected error")
        buf.task_done()
        return True

    async def _process_data(self, buf: Queue[bytes], on_data_callback, on_error_callback=None):
        self._on_data_callback = on_data_callback
        while self._is_running:
            while self._is_running and self._rawDataBuffer.empty():
                if self._is_running and self.isDataTransfering and not buf.empty():
                    self._drain_parsed_one(buf)
                else:
                    await asyncio.sleep(0.01)
                continue

            try:
                while self._is_running and not self._rawDataBuffer.empty():
                    data = self._rawDataBuffer.get_nowait()

                    if self.notifyDataFlag & DataSubscription.DNF_CONCAT_BLE != 0:
                        if len(self._concatDataBuffer) > _MAX_CONCAT_BUFFER_SIZE:
                            SdkLog.w(_TAG, f"Concat buffer exceeded {_MAX_CONCAT_BUFFER_SIZE}, clearing")
                            self._concatDataBuffer.clear()
                        self._concatDataBuffer.extend(data)
                    else:
                        self._processDataPackage(data, buf, on_error_callback)

                    self._rawDataBuffer.task_done()
                    self._last_progress_time = time.time()
            except Exception as e:
                SdkLog.exception(_TAG, "Unexpected error")

            if self.notifyDataFlag & DataSubscription.DNF_CONCAT_BLE != 0:
                try:
                    index = 0
                    last_cut = -1
                    data_size = len(self._concatDataBuffer)

                    while self._is_running:
                        if index >= data_size:
                            break

                        if self._concatDataBuffer[index] == 0x55:
                            if (index + 1) >= data_size:
                                index = data_size
                                continue
                            n = self._concatDataBuffer[index + 1]
                            if n < 2 or (index + 1 + n + 1) >= data_size:
                                index += 1
                                continue
                            crc8 = (self._concatDataBuffer[index + 1 + n + 1])
                            calc_crc = sensor_utils.calc_crc8(self._concatDataBuffer[index + 2: index + 2 + n])
                            if crc8 != calc_crc:
                                index += 1
                                continue
                            if self._is_data_transfering:
                                data_package = bytes(self._concatDataBuffer[index + 2: index + 2 + n])
                                if self._processDataPackage(data_package, buf, on_error_callback):
                                    last_cut = index = index + 2 + n
                            index += 1
                        else:
                            index += 1

                    if last_cut > 0:
                        self._concatDataBuffer = self._concatDataBuffer[last_cut + 1:]
                        last_cut = -1
                        index = 0

                    self._last_progress_time = time.time()
                except Exception as e:
                    SdkLog.exception(_TAG, "Unexpected error in concat data processing")

    def _processDataPackage(self, data: bytes, buf: Queue[bytes], on_error_callback=None) -> bool:
        if not data:
            return False
        v = data[0] & 0x7F

        def dispatch(sensor_data_type: SensorDataType, data_offset: int, data_gap: int) -> bool:
            sensor_data = self.sensorDatas[sensor_data_type]
            res = self.checkReadSamples(data, sensor_data, data_offset, data_gap, on_error_callback)
            if res == ReadSamplesResult.Error:
                return False
            if res == ReadSamplesResult.OK:
                self.sendSensorData(sensor_data, buf)
            return True

        if v == DataType.NTF_IMPEDANCE or v == DataType.NTF_IMPEDANCE_EXT:
            return self._process_impedance_samples(v, data, buf, on_error_callback)
        elif v == DataType.NTF_MAG_ANGLE_DATA:
            return dispatch(SensorDataType.DATA_TYPE_MAG_ANGLE, 4, 0)
        elif v == DataType.NTF_EMG:
            return dispatch(SensorDataType.DATA_TYPE_EMG, self.sensorDatas[SensorDataType.DATA_TYPE_EMG].packageIndexLength + 1, 0)
        elif v == DataType.NTF_GEST:
            return self._process_gesture_samples(v, data, buf, on_error_callback)
        elif v == DataType.NTF_EEG:
            return dispatch(SensorDataType.DATA_TYPE_EEG, 3, 0)
        elif v == DataType.NTF_ECG:
            return dispatch(SensorDataType.DATA_TYPE_ECG, 3, 0)
        elif v == DataType.NTF_BRTH:
            return dispatch(SensorDataType.DATA_TYPE_BRTH, 3, 0)
        elif v == DataType.NTF_IMU and self.hasIMU():
            return self._process_imu_samples(data, buf, on_error_callback)
        elif v == DataType.NTF_PPG and self.hasPPG() and self.notify_map.get("NTF_PPG") == "ON":
            return dispatch(SensorDataType.DATA_TYPE_PPG, 3, 0)
        elif v == DataType.NTF_SPO2 and self.hasPPG() and self.notify_map.get("NTF_SPO2") == "ON":
            return dispatch(SensorDataType.DATA_TYPE_SPO2, 3, 0)
        elif v == DataType.NTF_EULER_DATA and self.hasEuler():
            return dispatch(SensorDataType.DATA_TYPE_EULER, self.sensorDatas[SensorDataType.DATA_TYPE_EULER].packageIndexLength + 1, 0)
        elif v == DataType.NTF_QUATERNION and self.hasQuat():
            return dispatch(SensorDataType.DATA_TYPE_GFORCE_QUAT, self.sensorDatas[SensorDataType.DATA_TYPE_GFORCE_QUAT].packageIndexLength + 1, 0)
        elif v == DataType.NTF_ACC and self.hasAcc():
            return dispatch(SensorDataType.DATA_TYPE_ACC, self.sensorDatas[SensorDataType.DATA_TYPE_ACC].packageIndexLength + 1, 0)
        elif v == DataType.NTF_GYRO and self.hasGyro():
            return dispatch(SensorDataType.DATA_TYPE_GYRO, self.sensorDatas[SensorDataType.DATA_TYPE_GYRO].packageIndexLength + 1, 0)
        else:
            # Unknown data type is treated as a parse error; do not spam warnings.
            return False

    def _process_gesture_samples(self, type, data: bytes, buf: Queue[bytes], on_error_callback=None) -> bool:
        if (len(data) < 7):
            if on_error_callback:
                on_error_callback("Incomplete Gesture packet received")
            return False
        
        sensor_data = self.sensorDatas[SensorDataType.DATA_TYPE_GEST]
        if sensor_data is None or sensor_data.sampleRate <= 0:
            return True

        res = self.checkReadSamples(data, sensor_data, 0, -1, on_error_callback)
        if res == ReadSamplesResult.Error:
            return False
        if res == ReadSamplesResult.Repeated:
            return True
        sampleInterval = 1000.0 / sensor_data.sampleRate
        lastSampleIndex = sensor_data.lastPackageCounter * sensor_data.packageSampleCount
        sensor_data.channelSamples = []

        samples = []
        sample = self._object_pool.acquire_sample()
        
        offset = sensor_data.packageIndexLength + 1
        sample.data = data[offset]
        offset += 1
        sample.rawData = data[offset]
        offset += 1
        sample.impedance = data[offset]
        offset += 1 
        sample.saturation = data[offset]
        
        if (sample.data != sample.rawData) and (sample.data > 0):
            sample.data = 0

        if (sample.impedance > 100):
            sample.impedance = 100

        if (sample.saturation > 100):
            sample.saturation = 100
            
        sample.sampleIndex = lastSampleIndex
        sample.timeStampInMs = int(lastSampleIndex * sampleInterval)
        sample.channelIndex = 0
        samples.append(sample)
        sensor_data.channelSamples.append(samples)
        
        self.sendSensorData(sensor_data, buf)
        return True

    def _process_impedance_samples(self, type, data: bytes, buf: Queue[bytes], on_error_callback=None) -> bool:
        offset = 3

        impedanceData = []
        saturationData = []

        channelCount = self._device_info.EegChannelCount + self._device_info.EcgChannelCount + self._device_info.EmgChannelCount

        bytesPerChannel = 8
        if (type == DataType.NTF_IMPEDANCE_EXT):
            bytesPerChannel = 6

        if (len(data) < (offset + bytesPerChannel * channelCount)):
            if on_error_callback:
                on_error_callback("Incomplete Impedance packet received")
            return False

        sensor_data = self.sensorDatas[SensorDataType.DATA_TYPE_IMPEDANCE]
        res = self.checkReadSamples(data, sensor_data, 0, -1, on_error_callback)
        if res == ReadSamplesResult.Error:
            return False
        if res == ReadSamplesResult.Repeated:
            return True
        sampleInterval = 1000.0 / sensor_data.sampleRate if sensor_data.sampleRate > 0 else 0

        for index in range(channelCount):
            impedance = struct.unpack_from("<f", data, offset)[0]
            offset += 4
            impedanceData.append(impedance)

        for index in range(channelCount):
            if (type == DataType.NTF_IMPEDANCE):
                saturation = struct.unpack_from("<f", data, offset)[0]
                offset += 4
            else:
                saturation = struct.unpack_from("<H", data, offset)[0]
                offset += 2
            saturationData.append(saturation / 10)  # firmware value range 0 - 1000

        self.impedanceData = impedanceData
        self.saturationData = saturationData
        lastSampleIndex = sensor_data.lastPackageCounter * sensor_data.packageSampleCount

        sensor_data.channelSamples = []
        for index in range(channelCount):
            samples = []
            sample = self._object_pool.acquire_sample()

            impedanceValue = impedanceData[index]
            saturationValue = saturationData[index]
            if not math.isfinite(impedanceValue):
                impedanceValue = 0.0
            if not math.isfinite(saturationValue):
                saturationValue = 0.0

            sample.rawData = int(saturationValue)
            sample.data = impedanceValue
            sample.impedance = impedanceValue
            sample.saturation = saturationValue
            sample.sampleIndex = lastSampleIndex
            sample.timeStampInMs = int(lastSampleIndex * sampleInterval)
            sample.channelIndex = index
            samples.append(sample)
            sensor_data.channelSamples.append(samples)
        
        self.sendSensorData(sensor_data, buf)
        return True

    def _has_complete_imu_packet(
            self,
            data: bytes,
            sensor_data_ref: SensorData,
            sensor_data_quat: SensorData,
            dataOffset: int,
    ) -> bool:
        frameSize = 12
        if sensor_data_quat is not None:
            frameSize += 12

        expected = dataOffset + sensor_data_ref.packageSampleCount * frameSize
        return len(data) == expected
    
    def _process_imu_samples(self, data: bytes, buf: Queue[bytes], on_error_callback=None) -> bool:
        sensor_data_acc = self.sensorDatas[SensorDataType.DATA_TYPE_ACC]
        sensor_data_gyro = self.sensorDatas[SensorDataType.DATA_TYPE_GYRO]
        sensor_data_quat = None
        sensor_data_euler = None

        if self.isContainQAT6:
            sensor_data_quat = self.sensorDatas[SensorDataType.DATA_TYPE_QUATERNION]
            sensor_data_euler = self.sensorDatas[SensorDataType.DATA_TYPE_EULER]

        if not self._has_complete_imu_packet(data, sensor_data_acc, sensor_data_quat, 3):
            if on_error_callback:
                on_error_callback("Incomplete IMU packet received")
            return False

        def _dispatch_imu(sensor_data, data_offset, data_gap):
            res = self.checkReadSamples(data, sensor_data, data_offset, data_gap, on_error_callback)
            if res == ReadSamplesResult.Error:
                return False
            if res == ReadSamplesResult.OK:
                self.sendSensorData(sensor_data, buf)
            return True

        ok = True
        if not self.isContainQAT6:
            if not _dispatch_imu(sensor_data_acc, 3, 6):
                ok = False
            if not _dispatch_imu(sensor_data_gyro, 9, 6):
                ok = False
        else:
            if not _dispatch_imu(sensor_data_acc, 3, 18):
                ok = False
            if not _dispatch_imu(sensor_data_gyro, 9, 18):
                ok = False

            if sensor_data_euler.channelSamples is None or len(sensor_data_euler.channelSamples) == 0:
                sensor_data_euler.channelSamples = [[], [], []]
        
            res_quat = self.checkReadSamples(data, sensor_data_quat, 15, 12, on_error_callback)
            if res_quat == ReadSamplesResult.Error:
                ok = False
            elif res_quat == ReadSamplesResult.OK:
                #add w
                sampleCount = sensor_data_quat.packageSampleCount
                for sampleIndex in range(sampleCount):
                    try:
                        x_sample = sensor_data_quat.channelSamples[1][sampleIndex]
                        y_sample = sensor_data_quat.channelSamples[2][sampleIndex]
                        z_sample = sensor_data_quat.channelSamples[3][sampleIndex]
                        x = x_sample.data
                        y = y_sample.data
                        z = z_sample.data
                        w = math.sqrt(max(0.0, 1.0 - x ** 2 - y ** 2 - z ** 2))
                        dataItem = self._object_pool.acquire_sample()
                        dataItem.channelIndex = 0
                        dataItem.sampleIndex = x_sample.sampleIndex
                        dataItem.timeStampInMs = x_sample.timeStampInMs
                        dataItem.rawData = 0
                        dataItem.data = w
                        dataItem.isLost = x_sample.isLost
                        sensor_data_quat.channelSamples[0].append(dataItem)

                        #add euler
                        R = math.atan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2)) * 180 / math.pi
                        R_sample = self._object_pool.acquire_sample()
                        R_sample.channelIndex = 0
                        R_sample.sampleIndex = x_sample.sampleIndex
                        R_sample.timeStampInMs = x_sample.timeStampInMs
                        R_sample.rawData = 0
                        R_sample.data = R
                        R_sample.isLost = x_sample.isLost
                        sensor_data_euler.channelSamples[0].append(R_sample)
                        P = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))) * 180 / math.pi
                        P_sample = self._object_pool.acquire_sample()
                        P_sample.channelIndex = 1
                        P_sample.sampleIndex = x_sample.sampleIndex
                        P_sample.timeStampInMs = x_sample.timeStampInMs
                        P_sample.rawData = 0
                        P_sample.data = P
                        P_sample.isLost = x_sample.isLost
                        sensor_data_euler.channelSamples[1].append(P_sample)
                        Y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2)) * 180 / math.pi
                        Y_sample = self._object_pool.acquire_sample()
                        Y_sample.channelIndex = 2
                        Y_sample.sampleIndex = x_sample.sampleIndex
                        Y_sample.timeStampInMs = x_sample.timeStampInMs
                        Y_sample.rawData = 0
                        Y_sample.data = Y
                        Y_sample.isLost = x_sample.isLost
                        sensor_data_euler.channelSamples[2].append(Y_sample)
                    except Exception as e:
                        SdkLog.exception(_TAG, "Unexpected error")

            self.sendSensorData(sensor_data_quat, buf)
            self.sendSensorData(sensor_data_euler, buf)

        return ok

    @staticmethod
    def _is_consecutive_index(last_idx: int, new_idx: int, max_idx: int) -> bool:
        """序号连续：new == last + 1，或唯一合法翻转 last == max 且 new == 0（255->0 / 65535->0）。"""
        return new_idx == last_idx + 1 or (last_idx == max_idx and new_idx == 0)

    @staticmethod
    def _forward_index_delta(last_idx: int, new_idx: int, max_idx: int) -> int:
        """new 相对 last 的前向距离（含翻转），范围 [0, max]。"""
        return (new_idx - last_idx) % (max_idx + 1)

    def _decode_one_package(
        self,
        sensorData: SensorData,
        data: bytes,
        dataOffset: int,
        dataGap: int,
        newPackageIndex: int,
        maxPackageIndex: int,
    ):
        """解码一包（含丢失样本补偿），更新最后序号与包计数器。

        用 newPackageIndex 与最后序号的前向差值计算丢包数做补包。
        """
        lostPackageCounter = self._forward_index_delta(sensorData.lastPackageIndex, newPackageIndex, maxPackageIndex) - 1
        # 老 EMG 设备（isNewEMG=False，如 gForce200/gForcePro）的丢包属设备端固有行为，
        # 不上报丢包数据（lostPackageCount/占位样本）与丢包日志；当前包照常解码
        if lostPackageCounter > 0 and self.isNewEMG:
            sensorData.lostPackageCount = sensorData.lostPackageCount + lostPackageCounter
            lostSampleCount = sensorData.packageSampleCount * lostPackageCounter
            SdkLog.i(_TAG, (
                "MSG|LOST SAMPLE|MAC|" + str(sensorData.deviceMac)
                + "|TYPE|" + str(sensorData.dataType)
                + "|COUNT|" + str(lostSampleCount)
            ))
            if lostPackageCounter < _MAX_ALLOWED_PACKAGE_INDEX_DELTA:
                self.readSamples(data, sensorData, 0, dataGap, lostSampleCount)
            sensorData.lastPackageCounter += lostPackageCounter

        if dataGap >= 0:
            self.readSamples(data, sensorData, dataOffset, dataGap, 0)

        sensorData.lastPackageIndex = newPackageIndex
        sensorData.lastPackageCounter += 1
        self._illegal_jump_start_time = None

    def _pending_push(self, pending: List[Tuple[int, bytes]], newPackageIndex: int, data: bytes):
        """把不符合连续/翻转规则的包暂存到 pending queue，超出上限时丢弃最旧的。"""
        if len(pending) >= _PENDING_PACKAGE_MAX:
            dropped_idx, _ = pending.pop(0)
            if self.isNewEMG:
                SdkLog.data(_TAG, f"Pending queue full, drop oldest pending package idx={dropped_idx}")
        pending.append((newPackageIndex, bytes(data)))

    def _park_pending(self, sensorData: SensorData, pending: List[Tuple[int, bytes]], newPackageIndex: int, data: bytes):
        """把暂不能解码的包暂存 pending queue，等下一包数据到来时再判断。"""
        if self._illegal_jump_start_time is None:
            self._illegal_jump_start_time = time.time()
        if self.isNewEMG:
            SdkLog.data(_TAG, (
                "Pending package|MAC|" + str(sensorData.deviceMac)
                + "|TYPE|" + str(sensorData.dataType)
                + "|LAST_IDX|" + str(sensorData.lastPackageIndex)
                + "|CURR_IDX|" + str(newPackageIndex)
            ))
        self._pending_push(pending, newPackageIndex, data)

    def _commit_pending_in_order(
        self,
        sensorData: SensorData,
        pending: List[Tuple[int, bytes]],
        dataOffset: int,
        dataGap: int,
        maxPackageIndex: int,
    ) -> bool:
        """按序号顺序提交 pending queue 中的包（小前跳缺包未等到，按丢包补包解码）。

        每次取与最后序号前向差值最小的 pending 包解码（差值即丢包数），
        遇到重复或超差的包停止。返回是否有包被解码。
        """
        decoded_any = False
        while len(pending) > 0:
            best_pos = -1
            best_delta = None
            for pos in range(len(pending)):
                delta = self._forward_index_delta(sensorData.lastPackageIndex, pending[pos][0], maxPackageIndex)
                if best_delta is None or delta < best_delta:
                    best_delta = delta
                    best_pos = pos
            if best_delta is None or best_delta == 0 or best_delta > _MAX_ALLOWED_PACKAGE_INDEX_DELTA:
                break
            idx, pkt = pending.pop(best_pos)
            SdkLog.data(_TAG, f"Commit pending package idx={idx} last={sensorData.lastPackageIndex}")
            self._decode_one_package(sensorData, pkt, dataOffset, dataGap, idx, maxPackageIndex)
            decoded_any = True
        return decoded_any

    def _purge_stale_pending(self, sensorData: SensorData, pending: List[Tuple[int, bytes]], maxPackageIndex: int):
        """丢弃不可能再接续的 pending 包：序号与最后序号重复，或落在最后序号后方
        _MAX_ALLOWED_PACKAGE_INDEX_DELTA 窗口内（迟到/重复的乱序包）。"""
        kept: List[Tuple[int, bytes]] = []
        for idx, pkt in pending:
            back = (sensorData.lastPackageIndex - idx) % (maxPackageIndex + 1)
            if back <= _MAX_ALLOWED_PACKAGE_INDEX_DELTA:
                SdkLog.data(_TAG, f"Drop stale pending package idx={idx} last={sensorData.lastPackageIndex}")
                continue
            kept.append((idx, pkt))
        if len(kept) != len(pending):
            pending[:] = kept

    def _drain_pending(
        self,
        sensorData: SensorData,
        pending: List[Tuple[int, bytes]],
        dataOffset: int,
        dataGap: int,
        maxPackageIndex: int,
    ) -> bool:
        """尝试从 pending queue 中接续解码。

        规则 1：pending 中存在与最后序号连续（含合法翻转）的包时，反复找到并解码；
        规则 2：pending 中存在连续 3 包序号时提交它们，用最后序号与该连续段第一包的
        序号差值做补包。返回是否有包被解码。
        """
        decoded_any = False
        self._purge_stale_pending(sensorData, pending, maxPackageIndex)
        while len(pending) > 0:
            # 规则 1：找与最后序号连续的 pending 包
            connect_pos = -1
            for pos in range(len(pending)):
                if self._is_consecutive_index(sensorData.lastPackageIndex, pending[pos][0], maxPackageIndex):
                    connect_pos = pos
                    break
            if connect_pos >= 0:
                idx, pkt = pending[connect_pos]
                del pending[connect_pos]
                self._decode_one_package(sensorData, pkt, dataOffset, dataGap, idx, maxPackageIndex)
                decoded_any = True
                continue

            # 规则 2：pending 中存在连续 3 包（含翻转）时提交
            run_pos = -1
            for pos in range(len(pending) - 2):
                if (self._is_consecutive_index(pending[pos][0], pending[pos + 1][0], maxPackageIndex)
                        and self._is_consecutive_index(pending[pos + 1][0], pending[pos + 2][0], maxPackageIndex)):
                    run_pos = pos
                    break
            if run_pos < 0:
                break
            # 连续段之前的包已无法接续，丢弃
            for _ in range(run_pos):
                dropped_idx, _ = pending.pop(0)
                SdkLog.data(_TAG, f"Drop unconnectable pending package idx={dropped_idx}")
            SdkLog.data(_TAG, (
                f"Accept pending run: {pending[0][0]},{pending[1][0]},{pending[2][0]}"
                f" last={sensorData.lastPackageIndex}"
            ))
            for _ in range(3):
                idx, pkt = pending.pop(0)
                self._decode_one_package(sensorData, pkt, dataOffset, dataGap, idx, maxPackageIndex)
            decoded_any = True
        return decoded_any

    def checkReadSamples(self, data: bytes, sensorData: SensorData, dataOffset: int, dataGap: int, on_error_callback=None):
        offset = 1

        if not self._is_data_transfering:
            return ReadSamplesResult.Error
        if sensorData is None or sensorData.packageSampleCount <= 0 or sensorData.channelCount <= 0 or sensorData.minPackageSampleCount <= 0 or sensorData.K <= 0:
            return ReadSamplesResult.Error

        def _type_name():
            try:
                return DataType(sensorData.dataType).name
            except Exception:
                return str(sensorData.dataType)

        # 长度预检：在解析包序号/样本前就发现数据长度不足，并报告数据类型
        if dataGap >= 0 and sensorData.packageSampleCount > 0:
            if sensorData.resolutionBits in (7, 8):
                bytesPerChannel = 1
            elif sensorData.resolutionBits in (12, 16, 17, 0):
                bytesPerChannel = 2
            elif sensorData.resolutionBits == 24:
                bytesPerChannel = 3
            elif sensorData.resolutionBits in (31, 32, 33):
                bytesPerChannel = 4
            else:
                bytesPerChannel = 2

            realChannelCount = 0
            for i in range(sensorData.channelCount):
                if (sensorData.channelMask & (1 << i)) != 0:
                    realChannelCount += 1

            expected = (
                dataOffset
                + bytesPerChannel * realChannelCount * sensorData.packageSampleCount
                + dataGap * (sensorData.packageSampleCount - 1)
            )
            if dataGap == 0:
                # 单一流数据包：要求长度完全一致
                if expected != len(data):
                    SdkLog.data(_TAG, f"Invalid dataLength:{len(data)} (expected {expected}) for data type {_type_name()}")
                    return ReadSamplesResult.Error
            else:
                # IMU 复合包：允许包含多个子流，只检查长度不足
                if expected > len(data):
                    SdkLog.data(_TAG, f"Invalid dataLength:{len(data)} (expected at least {expected}) for data type {_type_name()}")
                    return ReadSamplesResult.Error

        try:
            packageIndex = 0
            maxPackageIndex = 0
            if (sensorData.packageIndexLength == 2):
                packageIndex = ((data[offset + 1] & 0xFF) << 8) | (data[offset] & 0xFF)
                maxPackageIndex = 65535
            elif (sensorData.packageIndexLength == 1):
                packageIndex = (data[offset] & 0xFF)
                maxPackageIndex = 255

            if sensorData.packageIndexLength <= 0:
                if sensorData.lastPackageCounter < 0:
                    sensorData.lastPackageIndex = 0
                    sensorData.lastPackageCounter = 0
                if (dataGap >= 0):
                    self.readSamples(data, sensorData, dataOffset, dataGap, 0)
                sensorData.lastPackageCounter += 1
                self._illegal_jump_start_time = None
                return ReadSamplesResult.OK

            offset += sensorData.packageIndexLength
            newPackageIndex = packageIndex

            # 首包：把上一包序号设为合法的前一个值，便于后续统一判断
            if sensorData.lastPackageCounter < 0:
                sensorData.lastPackageCounter = 0
                if newPackageIndex > 0:
                    sensorData.lastPackageIndex = newPackageIndex - 1
                else:
                    sensorData.lastPackageIndex = maxPackageIndex
                # 序号跟踪重新开始，丢弃历史暂存包
                self._pending_packages.pop(id(sensorData), None)

            if newPackageIndex == sensorData.lastPackageIndex:
                self._illegal_jump_start_time = None
                return ReadSamplesResult.Repeated

            pending = self._pending_packages.setdefault(id(sensorData), [])
            decoded_any = False
            if self._is_consecutive_index(sensorData.lastPackageIndex, newPackageIndex, maxPackageIndex):
                # 与最后序号连续（含唯一合法翻转 255->0 / 65535->0）：直接解码，
                # 之后到 pending queue 中反复寻找可以接续的包（规则 1）
                self._decode_one_package(sensorData, data, dataOffset, dataGap, newPackageIndex, maxPackageIndex)
                decoded_any = True
            else:
                # 非连续包按前向差值分类处理：
                # - 差值<=3（小前跳）：缺少的包可能随后到达（乱序），暂存 pending queue 等待恢复
                # - 差值<=100（中前跳）：先把 pending 中的小跳包按序号顺序补包提交，再重新分类当前包
                # - 差值>100 或回退：乱序或随机错误数据，暂存 pending queue（规则 1/2）
                while True:
                    deltaPackageIndex = self._forward_index_delta(sensorData.lastPackageIndex, newPackageIndex, maxPackageIndex)
                    if deltaPackageIndex == 0:
                        # 提交 pending 后当前包成为重复包
                        self._illegal_jump_start_time = None
                        return ReadSamplesResult.Repeated
                    if self._is_consecutive_index(sensorData.lastPackageIndex, newPackageIndex, maxPackageIndex):
                        self._decode_one_package(sensorData, data, dataOffset, dataGap, newPackageIndex, maxPackageIndex)
                        decoded_any = True
                        break
                    if deltaPackageIndex <= _SMALL_JUMP_MAX and dataGap >= 0:
                        # 小前跳：缺少的包可能随后到达（乱序），暂存等待恢复
                        self._park_pending(sensorData, pending, newPackageIndex, data)
                        break
                    if deltaPackageIndex <= _MAX_ALLOWED_PACKAGE_INDEX_DELTA:
                        # 中前跳：先提交 pending 中的小跳包，再重新分类当前包
                        if self._commit_pending_in_order(sensorData, pending, dataOffset, dataGap, maxPackageIndex):
                            decoded_any = True
                            continue
                        # pending 无可提交：按丢包数补包解码当前包
                        self._decode_one_package(sensorData, data, dataOffset, dataGap, newPackageIndex, maxPackageIndex)
                        decoded_any = True
                        break
                    # 超大前跳/回退（乱序或随机错误数据）：暂存 pending queue
                    if dataGap < 0:
                        # dataGap < 0 的流由调用方解析负载，pending 无法回溯解码，保持严格检测
                        if self._illegal_jump_start_time is None:
                            self._illegal_jump_start_time = time.time()
                        return ReadSamplesResult.Error
                    self._park_pending(sensorData, pending, newPackageIndex, data)
                    break

            # 规则 1/2：从 pending queue 中接续或按连续 3 包补包解码
            if len(pending) > 0:
                decoded_any = self._drain_pending(sensorData, pending, dataOffset, dataGap, maxPackageIndex) or decoded_any

            if not decoded_any:
                # 当前包已暂存 pending queue，等后续包到来再判断
                return ReadSamplesResult.Repeated
            return ReadSamplesResult.OK
        except Exception as e:
            SdkLog.exception(_TAG, "Unexpected error")
            return ReadSamplesResult.Error

    def transTrainData(self, data: int):
        xout = data >> 4
        exp = data & 0x0000000F
        xout = xout << exp
        return xout

    def readSamples(
            self,
            data: bytes,
            sensorData: SensorData,
            offset: int,
            dataGap: int,
            lostSampleCount: int,
    ):
        sampleCount = sensorData.packageSampleCount
        def _type_name():
            try:
                return DataType(sensorData.dataType).name
            except Exception:
                return str(sensorData.dataType)

        if lostSampleCount <= 0:
            if data is None or offset < 0 or offset > len(data):
                raise ValueError(f"Invalid data or offset for data type {_type_name()}")

        sampleInterval = (
            int(1000.0 / sensorData.sampleRate) if sensorData.sampleRate > 0 else 0
        )
        if lostSampleCount > 0:
            sampleCount = lostSampleCount

        K = sensorData.K
        lastSampleIndex = sensorData.lastPackageCounter * sensorData.packageSampleCount

        _impedanceData = self.impedanceData.copy()
        _saturationData = self.saturationData.copy()

        channelSamples = sensorData.channelSamples
        if not channelSamples:
            for channelIndex in range(sensorData.channelCount):
                channelSamples.append([])

        
        for sampleIndex in range(sampleCount):
            for channelIndex, impedanceChannelIndex in enumerate(range(sensorData.channelCount)):
                if (sensorData.channelMask & (1 << channelIndex)) != 0:
                    samples = channelSamples[channelIndex]
                    impedance = 0.0
                    saturation = 0.0

                    if sensorData.dataType == DataType.NTF_ECG:
                        impedanceChannelIndex = self.sensorDatas[SensorDataType.DATA_TYPE_EEG].channelCount

                    if impedanceChannelIndex < len(_impedanceData):
                        impedance = _impedanceData[impedanceChannelIndex]
                        saturation = _saturationData[impedanceChannelIndex]

                    impedanceChannelIndex += 1

                    dataItem = self._object_pool.acquire_sample()
                    dataItem.channelIndex = channelIndex
                    dataItem.sampleIndex = lastSampleIndex
                    dataItem.timeStampInMs = lastSampleIndex * sampleInterval
                    if lostSampleCount > 0:
                        dataItem.rawData = 0
                        dataItem.data = 0.0
                        dataItem.impedance = impedance
                        dataItem.saturation = saturation
                        dataItem.isLost = True
                    else:
                        rawData = 0
                        if sensorData.resolutionBits == 7:
                            rawData = data[offset]
                            rawData -= 119
                            offset += 1
                        elif sensorData.resolutionBits == 8:
                            rawData = data[offset] & 0xFF
                            offset += 1
                        elif sensorData.resolutionBits == 12:
                            rawData = struct.unpack_from("<h", data, offset)[0]
                            rawData -= 2000
                            offset += 2
                        elif sensorData.resolutionBits == 16:
                            if (sensorData.resolutionSigned):
                                rawData = struct.unpack_from("<h", data, offset)[0]
                            else:
                                rawData = struct.unpack_from("<H", data, offset)[0]
                            offset += 2
                        elif sensorData.resolutionBits == 17:
                            rawData = struct.unpack_from(">h", data, offset)[0]
                            offset += 2
                        elif sensorData.resolutionBits == 24:
                            rawData = (data[offset] << 16) | (data[offset + 1] << 8) | data[offset + 2]
                            if (sensorData.resolutionSigned):
                                rawData -= 8388608
                            offset += 3
                        elif sensorData.resolutionBits == 31:
                            if (sensorData.resolutionSigned):
                                rawData = struct.unpack_from("<i", data, offset)[0]
                            else:
                                rawData = struct.unpack_from("<I", data, offset)[0]
                            offset += 4
                        elif sensorData.resolutionBits == 32:
                            # 32-bit float: rawData 不存浮点，避免 FlatBuffers 整型字段异常
                            rawData = 0
                            converted = struct.unpack_from("f", data, offset)[0]
                            offset += 4
                        elif sensorData.resolutionBits == 33:
                            if (sensorData.resolutionSigned):
                                rawData = struct.unpack_from(">i", data, offset)[0]
                            else:
                                rawData = struct.unpack_from(">I", data, offset)[0]
                            offset += 4
                        elif sensorData.resolutionBits == 0:
                            rawData = struct.unpack_from("<h", data, offset)[0]
                            offset += 2
                            rawData = self.transTrainData(rawData)

                        if sensorData.resolutionBits == 32:
                            # converted 已在上面赋值（float 原始值）
                            pass
                        else:
                            converted = rawData * K
                        dataItem.rawData = rawData
                        dataItem.data = converted
                        dataItem.impedance = impedance
                        dataItem.saturation = saturation
                        dataItem.isLost = False

                    samples.append(dataItem)

            lastSampleIndex += 1
            offset += dataGap

    def sendSensorData(self, sensorData: SensorData, buf: Queue[bytes]):
        oldChannelSamples = sensorData.channelSamples

        if not self.isDataTransfering or len(oldChannelSamples) == 0:
            return

        realSampleCount = 0
        if len(oldChannelSamples) > 0:
            realSampleCount = len(oldChannelSamples[0])

        if realSampleCount < sensorData.minPackageSampleCount:
            return

        sensorData.channelSamples = []
        batchCount = realSampleCount // sensorData.minPackageSampleCount
        # leftSampleSize = realSampleCount - sensorData.minPackageSampleCount * batchCount

        sensorDataList = []
        startIndex = 0
        for batchIndex in range(batchCount):
            resultChannelSamples = []
            for channelIndex in range(sensorData.channelCount):
                oldSamples = oldChannelSamples[channelIndex]
                newSamples = []
                for sampleIndex in range(sensorData.minPackageSampleCount):
                    try:
                        newSamples.append(oldSamples[startIndex + sampleIndex])
                    except IndexError:
                        pass
                resultChannelSamples.append(newSamples)

            sensorDataResult = self._object_pool.acquire_sensor_data()
            sensorDataResult.channelSamples = resultChannelSamples
            sensorDataResult.dataType = sensorData.dataType
            sensorDataResult.deviceMac = sensorData.deviceMac
            sensorDataResult.sampleRate = sensorData.sampleRate
            sensorDataResult.channelCount = sensorData.channelCount
            sensorDataResult.minPackageSampleCount = sensorData.minPackageSampleCount
            sensorDataResult.lostPackageCount = sensorData.lostPackageCount
            sensorDataList.append(sensorDataResult)

            if self.debugCSVPath != None and self.debugCSVPath != "" and self.debugCSVWriter == None:
                try:
                    self.debugCSVWriter = csv.writer(open(self.debugCSVPath, "w", newline="", encoding="utf-8"))
                    header_append_keys = ["dataType", "sampleRate"]
                    channel_samples_header = list(vars(sensorDataResult.channelSamples[0][0]).keys())
                    for key_item in header_append_keys:
                        channel_samples_header.append(key_item)
                    self.debugCSVWriter.writerow(channel_samples_header)
                except Exception as e:
                    # print(e)
                    SdkLog.exception(_TAG, "Unexpected error")

            if self.debugCSVWriter != None:
                try:
                    for i, channel_sample_list in enumerate(sensorDataResult.channelSamples):
                        for channel_sample in channel_sample_list:
                            row_data = []

                            for key in vars(channel_sample).keys():
                                row_data.append(getattr(channel_sample, key))
                            row_data.append(sensorDataResult.dataType)
                            row_data.append(sensorDataResult.sampleRate)
                            self.debugCSVWriter.writerow(row_data)
                except Exception as e:
                    # print(e)
                    SdkLog.exception(_TAG, "Unexpected error")

            startIndex += sensorData.minPackageSampleCount

        leftChannelSamples = []
        for channelIndex in range(sensorData.channelCount):
            oldSamples = oldChannelSamples[channelIndex]
            newSamples = []
            for sampleIndex in range(startIndex, len(oldSamples)):
                newSamples.append(oldSamples[sampleIndex])

            leftChannelSamples.append(newSamples)

        sensorData.channelSamples = leftChannelSamples

        for sensorDataResult in sensorDataList:
            try:
                fb_bytes = sensorDataResult.to_flatbuffers()
                # 解码结果不丢：结果队列满时等待消费完成（自己先消费一条最旧结果
                # 腾出空间）；stopDataNotify / close / terminate 时放弃等待，丢弃该结果
                while self._is_running and self._is_data_transfering and not sensor_utils._terminated:
                    try:
                        buf.put(fb_bytes, timeout=0.1)
                        break
                    except Full:
                        self._drain_parsed_one(buf)
                        # 等待期间保持看门狗进度，避免误判解析卡死
                        self._last_progress_time = time.time()
            except Exception as e:
                SdkLog.e(_TAG, f"Failed to serialize SensorData to FlatBuffers: {e}")
            finally:
                # 结果对象已序列化为 FlatBuffers，可归还对象池
                self._object_pool.release_sensor_data(sensorDataResult)

    async def _processUniversalData(self, buf: Queue[bytes], on_data_callback, on_error_callback=None):
        self._on_data_callback = on_data_callback
        while self._is_running:
            while self._is_running and self._rawDataBuffer.empty():
                if self._is_running and self.isDataTransfering and not buf.empty():
                    self._drain_parsed_one(buf)
                else:
                    await asyncio.sleep(0.01)
                continue

            try:
                while self._is_running and not self._rawDataBuffer.empty():
                    data = self._rawDataBuffer.get_nowait()
                    if len(self._concatDataBuffer) > _MAX_CONCAT_BUFFER_SIZE:
                        SdkLog.w(_TAG, f"Concat buffer exceeded {_MAX_CONCAT_BUFFER_SIZE}, clearing")
                        self._concatDataBuffer.clear()
                    self._concatDataBuffer.extend(data)
                    self._rawDataBuffer.task_done()
                    self._last_progress_time = time.time()
            except Exception as e:
                SdkLog.exception(_TAG, "Error reading raw data buffer")

            try:
                index = 0
                last_cut = -1
                data_size = len(self._concatDataBuffer)

                while self._is_running:
                    if index >= data_size:
                        break

                    if self._concatDataBuffer[index] == 0x55:
                        if (index + 1) >= data_size:
                            index = data_size
                            continue
                        n = self._concatDataBuffer[index + 1]
                        if n < 2 or (index + 1 + n + 2) >= data_size:
                            index += 1
                            continue
                        crc16 = (self._concatDataBuffer[index + 1 + n + 2] << 8) | self._concatDataBuffer[index + 1 + n + 1]
                        calc_crc = sensor_utils.crc16_cal(self._concatDataBuffer[index + 2: index + 2 + n], n)
                        if crc16 != calc_crc:
                            index += 1
                            continue
                        if self._is_data_transfering:
                            data_package = bytes(self._concatDataBuffer[index + 2: index + 2 + n])
                            if self._processDataPackage(data_package, buf, on_error_callback):
                                last_cut = index = index + 2 + n + 1
                        index += 1
                    elif self._concatDataBuffer[index] == 0xAA:
                        if (index + 1) >= data_size:
                            index = data_size
                            continue
                        n = self._concatDataBuffer[index + 1]
                        if n < 2 or (index + 1 + n + 2) >= data_size:
                            index += 1
                            continue
                        crc16 = (self._concatDataBuffer[index + 1 + n + 2] << 8) | self._concatDataBuffer[index + 1 + n + 1]
                        calc_crc = sensor_utils.crc16_cal(self._concatDataBuffer[index + 2: index + 2 + n], n)
                        if crc16 != calc_crc:
                            index += 1
                            continue
                        data_package = bytes(self._concatDataBuffer[index + 2: index + 2 + n])

                        if not sensor_utils._terminated:
                            await self.gForce.async_on_cmd_response(data_package)
                        last_cut = index = index + 2 + n + 1
                        index += 1
                    else:
                        index += 1

                if last_cut > 0:
                    self._concatDataBuffer = self._concatDataBuffer[last_cut + 1:]
                    last_cut = -1
                    index = 0

                self._last_progress_time = time.time()
            except Exception as e:
                SdkLog.exception(_TAG, "Unexpected error in universal concat data processing")
