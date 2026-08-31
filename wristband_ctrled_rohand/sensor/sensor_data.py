from enum import IntEnum
from typing import List

import flatbuffers


class Sample:
    """单个通道采样点（对齐 C++ SDK include/SensorData.hpp 的 SensorData::Sample）。

    内部变量一律 _ 前缀；对外暴露与 C++ 公有字段同名的只读属性。
    """

    __slots__ = [
        "_rawData",
        "_data",
        "_impedance",
        "_saturation",
        "_sampleIndex",
        "_isLost",
        "_absTimeStampInSec",
        "_channelIndex",
    ]

    def __init__(self):
        self._rawData = 0
        self._data = 0.0
        self._impedance = 0.0
        self._saturation = 0.0
        self._sampleIndex = 0
        self._isLost = False
        self._absTimeStampInSec = 0.0
        self._channelIndex = 0

    def reset(self):
        self._rawData = 0
        self._data = 0.0
        self._impedance = 0.0
        self._saturation = 0.0
        self._sampleIndex = 0
        self._isLost = False
        self._absTimeStampInSec = 0.0
        self._channelIndex = 0

    # ---- 与 C++ SensorData::Sample 公有字段同名的只读属性 ----
    @property
    def rawData(self) -> int:
        return self._rawData

    @property
    def data(self) -> float:
        return self._data

    @property
    def impedance(self) -> float:
        return self._impedance

    @property
    def saturation(self) -> float:
        return self._saturation

    @property
    def sampleIndex(self) -> int:
        return self._sampleIndex

    @property
    def isLost(self) -> bool:
        return self._isLost

    @property
    def absTimeStampInSec(self) -> float:
        """LSL 风格绝对时间戳（Unix 秒，double）：起流墙钟 + sampleIndex/采样率，
        解码时计算；锚点未知（如无 stream_start 记录的 bin 回放）为 0。"""
        return self._absTimeStampInSec

    @property
    def channelIndex(self) -> int:
        return self._channelIndex


class DataType(IntEnum):
    NTF_ACC = 0x1
    NTF_GYRO = 0x2
    NTF_EULER_DATA = 0x4
    NTF_QUATERNION = 0x5
    NTF_GEST = 0x07
    NTF_EMG = 0x8
    NTF_MAG_ANGLE_DATA = 0x0D
    NTF_EEG = 0x10
    NTF_ECG = 0x11
    NTF_IMPEDANCE = 0x12
    NTF_IMU = 0x13
    NTF_ADS = 0x14
    NTF_BRTH = 0x15
    NTF_IMPEDANCE_EXT = 0x16
    NTF_SPO2 = 0x17
    NTF_PPG = 0x18


class SensorData:
    """多通道采样批（公有接口对齐 C++ SDK include/SensorData.hpp）。

    内部变量一律 _ 前缀，公有接口为 C++ 同名方法：
    getDeviceMac()/getDeviceName()/getDataType()/getLostPackageCount()/getSampleRate()/
    getChannelCount()/getChannelMask()/getSampleCount()/getStartTimeStamp()/
    getDelay()/getStartTimeSec()/isDataValid()/getChannelSample()/getData()/
    getTimeStampInMs()（计算值：sampleIndex * 1000 / sampleRate）/
    getAbsTimeStampInSec()/getSampleIndex()/getRawData()/getImpedance()/
    getSaturation()/isLost()/clone()；C++ 公有字段 channelSamples/
    startSampleIndex 以同名只读属性暴露。
    """

    __slots__ = [
        "_deviceMac",
        "_deviceName",
        "_dataType",
        "_sampleRate",
        "_channelCount",
        "_packageSampleCount",
        "_packageIndexLength",
        "_channelSamples",
        "_lastPackageCounter",
        "_lastPackageIndex",
        "_lostPackageCount",
        "_resolutionBits",
        "_resolutionSigned",
        "_channelMask",
        "_minPackageSampleCount",
        "_K",
        "_startTimeStamp",
        "_delay",
        "_startTimeSec",
    ]

    def __init__(self):
        self._deviceMac = ""
        self._deviceName = ""
        self._dataType = DataType.NTF_EEG
        self._sampleRate = 0.0
        self._channelCount = 0
        self._packageSampleCount = 0
        self._packageIndexLength = 2
        self._channelSamples: List[List[Sample]] = list()
        self._lastPackageCounter = 0
        self._lastPackageIndex = 0
        self._lostPackageCount = 0
        self._resolutionBits = 0
        self._resolutionSigned = 0
        self._channelMask = 0
        self._minPackageSampleCount = 0
        self._K = 0.0
        # 本次起流的 ATT 起流写发送时刻（32 位毫秒）与首包 delay（毫秒，
        # 首个原始数据包到达时刻 - startTimeStamp）；未起流为 0
        self._startTimeStamp = 0
        self._delay = 0
        # 起流墙钟锚点（Unix 秒，double，LSL 风格）；回放从 bin 记录时间戳还原，
        # 未知（如无 stream_start 记录的老 bin）为 0
        self._startTimeSec = 0.0

    def reset(self):
        self._deviceMac = ""
        self._deviceName = ""
        self._dataType = DataType.NTF_EEG
        self._sampleRate = 0.0
        self._channelCount = 0
        self._packageSampleCount = 0
        self._packageIndexLength = 2
        self._channelSamples.clear()
        self._lastPackageCounter = 0
        self._lastPackageIndex = 0
        self._lostPackageCount = 0
        self._resolutionBits = 0
        self._resolutionSigned = 0
        self._channelMask = 0
        self._minPackageSampleCount = 0
        self._K = 0.0
        self._startTimeStamp = 0
        self._delay = 0
        self._startTimeSec = 0.0

    def clear(self):
        self._channelSamples.clear()
        self._lastPackageCounter = -1
        self._lastPackageIndex = 0
        self._lostPackageCount = 0

    # ------------------------------------------------------------------
    # C++ 公有接口（include/SensorData.hpp）：元数据访问器
    # ------------------------------------------------------------------
    def getDeviceMac(self) -> str:
        return self._deviceMac

    def getDeviceName(self) -> str:
        return self._deviceName

    def getDataType(self) -> DataType:
        return self._dataType

    def getLostPackageCount(self) -> int:
        return self._lostPackageCount

    def getSampleRate(self) -> float:
        return self._sampleRate

    def getChannelCount(self) -> int:
        return self._channelCount

    def getChannelMask(self) -> int:
        return self._channelMask

    def getSampleCount(self) -> int:
        """每通道有效样本数（C++ Info::sampleCount，即历史 packageSampleCount）。"""
        return self._packageSampleCount

    def getStartTimeStamp(self) -> int:
        return self._startTimeStamp

    def getDelay(self) -> int:
        return self._delay

    def getStartTimeSec(self) -> float:
        """起流墙钟锚点（Unix 秒，double）；未知为 0。"""
        return self._startTimeSec

    # C++ 公有字段的同名只读属性
    @property
    def channelSamples(self) -> List[List[Sample]]:
        return self._channelSamples

    @property
    def startSampleIndex(self) -> int:
        """本批首个样本的绝对 sampleIndex（无样本时为 0）。"""
        if self._channelSamples and self._channelSamples[0]:
            return self._channelSamples[0][0].sampleIndex
        return 0

    # ------------------------------------------------------------------
    # C++ 公有接口：单点样本访问器
    # ------------------------------------------------------------------
    def isDataValid(self, channelIndex: int = 0, sampleIndex: int = 0) -> bool:
        """Python 侧无 arena 复用语义，统一返回 True。"""
        return True

    def getChannelSample(self, channelIndex: int, sampleIndex: int) -> Sample:
        if (channelIndex < 0 or channelIndex >= len(self._channelSamples)
                or sampleIndex < 0
                or sampleIndex >= len(self._channelSamples[channelIndex])):
            raise IndexError(
                f"SensorData: index out of range ({channelIndex}, {sampleIndex})")
        return self._channelSamples[channelIndex][sampleIndex]

    def getData(self, channelIndex: int, sampleIndex: int) -> float:
        return self.getChannelSample(channelIndex, sampleIndex).data

    def getTimeStampInMs(self, channelIndex: int, sampleIndex: int) -> int:
        """样本毫秒时间戳（计算值：sampleIndex * 1000 / sampleRate，
        采样率未知为 0；不再有存储字段）。"""
        idx = self.getChannelSample(channelIndex, sampleIndex).sampleIndex
        if self._sampleRate <= 0:
            return 0
        return int(idx * 1000.0 / self._sampleRate)

    def getAbsTimeStampInSec(self, channelIndex: int, sampleIndex: int) -> float:
        """样本绝对时间戳（LSL 风格 double 秒，纯访问；解码时已算好，
        任意采样率下分辨率均为 1/采样率 秒）。"""
        return self.getChannelSample(channelIndex, sampleIndex).absTimeStampInSec

    def getSampleIndex(self, channelIndex: int, sampleIndex: int) -> int:
        return self.getChannelSample(channelIndex, sampleIndex).sampleIndex

    def getRawData(self, channelIndex: int, sampleIndex: int) -> int:
        return self.getChannelSample(channelIndex, sampleIndex).rawData

    def getImpedance(self, channelIndex: int, sampleIndex: int) -> float:
        return self.getChannelSample(channelIndex, sampleIndex).impedance

    def getSaturation(self, channelIndex: int, sampleIndex: int) -> float:
        return self.getChannelSample(channelIndex, sampleIndex).saturation

    def isLost(self, channelIndex: int, sampleIndex: int) -> bool:
        return self.getChannelSample(channelIndex, sampleIndex).isLost

    def clone(self) -> "SensorData":
        """深拷贝有效数据窗口（样本与元数据均为独立副本）。"""
        out = SensorData()
        out._deviceMac = self._deviceMac
        out._deviceName = self._deviceName
        out._dataType = self._dataType
        out._sampleRate = self._sampleRate
        out._channelCount = self._channelCount
        out._packageSampleCount = self._packageSampleCount
        out._packageIndexLength = self._packageIndexLength
        out._lastPackageCounter = self._lastPackageCounter
        out._lastPackageIndex = self._lastPackageIndex
        out._lostPackageCount = self._lostPackageCount
        out._resolutionBits = self._resolutionBits
        out._resolutionSigned = self._resolutionSigned
        out._channelMask = self._channelMask
        out._minPackageSampleCount = self._minPackageSampleCount
        out._K = self._K
        out._startTimeStamp = self._startTimeStamp
        out._delay = self._delay
        out._startTimeSec = self._startTimeSec
        for channel in self._channelSamples:
            copied_channel = []
            for sample in channel:
                copied = Sample()
                copied._rawData = sample._rawData
                copied._data = sample._data
                copied._impedance = sample._impedance
                copied._saturation = sample._saturation
                copied._sampleIndex = sample._sampleIndex
                copied._isLost = sample._isLost
                copied._absTimeStampInSec = sample._absTimeStampInSec
                copied._channelIndex = sample._channelIndex
                copied_channel.append(copied)
            out._channelSamples.append(copied_channel)
        return out

    # ------------------------------------------------------------------
    # FlatBuffers 序列化（SDK 内部）
    # ------------------------------------------------------------------
    def to_flatbuffers(self) -> bytes:
        """将 SensorData 序列化为 FlatBuffers bytes。"""
        import sensor.fb.SensorData as FBSensorData
        import sensor.fb.Sample as FBSample

        builder = flatbuffers.Builder(1024)

        # 扁平化 channelSamples
        flat_samples = []
        samples_per_channel = []
        for channel in self._channelSamples:
            samples_per_channel.append(len(channel))
            for sample in channel:
                flat_samples.append(sample)

        # samples 向量（struct 数组）
        samples_offset = 0
        if flat_samples:
            FBSensorData.StartSamplesVector(builder, len(flat_samples))
            for sample in reversed(flat_samples):
                FBSample.CreateSample(
                    builder,
                    float(sample._absTimeStampInSec),
                    int(sample._channelIndex),
                    int(sample._sampleIndex),
                    int(sample._rawData),
                    float(sample._data),
                    float(sample._impedance),
                    float(sample._saturation),
                    bool(sample._isLost),
                )
            samples_offset = builder.EndVector()

        # samples_per_channel 向量
        spc_offset = 0
        if samples_per_channel:
            FBSensorData.StartSamplesPerChannelVector(builder, len(samples_per_channel))
            for count in reversed(samples_per_channel):
                builder.PrependInt32(count)
            spc_offset = builder.EndVector()

        # device_mac / device_name 字符串
        mac_offset = builder.CreateString(self._deviceMac) if self._deviceMac else 0
        name_offset = builder.CreateString(self._deviceName) if self._deviceName else 0

        FBSensorData.Start(builder)
        if mac_offset:
            FBSensorData.AddDeviceMac(builder, mac_offset)
        if name_offset:
            FBSensorData.AddDeviceName(builder, name_offset)
        FBSensorData.AddDataType(builder, int(self._dataType))
        FBSensorData.AddLastPackageCounter(builder, int(self._lastPackageCounter))
        FBSensorData.AddLastPackageIndex(builder, int(self._lastPackageIndex))
        FBSensorData.AddLostPackageCount(builder, int(self._lostPackageCount))
        FBSensorData.AddResolutionBits(builder, int(self._resolutionBits))
        FBSensorData.AddResolutionSigned(builder, int(self._resolutionSigned))
        FBSensorData.AddSampleRate(builder, float(self._sampleRate))
        FBSensorData.AddChannelCount(builder, int(self._channelCount))
        FBSensorData.AddChannelMask(builder, int(self._channelMask))
        FBSensorData.AddMinPackageSampleCount(builder, int(self._minPackageSampleCount))
        FBSensorData.AddPackageSampleCount(builder, int(self._packageSampleCount))
        FBSensorData.AddPackageIndexLength(builder, int(self._packageIndexLength))
        FBSensorData.AddK(builder, float(self._K))
        if samples_offset:
            FBSensorData.AddSamples(builder, samples_offset)
        if spc_offset:
            FBSensorData.AddSamplesPerChannel(builder, spc_offset)
        root = FBSensorData.End(builder)

        builder.Finish(root)
        return bytes(builder.Output())

    @classmethod
    def from_flatbuffers(cls, buf: bytes) -> "SensorData":
        """从 FlatBuffers bytes 反序列化为 SensorData。"""
        return cls.from_flatbuffers_pooled(buf, cls())

    @classmethod
    def from_flatbuffers_pooled(cls, buf: bytes, data: "SensorData", pool=None) -> "SensorData":
        """从 FlatBuffers bytes 反序列化到传入的 SensorData 实例（对象池复用）。"""
        import sensor.fb.SensorData as FBSensorData

        data.reset()
        fb = FBSensorData.SensorData.GetRootAs(buf, 0)

        mac = fb.DeviceMac()
        data._deviceMac = mac.decode('utf-8') if isinstance(mac, bytes) else (mac or "")
        name = fb.DeviceName()
        data._deviceName = name.decode('utf-8') if isinstance(name, bytes) else (name or "")
        data._dataType = DataType(fb.DataType())
        data._lastPackageCounter = fb.LastPackageCounter()
        data._lastPackageIndex = fb.LastPackageIndex()
        data._lostPackageCount = fb.LostPackageCount()
        data._resolutionBits = fb.ResolutionBits()
        data._resolutionSigned = fb.ResolutionSigned()
        data._sampleRate = fb.SampleRate()
        data._channelCount = fb.ChannelCount()
        data._channelMask = fb.ChannelMask()
        data._minPackageSampleCount = fb.MinPackageSampleCount()
        data._packageSampleCount = fb.PackageSampleCount()
        data._packageIndexLength = fb.PackageIndexLength()
        data._K = fb.K()

        data._channelSamples.clear()
        samples_len = fb.SamplesLength()
        spc_len = fb.SamplesPerChannelLength()

        offset = 0
        for i in range(spc_len):
            count = fb.SamplesPerChannel(i)
            channel = []
            for j in range(count):
                if offset >= samples_len:
                    break
                fb_sample = fb.Samples(offset)
                sample = pool.acquire_sample() if pool is not None else Sample()
                sample._absTimeStampInSec = fb_sample.AbsTimeStampInSec()
                sample._channelIndex = fb_sample.ChannelIndex()
                sample._sampleIndex = fb_sample.SampleIndex()
                sample._rawData = fb_sample.RawData()
                sample._data = fb_sample.Data()
                sample._impedance = fb_sample.Impedance()
                sample._saturation = fb_sample.Saturation()
                sample._isLost = fb_sample.IsLost()
                channel.append(sample)
                offset += 1
            data._channelSamples.append(channel)

        return data
