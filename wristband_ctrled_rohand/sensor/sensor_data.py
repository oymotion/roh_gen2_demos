from enum import IntEnum
from typing import List

import flatbuffers


class Sample:
    __slots__ = [
        "rawData",
        "data",
        "impedance",
        "saturation",
        "sampleIndex",
        "isLost",
        "timeStampInMs",
        "channelIndex",
    ]

    def __init__(self):
        self.rawData = 0
        self.data = 0.0
        self.impedance = 0.0
        self.saturation = 0.0
        self.sampleIndex = 0
        self.isLost = False
        self.timeStampInMs = 0
        self.channelIndex = 0

    def reset(self):
        self.rawData = 0
        self.data = 0.0
        self.impedance = 0.0
        self.saturation = 0.0
        self.sampleIndex = 0
        self.isLost = False
        self.timeStampInMs = 0
        self.channelIndex = 0


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
    __slots__ = [
        "deviceMac",
        "dataType",
        "sampleRate",
        "channelCount",
        "packageSampleCount",
        "packageIndexLength",
        "channelSamples",
        "lastPackageCounter",
        "lastPackageIndex",
        "lostPackageCount",
        "resolutionBits",
        "resolutionSigned",
        "channelMask",
        "minPackageSampleCount",
        "K",
    ]

    def __init__(self):
        self.deviceMac = ""
        self.dataType = DataType.NTF_EEG
        self.sampleRate = 0.0
        self.channelCount = 0
        self.packageSampleCount = 0
        self.packageIndexLength = 2
        self.channelSamples: List[List[Sample]] = list()
        self.lastPackageCounter = 0
        self.lastPackageIndex = 0
        self.lostPackageCount = 0
        self.resolutionBits = 0
        self.resolutionSigned = 0
        self.channelMask = 0
        self.minPackageSampleCount = 0
        self.K = 0.0

    def reset(self):
        self.deviceMac = ""
        self.dataType = DataType.NTF_EEG
        self.sampleRate = 0.0
        self.channelCount = 0
        self.packageSampleCount = 0
        self.packageIndexLength = 2
        self.channelSamples.clear()
        self.lastPackageCounter = 0
        self.lastPackageIndex = 0
        self.lostPackageCount = 0
        self.resolutionBits = 0
        self.resolutionSigned = 0
        self.channelMask = 0
        self.minPackageSampleCount = 0
        self.K = 0.0

    def clear(self):
        self.channelSamples.clear()
        self.lastPackageCounter = -1
        self.lastPackageIndex = 0
        self.lostPackageCount = 0

    def to_flatbuffers(self) -> bytes:
        """将 SensorData 序列化为 FlatBuffers bytes。"""
        import sensor.fb.SensorData as FBSensorData
        import sensor.fb.Sample as FBSample

        builder = flatbuffers.Builder(1024)

        # 扁平化 channelSamples
        flat_samples = []
        samples_per_channel = []
        for channel in self.channelSamples:
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
                    int(sample.timeStampInMs),
                    int(sample.channelIndex),
                    int(sample.sampleIndex),
                    int(sample.rawData),
                    float(sample.data),
                    float(sample.impedance),
                    float(sample.saturation),
                    bool(sample.isLost),
                )
            samples_offset = builder.EndVector()

        # samples_per_channel 向量
        spc_offset = 0
        if samples_per_channel:
            FBSensorData.StartSamplesPerChannelVector(builder, len(samples_per_channel))
            for count in reversed(samples_per_channel):
                builder.PrependInt32(count)
            spc_offset = builder.EndVector()

        # device_mac 字符串
        mac_offset = builder.CreateString(self.deviceMac) if self.deviceMac else 0

        FBSensorData.Start(builder)
        if mac_offset:
            FBSensorData.AddDeviceMac(builder, mac_offset)
        FBSensorData.AddDataType(builder, int(self.dataType))
        FBSensorData.AddLastPackageCounter(builder, int(self.lastPackageCounter))
        FBSensorData.AddLastPackageIndex(builder, int(self.lastPackageIndex))
        FBSensorData.AddLostPackageCount(builder, int(self.lostPackageCount))
        FBSensorData.AddResolutionBits(builder, int(self.resolutionBits))
        FBSensorData.AddResolutionSigned(builder, int(self.resolutionSigned))
        FBSensorData.AddSampleRate(builder, float(self.sampleRate))
        FBSensorData.AddChannelCount(builder, int(self.channelCount))
        FBSensorData.AddChannelMask(builder, int(self.channelMask))
        FBSensorData.AddMinPackageSampleCount(builder, int(self.minPackageSampleCount))
        FBSensorData.AddPackageSampleCount(builder, int(self.packageSampleCount))
        FBSensorData.AddPackageIndexLength(builder, int(self.packageIndexLength))
        FBSensorData.AddK(builder, float(self.K))
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
        data.deviceMac = mac.decode('utf-8') if isinstance(mac, bytes) else (mac or "")
        data.dataType = DataType(fb.DataType())
        data.lastPackageCounter = fb.LastPackageCounter()
        data.lastPackageIndex = fb.LastPackageIndex()
        data.lostPackageCount = fb.LostPackageCount()
        data.resolutionBits = fb.ResolutionBits()
        data.resolutionSigned = fb.ResolutionSigned()
        data.sampleRate = fb.SampleRate()
        data.channelCount = fb.ChannelCount()
        data.channelMask = fb.ChannelMask()
        data.minPackageSampleCount = fb.MinPackageSampleCount()
        data.packageSampleCount = fb.PackageSampleCount()
        data.packageIndexLength = fb.PackageIndexLength()
        data.K = fb.K()

        data.channelSamples.clear()
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
                sample.timeStampInMs = fb_sample.TimeStampInMs()
                sample.channelIndex = fb_sample.ChannelIndex()
                sample.sampleIndex = fb_sample.SampleIndex()
                sample.rawData = fb_sample.RawData()
                sample.data = fb_sample.Data()
                sample.impedance = fb_sample.Impedance()
                sample.saturation = fb_sample.Saturation()
                sample.isLost = fb_sample.IsLost()
                channel.append(sample)
                offset += 1
            data.channelSamples.append(channel)

        return data
