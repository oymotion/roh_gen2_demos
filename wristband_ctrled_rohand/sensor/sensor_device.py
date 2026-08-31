

from enum import Enum


class DeviceInfo:
    # """
    # Initialize a DeviceInfo instance.












    # """
    # def __init__(self, device_name: str, model_name: str, hardware_version: str, firmware_version: str,
    #              emg_channel_count: int, eeg_channel_count: int, ecg_channel_count: int,
    #              acc_channel_count: int, gyro_channel_count: int, brth_channel_count: int, mtu_size: int):
    #     self.DeviceName = device_name
    #     self.ModelName = model_name
    #     self.HardwareVersion = hardware_version
    #     self.FirmwareVersion = firmware_version
    #     self.EmgChannelCount = emg_channel_count
    #     self.EegChannelCount = eeg_channel_count
    #     self.EcgChannelCount = ecg_channel_count
    #     self.AccChannelCount = acc_channel_count
    #     self.GyroChannelCount = gyro_channel_count
    #     self.BrthChannelCount = brth_channel_count
    #     self.MTUSize = mtu_size

    def __init__(self):
        self.DeviceName = ""
        self.ModelName = ""
        self.HardwareVersion = ""
        self.FirmwareVersion = ""
        self.PpgChannelCount =0
        self.PpgSampleRate = 0
        self.Spo2ChannelCount = 0
        self.Spo2SampleRate = 0
        self.ImpeChannelCount = 0
        self.ImpeSampleRate = 0
        self.EmgChannelCount = 0
        self.EmgSampleRate = 0
        self.EegChannelCount = 0
        self.EegSampleRate = 0
        self.EcgChannelCount = 0
        self.EcgSampleRate = 0
        # 设备能力查询（get_emg/eeg/ecg_raw_data_cap）返回的最高采样率；
        # 0 表示设备未上报或不支持该查询
        self.EmgMaxSampleRate = 0
        self.EegMaxSampleRate = 0
        self.EcgMaxSampleRate = 0
        self.AccChannelCount = 0
        self.AccSampleRate = 0
        self.GyroChannelCount = 0
        self.GyroSampleRate = 0
        self.BrthChannelCount = 0
        self.BrthSampleRate = 0
        self.MagAngleChannelCount = 0
        self.MagAngleSampleRate = 0
        self.EulerChannelCount = 0
        self.EulerSampleRate = 0
        self.QuatChannelCount = 0
        self.QuatSampleRate = 0
        # NTF_IMU 聚合流信息（acc+gyro+euler+quat 合并广播，仅新 EMG 设备提供；
        # 0 表示无聚合流，应用应改用 ACC/GYRO/EULER/QUAT 独立流）
        self.ImuChannelCount = 0
        self.ImuSampleRate = 0
        self.MTUSize = 0
        # 链路连接参数（仅 bumble 后端可获取，原生 bleak 后端为未知值；
        # 外设连接后可能通过 L2CAP 更新，经 onDeviceInfoUpdate 事件刷新）
        self.ConnectionIntervalMs = 0.0  # 连接间隔，毫秒；0 = 未知
        self.PeripheralLatency = -1      # 从设备延迟，事件数；-1 = 未知（0 是合法值）
        self.SupervisionTimeoutMs = 0    # 监督超时，毫秒；0 = 未知


class DeviceStateEx(Enum):
    Disconnected = 0
    Connecting = 1
    Connected = 2
    Ready = 3
    Disconnecting = 4
    Invalid = 5


class BLEChipType(Enum):
    Unknown = -1
    OYM = 0
    RFSTAR = 1


class BLEDevice:


    def __init__(self, name: str, address: str, rssi: int):


        self.Name = name
        self.Address = address
        self.RSSI = rssi
