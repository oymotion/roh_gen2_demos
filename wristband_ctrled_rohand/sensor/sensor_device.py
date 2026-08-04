

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
        self.MTUSize = 0


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
