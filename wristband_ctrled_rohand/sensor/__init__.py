from sensor.sensor_controller import SensorController, SensorControllerInstance
from sensor.sensor_profile import SensorProfile
from sensor.sensor_device import BLEDevice, DeviceInfo, DeviceStateEx
from sensor.sensor_data import DataType, Sample, SensorData
from sensor.winrt_high_throughput import apply as _apply_winrt_high_throughput_patch
# SDK 版本号（单一数据源，setup.py 打包时从这里读取）
__version__ = "0.3.2"

# Windows 下尝试给 bleak WinRT backend 打高吞吐率连接参数补丁
_apply_winrt_high_throughput_patch()

__all__ = [
    "SensorController",
    "SensorControllerInstance",
    "SensorProfile",
    "BLEDevice",
    "DeviceInfo",
    "DeviceStateEx",
    "DataType",
    "Sample",
    "SensorData",
    "__version__",
]
