# PyInstaller module hook for the sensor package.
# Bundles the dongle setup scripts shipped in the wheel (sensor/tools/)
# into the frozen executable. The SDK locates them relative to the module
# file (__file__), which under PyInstaller resolves to <_MEIPASS>/sensor/,
# so collecting the data files into sensor/tools keeps that lookup working.
from PyInstaller.utils.hooks import collect_data_files

datas = collect_data_files("sensor", includes=["tools/*"])
