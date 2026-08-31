"""PyInstaller hook 目录入口。

经 setup_cython.py 的 ``pyinstaller40`` entry point（``hook-dirs``）注册，
PyInstaller 打包任何依赖 sensor-sdk 的应用时会自动加载本目录下的
``hook-sensor.py``，把随 wheel 分发的 ``sensor/tools/`` dongle 安装脚本
一并打进冻结程序，无需用户手动配置 ``--add-data``。
"""

import os


def get_hook_dirs():
    """返回 PyInstaller hook 文件所在目录。"""
    return [os.path.dirname(os.path.abspath(__file__))]
