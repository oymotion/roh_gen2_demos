from sensor.sensor_controller import SensorController, SensorControllerInstance
from sensor.sensor_profile import SensorProfile
from sensor.sensor_device import BLEDevice, DeviceInfo, DeviceStateEx
from sensor.sensor_data import DataType, Sample, SensorData
from sensor.sensor_utils import checkSetupDongle
from sensor.winrt_high_throughput import apply as _apply_winrt_high_throughput_patch

__version__ = "0.9.7"

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
    "checkSetupDongle",
    "__version__",
]


import sys as _sys


from sensor import sdk_log as _sdk_log  # noqa: F401
from sensor import sensor_utils as _sensor_utils  # noqa: F401
from sensor import sensor_device as _sensor_device  # noqa: F401
from sensor import sensor_data as _sensor_data  # noqa: F401
from sensor import sensor_data_pool as _sensor_data_pool  # noqa: F401
from sensor import sensor_data_context as _sensor_data_context  # noqa: F401
from sensor import gforce as _gforce  # noqa: F401
from sensor import bleak_host as _bleak_host  # noqa: F401
from sensor import bleak_process as _bleak_process  # noqa: F401
from sensor import bumble_dongle as _bumble_dongle  # noqa: F401
from sensor import bin_recorder as _bin_recorder  # noqa: F401
from sensor import bin_to_csv as _bin_to_csv  # noqa: F401
from sensor import winrt_high_throughput as _winrt_high_throughput  # noqa: F401
import sensor.fb.DataType as _fb_datatype  # noqa: F401
import sensor.fb.Sample as _fb_sample  # noqa: F401
import sensor.fb.SensorData as _fb_sensordata  # noqa: F401
from sensor.bleak_bumble import client as _bb_client  # noqa: F401
from sensor.bleak_bumble import scanner as _bb_scanner  # noqa: F401
from sensor.bleak_bumble import utils as _bb_utils  # noqa: F401

import bleak as _bleak  # noqa: F401
import flatbuffers as _flatbuffers  # noqa: F401
import numpy as _numpy  # noqa: F401
import typing_extensions as _typing_extensions  # noqa: F401
import bumble.core as _bumble_core  # noqa: F401
import bumble.controller as _bumble_controller  # noqa: F401
import bumble.device as _bumble_device  # noqa: F401
import bumble.hci as _bumble_hci  # noqa: F401
import bumble.host as _bumble_host  # noqa: F401
import bumble.l2cap as _bumble_l2cap  # noqa: F401
import bumble.link as _bumble_link  # noqa: F401
import bumble.transport.usb as _bumble_transport_usb  # noqa: F401
import usb1 as _usb1  # noqa: F401
import libusb_package as _libusb_package  # noqa: F401

import asyncio as _asyncio  # noqa: F401
import atexit as _atexit  # noqa: F401
import collections as _collections  # noqa: F401
import collections.abc as _collections_abc  # noqa: F401
import concurrent.futures as _concurrent_futures  # noqa: F401
import contextlib as _contextlib  # noqa: F401
import csv as _csv  # noqa: F401
import dataclasses as _dataclasses  # noqa: F401
import datetime as _datetime  # noqa: F401
import enum as _enum  # noqa: F401
import errno as _errno  # noqa: F401
import functools as _functools  # noqa: F401
import logging as _logging  # noqa: F401
import logging.handlers as _logging_handlers  # noqa: F401
import math as _math  # noqa: F401
import multiprocessing as _multiprocessing  # noqa: F401
import os as _os  # noqa: F401
import pathlib as _pathlib  # noqa: F401
import platform as _platform  # noqa: F401
import queue as _queue  # noqa: F401
import shlex as _shlex  # noqa: F401
import shutil as _shutil  # noqa: F401
import signal as _signal  # noqa: F401
import struct as _struct  # noqa: F401
import subprocess as _subprocess  # noqa: F401
import tempfile as _tempfile  # noqa: F401
import threading as _threading  # noqa: F401
import time as _time  # noqa: F401
import typing as _typing  # noqa: F401
import uuid as _uuid  # noqa: F401
import warnings as _warnings  # noqa: F401

if _sys.platform == "win32":
    try:
        import winrt.windows.devices.bluetooth as _winrt_bt  # noqa: F401
        from bleak.backends.winrt import client as _bleak_winrt_client  # noqa: F401
    except ImportError:
        pass
