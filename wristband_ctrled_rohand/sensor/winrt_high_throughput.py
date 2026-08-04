"""Windows 下给 bleak WinRT backend 打补丁，请求高吞吐率连接参数。

通过 BluetoothLEDevice.RequestPreferredConnectionParameters(
    BluetoothLEPreferredConnectionParameters.ThroughputOptimized()
) 在连接成功后把连接参数切到“吞吐量优化”模式，提升高采样率下的吞吐。

仅在 Windows 且 winrt 相关模块可用时生效；其它平台自动无操作。
"""

import logging
import platform
from functools import wraps

logger = logging.getLogger(__name__)


_REQUEST_ATTR_CANDIDATES = (
    "request_preferred_connection_parameters",
    "RequestPreferredConnectionParameters",
)


def _set_high_throughput(backend) -> bool:
    """尝试对已经连接成功的 bleak WinRT backend 设置高吞吐率连接参数。"""
    if platform.system() != "Windows":
        return False

    requester = getattr(backend, "_requester", None)
    if requester is None:
        return False

    try:
        from winrt.windows.devices.bluetooth import (
            BluetoothLEPreferredConnectionParameters,
        )
    except Exception as e:
        logger.debug("winrt BluetoothLEPreferredConnectionParameters not available: %s", e)
        return False

    params = None
    for attr in ("throughput_optimized", "ThroughputOptimized"):
        try:
            params = getattr(BluetoothLEPreferredConnectionParameters, attr)()
            break
        except Exception:
            continue

    if params is None:
        logger.debug("BluetoothLEPreferredConnectionParameters.throughput_optimized not found")
        return False

    request_fn = None
    for attr in _REQUEST_ATTR_CANDIDATES:
        request_fn = getattr(requester, attr, None)
        if request_fn is not None:
            break

    if request_fn is None:
        logger.debug("BluetoothLEDevice.request_preferred_connection_parameters not found")
        return False

    try:
        # 返回的 request 对象需要保持引用，否则系统可能恢复默认参数
        backend._high_throughput_request = request_fn(params)
        logger.debug("Requested WinRT high-throughput connection parameters")
        return True
    except Exception as e:
        logger.debug("Failed to request high-throughput connection parameters: %s", e)
        return False


def _try_patch_connect() -> None:
    """给 BleakClientWinRT.connect 打补丁，连接成功后自动设置高吞吐率参数。"""
    if platform.system() != "Windows":
        return

    try:
        from bleak.backends.winrt.client import BleakClientWinRT
    except Exception as e:
        logger.debug("bleak WinRT backend not available: %s", e)
        return

    if getattr(BleakClientWinRT, "_high_throughput_patched", False):
        return

    orig_connect = BleakClientWinRT.connect

    @wraps(orig_connect)
    async def patched_connect(self, *args, **kwargs):
        result = await orig_connect(self, *args, **kwargs)
        _set_high_throughput(self)
        return result

    BleakClientWinRT.connect = patched_connect
    BleakClientWinRT._high_throughput_patched = True  # type: ignore[attr-defined]
    logger.debug("Applied bleak WinRT high-throughput patch")


def apply() -> None:
    """在 Windows 上自动应用补丁。非 Windows 平台无操作。"""
    try:
        _try_patch_connect()
    except Exception as e:
        logger.debug("Failed to apply WinRT high-throughput patch: %s", e)
