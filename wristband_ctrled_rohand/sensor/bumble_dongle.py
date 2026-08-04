"""USB 蓝牙 dongle 检测与 bleak-bumble 后端选择。

在 macOS 上检测到 USB 蓝牙 HCI dongle 时，自动使用 bleak_bumble
（https://github.com/ekspla/bleak-bumble_dev_host_mode）作为 bleak 后端：
经 bumble host 模式 + usb:VID:PID 传输直接驱动 dongle，绕过 CoreBluetooth。

环境变量：
- SENSOR_SDK_BLE_BACKEND: "bleak" 强制原生后端；"bumble" 强制 bleak_bumble 后端；
  缺省为自动模式（仅 macOS 自动启用，其他平台需显式强制）。
- SENSOR_SDK_BUMBLE_TRANSPORT: 完整 bumble 传输 spec（如 "usb:0"、"usb:10d7:b012"），
  设置后优先于 USB 自动检测结果。

注意：bleak_bumble 为可选依赖（git 安装），未安装时本模块所有函数安全回退。
"""

import os
import platform
import threading
import time
from typing import Optional

from sensor.sdk_log import SdkLog

_TAG = "BumbleDongle"

_BACKEND_ENV = "SENSOR_SDK_BLE_BACKEND"
_TRANSPORT_ENV = "SENSOR_SDK_BUMBLE_TRANSPORT"

# USB 蓝牙 HCI 类码（与 bumble transport/usb.py 的判定一致）
_USB_CLASS_WIRELESS_CONTROLLER = 0xE0
_USB_SUBCLASS_RF_CONTROLLER = 0x01
_USB_PROTOCOL_BLUETOOTH_PRIMARY = 0x01
_USB_CLASS_DEVICE = 0x00

_USB_BT_HCI_CLASS_TUPLE = (
    _USB_CLASS_WIRELESS_CONTROLLER,
    _USB_SUBCLASS_RF_CONTROLLER,
    _USB_PROTOCOL_BLUETOOTH_PRIMARY,
)

# 已知支持的 USB 蓝牙 dongle 型号（VID, PID）；
# 类码非标准 HCI 的型号会以 "!" 强制模式打开（使用第一个接口）
_KNOWN_DONGLE_IDS = {
    (0x10D7, 0xB012),  # Actions "general adapter"
    (0x33FA, 0x0012),  # 新型号
}


def _load_libusb() -> None:
    """优先加载 libusb_package 自带的 libusb 动态库（与 bumble 的做法一致）。"""
    try:
        import ctypes

        import libusb_package
        import usb1

        libusb_path = libusb_package.get_library_path()
        if libusb_path:
            dll = ctypes.CDLL(str(libusb_path), use_errno=True, use_last_error=True)
            usb1.loadLibrary(dll)
    except Exception:
        # 没有 libusb_package 时由 usb1 自行在系统路径中查找
        pass


def _device_is_bluetooth_hci(device) -> bool:
    """判定 USB 设备是否为蓝牙 HCI 控制器（镜像 bumble 的匹配逻辑）。"""
    try:
        if (
            device.getDeviceClass(),
            device.getDeviceSubClass(),
            device.getDeviceProtocol(),
        ) == _USB_BT_HCI_CLASS_TUPLE:
            return True
        # 设备类为 0 时类信息定义在接口级，逐接口匹配
        if device.getDeviceClass() == _USB_CLASS_DEVICE:
            for configuration in device:
                for interface in configuration:
                    for setting in interface:
                        if (
                            setting.getClass(),
                            setting.getSubClass(),
                            setting.getProtocol(),
                        ) == _USB_BT_HCI_CLASS_TUPLE:
                            return True
    except Exception:
        return False
    return False


def detect_usb_dongle_specs(context=None) -> list:
    """枚举所有 USB 蓝牙 dongle，返回 bumble 传输 spec 列表。

    匹配规则：USB 蓝牙 HCI 类（E0/01/01），或 _KNOWN_DONGLE_IDS 中的型号
    （类码非标准 HCI 的型号加 "!" 强制模式后缀）；同一 VID:PID 插入多只时
    以 "#index" 后缀区分。无 dongle、缺 libusb 依赖或枚举出错时返回空列表。

    context: 可选的已打开 usb1.USBContext（热插拔监控复用同一上下文）；
             缺省时每次临时创建并在结束时关闭。
    """
    found = []  # (vid, pid, is_hci)
    try:
        import usb1
    except ImportError:
        SdkLog.i(_TAG, "python-libusb1 not installed, skip USB dongle detection")
        return found

    own_context = context is None
    try:
        _load_libusb()
        if own_context:
            context = usb1.USBContext()
            context.open()
        for device in context.getDeviceIterator(skip_on_error=True):
            try:
                vid = device.getVendorID()
                pid = device.getProductID()
                is_hci = _device_is_bluetooth_hci(device)
                if is_hci or (vid, pid) in _KNOWN_DONGLE_IDS:
                    found.append((vid, pid, is_hci))
            finally:
                device.close()
    except Exception as e:
        SdkLog.w(_TAG, f"USB dongle detection failed: {e}")
        return []
    finally:
        if own_context and context is not None:
            try:
                context.close()
            except Exception:
                pass

    totals = {}
    for vid, pid, _ in found:
        totals[(vid, pid)] = totals.get((vid, pid), 0) + 1
    seen = {}
    specs = []
    for vid, pid, is_hci in found:
        key = (vid, pid)
        index = seen.get(key, 0)
        seen[key] = index + 1
        spec = "usb:%04x:%04x" % (vid, pid)
        if totals[key] > 1:
            spec += "#%d" % index
        if not is_hci:
            spec += "!"  # 类码非标准 HCI：强制模式（使用第一个接口）
        specs.append(spec)
    if specs:
        SdkLog.i(_TAG, f"USB Bluetooth dongles detected: {specs}")
    return specs


def _spec_key(spec: str):
    """从 bumble 传输 spec（"usb:<vid>:<pid>[#i][!]"）解析 (vid, pid)，失败返回 (0, 0)。"""
    try:
        parts = spec.split(":")
        vid = int(parts[1], 16)
        pid = int(parts[2].split("#")[0].rstrip("!"), 16)
        return (vid, pid)
    except Exception:
        return (0, 0)


class BumbleBackend:
    """bleak_bumble 后端配置：持有多只 dongle 的传输 spec，并按需分配/释放。

    每只 dongle 同一时刻只能服务一个连接（或一次扫描）：
    allocate() 领取空闲 dongle 的 spec，release() 归还。
    热插拔监控通过 reconcile() 增删池内 spec（线程安全）。
    """

    def __init__(self, transport_specs):
        # bumble 传输 spec 列表，如 ["usb:10d7:b012", "usb:33fa:0012"]
        self.transport_specs = list(transport_specs)
        self._busy = set()      # 已分配且物理上仍在的 spec
        self._departed = set()  # 已分配但已拔出的 spec（release 时清理）
        self._lock = threading.Lock()

    def allocate(self) -> Optional[str]:
        """领取一只空闲 dongle 的 spec；无空闲时返回 None。"""
        with self._lock:
            for spec in self.transport_specs:
                if spec not in self._busy:
                    self._busy.add(spec)
                    return spec
            return None

    def release(self, spec: str) -> None:
        """归还 dongle（幂等；已拔出的 spec 同样安全）。"""
        with self._lock:
            self._busy.discard(spec)
            self._departed.discard(spec)

    def reconcile(self, new_specs: list) -> list:
        """按最新 USB 枚举对账 dongle 池（热插拔监控驱动）。

        - 无占用的型号：按新枚举全量重建（新插入即入池可用）；
        - 占用中且名称仍在新枚举中的型号：busy 保留，空闲槽按新枚举重建；
        - 占用中但名称消失、且该型号仍有设备在枚举中：拔出/换名存在歧义，
          冻结该型号（保留 busy 旧名、不加新槽，防止同一物理 dongle 被重复
          打开）；连接若已随拔出死亡，会走正常断连清理，release 后再次
          reconcile 即可解冻重建；
        - 占用中且该型号已无任何设备：无歧义拔出，转入 departed 并返回，
          由调用方主动断开受影响连接。

        返回本次无歧义拔出的占用中 spec 列表。
        """
        with self._lock:
            new_groups = {}
            for spec in new_specs:
                new_groups.setdefault(_spec_key(spec), []).append(spec)

            keys = ({_spec_key(s) for s in self.transport_specs}
                    | {_spec_key(s) for s in self._busy}
                    | set(new_groups))
            keys.discard((0, 0))

            departed_busy = []
            per_key = {}  # key -> 该型号对账后的 spec 列表
            for key in keys:
                n = new_groups.get(key, [])
                busy_key = [s for s in self._busy if _spec_key(s) == key]
                unmapped = [s for s in busy_key if s not in n]
                if not n:
                    # 该型号已无设备：busy 全部无歧义拔出
                    for s in busy_key:
                        self._busy.discard(s)
                        self._departed.add(s)
                        departed_busy.append(s)
                    per_key[key] = []
                elif unmapped:
                    # 拔出/换名歧义：冻结该型号，仅保留 busy 旧名
                    per_key[key] = busy_key
                    SdkLog.w(_TAG, f"Dongle spec ambiguity for {key[0]:04x}:{key[1]:04x} "
                                   f"(busy={busy_key}, enumerated={n}), freeze new slots")
                else:
                    # busy 全部存活：按新枚举重建（含新增/无变化）
                    per_key[key] = n

            # 稳定顺序：已有型号保持原相对位置，新出现型号按枚举顺序追加
            keep = []
            emitted = set()
            for s in self.transport_specs:
                key = _spec_key(s)
                if key in emitted:
                    continue
                emitted.add(key)
                keep.extend(per_key.get(key, []))
            for s in new_specs:
                key = _spec_key(s)
                if key in emitted:
                    continue
                emitted.add(key)
                keep.extend(per_key.get(key, []))

            if keep != self.transport_specs:
                SdkLog.i(_TAG, f"Dongle pool updated: {self.transport_specs} -> {keep}")
            self.transport_specs = keep
            return departed_busy

    @staticmethod
    def _make_cfg(spec: str):
        from sensor.bleak_bumble import BumbleTransportCfg, TransportScheme

        scheme_str, _, args = spec.partition(":")
        return BumbleTransportCfg(
            TransportScheme.from_string(scheme_str), args if args else None
        )

    def scanner_kwargs(self, spec: str) -> dict:
        """BleakScanner 构造参数（backend/cfg/host_mode）。"""
        from sensor.bleak_bumble.scanner import BleakScannerBumble

        return {
            "backend": BleakScannerBumble,
            "cfg": self._make_cfg(spec),
            "host_mode": True,
        }

    def client_kwargs(self, spec: str) -> dict:
        """BleakClient 构造参数（backend/cfg/host_mode）。"""
        from sensor.bleak_bumble.client import BleakClientBumble

        return {
            "backend": BleakClientBumble,
            "cfg": self._make_cfg(spec),
            "host_mode": True,
        }


def _patch_bumble_host_reset() -> None:
    """给 bumble Host 打补丁：规避若干 dongle 固件缺陷（仅子进程内生效，只打一次）。

    1. 某些 dongle（如 Actions 10d7:b012）在 local supported commands 掩码中声称
       支持 HCI_LE_EXTENDED_CREATE_CONNECTION，实际下发却返回 UNKNOWN_HCI_COMMAND，
       导致连接失败。对这些已知虚报的型号（按 transport 的 VID:PID 识别），
       reset 后清除该位，回退 legacy 连接命令；其余型号保持原生行为。
    2. 某些 dongle（如 33fa:0012）的 HCI_READ_LOCAL_EXTENDED_FEATURES 应答畸形
       （page 0 长度字段少报 1 字节，残留字节污染后续事件解析；page>0 返回垃圾），
       导致 reset 挂起、扫描/连接链路静默失败。该命令仅承载经典 BR/EDR 特性
       （BLE 不需要），所有 page 一律由补丁合成空应答，不下发真实命令。
    """
    from bumble import hci
    from bumble.host import Host

    if getattr(Host, "_sensor_sdk_reset_patched", False):
        return

    # 虚报 HCI_LE_EXTENDED_CREATE_CONNECTION 支持的固件（实际下发返回错误）
    _CLEAR_EXT_CREATE_IDS = {(0x10D7, 0xB012)}

    orig_reset = Host.reset
    clear_mask = hci.HCI_SUPPORTED_COMMANDS_MASKS.get(
        hci.HCI_LE_EXTENDED_CREATE_CONNECTION_COMMAND, 0
    )

    async def patched_reset(self, *args, **kwargs):
        await orig_reset(self, *args, **kwargs)
        # transport 元数据中的 VID:PID（USB 来源），用于按型号施加规避
        meta = getattr(self, "hci_metadata", None) or {}
        vid_pid = (meta.get("vendor_id"), meta.get("product_id"))
        if (clear_mask and vid_pid in _CLEAR_EXT_CREATE_IDS
                and (self.local_supported_commands & clear_mask)):
            self.local_supported_commands &= ~clear_mask

    Host.reset = patched_reset

    orig_send_sync_command = Host.send_sync_command

    async def patched_send_sync_command(self, command, *args, **kwargs):
        if isinstance(command, hci.HCI_Read_Local_Extended_Features_Command):
            return hci.HCI_Read_Local_Extended_Features_ReturnParameters(
                status=hci.HCI_SUCCESS,
                page_number=command.page_number,
                maximum_page_number=command.page_number,
                extended_lmp_features=bytes(8),
            )
        return await orig_send_sync_command(self, command, *args, **kwargs)

    Host.send_sync_command = patched_send_sync_command

    Host._sensor_sdk_reset_patched = True


def _patch_bumble_client_connect() -> None:
    """给 BleakClientBumble.connect 打补丁：连接成功后交换 ATT MTU。

    bleak_bumble 连接后不交换 MTU，ATT_MTU 保持默认 23，较长的响应
    （如固件版本号）会被截断/无法送达导致 init 超时；CoreBluetooth 默认 185。
    这里统一交换到 247（对端可在协商中降低），失败不阻断连接。

    另外在首个 GATT 请求（服务发现）前加短延迟：部分固件连接后 ATT 注册
    较慢，过早的 ATT 请求会被直接丢弃（不应答），导致服务发现超时。
    """
    import asyncio as _asyncio

    from sensor.bleak_bumble.client import BleakClientBumble

    if getattr(BleakClientBumble, "_sensor_sdk_mtu_patched", False):
        return

    orig_connect = BleakClientBumble.connect

    async def patched_connect(self, *args, **kwargs):
        await orig_connect(self, *args, **kwargs)
        try:
            peer = getattr(self, "_peer", None)
            connection = getattr(self, "_connection", None)
            if (peer is not None and connection is not None
                    and connection.att_mtu < 247):
                await peer.request_mtu(247)
        except Exception as e:
            SdkLog.w(_TAG, f"bumble MTU exchange failed: {e}")

    BleakClientBumble.connect = patched_connect

    orig_get_services = BleakClientBumble.get_services

    async def patched_get_services(self, *args, **kwargs):
        await _asyncio.sleep(0.4)
        return await orig_get_services(self, *args, **kwargs)

    BleakClientBumble.get_services = patched_get_services

    BleakClientBumble._sensor_sdk_mtu_patched = True


def _patch_bumble_usb_close() -> None:
    """给 bumble UsbTransport.close 打补丁：先等 USB 事件线程退出再关 libusb 上下文。

    原版 close() 在事件线程仍可能陷在 handleEvents 时就执行 device.close()/
    context.close()，间歇触发 libusb pthread_mutex_destroy 断言使整个子进程
    崩溃（SIGABRT），表现为连接/断开偶发失败。调整为先等待事件线程退出
    （带超时兜底），再释放 libusb 资源。
    """
    import asyncio as _asyncio

    from bumble.transport.usb import UsbTransport

    if getattr(UsbTransport, "_sensor_sdk_close_patched", False):
        return

    async def patched_close(self):
        self.source.close()
        self.sink.close()
        for endpoint in (self.source, self.sink):
            try:
                await endpoint.terminate()
            except Exception:
                # 跨事件循环调用时 terminate 可能立即失败，忽略并继续
                pass

        # 请求事件线程退出，并 join 等待线程函数真正返回；
        # 之后再销毁 libusb 上下文，不再与 handleEvents 竞争
        with self.lock:
            self.event_loop_should_exit = True
        try:
            self.context.interruptEventHandler()
        except Exception:
            pass
        try:
            await _asyncio.get_running_loop().run_in_executor(
                None, self.event_thread.join, 2.0
            )
        except Exception:
            pass

        # 事件线程已退出，可以安全释放 libusb 资源
        try:
            self.device.releaseInterface(self.acl_interface.getNumber())
            if self.sco_interface:
                self.device.releaseInterface(self.sco_interface.getNumber())
            self.device.close()
            self.context.close()
        except Exception as e:
            SdkLog.w(_TAG, f"bumble USB close cleanup failed: {e}")

    UsbTransport.close = patched_close
    UsbTransport._sensor_sdk_close_patched = True


def _patch_bumble_transport_reuse() -> None:
    """host 模式下复用 USB transport：扫描停止、断开连接、连接超时时都不再关闭。

    libusb 上下文销毁与 USB 事件线程之间存在无法彻底消除的竞态，间歇触发
    usbi_mutex_destroy/lock 断言使整个子进程崩溃（SIGABRT）。transport 复用后
    进程生命周期内不再销毁 libusb 上下文；dongle 由 SDK 子进程持有，退出前由
    BleakProcess 统一关闭。附带收益：扫描→连接切换更快。
    """
    from sensor.bleak_bumble.client import BleakClientBumble
    from sensor.bleak_bumble.scanner import BleakScannerBumble

    if getattr(BleakScannerBumble, "_sensor_sdk_reuse_patched", False):
        return

    # scanner.stop：跳过 host_mode 下的 transport 关闭，其余行为不变
    async def patched_scanner_stop(self):
        if self._dev is None:
            raise RuntimeError("Scanner not started")
        await self._dev.stop_scanning()
        # 原版在此 pop 并 close 共享 transport；复用模式下跳过
        await self._dev.power_off()
        self._dev = None

    BleakScannerBumble.stop = patched_scanner_stop

    # client：_close_transport 退化为仅释放 Device 引用，transport 保持复用
    async def patched_close_transport(self):
        self._dev = None

    BleakClientBumble._close_transport = patched_close_transport

    BleakScannerBumble._sensor_sdk_reuse_patched = True


def _patch_bumble_connection_params() -> None:
    """调整 bumble 连接参数策略，规避 Actions（10d7:b012）固件缺陷。

    背景：该 dongle 的固件执行 host 下发的 HCI_LE_Connection_Update_Command 时
    间歇挂起（命令 PENDING 后无 complete 事件，链路静默）；而 OY 设备连接后
    固定会通过 L2CAP 请求更新参数（约 12.5-25ms / latency 0 / timeout 3s），
    假意 accepted 不应用会被对端干等、rejected 会被对端直接断开。

    策略：
    1. 把 bumble 的默认连接参数压到 BLE 最小间隔 7.5ms（latency 0、
       supervision timeout 3s），使事件频率最大化，降低通知挤占；
    2. 对仍发起 L2CAP 参数更新请求的情况，L2CAP 层应答 accepted，并把
       HCI 命令延迟 300ms 下发——间隔固定取最小值 6（7.5ms），
       latency/timeout 沿用对端请求，避开连接初期 LL 过程碰撞导致的固件挂起。
    """
    import asyncio as _asyncio

    from bumble import hci
    from bumble.device import ConnectionParametersPreferences
    from bumble.l2cap import (
        L2CAP_CONNECTION_PARAMETERS_ACCEPTED_RESULT,
        L2CAP_CONNECTION_PARAMETERS_REJECTED_RESULT,
        ChannelManager,
        L2CAP_Connection_Parameter_Update_Response,
    )

    if getattr(ChannelManager, "_sensor_sdk_param_update_patched", False):
        return

    # BLE 规范最小连接间隔：7.5ms = 6 × 1.25ms
    _MIN_INTERVAL_UNITS = 6

    ConnectionParametersPreferences.default = ConnectionParametersPreferences(
        connection_interval_min=7.5,
        connection_interval_max=7.5,
        max_latency=0,
        supervision_timeout=3000,
    )

    def patched_on_param_update(self, connection, cid, request):
        if connection.role == hci.Role.CENTRAL:
            self.send_control_frame(
                connection,
                cid,
                L2CAP_Connection_Parameter_Update_Response(
                    identifier=request.identifier,
                    result=L2CAP_CONNECTION_PARAMETERS_ACCEPTED_RESULT,
                ),
            )

            # 延迟下发，避开连接初期的 LL 过程碰撞窗口；
            # 间隔固定取最小值（7.5ms），latency/timeout 沿用对端请求
            def _issue_update():
                try:
                    self.host.send_command_sync(
                        hci.HCI_LE_Connection_Update_Command(
                            connection_handle=connection.handle,
                            connection_interval_min=_MIN_INTERVAL_UNITS,
                            connection_interval_max=_MIN_INTERVAL_UNITS,
                            max_latency=request.latency,
                            supervision_timeout=request.timeout,
                            min_ce_length=0,
                            max_ce_length=0,
                        )
                    )
                except Exception:
                    pass

            _asyncio.get_running_loop().call_later(0.3, _issue_update)
        else:
            self.send_control_frame(
                connection,
                cid,
                L2CAP_Connection_Parameter_Update_Response(
                    identifier=request.identifier,
                    result=L2CAP_CONNECTION_PARAMETERS_REJECTED_RESULT,
                ),
            )

    ChannelManager.on_l2cap_connection_parameter_update_request = (
        patched_on_param_update
    )
    ChannelManager._sensor_sdk_param_update_patched = True


def resolve_bumble_backend() -> Optional[BumbleBackend]:
    """按环境变量与硬件检测决定是否启用 bleak_bumble 后端。

    返回 BumbleBackend 实例；不启用或条件不满足（缺依赖/无 dongle）时返回 None。
    """
    backend_env = (os.environ.get(_BACKEND_ENV) or "").strip().lower()
    if backend_env == "bleak":
        return None
    if backend_env and backend_env != "bumble":
        SdkLog.w(_TAG, f"Unknown {_BACKEND_ENV}={backend_env}, fallback to auto")

    forced = backend_env == "bumble"
    if not forced and platform.system() != "Darwin":
        # 自动模式仅限 macOS：PC 的内置蓝牙常是 USB HCI 设备，避免误切换；
        # Apple 主机的内置蓝牙不走 USB，检测到的 USB HCI 设备必为外插 dongle
        return None

    try:
        from sensor import bleak_bumble  # noqa: F401
        import bumble  # noqa: F401
    except ImportError as e:
        SdkLog.w(_TAG, f"bleak_bumble/bumble not installed, use native bleak: {e}")
        return None

    env_spec = (os.environ.get(_TRANSPORT_ENV) or "").strip()
    if env_spec:
        specs = [env_spec]
    else:
        specs = detect_usb_dongle_specs()
    if not specs:
        SdkLog.w(_TAG, "No USB Bluetooth dongle detected, use native bleak")
        return None

    _apply_bumble_patches()
    SdkLog.i(_TAG, f"Use bleak_bumble backend, transports={specs}")
    return BumbleBackend(specs)


_patches_applied = False


def _apply_bumble_patches() -> None:
    """应用 dongle/bumble 兼容补丁（幂等；仅 bumble 后端激活时调用）。"""
    global _patches_applied
    if _patches_applied:
        return
    _patches_applied = True
    try:
        _patch_bumble_host_reset()
        _patch_bumble_client_connect()
        _patch_bumble_usb_close()
        _patch_bumble_transport_reuse()
        _patch_bumble_connection_params()
    except Exception as e:
        # 补丁失败不致命：扫描仍可用，仅老旧固件上连接可能失败
        SdkLog.w(_TAG, f"bumble patches failed: {e}")


def create_runtime_backend(transport_specs: list) -> Optional[BumbleBackend]:
    """运行中构建 bumble 后端：启动时无 dongle、热插拔监控发现插入后的切换路径。"""
    try:
        from sensor import bleak_bumble  # noqa: F401
        import bumble  # noqa: F401
    except ImportError as e:
        SdkLog.w(_TAG, f"bleak_bumble/bumble not installed, cannot switch backend: {e}")
        return None
    _apply_bumble_patches()
    SdkLog.i(_TAG, f"Hotplug: switch to bleak_bumble backend, transports={transport_specs}")
    return BumbleBackend(transport_specs)


class DongleHotplugMonitor:
    """USB 蓝牙 dongle 热插拔监控（仅 BLE 子进程内使用，守护线程）。

    优先使用 libusb hotplug 回调；不可用或出错时退化为周期轮询。
    枚举结果发生变化时经 on_change(new_specs) 上报（监控线程上下文，
    由调用方序列化到事件循环处理）。stop() 只置停止标志，不销毁
    libusb 上下文（守护线程随进程退出回收，避免析构期崩溃）。
    """

    def __init__(self, on_change, initial_specs, poll_interval=2.0, debounce=0.4):
        self._on_change = on_change
        self._last_specs = list(initial_specs)
        self._poll_interval = poll_interval
        self._debounce = debounce
        self._stop_event = threading.Event()
        self._dirty = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="DongleHotplugMonitor", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def _run(self) -> None:
        context = None
        use_hotplug = False
        try:
            _load_libusb()
            import usb1

            context = usb1.USBContext()
            context.open()
            if usb1.hasCapability(usb1.CAP_HAS_HOTPLUG):
                def _hotplug_cb(_ctx, _device, _event):
                    self._dirty.set()
                    return False  # 保持注册（truthy 返回值会注销回调）

                context.hotplugRegisterCallback(
                    _hotplug_cb,
                    events=(usb1.HOTPLUG_EVENT_DEVICE_ARRIVED
                            | usb1.HOTPLUG_EVENT_DEVICE_LEFT),
                    vendor_id=usb1.HOTPLUG_MATCH_ANY,
                    product_id=usb1.HOTPLUG_MATCH_ANY,
                    dev_class=usb1.HOTPLUG_MATCH_ANY,
                )
                use_hotplug = True
        except Exception as e:
            SdkLog.w(_TAG, f"USB hotplug unavailable, fallback to polling: {e}")
            context = None

        SdkLog.i(_TAG, f"Dongle hotplug monitor started "
                       f"({'hotplug' if use_hotplug else 'polling'}, "
                       f"initial={self._last_specs})")
        while not self._stop_event.is_set():
            try:
                if use_hotplug:
                    # 阻塞至任意 USB 事件或超时；hotplug 回调仅置脏标志
                    context.handleEventsTimeout(self._poll_interval)
                    if not self._dirty.is_set():
                        continue
                    self._dirty.clear()
                    time.sleep(self._debounce)  # 等待枚举稳定，合并连续事件
                else:
                    time.sleep(self._poll_interval)
                self._check(context)
            except Exception as e:
                SdkLog.w(_TAG, f"Dongle hotplug monitor error: {e}")
                time.sleep(self._poll_interval)

    def _check(self, context) -> None:
        new_specs = detect_usb_dongle_specs(context=context)
        if new_specs == self._last_specs:
            return
        self._last_specs = new_specs
        try:
            self._on_change(new_specs)
        except Exception as e:
            SdkLog.w(_TAG, f"Dongle hotplug on_change failed: {e}")


def create_hotplug_monitor(on_change, initial_specs) -> Optional[DongleHotplugMonitor]:
    """按环境判定创建 dongle 热插拔监控（规则与 resolve_bumble_backend 一致）。

    SENSOR_SDK_BLE_BACKEND=bleak 或显式设置 SENSOR_SDK_BUMBLE_TRANSPORT 时不监控；
    自动模式仍限 macOS。无论当前是否已启用 bumble 后端都会监控
    （启动时无 dongle、之后插入时需要切换到 bumble 后端）。
    """
    backend_env = (os.environ.get(_BACKEND_ENV) or "").strip().lower()
    if backend_env == "bleak":
        return None
    if (os.environ.get(_TRANSPORT_ENV) or "").strip():
        return None
    if backend_env != "bumble" and platform.system() != "Darwin":
        return None
    try:
        import usb1  # noqa: F401
    except ImportError:
        return None
    return DongleHotplugMonitor(on_change, initial_specs)
