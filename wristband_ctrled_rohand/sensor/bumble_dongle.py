"""USB 蓝牙 dongle 检测与 bleak-bumble 后端选择。

检测到装好驱动的 USB 蓝牙 HCI dongle 时（任何平台），自动使用 bleak_bumble
（https://github.com/ekspla/bleak-bumble_dev_host_mode）作为 bleak 后端：
经 bumble host 模式 + usb:VID:PID 传输直接驱动 dongle，绕过系统蓝牙栈。

"装好驱动"由 detect_usb_dongle_specs 的枚举天然保证：只有 libusb 能实际
打开的设备才会被检出——Windows 需绑定 WinUSB（sensor/tools/setup_dongle_winusb.ps1），
Linux 需 udev 权限（sensor/tools/setup_dongle_udev.sh），macOS 免驱；未装驱动的
内置蓝牙（BTHUSB / 无权限）打不开、不会被误当 dongle。安装脚本随 wheel 打包在
sensor/tools/ 下，也可由 SensorController.checkSetupDongle() 自动提权调用
（见本模块 check_setup_dongle）。

环境变量：
- SENSOR_SDK_BLE_BACKEND: "bleak" 强制原生后端；"bumble" 强制 bleak_bumble 后端；
  缺省为自动模式（全平台：检测到装好驱动的 dongle 即启用）。
- SENSOR_SDK_BUMBLE_TRANSPORT: 完整 bumble 传输 spec（如 "usb:0"、"usb:10d7:b012"），
  设置后优先于 USB 自动检测结果。

注意：bleak_bumble 为可选依赖（git 安装），未安装时本模块所有函数安全回退。
"""

import asyncio
import json
import logging
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
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
    (0x10D7, 0xB012),  # Actions ATS2851 Bluetooth 5.3 [UGREEN CM591]
    (0x10D7, 0xB008),  # Actions ATS2831 Bluetooth 5.2 [UGREEN CM672]
    (0x33FA, 0x0001),  # BARROT BRLINK Generic Bluetooth Adapter
    (0x33FA, 0x0010),  # UGREEN BCM20702A1 Bluetooth Dongle
    (0x33FA, 0x0012),  # BARROT 新型号
}

# 已知不可用的型号，全平台排除（即使类码匹配 E0/01/01 也不算 dongle）：
# Realtek/MTK USB 网卡的固件需 OS 驱动加载（Windows BthUSB / Linux btusb），
# host 模式下 HCI 能正常应答但射频不工作（实测 2b89:6275 RTL8761B：
# 扫描 0 广播、按 MAC 直连超时），借到它扫描/连接必然失败
_EXCLUDED_DONGLE_VIDS = {0x0BDA}  # Realtek 自家 VID，全部为 Realtek 芯片
_EXCLUDED_DONGLE_IDS = {          # 绿联 VID（2b89）下的 Realtek 型号 + MT7921U
    (0x2B89, 0x6268),  # Realtek RTL8761BUV [UGREEN CM408]
    (0x2B89, 0x6272),  # Realtek RTL8761BUV [UGREEN CM390]
    (0x2B89, 0x6275),  # Realtek RTL8761B   [UGREEN CM748]
    (0x2B89, 0x6276),  # Realtek RTL8761B   [UGREEN CM749]
    (0x2B89, 0x6278),  # Realtek RTL8761B   [UGREEN CM591]
    (0x0B05, 0x190E),  # MT7921U            [UGREEN CM749/CM748-75073]
}

# dongle 安装脚本/驱动目录（随 wheel 以 package_data 打包在 sensor/tools/ 下；
# 源码与编译为 .pyd/.so 后 __file__ 均在包目录内，两种形态都能定位）
_TOOLS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tools")


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


def _device_usable(device) -> bool:
    """试开/关一次，判断 dongle 是否装好驱动/权限、可供 bumble 使用。

    枚举可见不代表可用：Windows 未绑 WinUSB 时 open 失败
    （USBErrorNotSupported）；Linux 无 udev 权限时失败（USBErrorAccess）。
    Windows 上设备已被占用时 open 返回 USBErrorAccess——占用者可能就是
    本 SDK 的连接/扫描，驱动本身可用，故 Windows 的 ACCESS 按可用计；
    Linux 不独占 USB 设备，ACCESS 即无权限。macOS 免驱且独占语义不同，
    保持仅枚举判定（不试开）。
    """
    if sys.platform == "darwin":
        return True
    try:
        handle = device.open()
    except Exception as e:
        if type(e).__name__ == "USBErrorAccess":
            return sys.platform.startswith("win")
        return False  # NotSupported（未绑驱动）/ NotFound（刚拔掉）等
    try:
        handle.close()
    except Exception:
        pass
    return True


def detect_usb_dongle_specs(context=None, with_topology=False):
    """枚举所有 USB 蓝牙 dongle，返回 bumble 传输 spec 列表。

    匹配规则：USB 蓝牙 HCI 类（E0/01/01），或 _KNOWN_DONGLE_IDS 中的型号
    （类码非标准 HCI 的型号加 "!" 强制模式后缀）；_EXCLUDED_DONGLE_VIDS /
    _EXCLUDED_DONGLE_IDS 中的已知不可用型号（Realtek）最先排除，不参与匹配。spec 一律使用 bumble 的
    "usb:<bus>-<port.path>" 拓扑形式——名字即物理槽位，新设备插入/拔出
    不会引起其他设备重命名，占用记录（busy 表 / bleak_bumble transports
    缓存）永远指向正确的物理设备。早期版本用 "usb:vid:pid#index"（按拓扑
    排序编号），但新设备插到排序中间时后续设备全部重号，busy 名字瞬间
    指向别的物理 dongle（扫描/连接打开已被占用的设备，claimInterface 必报
    LIBUSB_ERROR_ACCESS）。
    每个候选设备都会试开一次，libusb 打不开的
    （Windows 未绑 WinUSB、Linux 无 udev 权限）不算可用 dongle。
    无 dongle、缺 libusb 依赖或枚举出错时返回空列表。

    context: 可选的已打开 usb1.USBContext（热插拔监控复用同一上下文）；
             缺省时每次临时创建并在结束时关闭。
    with_topology: True 时返回 (specs, topo_map)，topo_map 为
             {spec: "usb:vid:pid@bus:port.path"}（带型号信息的拓扑身份，
             供日志与连接偏好用）。已知型号 dongle 的 USB serial 全同
             （实测两只 Actions 10d7:b012 均为 "ACTIONS1234"）、
             HCI_Read_BD_ADDR 不被固件支持（UNKNOWN_HCI_COMMAND），
             硬件层无唯一身份，拓扑是唯一可用的持久标识（换口即变，
             偏好未命中时调用方退回任意空闲 dongle）。
    """
    found = []  # (vid, pid, is_hci, bus, ports)
    try:
        import usb1
    except ImportError:
        SdkLog.i(_TAG, "python-libusb1 not installed, skip USB dongle detection")
        return ([], {}) if with_topology else found

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
                if (vid in _EXCLUDED_DONGLE_VIDS
                        or (vid, pid) in _EXCLUDED_DONGLE_IDS):
                    continue  # 已知不可用的 Realtek 型号，即使类码匹配也排除
                is_hci = _device_is_bluetooth_hci(device)
                if is_hci or (vid, pid) in _KNOWN_DONGLE_IDS:
                    if not _device_usable(device):
                        continue  # 枚举可见但不可用（未装驱动/无权限），不算 dongle
                    try:
                        bus = device.getBusNumber()
                        ports = tuple(device.getPortNumberList())
                    except Exception:
                        bus, ports = 0, ()
                    found.append((vid, pid, is_hci, bus, ports))
            finally:
                device.close()
    except Exception as e:
        SdkLog.w(_TAG, f"USB dongle detection failed: {e}")
        return ([], {}) if with_topology else []
    finally:
        if own_context and context is not None:
            try:
                context.close()
            except Exception:
                pass

    # 按物理拓扑（总线号+端口路径）排序，保证返回列表顺序稳定
    found.sort(key=lambda item: (item[3], item[4]))

    specs = []
    topo_map = {}
    for vid, pid, is_hci, bus, ports in found:
        if not bus or not ports:
            # 读不到拓扑就无法安全命名/寻址，宁可跳过
            SdkLog.w(_TAG, f"USB dongle {vid:04x}:{pid:04x} topology unavailable, skipped")
            continue
        port_path = ".".join(str(p) for p in ports)
        spec = "usb:%d-%s" % (bus, port_path)
        if not is_hci:
            spec += "!"  # 类码非标准 HCI：强制模式（使用第一个接口）
        specs.append(spec)
        topo_map[spec] = "usb:%04x:%04x@%d:%s" % (vid, pid, bus, port_path)
    if with_topology:
        return specs, topo_map
    return specs


def _translate_usb_spec_to_topology(spec: str) -> str:
    """把 "vid:pid#index[!]" 形式的 USB spec 翻译成 "bus-port.path" 拓扑形式。

    SDK 侧按物理拓扑排序编号（detect_usb_dongle_specs），而 bumble 原生按
    全新 libusb context 的枚举顺序数同型号第 index 个；两者顺序并不一致
    （macOS 实测 bus=2 的设备枚举在 bus=1 之前），同型号多只时 "#index"
    会指向错误的物理设备——指向已被连接占用的 dongle 时 claimInterface
    必报 LIBUSB_ERROR_ACCESS（第 4 只 dongle 插入后无法使用的根因）。
    这里用与编号一致的拓扑排序重新解析 index，换成 bumble 的
    "<bus>-<port.path>" 形式（按物理路径寻址，无歧义）。
    解析失败/设备未找到时原样返回，由 bumble 按原逻辑报错。
    """
    core = spec
    suffix = ""
    if core.endswith("!"):
        core, suffix = core[:-1], "!"
    if "+sco=" in core or ":" not in core:
        return spec  # SDK 不使用 sco；非 vid:pid 形式不翻译
    vid_s, _, rest = core.partition(":")
    pid_s, sep, idx_s = rest.partition("#")
    if not sep or not idx_s.isdigit():
        return spec  # 无 #index 后缀（SDK 生成的 spec 一律带），保持原样
    try:
        vid, pid = int(vid_s, 16), int(pid_s, 16)
    except ValueError:
        return spec
    index = int(idx_s)
    try:
        import usb1

        _load_libusb()
        context = usb1.USBContext()
        context.open()
    except Exception:
        return spec
    try:
        candidates = []
        for device in context.getDeviceIterator(skip_on_error=True):
            try:
                if device.getVendorID() == vid and device.getProductID() == pid:
                    try:
                        candidates.append(
                            (device.getBusNumber(), tuple(device.getPortNumberList())))
                    except Exception:
                        candidates.append((0, ()))
            finally:
                device.close()
        # 与 detect_usb_dongle_specs 编号一致的排序：总线号 + 端口路径
        candidates.sort(key=lambda item: (item[0], item[1]))
        if index >= len(candidates):
            return spec
        bus, ports = candidates[index]
        return "%d-%s%s" % (bus, ".".join(str(p) for p in ports), suffix)
    except Exception:
        return spec
    finally:
        try:
            context.close()
        except Exception:
            pass


def _known_dongle_plugged() -> bool:
    """枚举 USB 设备（不要求可打开），检查是否插着已知型号的 dongle。"""
    try:
        import usb1
    except ImportError:
        return False

    context = None
    try:
        _load_libusb()
        context = usb1.USBContext()
        context.open()
        for device in context.getDeviceIterator(skip_on_error=True):
            try:
                if (device.getVendorID(), device.getProductID()) in _KNOWN_DONGLE_IDS:
                    return True
            finally:
                device.close()
    except Exception as e:
        SdkLog.w(_TAG, f"USB dongle presence check failed: {e}")
    finally:
        if context is not None:
            try:
                context.close()
            except Exception:
                pass
    return False


def _is_windows_admin() -> bool:
    """当前进程是否具有管理员权限。"""
    try:
        import ctypes

        return bool(ctypes.windll.shell32.IsUserAnAdmin())
    except Exception:
        return False


def _read_setup_log(log_file: str) -> str:
    """读取提权安装脚本日志的尾部（PowerShell *> 默认按 UTF-16 写出，按 BOM 嗅探解码）。"""
    try:
        with open(log_file, "rb") as f:
            raw = f.read()
    except Exception:
        return ""
    if raw[:2] in (b"\xff\xfe", b"\xfe\xff"):
        text = raw.decode("utf-16", errors="replace")
    else:
        text = raw.decode("utf-8", errors="replace")
    return text.strip()[-2000:]


def _run_tool_ps1(script: str, extra_args: list = None) -> tuple:
    """运行 sensor/tools 下的 ps1 脚本，返回 (退出码, 输出尾部)。

    已是管理员时直接运行并捕获输出；否则经 Start-Process -Verb RunAs 弹 UAC
    提权（-Wait 等待并透传脚本退出码），提权窗口内的脚本输出重定向到临时
    日志，失败时读取其尾部作为系统出错信息。
    """
    extra_args = extra_args or []
    if _is_windows_admin():
        try:
            result = subprocess.run(
                ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
                 "-File", script] + extra_args,
                capture_output=True, text=True, errors="replace")
        except Exception as e:
            return 1, f"failed to launch {os.path.basename(script)}: {e}"
        detail = ((result.stdout or "") + (result.stderr or "")).strip()
        return result.returncode, detail
    log_file = os.path.join(
        tempfile.gettempdir(),
        "sensor_sdk_%s.log" % os.path.splitext(os.path.basename(script))[0])
    try:
        os.remove(log_file)
    except OSError:
        pass
    inner_command = "& '%s' %s *> '%s'; exit $LASTEXITCODE" % (
        script, " ".join(extra_args), log_file)
    ps_command = (
        "try { $p = Start-Process -FilePath 'powershell.exe' -Verb RunAs -Wait -PassThru "
        "-ArgumentList @('-NoProfile','-ExecutionPolicy','Bypass','-Command','%s') "
        "-ErrorAction Stop; exit $p.ExitCode } catch { Write-Error $_.Exception.Message; exit 1 }"
    ) % inner_command.replace("'", "''")
    try:
        result = subprocess.run(["powershell", "-NoProfile", "-Command", ps_command],
                                capture_output=True, text=True, errors="replace")
    except Exception as e:
        return 1, f"failed to launch elevated {os.path.basename(script)}: {e}"
    detail = _read_setup_log(log_file)
    outer_err = (result.stderr or "").strip()
    if outer_err and not detail:
        detail = outer_err  # 例如用户取消 UAC：The operation was canceled by the user
    return result.returncode, detail


def _setup_dongle_windows() -> tuple:
    """运行 setup_dongle_winusb.ps1 把 dongle 驱动换绑为 WinUSB，返回 (成功与否, 出错信息)。

    退出码：0=成功/已是 WinUSB，2=未找到 dongle，1=失败或用户取消 UAC。
    """
    script = os.path.join(_TOOLS_DIR, "setup_dongle_winusb.ps1")
    if not os.path.isfile(script):
        return False, f"dongle setup script not found: {script}"
    rc, detail = _run_tool_ps1(script)
    if rc == 0:
        return True, ""
    if rc == 2:
        return False, "no USB BLE dongle found by the setup script, plug one in and retry"
    base = f"dongle setup script failed (exit {rc})"
    return False, f"{base}: {detail}" if detail else base


def _recover_dongle_status_windows() -> tuple:
    """提权运行 check_dongle_status.ps1 -Recover 恢复状态异常的 dongle。

    被禁用（problem code 22，libusb 枚举不到）的设备重新启用，其他异常码的
    设备做禁用+启用重启。返回 (成功与否, 出错信息)；恢复后仍有异常的设备
    由调用方通过 _windows_pnp_dongle_status() 复查并文字提示。
    """
    script = os.path.join(_TOOLS_DIR, "check_dongle_status.ps1")
    if not os.path.isfile(script):
        return False, f"dongle check script not found: {script}"
    rc, detail = _run_tool_ps1(script, ["-Recover"])
    if rc == 0:
        return True, ""
    base = f"dongle status recovery failed (exit {rc})"
    return False, f"{base}: {detail}" if detail else base


def _windows_pnp_dongle_status():
    """经 PnP 枚举已知 dongle 的数量与状态（libusb 枚举不到的被禁用设备也能看到）。

    以 problem code 判定健康（0=正常；22=被禁用等），同时给出驱动服务名
    （非 WinUSB 即尚未换绑驱动）。返回 {"total": int, "devices": [...]}；
    脚本缺失或执行失败返回 None（调用方退化为仅 libusb 检测的旧行为）。
    """
    script = os.path.join(_TOOLS_DIR, "check_dongle_status.ps1")
    if not os.path.isfile(script):
        return None
    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
             "-File", script, "-Json"],
            capture_output=True, text=True, errors="replace")
    except Exception as e:
        SdkLog.w(_TAG, f"PnP dongle status check failed: {e}")
        return None
    try:
        payload = json.loads((result.stdout or "").strip())
        devices = payload.get("devices") or []
        if isinstance(devices, dict):  # 单个设备时 ConvertTo-Json 不包数组
            devices = [devices]
        return {"total": int(payload.get("total") or 0), "devices": devices}
    except (ValueError, TypeError, AttributeError) as e:
        SdkLog.w(_TAG, f"PnP dongle status check returned no usable data "
                       f"(exit {result.returncode}): {e}")
        return None


def _run_linux_setup(cmd: list) -> tuple:
    """直接运行安装命令并解释退出码，返回 (成功与否, 出错信息)。"""
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, errors="replace")
    except Exception as e:
        return False, f"failed to launch dongle setup: {e}"
    if result.returncode == 0:
        return True, ""
    detail = ((result.stderr or "") + (result.stdout or "")).strip()[-1000:]
    base = f"dongle setup script failed (exit {result.returncode})"
    return False, f"{base}: {detail}" if detail else base


def _has_controlling_terminal() -> bool:
    """是否有控制终端（sudo 能在 /dev/tty 上提示并读取密码）。"""
    try:
        fd = os.open("/dev/tty", os.O_RDONLY | os.O_NOCTTY)
    except OSError:
        return False
    os.close(fd)
    return True


def _linux_terminal_argv(script: str, rc_path: str):
    """探测可用的终端模拟器，返回 (argv, factory)；找不到返回 None。

    窗口内执行 sudo 安装脚本，退出码写入 rc_path 后等待用户按键关窗
    （各终端进程退出行为不一致，结果一律以 rc 文件为准）。factory=True
    表示终端进程会立即返回（gnome-terminal/mate-terminal 经 D-Bus 工厂
    启动），调用方需轮询 rc 文件而不能等待进程退出。
    """
    inner = (
        f"sudo sh {shlex.quote(script)}; rc=$?; "
        f"echo $rc > {shlex.quote(rc_path)}; "
        "echo; echo 'Press Enter to close this window...'; read dummy"
    )
    candidates = (
        ("x-terminal-emulator", ["-e", "sh", "-c", inner], False),
        ("gnome-terminal",      ["--", "sh", "-c", inner], True),
        ("konsole",             ["-e", "sh", "-c", inner], False),
        ("xfce4-terminal",      ["-x", "sh", "-c", inner], False),
        ("mate-terminal",       ["-x", "sh", "-c", inner], True),
        # lxterminal 的 -e 只接受单个命令字符串
        ("lxterminal",          ["-e", f"sh -c {shlex.quote(inner)}"], False),
        ("xterm",               ["-e", "sh", "-c", inner], False),
    )
    for exe, args, factory in candidates:
        path = shutil.which(exe)
        if path:
            return [path] + args, factory
    return None


def _run_linux_setup_in_terminal(argv: list, rc_path: str, factory: bool) -> tuple:
    """在弹出的终端窗口里执行安装脚本，以 rc 文件判定结果。

    xterm/konsole 等终端进程随窗口命令结束退出（窗口被直接关闭视为取消）；
    gnome-terminal/mate-terminal 立即返回，轮询 rc 文件（最长 10 分钟）。
    """
    try:
        os.unlink(rc_path)
    except OSError:
        pass
    try:
        proc = subprocess.Popen(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        return False, f"failed to open terminal for dongle setup: {e}"
    started = time.monotonic()
    while True:
        if os.path.isfile(rc_path):
            break
        rc = proc.poll()
        if rc is not None:
            if factory and time.monotonic() - started < 2:
                if rc != 0:
                    return False, f"failed to open terminal for dongle setup (exit {rc})"
                # 工厂模式立即正常返回，继续等 rc 文件
            elif not factory:
                return False, "dongle setup cancelled (terminal window closed)"
        if factory and time.monotonic() - started > 600:
            return False, "dongle setup timed out waiting for the terminal window"
        time.sleep(0.3)
    try:
        with open(rc_path) as f:
            code = int(f.read().strip() or "1")
    except Exception:
        code = 1
    try:
        os.unlink(rc_path)
    except OSError:
        pass
    if code == 0:
        return True, ""
    return False, f"dongle setup script failed (exit {code})"


def _setup_dongle_linux() -> tuple:
    """安装 udev 规则使 dongle 可被 libusb 打开，返回 (成功与否, 出错信息)。

    提权路径按环境选择：
    - 已是 root：直接运行（兼容无 sudo 的最小系统/容器）；
    - 有控制终端：sudo（密码提示走 /dev/tty，捕获输出不影响终端输入密码）；
    - 无终端（GUI 直启）且有显示服务器：弹终端模拟器窗口运行脚本（窗口内
      输 sudo 密码，结果经临时 rc 文件回传），找不到终端模拟器时退到
      pkexec 图形提权框；
    - 以上都不行：仍走 sudo，系统报错原样返回。
    """
    script = os.path.join(_TOOLS_DIR, "setup_dongle_udev.sh")
    if not os.path.isfile(script):
        return False, f"dongle setup script not found: {script}"
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        return _run_linux_setup(["sh", script])
    if _has_controlling_terminal():
        return _run_linux_setup(["sudo", "sh", script])
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        rc_path = os.path.join(
            tempfile.gettempdir(),
            f"sensor_dongle_setup_rc_{os.getuid()}_{os.getpid()}")
        term = _linux_terminal_argv(script, rc_path)
        if term is not None:
            argv, factory = term
            SdkLog.i(_TAG, f"Opening terminal window for dongle setup: {argv[0]}")
            return _run_linux_setup_in_terminal(argv, rc_path, factory)
        pkexec = shutil.which("pkexec")
        if pkexec is not None:
            return _run_linux_setup([pkexec, "sh", script])
    return _run_linux_setup(["sudo", "sh", script])


def _check_setup_dongle_windows() -> str:
    """Windows 侧 dongle 检查/安装流程（check_setup_dongle 的平台分支）。

    libusb 只能枚举总线上的设备——被禁用（problem code 22）或错误状态的
    dongle 根本不可见，所以用 PowerShell PnP 检测（check_dongle_status.ps1）
    交叉核对真实插着的数量：PnP 多于 libusb 可用数时，先提权恢复异常状态
    （启用/重启设备），再按需运行 WinUSB 换绑脚本（存在非 WinUSB 设备时），
    最后复查；恢复不成功的设备在返回值中以文字提示列出。
    """
    specs = detect_usb_dongle_specs()
    pnp = _windows_pnp_dongle_status()

    if pnp is not None and pnp["total"] > len(specs):
        # 插着的已知 dongle 多于 libusb 可用数：有设备被禁用/出错
        # （libusb 枚举不到）或驱动未换绑 WinUSB
        if any(d.get("problem") for d in pnp["devices"]):
            _recover_dongle_status_windows()
            specs = detect_usb_dongle_specs()
            pnp = _windows_pnp_dongle_status() or pnp
        if any((d.get("service") or "").lower() != "winusb" for d in pnp["devices"]):
            ok, detail = _setup_dongle_windows()
            if not ok and not specs:
                SdkLog.w(_TAG, detail)
                return f"Error: {detail}"
            specs = detect_usb_dongle_specs()
            pnp = _windows_pnp_dongle_status() or pnp
    elif not specs:
        # 一只可用 dongle 都没有：保持原行为，直接运行驱动安装脚本
        ok, detail = _setup_dongle_windows()
        if not ok:
            SdkLog.w(_TAG, detail)
            return f"Error: {detail}"
        specs = detect_usb_dongle_specs()
        pnp = _windows_pnp_dongle_status()

    if not specs:
        msg = ("driver installed but the dongle is still not accessible; "
               "replug the dongle and check again")
        SdkLog.w(_TAG, msg)
        return f"Error: {msg}"
    # 最终复查：仍有插着但不可用的 dongle → 附文字提示（"OK: N" 前缀不变）
    if pnp is not None and pnp["total"] > len(specs):
        unusable = []
        for d in pnp["devices"]:
            problem = d.get("problem")
            if problem:
                unusable.append("%s (device error, problem code %s)"
                                % (d.get("instance_id"), problem))
            elif (d.get("service") or "").lower() != "winusb":
                unusable.append("%s (driver not switched to WinUSB)"
                                % d.get("instance_id"))
        if unusable:
            msg = ("OK: %d\n%d of %d USB BLE dongle(s) unusable and could not "
                   "be recovered: %s"
                   % (len(specs), pnp["total"] - len(specs), pnp["total"],
                      "; ".join(unusable)))
            SdkLog.w(_TAG, msg)
            return msg
    return f"OK: {len(specs)}"


def check_setup_dongle() -> str:
    """检查 USB BLE dongle 是否可用，不可用时按平台调用安装脚本并重新检测。

    返回 "OK: N"（N 为检测到的可用 dongle 数量）表示至少一只 dongle 已装好
    驱动/权限、可被 libusb 打开（即 SDK 可切换 bumble 后端）；失败时返回
    "Error: ..."（含系统出错信息）。
    Windows 弹 UAC 提权窗口；Linux 经 sudo 提权——有控制终端时在终端输密码，
    无终端（GUI 直启）时弹终端模拟器窗口运行安装脚本（找不到终端退到
    pkexec 图形提权框）；调用会阻塞等待用户确认；macOS 免驱，仅做检测。

    Windows 额外经 PowerShell PnP 检测（sensor/tools/check_dongle_status.ps1）
    核对插着的 dongle 总数：被禁用/错误状态的 dongle libusb 枚举不到，PnP
    发现数量不符时先提权恢复（启用被禁用的设备、重启错误设备），再按需运行
    WinUSB 换绑脚本；恢复不成功的设备以 "OK: N\\n<文字提示>" 的形式附在
    返回值中（保持 "OK" 前缀契约不变）。
    """
    if sys.platform.startswith("win"):
        return _check_setup_dongle_windows()

    specs = detect_usb_dongle_specs()
    if specs:
        return f"OK: {len(specs)}"

    platform = sys.platform
    if platform == "darwin":
        msg = ("no usable USB BLE dongle detected (macOS is driverless; "
               "check the dongle is plugged in and not claimed by macOS)")
        SdkLog.w(_TAG, msg)
        return f"Error: {msg}"
    if platform.startswith("linux"):
        if not _known_dongle_plugged():
            msg = ("no known USB BLE dongle plugged in (%s)"
                   % ", ".join("%04x:%04x" % i for i in sorted(_KNOWN_DONGLE_IDS)))
            SdkLog.w(_TAG, msg)
            return f"Error: {msg}"
        ok, detail = _setup_dongle_linux()
        if not ok:
            SdkLog.w(_TAG, detail)
            return f"Error: {detail}"
        specs = detect_usb_dongle_specs()
        if specs:
            return f"OK: {len(specs)}"
        msg = ("udev rules installed but the dongle is still not accessible; "
               "replug the dongle and check again")
        SdkLog.w(_TAG, msg)
        return f"Error: {msg}"
    msg = f"check_setup_dongle not supported on {platform}"
    SdkLog.w(_TAG, msg)
    return f"Error: {msg}"


class BumbleBackend:
    """bleak_bumble 后端配置：持有多只 dongle 的传输 spec，并按需分配/释放。

    每只 dongle 同一时刻只能服务一个连接（或一次扫描）：
    allocate() 领取空闲 dongle 的 spec，release() 归还。
    热插拔监控通过 reconcile() 增删池内 spec（线程安全）。
    """

    def __init__(self, transport_specs):
        # bumble 传输 spec 列表，如 ["usb:1-1.3.2", "usb:2-1.2"]（拓扑形式）
        self.transport_specs = list(transport_specs)
        self._busy = set()      # 已分配且物理上仍在的 spec
        self._departed = set()  # 已分配但已拔出的 spec（release 时清理）
        self._lock = threading.Lock()
        # per-dongle 事件循环：spec -> (loop, thread)，见 get_loop
        self._loops = {}
        self._loop_threads = {}

    def get_loop(self, spec: str):
        """返回 spec 专属事件循环（懒创建，daemon 线程）；spec 不在池中返回 None。

        per-dongle loop：该 dongle 的传输/连接/扫描借用全部在此 loop 上运行，
        各 dongle 的 HCI/ATT 处理互不排队（一台设备 connect 的服务发现+MTU
        不再阻塞其他设备的命令与通知分发）；传输打开时绑定本 loop，终身不变，
        扫描↔连接交接无需重开传输。loop 创建后随进程存续：spec 拔出后不停止
        （避免与仍在收尾的传输 close/收包投递竞争），空闲 daemon 线程成本
        可忽略，同口复插直接复用。
        """
        with self._lock:
            if spec not in self.transport_specs:
                return None
            loop = self._loops.get(spec)
            if loop is None or loop.is_closed():
                from sensor import sensor_utils

                loop = asyncio.new_event_loop()
                thread = threading.Thread(
                    target=sensor_utils.start_loop, args=(loop,)
                )
                thread.daemon = True
                thread.name = "dongle_event_" + spec.replace(":", "_").replace("@", "_")
                thread.start()
                self._loops[spec] = loop
                self._loop_threads[spec] = thread
            return loop

    def allocate(self, preferred: Optional[str] = None) -> Optional[str]:
        """领取一只空闲 dongle 的 spec；无空闲时返回 None。

        preferred 非空且该 spec 空闲时优先领取它（设备断连/热插拔后优先用
        原 dongle 的偏好）；preferred 占用/已拔出/不存在时立即退回第一只
        空闲 dongle——偏好只是倾向，绝不为偏好等待。
        """
        with self._lock:
            if (preferred is not None and preferred in self.transport_specs
                    and preferred not in self._busy):
                self._busy.add(preferred)
                return preferred
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

    def has_free(self) -> bool:
        """是否有空闲可分配的 dongle。"""
        with self._lock:
            return any(s not in self._busy for s in self.transport_specs)

    def free_count(self) -> int:
        """空闲可分配的 dongle 数量。"""
        with self._lock:
            return sum(1 for s in self.transport_specs if s not in self._busy)

    def reconcile(self, new_specs: list) -> list:
        """按最新 USB 枚举对账 dongle 池（热插拔监控驱动）。

        spec 即物理槽位（"usb:<bus>-<port.path>"，见 detect_usb_dongle_specs），
        不存在重命名歧义（旧 "#index" 编号时代的型号分组/冻结逻辑已删除）：
        - busy spec 仍在枚举中：保留；
        - busy spec 消失：无歧义拔出，转入 departed 并返回，由调用方主动断开
          受影响连接；
        - 空闲 spec 消失：直接出池；
        - 新出现的 spec：直接入池，立即可分配。

        返回本次拔出的占用中 spec 列表。
        """
        with self._lock:
            new_set = set(new_specs)
            departed_busy = []
            keep = []
            for s in self.transport_specs:
                if s in new_set:
                    keep.append(s)
                elif s in self._busy:
                    self._busy.discard(s)
                    self._departed.add(s)
                    departed_busy.append(s)
                # 空闲且已消失：直接出池
            for s in new_specs:
                if s not in keep:
                    keep.append(s)

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

    1. Actions 10d7 全系列 dongle（如 b012 ATS2851、b008 ATS2831）在 local
       supported commands 掩码中声称支持 HCI_LE_EXTENDED_CREATE_CONNECTION，
       实际下发却返回 UNKNOWN_HCI_COMMAND，导致连接失败。对该厂商的所有
       型号（按 transport 的 VID 识别），reset 后清除该位，回退 legacy 连接
       命令；其余厂商型号保持原生行为。
    2. 某些 dongle（如 33fa:0012）的 HCI_READ_LOCAL_EXTENDED_FEATURES 应答畸形
       （page 0 长度字段少报 1 字节，残留字节污染后续事件解析；page>0 返回垃圾），
       导致 reset 挂起、扫描/连接链路静默失败。该命令仅承载经典 BR/EDR 特性
       （BLE 不需要），所有 page 一律由补丁合成空应答，不下发真实命令。
    """
    from bumble import hci
    from bumble.host import Host

    if getattr(Host, "_sensor_sdk_reset_patched", False):
        return

    # 虚报 HCI_LE_EXTENDED_CREATE_CONNECTION 支持的固件厂商（实际下发返回错误）：
    # Actions 10d7 全系列
    _CLEAR_EXT_CREATE_VIDS = {0x10D7}

    orig_reset = Host.reset
    clear_mask = hci.HCI_SUPPORTED_COMMANDS_MASKS.get(
        hci.HCI_LE_EXTENDED_CREATE_CONNECTION_COMMAND, 0
    )

    async def patched_reset(self, *args, **kwargs):
        await orig_reset(self, *args, **kwargs)
        # transport 元数据中的 VID:PID（USB 来源），用于按厂商施加规避
        meta = getattr(self, "hci_metadata", None) or {}
        if (clear_mask and meta.get("vendor_id") in _CLEAR_EXT_CREATE_VIDS
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


def _address_to_log_mac(address) -> Optional[str]:
    """bumble 设备地址（可能带 /P、/R 类型后缀）规整为 colon 大写 MAC，

    用于 SdkLog 的 profile 路由（不匹配时自然回落公共通道）。"""
    if not address:
        return None
    mac = str(address).split("/")[0].strip().upper()
    return mac if len(mac) == 17 else None


def _format_conn_handle(handle) -> str:
    """连接句柄显示为 0xXXXX；缺失时显示 ?。"""
    return f"0x{handle:04X}" if isinstance(handle, int) else "?"


def _patch_bumble_client_connect() -> None:
    """给 BleakClientBumble 的连接/断连路径打补丁：MTU 交换 + 全程日志。

    功能补丁：
    1. connect 成功后交换 ATT MTU：bleak_bumble 连接后不交换 MTU，ATT_MTU
       保持默认 23，较长的响应（如固件版本号）会被截断/无法送达导致 init
       超时；CoreBluetooth 默认 185。交换目标由 GForce 按芯片类型在
       connect 前经 `_sdk_att_mtu` 下发：RFSTAR(BLE 5.3)尝试 511，
       其余设备 247（对端可在协商中降低），失败不阻断连接。
    2. 首个 GATT 请求（服务发现）前加短延迟：部分固件连接后 ATT 注册较慢，
       过早的 ATT 请求会被直接丢弃（不应答），导致服务发现超时。

    日志补丁（定位「HCI 链路建立后 ATT 阶段卡死、外设随后掉链」类问题）：
    - on_connection：HCI 链路建立即记录（地址/spec/句柄），并记下链路建立
      时间戳供断连日志计算存活时长；
    - get_services：服务发现开始/完成（耗时、服务数）；
    - connect：开始（地址/spec）与完成（总耗时、ATT MTU、对端名、协商到的
      连接参数 间隔/延迟/超时）；
    - MTU 交换：开始/结果（协商后 MTU、耗时）；
    - on_disconnection：断连原因码（含 HCI 错误名）、句柄、链路存活时长——
      vendored client 原本把 reason 直接丢弃。
    """
    import asyncio as _asyncio

    from sensor.bleak_bumble.client import BleakClientBumble

    if getattr(BleakClientBumble, "_sensor_sdk_mtu_patched", False):
        return

    orig_connect = BleakClientBumble.connect
    orig_on_connection = BleakClientBumble.on_connection
    orig_on_disconnection = BleakClientBumble.on_disconnection

    def patched_on_connection(self, connection):
        orig_on_connection(self, connection)
        try:
            mac = _address_to_log_mac(getattr(self, "address", None))
            handle = getattr(connection, "handle", None)
            self._sdk_link_handle = handle
            self._sdk_link_up_ts = time.monotonic()
            SdkLog.i(_TAG, f"HCI link up: {self.address} via {self._cfg}, "
                           f"handle={_format_conn_handle(handle)}", mac=mac)
        except Exception:
            pass

    def patched_on_disconnection(self, reason):
        try:
            mac = _address_to_log_mac(getattr(self, "address", None))
            handle = getattr(self, "_sdk_link_handle", None)
            up_ts = getattr(self, "_sdk_link_up_ts", None)
            lived = (f", link lived {time.monotonic() - up_ts:.1f}s"
                     if up_ts is not None else "")
            try:
                from bumble.hci import HCI_Constant
                reason_text = f"0x{reason:02X} ({HCI_Constant.error_name(reason)})"
            except Exception:
                reason_text = str(reason)
            SdkLog.w(_TAG, f"HCI link down: {self.address} via {self._cfg}, "
                           f"handle={_format_conn_handle(handle)}, "
                           f"reason={reason_text}{lived}", mac=mac)
        except Exception:
            pass
        orig_on_disconnection(self, reason)

    async def patched_connect(self, *args, **kwargs):
        mac = _address_to_log_mac(getattr(self, "address", None))
        connect_t0 = time.monotonic()
        SdkLog.i(_TAG, f"bumble connect start: {self.address} via {self._cfg}", mac=mac)
        try:
            await orig_connect(self, *args, **kwargs)
        except Exception as e:
            SdkLog.w(_TAG, f"bumble connect failed: {self.address} via {self._cfg}, "
                           f"{type(e).__name__}: {e}, "
                           f"{(time.monotonic() - connect_t0) * 1000:.0f}ms", mac=mac)
            raise
        connection = getattr(self, "_connection", None)
        params = getattr(connection, "parameters", None)
        if all(hasattr(params, a) for a in ("connection_interval", "peripheral_latency", "supervision_timeout")):
            conn_params_text = (f", conn_params={params.connection_interval}ms/"
                                f"latency {params.peripheral_latency}/"
                                f"timeout {params.supervision_timeout}ms")
        else:
            conn_params_text = ""
        SdkLog.i(_TAG, f"bumble connect done: {self.address} via {self._cfg}, "
                       f"handle={_format_conn_handle(getattr(self, '_sdk_link_handle', None))}, "
                       f"att_mtu={getattr(connection, 'att_mtu', '?')}, "
                       f"peer_name={getattr(self, '_name', '')!r}{conn_params_text}, "
                       f"{(time.monotonic() - connect_t0) * 1000:.0f}ms", mac=mac)
        try:
            # 标记连接传输的属主事件循环/线程（传输在 connect 内创建于本
            # loop）：patched_close 据此输出 cross_loop 诊断，证实异常清理
            # 在 device loop 上 close 属主为 gforce loop 的传输这一假设
            from sensor.bleak_bumble import transports as _bb_transports
            transport = _bb_transports.get(str(self._cfg))
            if transport is not None and getattr(transport, "_sdk_owner_loop", None) is None:
                transport._sdk_owner_loop = _asyncio.get_running_loop()
                transport._sdk_owner_thread = threading.current_thread().name
        except Exception:
            pass
        try:
            peer = getattr(self, "_peer", None)
            connection = getattr(self, "_connection", None)
            # 期望 MTU 由 GForce 按芯片类型在 connect 前下发（_sdk_att_mtu）：
            # RFSTAR 尝试 511（ATT 单次交换定终身，必须一次到位），默认 247
            desired_mtu = getattr(self, "_sdk_att_mtu", 247)
            if (peer is not None and connection is not None
                    and connection.att_mtu < desired_mtu):
                SdkLog.d(_TAG, f"ATT MTU exchange start: {self.address} "
                               f"via {self._cfg}, current={connection.att_mtu}, "
                               f"target={desired_mtu}", mac=mac)
                mtu_t0 = time.monotonic()
                await peer.request_mtu(desired_mtu)
                SdkLog.i(_TAG, f"ATT MTU exchanged: {self.address} via {self._cfg}, "
                               f"mtu={connection.att_mtu}, "
                               f"{(time.monotonic() - mtu_t0) * 1000:.0f}ms", mac=mac)
        except Exception as e:
            SdkLog.w(_TAG, f"bumble MTU exchange failed: {self.address} "
                           f"via {self._cfg}: {e}", mac=mac)

    BleakClientBumble.connect = patched_connect
    BleakClientBumble.on_connection = patched_on_connection
    BleakClientBumble.on_disconnection = patched_on_disconnection

    orig_get_services = BleakClientBumble.get_services

    async def patched_get_services(self, *args, **kwargs):
        mac = _address_to_log_mac(getattr(self, "address", None))
        await _asyncio.sleep(0.4)
        SdkLog.d(_TAG, f"GATT service discovery start: {self.address} "
                       f"via {self._cfg}", mac=mac)
        discovery_t0 = time.monotonic()
        services = await orig_get_services(self, *args, **kwargs)
        service_count = len(getattr(services, "services", None) or ())
        SdkLog.i(_TAG, f"GATT service discovery done: {self.address} "
                       f"via {self._cfg}, services={service_count}, "
                       f"{(time.monotonic() - discovery_t0) * 1000:.0f}ms", mac=mac)
        return services

    BleakClientBumble.get_services = patched_get_services

    BleakClientBumble._sensor_sdk_mtu_patched = True


def _patch_bumble_usb_spec_resolution() -> None:
    """打开 USB transport 前把 "vid:pid#index" spec 翻译成拓扑形式，并记录解析日志。

    bumble.transport._open_transport 每次调用时在函数体内局部 import
    open_usb_transport，替换 bumble.transport.usb 模块属性即生效。
    翻译见 _translate_usb_spec_to_topology；打开失败时补记原始 spec 与
    解析结果（bumble 原生日志不含设备身份，无法定位是哪只 dongle 失败）。
    """
    import bumble.transport.usb as _usb_transport_mod

    if getattr(_usb_transport_mod, "_sensor_sdk_spec_patched", False):
        return
    orig_open = _usb_transport_mod.open_usb_transport

    async def patched_open_usb_transport(spec: str):
        translated = _translate_usb_spec_to_topology(spec)
        if translated != spec:
            SdkLog.d(_TAG, f"open USB transport: usb:{spec} -> usb:{translated}")
        try:
            return await orig_open(translated)
        except Exception as e:
            SdkLog.w(_TAG, f"open USB transport failed: usb:{spec} "
                           f"(resolved usb:{translated}): {e}")
            raise

    _usb_transport_mod.open_usb_transport = patched_open_usb_transport
    _usb_transport_mod._sensor_sdk_spec_patched = True


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
        # 诊断日志：证实/证伪「terminate 卡死 + 跨事件循环 close」假设。
        # 属主 loop 由 patched_connect / patched_scanner_start 在传输
        # 创建后标记（_sdk_owner_loop）
        try:
            vid_pid = "%04x:%04x" % (self.device.getVendorID(), self.device.getProductID())
        except Exception:
            vid_pid = "?"
        cur_loop = _asyncio.get_running_loop()
        owner_loop = getattr(self, "_sdk_owner_loop", None)
        SdkLog.d(_TAG, f"bumble USB close start: usb:{vid_pid}, "
                       f"cross_loop={owner_loop is not None and owner_loop is not cur_loop}, "
                       f"loop={id(cur_loop)}, "
                       f"owner_loop={id(owner_loop) if owner_loop is not None else None}, "
                       f"owner_thread={getattr(self, '_sdk_owner_thread', None)}, "
                       f"thread={threading.current_thread().name}")
        t0 = time.monotonic()
        self.source.close()
        self.sink.close()
        for idx, endpoint in enumerate((self.source, self.sink)):
            ts = time.monotonic()
            try:
                # terminate 可能无界等待传输取消完成（远端已断电、跨事件循环
                # 调用时实测卡死 27s+），加 1s 上限；超时必须继续走到下面的
                # releaseInterface/device.close/context.close()——若整个 close
                # 被外部取消，接口占用永不释放，之后打开该 dongle 永远
                # LIBUSB_ERROR_ACCESS（自动重连全灭）
                await _asyncio.wait_for(endpoint.terminate(), timeout=1.0)
                SdkLog.d(_TAG, f"bumble USB close: endpoint[{idx}] terminate done in {time.monotonic() - ts:.3f}s")
            except _asyncio.TimeoutError:
                SdkLog.w(_TAG, f"bumble USB close: endpoint[{idx}] terminate timeout (1s), continuing close")
            except Exception as e:
                # 跨事件循环调用时 terminate 可能立即失败，忽略并继续
                SdkLog.d(_TAG, f"bumble USB close: endpoint[{idx}] terminate failed fast: {e}")

        # 请求事件线程退出，并 join 等待线程函数真正返回；
        # 之后再销毁 libusb 上下文，不再与 handleEvents 竞争
        with self.lock:
            self.event_loop_should_exit = True
        try:
            self.context.interruptEventHandler()
        except Exception:
            pass
        try:
            await cur_loop.run_in_executor(
                None, self.event_thread.join, 2.0
            )
        except Exception:
            pass
        if self.event_thread.is_alive():
            SdkLog.w(_TAG, "bumble USB close: event thread still alive after 2s join")

        # 事件线程已退出，可以安全释放 libusb 资源
        try:
            self.device.releaseInterface(self.acl_interface.getNumber())
            if self.sco_interface:
                self.device.releaseInterface(self.sco_interface.getNumber())
            self.device.close()
            self.context.close()
            SdkLog.d(_TAG, f"bumble USB close done: usb:{vid_pid}, total {time.monotonic() - t0:.3f}s")
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
    from sensor.bleak_bumble import transports
    from sensor.bleak_bumble.client import BleakClientBumble
    from sensor.bleak_bumble.scanner import BleakScannerBumble

    if getattr(BleakScannerBumble, "_sensor_sdk_reuse_patched", False):
        return

    # scanner.start：传输在 start_transport 内于本 loop 打开/复用，标记属主
    # loop/线程（与 patched_connect 对连接传输的标记一致）——per-dongle loop
    # 下传输终身绑在其 spec 的专属 loop 上，该标记供 patched_close 的
    # cross_loop 诊断与排查使用
    orig_scanner_start = BleakScannerBumble.start

    async def patched_scanner_start(self):
        await orig_scanner_start(self)
        try:
            transport = transports.get(str(self._cfg))
            if transport is not None and getattr(transport, "_sdk_owner_loop", None) is None:
                transport._sdk_owner_loop = asyncio.get_running_loop()
                transport._sdk_owner_thread = threading.current_thread().name
        except Exception:
            pass

    BleakScannerBumble.start = patched_scanner_start

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


class SyncWriteGate:
    """多设备同步起/停流的写命令门闩（跨事件循环/线程）。

    各参与设备的起/停流写命令（OYM 的 start_notify/stop_notify CCCD 写、
    RFSTAR 的 set_subscription CMD 写）在 bumble 后端发送前调用
    arrive_and_wait() 等待同一个 threading.Event：最后一个到达者放行。
    等待经 run_in_executor 下放到线程，不阻塞任何事件循环——per-dongle
    loop 后各写命令在各自 dongle 的 loop 上真正并发下发。
    """

    def __init__(self, parties, timeout=5.0):
        self._parties = parties
        self._timeout = timeout
        self._ready = 0
        self._lock = threading.Lock()
        self._go = threading.Event()

    @property
    def parties(self):
        return self._parties

    async def arrive_and_wait(self):
        """到达并等待放行；超时（有设备在写前失败）仅告警放行，不拖死其他设备。"""
        with self._lock:
            self._ready += 1
            if self._ready >= self._parties:
                self._go.set()
        released = await asyncio.get_running_loop().run_in_executor(
            None, self._go.wait, self._timeout
        )
        if not released:
            SdkLog.w(
                _TAG,
                f"SyncWriteGate: released on timeout "
                f"({self._ready}/{self._parties} arrived)",
            )


def arm_sync_write_gate(bleak_client, gate, expect=None) -> None:
    """把同步门闩挂到 bleak 客户端的 bumble 后端实例上（下一次匹配写消费一次）。

    GForce.client 是 bleak 门面 BleakClient，写路径补丁作用在 bumble 后端
    （BleakClientBumble）上，故属性须挂到 client._backend。

    ``expect`` 声明本次门闩期望的写身份，写包装校验匹配才消费门闩、才更新
    发送打点；不匹配的写（如电量轮询）在门闩挂起期间直通且不打点，避免
    抢门闩/污染起流时刻：
    - ("write", cmd)：CMD 特征写，data[0] 须等于 cmd（如 SET_DATA_NOTIF_SWITCH）
    - ("start_notify",)：OYM CCCD 起流写
    - ("stop_notify",)：OYM CCCD 停流写
    """
    backend = getattr(bleak_client, "_backend", bleak_client)
    backend._sync_write_gate = (gate, expect)


def _sync_gate_match(expect, kind: str, data) -> bool:
    """写身份与门闩期望的匹配判定；未声明期望（None）时视为匹配（兼容直通透传）。"""
    if expect is None:
        return True
    if expect[0] != kind:
        return False
    if kind == "write":
        return bool(data) and data[0] == int(expect[1])
    return True


def _backend_dongle_loop(backend):
    """从 bumble 后端实例取 dongle loop（USB transport sink 绑定的 loop，
    发送队列 process_queue 所在）；取不到或已关闭返回 None。"""
    try:
        dev = getattr(backend, "_dev", None)
        host = getattr(dev, "host", None)
        sink = getattr(host, "hci_sink", None)
        loop = getattr(sink, "loop", None)
        if loop is not None and not loop.is_closed():
            return loop
    except Exception:
        pass
    return None


def _patch_bumble_sync_write_gate() -> None:
    """多设备同步起/停流：起/停流写命令在 bumble 发送前等待同一个 SyncWriteGate。

    包装 BleakClientBumble.write_gatt_char（覆盖 RFSTAR set_subscription
    的 CMD 特征写）与 start_notify/stop_notify（覆盖 OYM 起流/停流的
    CCCD 写）：若后端实例持有 _sync_write_gate（由 arm_sync_write_gate
    挂载，携带期望写身份），写身份匹配时取出并消费；不匹配的写（如电量
    轮询）直通且不更新发送打点，不会抢门闩或污染起流时刻。

    门闩等待与真正下发下沉到 dongle loop：匹配的写经
    run_coroutine_threadsafe 投到该连接的 dongle loop，在那里
    arrive_and_wait、打点、调原始写——放行后各设备在**自己的** dongle
    loop 上并行完成打点+入队（同线程 put_nowait 即时唤醒发送队列），
    消除共享 gforce loop 的续体串行与跨线程入队的唤醒延迟。取不到
    dongle loop 时退回在调用方 loop 等待放行。未挂载门闩的写零开销直通。
    """
    from sensor.bleak_bumble.client import BleakClientBumble

    if getattr(BleakClientBumble, "_sensor_sdk_sync_gate_patched", False):
        return

    def _stamp(self):
        # bumble 层发送打点（门闩放行后、真正下发前），供 bin 0x07 记录
        # 与起/停流时刻的 wall ms（32 位 startTimeStamp 取发送时刻而非写完成时刻）
        self._last_write_perf_ns = time.perf_counter_ns()
        self._last_write_wall_ms = int(time.time() * 1000)

    async def _gated(self, gate, orig, args, kwargs):
        """门闩写公共流程：优先在 dongle loop 上等待放行并下发。"""
        loop = _backend_dongle_loop(self)
        if loop is not None and loop is not asyncio.get_running_loop():
            async def gated_write():
                await gate.arrive_and_wait()
                _stamp(self)
                await orig(self, *args, **kwargs)

            return await asyncio.wrap_future(
                asyncio.run_coroutine_threadsafe(gated_write(), loop))
        # 取不到 dongle loop：退回当前 loop 等待放行
        await gate.arrive_and_wait()
        _stamp(self)
        return await orig(self, *args, **kwargs)

    orig_write_gatt_char = BleakClientBumble.write_gatt_char

    async def write_gatt_char(self, characteristic, data, response):
        armed = getattr(self, "_sync_write_gate", None)
        if armed is not None:
            if not _sync_gate_match(armed[1], "write", data):
                # 门闩挂起期间的不匹配写（如电量轮询）：直通且不打点，
                # 避免覆盖起流写的发送时刻
                return await orig_write_gatt_char(self, characteristic, data, response)
            self._sync_write_gate = None
            return await self._sensor_sdk_gated_write(
                armed[0], orig_write_gatt_char, (characteristic, data, response), {})
        _stamp(self)
        return await orig_write_gatt_char(self, characteristic, data, response)

    orig_start_notify = BleakClientBumble.start_notify

    async def start_notify(self, characteristic, callback, **kwargs):
        armed = getattr(self, "_sync_write_gate", None)
        if armed is not None:
            if not _sync_gate_match(armed[1], "start_notify", None):
                return await orig_start_notify(self, characteristic, callback, **kwargs)
            self._sync_write_gate = None
            return await self._sensor_sdk_gated_write(
                armed[0], orig_start_notify, (characteristic, callback), kwargs)
        _stamp(self)
        return await orig_start_notify(self, characteristic, callback, **kwargs)

    orig_stop_notify = BleakClientBumble.stop_notify

    async def stop_notify(self, characteristic, **kwargs):
        armed = getattr(self, "_sync_write_gate", None)
        if armed is not None:
            if not _sync_gate_match(armed[1], "stop_notify", None):
                return await orig_stop_notify(self, characteristic, **kwargs)
            self._sync_write_gate = None
            return await self._sensor_sdk_gated_write(
                armed[0], orig_stop_notify, (characteristic,), kwargs)
        _stamp(self)
        return await orig_stop_notify(self, characteristic, **kwargs)

    BleakClientBumble._sensor_sdk_gated_write = _gated
    BleakClientBumble.write_gatt_char = write_gatt_char
    BleakClientBumble.start_notify = start_notify
    BleakClientBumble.stop_notify = stop_notify
    BleakClientBumble._sensor_sdk_sync_gate_patched = True


def _patch_bumble_precise_ts() -> None:
    """bumble 层接收打点：通知分发入口记录 time.perf_counter_ns()。

    包装 BleakClientBumble 的私有 __notify_handler（ATT 通知经 bumble
    subscribe 回调进入 bleak 封装层的第一个 SDK 可达点），在用户回调
    （gforce._on_data_response 等）执行前把接收时刻记到后端实例的
    _last_notify_perf_ns，供 gforce 写 bin 0x07 记录时读取。
    处理器同步调用用户回调，回调内读取不存在交错窗口。
    发送侧打点（_last_write_perf_ns）在 _patch_bumble_sync_write_gate
    的三个写包装内完成。
    """
    from sensor.bleak_bumble.client import BleakClientBumble

    if getattr(BleakClientBumble, "_sensor_sdk_precise_ts_patched", False):
        return

    orig_notify_handler = BleakClientBumble._BleakClientBumble__notify_handler

    def __notify_handler(self, characteristic, value):
        self._last_notify_perf_ns = time.perf_counter_ns()
        orig_notify_handler(self, characteristic, value)

    BleakClientBumble._BleakClientBumble__notify_handler = __notify_handler
    BleakClientBumble._sensor_sdk_precise_ts_patched = True


def _patch_bumble_gatt_no_subscriber_log() -> None:
    """给 bumble gatt_client 的无订阅者警告补上对端地址。

    bumble 原日志 '!!! received notification/indication with no subscriber'
    不含设备地址，无法按 MAC 路由到对应 profile log，多设备时无法分辨
    来自哪台设备。补丁后的实现与 bumble 0.0.233 原逻辑一致，仅警告文本
    附加 peer_address 与 attribute handle，使 SdkLog.route_external 能命中
    已注册 profile 的 MAC。
    """
    from bumble.gatt_client import Client, logger

    if getattr(Client, "_sensor_sdk_gatt_log_patched", False):
        return

    def on_att_handle_value_notification(self, notification):
        subscribers = self.notification_subscribers.get(
            notification.attribute_handle, set()
        )
        if not subscribers:
            logger.warning(
                f"!!! received notification with no subscriber "
                f"(peer={self.connection.peer_address}, "
                f"handle=0x{notification.attribute_handle:04X})"
            )
        self.cache_value(notification.attribute_handle, notification.attribute_value)
        for subscriber in subscribers:
            if callable(subscriber):
                subscriber(notification.attribute_value)
            else:
                subscriber.emit(subscriber.EVENT_UPDATE, notification.attribute_value)

    def on_att_handle_value_indication(self, indication):
        subscribers = self.indication_subscribers.get(
            indication.attribute_handle, set()
        )
        if not subscribers:
            logger.warning(
                f"!!! received indication with no subscriber "
                f"(peer={self.connection.peer_address}, "
                f"handle=0x{indication.attribute_handle:04X})"
            )
        self.cache_value(indication.attribute_handle, indication.attribute_value)
        for subscriber in subscribers:
            if callable(subscriber):
                subscriber(indication.attribute_value)
            else:
                subscriber.emit(subscriber.EVENT_UPDATE, indication.attribute_value)

    Client.on_att_handle_value_notification = on_att_handle_value_notification
    Client.on_att_handle_value_indication = on_att_handle_value_indication
    Client._sensor_sdk_gatt_log_patched = True


def resolve_bumble_backend() -> Optional[BumbleBackend]:
    """按环境变量与硬件检测决定是否启用 bleak_bumble 后端。

    返回 BumbleBackend 实例；不启用或条件不满足（缺依赖/无 dongle）时返回 None。
    """
    backend_env = (os.environ.get(_BACKEND_ENV) or "").strip().lower()
    if backend_env == "bleak":
        return None
    if backend_env and backend_env != "bumble":
        SdkLog.w(_TAG, f"Unknown {_BACKEND_ENV}={backend_env}, fallback to auto")

    # 自动模式全平台启用：detect_usb_dongle_specs 枚举时要求 libusb 能实际
    # 打开设备——Windows 上只有绑了 WinUSB（如经 setup_dongle_winusb.ps1 安装）
    # 的 dongle 才满足，Linux 上需要 udev 权限（setup_dongle_udev.sh），
    # 未装驱动的内置蓝牙（BTHUSB / 无权限）天然被过滤，不会误切换。
    # macOS 上内置蓝牙不走 USB，检测到的 USB HCI 设备必为外插 dongle。
    forced = backend_env == "bumble"

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


_SCAN_LOG_KEYWORDS = ("scan", "advertis", "discover")


def _record_scan_related(record) -> bool:
    """判断 logging 记录是否与扫描相关（按 logger 名与消息文本关键字过滤）。"""
    try:
        text = f"{record.name} {record.getMessage()}".lower()
    except Exception:
        return False
    return any(k in text for k in _SCAN_LOG_KEYWORDS)


def _patch_bumble_logging() -> None:
    """把 bumble 各模块的 logging 输出转发进 SdkLog。

    bumble 后端激活时调用一次：bumble 的日志（HCI 命令失败、连接事件等）
    按级别映射到 SdkLog，经启动缓存/异步队列写入 SDK 日志文件（文件日志
    未启用时先缓存，启用后回放）；消息文本命中已注册 profile 的 MAC 的记录
    （与该 profile 当前连接相关）路由到该 profile log，扫描相关记录固定写入
    controller log（扫描日志常含设备地址，不能按 MAC 路由）；bumble 自身的
    控制台输出保持不变。
    """

    class _BumbleToSdkHandler(logging.Handler):
        _is_bumble_to_sdk_handler = True

        def emit(self, record: logging.LogRecord) -> None:
            try:
                msg = self.format(record)  # 默认格式：消息文本（含异常堆栈）
                tag = record.name
                # 扫描相关记录固定进公共通道（controller log）
                if _record_scan_related(record):
                    SdkLog.controller(tag, msg)
                    return
                if record.levelno >= logging.ERROR:
                    level = logging.ERROR
                elif record.levelno >= logging.WARNING:
                    level = logging.WARNING
                elif record.levelno >= logging.INFO:
                    level = logging.INFO
                else:
                    level = logging.DEBUG
                # 命中已注册 profile 的 MAC -> 该 profile log，否则公共通道
                SdkLog.route_external(tag, msg, level)
            except Exception:
                pass

    bumble_logger = logging.getLogger("bumble")
    if not any(getattr(h, "_is_bumble_to_sdk_handler", False) for h in bumble_logger.handlers):
        bumble_logger.addHandler(_BumbleToSdkHandler())

    # vendored bleak_bumble 后端的 logger（sensor.bleak_bumble.*）不在 "bumble"
    # 命名空间下，单独接入转发：连接/断开/配对等后端日志同样按 MAC 路由
    bb_logger = logging.getLogger("sensor.bleak_bumble")
    if not any(getattr(h, "_is_bumble_to_sdk_handler", False) for h in bb_logger.handlers):
        bb_logger.addHandler(_BumbleToSdkHandler())


def patch_bleak_scan_logging() -> None:
    """把 bleak 的原始日志转发进 SdkLog（幂等）。

    扫描相关记录写入 controller log（原生 bleak 后端下扫描由 OS 蓝牙栈执行，
    bleak 的日志是唯一的原始扫描日志来源）；非扫描记录只在消息文本命中已
    注册 profile 的 MAC 时（与该 profile 当前连接相关）路由到该 profile log，
    其余不转发。子进程启动时调用一次，两种后端都适用。
    """

    class _BleakScanToSdkHandler(logging.Handler):
        _is_bleak_scan_sdk_handler = True

        def emit(self, record: logging.LogRecord) -> None:
            try:
                msg = self.format(record)
                if _record_scan_related(record):
                    SdkLog.controller(record.name, msg)
                    return
                # 非扫描记录：只转发命中已注册 profile MAC 的（连接相关）
                norm = msg.replace(":", "").replace("-", "").upper()
                for mac in SdkLog._profiles:
                    if SdkLog._norm_mac(mac) in norm:
                        SdkLog.route_external(record.name, msg, record.levelno)
                        return
            except Exception:
                pass

    bleak_logger = logging.getLogger("bleak")
    if not any(getattr(h, "_is_bleak_scan_sdk_handler", False) for h in bleak_logger.handlers):
        bleak_logger.addHandler(_BleakScanToSdkHandler())


def _apply_bumble_patches() -> None:
    """应用 dongle/bumble 兼容补丁（幂等；仅 bumble 后端激活时调用）。"""
    global _patches_applied
    if _patches_applied:
        return
    _patches_applied = True
    try:
        _patch_bumble_host_reset()
        _patch_bumble_client_connect()
        _patch_bumble_usb_spec_resolution()
        _patch_bumble_usb_close()
        _patch_bumble_transport_reuse()
        _patch_bumble_connection_params()
        _patch_bumble_gatt_no_subscriber_log()
        _patch_bumble_sync_write_gate()
        _patch_bumble_precise_ts()
        _patch_bumble_logging()
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
        new_specs, topo_map = detect_usb_dongle_specs(context=context, with_topology=True)
        if new_specs == self._last_specs:
            return
        self._last_specs = new_specs
        try:
            self._on_change(new_specs, topo_map)
        except Exception as e:
            SdkLog.w(_TAG, f"Dongle hotplug on_change failed: {e}")


def create_hotplug_monitor(on_change, initial_specs) -> Optional[DongleHotplugMonitor]:
    """按环境判定创建 dongle 热插拔监控（规则与 resolve_bumble_backend 一致）。

    SENSOR_SDK_BLE_BACKEND=bleak 或显式设置 SENSOR_SDK_BUMBLE_TRANSPORT 时不监控；
    自动模式全平台启用。无论当前是否已启用 bumble 后端都会监控
    （启动时无 dongle、之后插入时需要切换到 bumble 后端）。
    """
    backend_env = (os.environ.get(_BACKEND_ENV) or "").strip().lower()
    if backend_env == "bleak":
        return None
    if (os.environ.get(_TRANSPORT_ENV) or "").strip():
        return None
    try:
        import usb1  # noqa: F401
    except ImportError:
        return None
    return DongleHotplugMonitor(on_change, initial_specs)
