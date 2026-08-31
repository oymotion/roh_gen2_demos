#requires -Version 5.1
<#
.SYNOPSIS
    Restart (bus-reset) USB BLE dongles so a wedged Bluetooth controller
    re-enumerates. Requires Administrator privileges.

.DESCRIPTION
    Recovery helper for the sensor-sdk bumble host-mode backend. Use it when a
    dongle stops answering (repeated connect timeouts / half-dead link in the
    SDK log) instead of physically unplugging and replugging it:
    - Targets known dongles by default: Actions 10d7:b012/b008,
      Barrot 33fa:0001/0010/0012
      (extend with -HardwareId), or a specific -InstanceId.
    - Restarts each matched device with Restart-PnpDevice, falling back to
      Disable-PnpDevice + Enable-PnpDevice when unavailable.
    - The SDK hot-plug monitor sees the restart as unplug + replug: affected
      connections are dropped (auto-reconnect recovers) and scans resume
      automatically once the dongle re-enumerates.
    Note: do NOT trigger this on the routine UNKNOWN_HCI_COMMAND warnings
    (HCI_LE_READ_NUMBER_OF_SUPPORTED_ADVERTISING_SETS / READ_MAXIMUM_ADVERTISING
    _DATA_LENGTH) - those are normal for these firmwares and need no recovery.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File sensor\tools\restart_dongle.ps1
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File sensor\tools\restart_dongle.ps1 -InstanceId 'USB\VID_10D7&PID_B012\5&12345678&0&1'
#>
[CmdletBinding()]
param(
    # Restart only specific PnP instance(s) (wildcards allowed)
    [string[]]$InstanceId = @(),
    # Extra HardwareID fragments to match (e.g. VID_1234&PID_5678)
    [string[]]$HardwareId = @(),
    # Allow resetting a ROOT hub as a last resort (bounces every device on that
    # controller, including HID/storage). Off by default.
    [switch]$AllowRootHub
)

$ErrorActionPreference = 'Stop'

function Write-Info([string]$m) { Write-Host "[INFO] $m" }
function Write-Warn([string]$m) { Write-Host "[WARN] $m" -ForegroundColor Yellow }
function Write-Fail([string]$m) { Write-Host "[FAIL] $m" -ForegroundColor Red }

# ---- 0. Administrator check ----
$principal = [Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Write-Fail "Administrator privileges required; please run this script from an elevated (Administrator) PowerShell"
    exit 1
}

# ---- 1. Find targets ----
$targets = @()
if ($InstanceId.Count -gt 0) {
    foreach ($iid in $InstanceId) {
        $targets += @(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
            Where-Object { $_.InstanceId -like $iid })
    }
} else {
    $known = @('VID_10D7&PID_B012', 'VID_10D7&PID_B008',
        'VID_33FA&PID_0001', 'VID_33FA&PID_0010', 'VID_33FA&PID_0012') + $HardwareId
    foreach ($id in $known) {
        $targets += @(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
            Where-Object { $_.InstanceId -like "*$id*" })
    }
}
if ($targets.Count -eq 0) {
    Write-Fail 'No USB BLE dongle matched (8 known models by default; use -HardwareId or -InstanceId)'
    exit 2
}

# ---- 2. Restart each target ----
function Restart-ViaParentHub {
    param([Parameter(Mandatory = $true)]$Device)
    # Reset the parent USB hub to power-cycle the port. Needed on systems where
    # direct PnP restart of the (WinUSB-bound) dongle is denied or unsupported.
    $parent = (Get-PnpDeviceProperty -InstanceId $Device.InstanceId `
        -KeyName 'DEVPKEY_Device_Parent' -ErrorAction SilentlyContinue).Data
    if (-not $parent) { throw "no parent hub found for $($Device.InstanceId)" }
    if ($parent -like 'USB\ROOT_HUB*' -and -not $AllowRootHub) {
        throw "parent is a ROOT hub ($parent); restarting it bounces every device on that controller. Re-run with -AllowRootHub to accept."
    }
    Write-Info "resetting parent hub $parent"
    Disable-PnpDevice -InstanceId $parent -Confirm:$false -ErrorAction Stop
    Start-Sleep -Milliseconds 800
    Enable-PnpDevice -InstanceId $parent -Confirm:$false -ErrorAction Stop
    Start-Sleep -Seconds 2
    # the child may re-enumerate in the disabled state (code 22) after the bounce
    $now = Get-PnpDevice -InstanceId $Device.InstanceId -ErrorAction SilentlyContinue
    if ($now -and $now.Problem -eq 'CM_PROB_DISABLED') {
        Write-Info 'child came back disabled, re-enabling'
        Enable-PnpDevice -InstanceId $Device.InstanceId -Confirm:$false -ErrorAction Stop
        Start-Sleep -Milliseconds 500
    }
}

$failed = @()
foreach ($dev in $targets) {
    Write-Info ("Restarting: {0} | {1}" -f $dev.FriendlyName, $dev.InstanceId)
    $done = $false
    # Mechanism 1: pnputil /restart-device (inbox cfgmgr32 path)
    & "$env:SystemRoot\System32\pnputil.exe" /restart-device "$($dev.InstanceId)" | Out-Null
    if ($LASTEXITCODE -eq 0) {
        $done = $true
    } else {
        Write-Info "pnputil /restart-device failed (exit $LASTEXITCODE); trying Restart-PnpDevice"
        # Mechanism 2: Restart-PnpDevice cmdlet (not present on all PowerShell builds)
        try {
            Restart-PnpDevice -InstanceId $dev.InstanceId -Confirm:$false -ErrorAction Stop
            $done = $true
        } catch {
            Write-Info "Restart-PnpDevice unavailable or failed ($($_.Exception.Message)); falling back to disable+enable"
            # Mechanism 3: Disable-PnpDevice + Enable-PnpDevice (WMI path; can be
            # denied while the device is held open by a userspace driver)
            try {
                Disable-PnpDevice -InstanceId $dev.InstanceId -Confirm:$false -ErrorAction Stop
                Start-Sleep -Milliseconds 500
                Enable-PnpDevice -InstanceId $dev.InstanceId -Confirm:$false -ErrorAction Stop
                $done = $true
            } catch {
                Write-Info "direct restart failed ($($_.Exception.Message)); trying parent hub reset"
                # Mechanism 4: parent hub power-cycle
                try {
                    Restart-ViaParentHub -Device $dev
                    $done = $true
                } catch {
                    Write-Fail "restart failed: $($_.Exception.Message) ($($dev.InstanceId))"
                    $failed += $dev
                    continue
                }
            }
        }
    }
}
Start-Sleep -Seconds 1

# ---- 3. Verify the devices came back ----
foreach ($dev in $targets) {
    if ($failed -contains $dev) { continue }
    $now = Get-PnpDevice -InstanceId $dev.InstanceId -ErrorAction SilentlyContinue
    if ($now) {
        Write-Info "Back online: $($dev.InstanceId)"
    } else {
        Write-Fail "device did not come back: $($dev.InstanceId)"
        $failed += $dev
    }
}

Write-Host '============================================================'
if ($failed.Count -gt 0) {
    Write-Fail "$($failed.Count) device(s) failed to restart"
    exit 1
}
Write-Info 'Done. The SDK will re-enumerate the dongle(s) automatically.'
exit 0
