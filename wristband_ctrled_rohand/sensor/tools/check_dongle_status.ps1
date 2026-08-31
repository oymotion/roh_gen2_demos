#requires -Version 5.1
<#
.SYNOPSIS
    Check the PnP status of known USB Bluetooth dongles and optionally try to
    recover abnormal ones (e.g. devices disabled in Device Manager).

.DESCRIPTION
    libusb can only enumerate devices that are present on the USB bus: a dongle
    that is disabled in Device Manager (problem code 22) never shows up in USB
    enumeration, so the SDK's libusb-based detection reports fewer dongles than
    are actually plugged in. This script enumerates known dongles through PnP
    instead, which also sees disabled / error-state devices.

    - Detects known dongles: Actions 10d7:b012/b008,
      Barrot 33fa:0001/0010/0012 (extend with -HardwareId)
    - Reports every dongle's status / problem code / bound driver service
    - With -Recover (requires Administrator), tries to recover abnormal devices:
      problem code 22 (disabled) -> Enable-PnpDevice; any other problem code ->
      Disable-PnpDevice + Enable-PnpDevice restart. Devices that are still
      abnormal afterwards are reported as failures.
    - With -Json, prints only a machine-readable JSON object (text output is
      suppressed): {"total": N, "devices": [{instance_id, name, status,
      problem, service}, ...]}

    Exit codes: 0 = all detected dongles healthy; 1 = some dongles abnormal
    (still abnormal after the -Recover attempt); 2 = no known dongle found;
    3 = -Recover requires Administrator privileges.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File sensor\tools\check_dongle_status.ps1
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File sensor\tools\check_dongle_status.ps1 -Recover
#>
[CmdletBinding()]
param(
    # Try to recover abnormal devices (enable disabled ones, restart error ones)
    [switch]$Recover,
    # Print only the machine-readable JSON result (suppresses text output)
    [switch]$Json,
    # Extra HardwareID fragments to match (e.g. VID_1234&PID_5678) for dongle
    # models beyond the known ones
    [string[]]$HardwareId = @()
)

$ErrorActionPreference = 'Stop'

function Write-Info([string]$m) { if (-not $Json) { Write-Host "[INFO] $m" } }
function Write-Warn([string]$m) { if (-not $Json) { Write-Host "[WARN] $m" -ForegroundColor Yellow } }
function Write-Fail([string]$m) { if (-not $Json) { Write-Host "[FAIL] $m" -ForegroundColor Red } }

$problemText = @{
    0  = 'working'
    10 = 'device cannot start'
    22 = 'disabled'
    28 = 'drivers not installed'
    43 = 'device reported a failure'
    45 = 'not connected'
}

function Get-ProblemCode([string]$instanceId) {
    try {
        $p = Get-PnpDeviceProperty -InstanceId $instanceId -KeyName 'DEVPKEY_Device_ProblemCode' -ErrorAction Stop
        return [int]$p.Data
    } catch {
        return -1
    }
}

function Get-ProblemText([int]$code) {
    if ($problemText.ContainsKey($code)) { return $problemText[$code] }
    return 'unknown'
}

function Emit-Json([array]$results) {
    $payload = [PSCustomObject]@{ total = $results.Count; devices = @($results) }
    $payload | ConvertTo-Json -Depth 4 -Compress
}

# ---- 1. Detect known dongles through PnP (sees disabled devices too) ----
$known = @('VID_10D7&PID_B012', 'VID_10D7&PID_B008',
    'VID_33FA&PID_0001', 'VID_33FA&PID_0010', 'VID_33FA&PID_0012') + $HardwareId
$dongles = @()
foreach ($id in $known) {
    $dongles += @(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
        Where-Object { $_.InstanceId -like "*$id*" })
}
if ($dongles.Count -eq 0) {
    Write-Fail 'No USB Bluetooth dongle found (matched 8 known models by default; use -HardwareId to add more)'
    if ($Json) { Emit-Json @() }
    exit 2
}

# ---- 2. -Recover requires Administrator ----
if ($Recover) {
    $principal = [Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()
    if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        Write-Fail 'Administrator privileges required for -Recover; please run from an elevated (Administrator) PowerShell'
        exit 3
    }
}

# ---- 3. Check (and optionally recover) each dongle ----
$results = @()
$bad = 0
foreach ($dev in $dongles) {
    $problem = Get-ProblemCode $dev.InstanceId
    if ($problem -ne 0) {
        Write-Warn ("Abnormal dongle: {0} | {1} | problem code {2} ({3})" -f `
            $dev.FriendlyName, $dev.InstanceId, $problem, (Get-ProblemText $problem))
        if ($Recover) {
            try {
                if ($problem -eq 22) {
                    Write-Info '  enabling the disabled device'
                    Enable-PnpDevice -InstanceId $dev.InstanceId -Confirm:$false -ErrorAction Stop
                } else {
                    Write-Info '  restarting the device (disable + enable)'
                    Disable-PnpDevice -InstanceId $dev.InstanceId -Confirm:$false -ErrorAction Stop
                    Start-Sleep -Milliseconds 500
                    Enable-PnpDevice -InstanceId $dev.InstanceId -Confirm:$false -ErrorAction Stop
                }
                Start-Sleep -Milliseconds 500
            } catch {
                Write-Warn "  recovery action failed: $($_.Exception.Message)"
            }
            $problem = Get-ProblemCode $dev.InstanceId
            $refreshed = Get-PnpDevice -InstanceId $dev.InstanceId -ErrorAction SilentlyContinue
            if ($refreshed) { $dev = $refreshed }
        }
    }
    $svc = if ($dev.Service) { [string]$dev.Service } else { '' }
    if ($problem -eq 0) {
        Write-Info ("OK: {0} | {1} | driver: {2}" -f $dev.FriendlyName, $dev.InstanceId, $(if ($svc) { $svc } else { '(none)' }))
    } else {
        $bad++
        Write-Fail ("{0} | {1} | still abnormal, problem code {2} ({3})" -f `
            $dev.FriendlyName, $dev.InstanceId, $problem, (Get-ProblemText $problem))
    }
    $results += [PSCustomObject]@{
        instance_id = [string]$dev.InstanceId
        name        = [string]$dev.FriendlyName
        status      = [string]$dev.Status
        problem     = $problem
        service     = $svc
    }
}

# ---- 4. Summary ----
if ($Json) {
    Emit-Json $results
} else {
    Write-Host '============================================================'
    if ($bad -eq 0) {
        Write-Info "All $($results.Count) dongle(s) healthy"
    } else {
        Write-Fail "$bad of $($results.Count) dongle(s) abnormal"
    }
}
if ($bad -eq 0) { exit 0 }
exit 1
