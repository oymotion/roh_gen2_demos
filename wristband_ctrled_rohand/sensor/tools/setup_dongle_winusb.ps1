#requires -Version 5.1
<#
.SYNOPSIS
    Switch the USB Bluetooth dongle driver to WinUSB (libusb-accessible) so the
    sensor-sdk bumble host-mode backend can use it on Windows. Requires
    Administrator privileges.

.DESCRIPTION
    - Detects known dongles: Actions 10d7:b012/b008,
      Barrot 33fa:0001/0010/0012 (extend with -HardwareId)
    - Skips devices already on WinUSB; the rest (stock BthUSB or third-party
      drivers) are switched to WinUSB
    - On first run, creates a self-signed code-signing certificate installed into
      LocalMachine Root / TrustedPublisher, then generates a signed catalog for
      sensor_dongle_winusb.inf offline with New-FileCatalog +
      Set-AuthenticodeSignature (no Zadig / Windows SDK needed)
    - Installs the driver with pnputil /add-driver /install, then restarts the
      devices so the new binding takes effect
    Note: after switching, the dongle disappears from Windows Bluetooth settings
    (the SDK drives it directly over HCI - this is expected).
    To revert: pnputil /delete-driver sensor_dongle_winusb.inf /uninstall,
    then unplug and replug the dongle.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File sensor\tools\setup_dongle_winusb.ps1
#>
[CmdletBinding()]
param(
    # Extra HardwareID fragments to match (e.g. VID_1234&PID_5678) for dongle
    # models beyond the known ones
    [string[]]$HardwareId = @()
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

$InfPath = Join-Path $PSScriptRoot 'sensor_dongle_winusb.inf'
$CatPath = [System.IO.Path]::ChangeExtension($InfPath, '.cat')
$CertSubject = 'CN=Synchroni SDK Dongle Driver Signing'
if (-not (Test-Path $InfPath)) {
    Write-Fail "INF file not found: $InfPath"
    exit 1
}

# ---- 1. Detect dongles ----
$known = @('VID_10D7&PID_B012', 'VID_10D7&PID_B008',
    'VID_33FA&PID_0001', 'VID_33FA&PID_0010', 'VID_33FA&PID_0012') + $HardwareId
$dongles = @()
foreach ($id in $known) {
    $dongles += @(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
        Where-Object { $_.InstanceId -like "*$id*" })
}
if ($dongles.Count -eq 0) {
    Write-Fail 'No USB Bluetooth dongle found (matched 8 known models by default; use -HardwareId to add more)'
    exit 2
}

$needConvert = @()
foreach ($dev in $dongles) {
    $svc = if ($dev.Service) { $dev.Service } else { '(none)' }
    Write-Info ("Found dongle: {0} | {1} | current driver: {2}" -f $dev.FriendlyName, $dev.InstanceId, $svc)
    if ($dev.Service -ieq 'WinUSB') {
        Write-Info '  already WinUSB, skipping'
    } else {
        $needConvert += $dev
    }
}

if ($needConvert.Count -eq 0) {
    Write-Info 'All dongles are already on WinUSB, nothing to do'
    exit 0
}

# ---- 2. Signing certificate (created on first run, reused afterwards) ----
$cert = @(Get-ChildItem Cert:\LocalMachine\My | Where-Object { $_.Subject -eq $CertSubject })[0]
if ($cert) {
    # The cert may outlive its private key (the CNG key container under
    # ProgramData\Microsoft\Crypto\Keys gets removed by cleanup tools or
    # system maintenance). Signing then fails with "No provider was
    # specified for the store or object" — detect the unusable key and
    # recreate the cert from scratch in all three stores.
    $keyOk = $false
    try {
        $keyOk = [System.Security.Cryptography.X509Certificates.RSACertificateExtensions]::GetRSAPrivateKey($cert) -ne $null
    } catch {
        $keyOk = $false
    }
    if (-not $keyOk) {
        Write-Warn 'Existing signing certificate has a missing/unusable private key; recreating it'
        foreach ($storeName in 'My', 'Root', 'TrustedPublisher') {
            $store = New-Object System.Security.Cryptography.X509Certificates.X509Store($storeName, 'LocalMachine')
            $store.Open('ReadWrite')
            foreach ($old in @($store.Certificates | Where-Object { $_.Subject -eq $CertSubject })) {
                $store.Remove($old)
            }
            $store.Close()
        }
        $cert = $null
    }
}
if (-not $cert) {
    Write-Info "Creating self-signed code-signing certificate: $CertSubject"
    $cert = New-SelfSignedCertificate -Subject $CertSubject -Type CodeSigningCert `
        -CertStoreLocation Cert:\LocalMachine\My -KeyExportPolicy NonExportable `
        -HashAlgorithm SHA256 -KeyLength 2048 -NotAfter (Get-Date).AddYears(10)
    foreach ($storeName in 'Root', 'TrustedPublisher') {
        $store = New-Object System.Security.Cryptography.X509Certificates.X509Store($storeName, 'LocalMachine')
        $store.Open('ReadWrite')
        $store.Add($cert)
        $store.Close()
    }
    Write-Info 'Certificate installed to LocalMachine\Root and LocalMachine\TrustedPublisher'
}

# ---- 3. Generate the signed catalog and install the driver ----
if (Test-Path $CatPath) { Remove-Item $CatPath -Force }
Write-Info "Generating signed catalog: $CatPath"
New-FileCatalog -Path $InfPath -CatalogFilePath $CatPath -CatalogVersion 2.0 | Out-Null
$sig = Set-AuthenticodeSignature -FilePath $CatPath -Certificate $cert -HashAlgorithm SHA256
if ($sig.Status -ne 'Valid') {
    Write-Fail "catalog signing failed: $($sig.StatusMessage)"
    exit 1
}

Write-Info "Installing driver package: $InfPath"
& "$env:SystemRoot\System32\pnputil.exe" /add-driver "$InfPath" /install
if ($LASTEXITCODE -ne 0) {
    Write-Fail "pnputil install failed (exit $LASTEXITCODE)"
    exit 1
}

# ---- 4. Restart the devices so the new binding applies, then verify ----
foreach ($dev in $needConvert) {
    Write-Info "Restarting device to apply the new driver: $($dev.InstanceId)"
    try {
        Disable-PnpDevice -InstanceId $dev.InstanceId -Confirm:$false -ErrorAction Stop
        Start-Sleep -Milliseconds 500
        Enable-PnpDevice -InstanceId $dev.InstanceId -Confirm:$false -ErrorAction Stop
    } catch {
        Write-Warn "device restart failed (it may have been re-bound automatically): $($_.Exception.Message)"
    }
}
Start-Sleep -Seconds 1

$failed = @()
foreach ($dev in $needConvert) {
    $now = Get-PnpDevice -InstanceId $dev.InstanceId -ErrorAction SilentlyContinue
    $svc = if ($now -and $now.Service) { $now.Service } else { '(unknown)' }
    if ($now -and $now.Service -ieq 'WinUSB') {
        Write-Info "Switched to WinUSB: $($dev.InstanceId)"
    } else {
        Write-Fail "switch failed, current driver: $svc ($($dev.InstanceId))"
        $failed += $dev
    }
}

if ($failed.Count -gt 0) {
    # A device node with a stale configuration can ignore the driver update
    # and the disable/enable restart (SetupAPI then reports "device does not
    # need an update" forever). Removing the node forces a fresh enumeration
    # where the newly staged package wins the driver selection by rank.
    Write-Info 'Retrying failed device(s) with node re-enumeration'
    $removed = $false
    foreach ($dev in $failed) {
        Write-Info "Removing device node: $($dev.InstanceId)"
        & "$env:SystemRoot\System32\pnputil.exe" /remove-device $dev.InstanceId | Out-Null
        if ($LASTEXITCODE -eq 0) { $removed = $true }
    }
    if ($removed) {
        & "$env:SystemRoot\System32\pnputil.exe" /scan-devices | Out-Null
        if ($LASTEXITCODE -ne 0) {
            # older pnputil builds name the rescan verb /scan-hardware
            & "$env:SystemRoot\System32\pnputil.exe" /scan-hardware | Out-Null
        }
        Start-Sleep -Seconds 4
        $stillFailed = @()
        foreach ($dev in $failed) {
            $now = Get-PnpDevice -InstanceId $dev.InstanceId -ErrorAction SilentlyContinue
            if ($now -and $now.Service -ieq 'WinUSB') {
                Write-Info "Switched to WinUSB after re-enumeration: $($dev.InstanceId)"
            } else {
                $svc = if ($now -and $now.Service) { $now.Service } else { '(not present, replug the dongle)' }
                Write-Fail "switch failed, current driver: $svc ($($dev.InstanceId))"
                $stillFailed += $dev
            }
        }
        $failed = $stillFailed
    }
}

Write-Host '============================================================'
if ($failed.Count -gt 0) {
    Write-Fail "$($failed.Count) device(s) failed to switch"
    exit 1
}
Write-Info 'Done. Usage: set SENSOR_SDK_BLE_BACKEND=bumble, then run your program'
exit 0
