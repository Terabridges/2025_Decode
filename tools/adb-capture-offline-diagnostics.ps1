param(
    [string]$Endpoint = "192.168.43.1:5555",
    [int]$ServerTraceSeconds = 8
)

$ErrorActionPreference = "Stop"

$workspaceRoot = Split-Path -Parent $PSScriptRoot
$timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
$outDir = Join-Path $workspaceRoot "build\reports\adb-offline\$timestamp"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

$summaryPath = Join-Path $outDir "summary.txt"
$deviceListingPath = Join-Path $outDir "adb-devices-before.txt"
$hostTracePath = Join-Path $outDir "host-adb-trace.txt"
$hostActionsPath = Join-Path $outDir "host-adb-actions.txt"
$hubStatePath = Join-Path $outDir "hub-state.txt"
$hubLogcatPath = Join-Path $outDir "hub-logcat.txt"

"Capture start: $(Get-Date -Format o)" | Set-Content $summaryPath
"Endpoint: $Endpoint" | Add-Content $summaryPath

"=== adb devices -l (before) ===" | Set-Content $deviceListingPath
adb devices -l | Add-Content $deviceListingPath

try {
    "=== hub quick state ===" | Set-Content $hubStatePath
    "get-state:" | Add-Content $hubStatePath
    adb -s $Endpoint get-state | Add-Content $hubStatePath
    "init.svc.adbd:" | Add-Content $hubStatePath
    adb -s $Endpoint shell getprop init.svc.adbd | Add-Content $hubStatePath
    "service.adb.tcp.port:" | Add-Content $hubStatePath
    adb -s $Endpoint shell getprop service.adb.tcp.port | Add-Content $hubStatePath
    "persist.adb.tcp.port:" | Add-Content $hubStatePath
    adb -s $Endpoint shell getprop persist.adb.tcp.port | Add-Content $hubStatePath
    "ps | grep adbd:" | Add-Content $hubStatePath
    adb -s $Endpoint shell ps | findstr adbd | Add-Content $hubStatePath
}
catch {
    "Hub state capture failed: $($_.Exception.Message)" | Add-Content $summaryPath
}

try {
    adb -s $Endpoint logcat -b all -d -v threadtime > $hubLogcatPath
    "Hub logcat captured: $hubLogcatPath" | Add-Content $summaryPath
}
catch {
    "Hub logcat capture failed: $($_.Exception.Message)" | Add-Content $summaryPath
}

"=== Host ADB trace capture ===" | Set-Content $hostActionsPath
adb kill-server | Add-Content $hostActionsPath

$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = "adb"
$psi.Arguments = "nodaemon server"
$psi.UseShellExecute = $false
$psi.RedirectStandardOutput = $true
$psi.RedirectStandardError = $true
$psi.Environment["ADB_TRACE"] = "adb,sockets,transport,services"

$serverProc = New-Object System.Diagnostics.Process
$serverProc.StartInfo = $psi
[void]$serverProc.Start()

Start-Sleep -Milliseconds 500

try {
    "adb connect $Endpoint" | Add-Content $hostActionsPath
    adb connect $Endpoint | Add-Content $hostActionsPath
    "adb devices -l" | Add-Content $hostActionsPath
    adb devices -l | Add-Content $hostActionsPath
    "adb -s $Endpoint get-state" | Add-Content $hostActionsPath
    adb -s $Endpoint get-state | Add-Content $hostActionsPath
    Start-Sleep -Seconds $ServerTraceSeconds
}
finally {
    if (-not $serverProc.HasExited) {
        $serverProc.Kill()
        $serverProc.WaitForExit(2000) | Out-Null
    }

    $serverStdout = $serverProc.StandardOutput.ReadToEnd()
    $serverStderr = $serverProc.StandardError.ReadToEnd()
    "=== adb nodaemon server stdout ===" | Set-Content $hostTracePath
    $serverStdout | Add-Content $hostTracePath
    "`n=== adb nodaemon server stderr ===" | Add-Content $hostTracePath
    $serverStderr | Add-Content $hostTracePath
}

adb start-server | Out-Null

"Capture complete: $(Get-Date -Format o)" | Add-Content $summaryPath
"Output directory: $outDir" | Add-Content $summaryPath

Write-Host "✅ Offline diagnostics captured"
Write-Host "   Output: $outDir"