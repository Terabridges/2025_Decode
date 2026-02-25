param(
    [string]$Ssid = "18603-RC"
)

$ErrorActionPreference = "Stop"
$netshOutput = netsh wlan show interfaces

$state = ""
$currentSsid = ""

foreach ($line in $netshOutput) {
    if ($line -match '^\s*State\s*:\s*(.+)$') {
        $state = $Matches[1].Trim()
        continue
    }

    if ($line -match '^\s*SSID\s*:\s*(.+)$') {
        $currentSsid = $Matches[1].Trim()
        continue
    }
}

if ($state -eq "connected" -and $currentSsid -eq $Ssid) {
    Write-Host "✅ On robot Wi-Fi: '$Ssid'"
    exit 0
}

if ($state -eq "connected" -and $currentSsid -ne "") {
    Write-Host "ℹ️ Connected to '$currentSsid' (not robot SSID '$Ssid')."
    exit 1
}

Write-Host "❌ Wi-Fi disconnected."
exit 1
