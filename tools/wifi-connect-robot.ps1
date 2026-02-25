param(
    [string]$Ssid = "18603-RC",
    [string]$Interface = "Wi-Fi"
)

$ErrorActionPreference = "Stop"

Write-Host "Connecting to Wi-Fi SSID '$Ssid' on interface '$Interface'..."
netsh wlan connect name="$Ssid" ssid="$Ssid" interface="$Interface" | Out-Host

Start-Sleep -Seconds 2

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
    Write-Host "✅ Connected to '$Ssid'."
    exit 0
}

if ($state -eq "connected" -and $currentSsid -ne "") {
    Write-Host "⚠️ Connected, but on '$currentSsid' (expected '$Ssid')."
    exit 1
}

Write-Host "❌ Not connected to Wi-Fi."
exit 1
