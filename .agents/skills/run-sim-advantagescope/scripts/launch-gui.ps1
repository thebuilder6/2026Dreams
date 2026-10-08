# Tier 1 GUI launch: starts simulateJava, then auto-launches Elastic +
# AdvantageScope when found. Prints manual fallback otherwise.
# -RealDs loads halsim_ds_socket too (build.gradle -PrealDs flag) so a real
# Driver Station app can drive match phases; keep SimGUI for sticks/NT view.
param([switch]$RealDs)
$ErrorActionPreference = "Continue"
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
$GradleRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\..\..\..\TitanRoboticsBuildSeason"))
Set-Location $GradleRoot

Import-Module (Join-Path $GradleRoot "tools\lock\Lock.psm1") -Force

# A GUI sim owns NT4 5810, WebServer 5800 and CameraServer 1181-1182, all
# hardcoded WPILib constants. One sim repo-wide. See tools/score/sweep.ps1 for
# the port analysis and KNOWN_ISSUES.md section E for why a stray NT client is
# worse than a bind error.
#
# The lock is anchored on the sim window's own PID, not on this script. This
# script returns immediately while the sim keeps running, so anchoring on $PID
# would free the lock the moment the launcher exited.
$simHost = $null
try {
  $simFlags = ".\gradlew simulateJava --offline"
  if ($RealDs) { $simFlags += " -PrealDs" }
  $cmdStr = "cd '$GradleRoot'; `$env:JAVA_HOME = '$env:JAVA_HOME'; `$env:PATH = `"`$env:JAVA_HOME\bin;`$env:PATH`"; $simFlags"
  $simHost = Start-Process powershell -PassThru -WindowStyle Normal -ArgumentList "-NoExit", "-Command", $cmdStr
  $null = Enter-Lock -Resource sim-gui -AnchorPid $simHost.Id `
                     -Reason "SimGUI launched by $(Get-OwnerId)" -TimeoutSec 300
  Write-Output "simulateJava launched in a new window from $GradleRoot (host pid $($simHost.Id))."
  Write-Output "sim-gui lock held by pid $($simHost.Id). Release it when you close the sim:"
  Write-Output "  powershell -File tools/lock/release.ps1 -Resource sim-gui"
} catch {
  if ($simHost) { try { Stop-Process -Id $simHost.Id -Force -ErrorAction SilentlyContinue } catch { } }
  Write-Error $_.Exception.Message
  Write-Output "Sim not started. If a previous run is still holding sim-gui:"
  Write-Output "  powershell -File tools/lock/status.ps1"
  exit 3
}

function Find-App($names, $paths) {
  foreach ($n in $names) {
    $c = Get-Command $n -ErrorAction SilentlyContinue
    if ($c) { return $c.Source }
  }
  foreach ($p in $paths) { if (Test-Path $p) { return $p } }
  return $null
}

function Find-StartMenuLnk($pattern) {
  $dir = "C:\ProgramData\Microsoft\Windows\Start Menu\Programs\2026 WPILib Tools"
  if (-not (Test-Path $dir)) { return $null }
  $lnk = Get-ChildItem $dir -Filter "*.lnk" | Where-Object { $_.Name -like $pattern } | Select-Object -First 1
  if (-not $lnk) { return $null }
  try {
    $target = (New-Object -ComObject WScript.Shell).CreateShortcut($lnk.FullName).TargetPath
    if ($target -and (Test-Path $target)) { return $target }
  } catch { }
  return $null
}

$elastic = Find-App @("elastic", "ElasticDashboard") @(
  "$env:LOCALAPPDATA\Programs\FRC Elastic\elastic_dashboard.exe",
  "$env:LOCALAPPDATA\Elastic Dashboard\Elastic Dashboard.exe",
  "$env:ProgramFiles\Elastic Dashboard\Elastic Dashboard.exe")
$advScope = Find-App @("advantagescope") @(
  "$env:LOCALAPPDATA\AdvantageScope\AdvantageScope.exe",
  "$env:ProgramFiles\AdvantageScope\AdvantageScope.exe")
if (-not $advScope) { $advScope = Find-StartMenuLnk "AdvantageScope*.lnk" }

# Sim blocks: launched above under the sim-gui lock, so dashboards can attach.


if ($elastic) { Start-Process $elastic | Out-Null; Write-Output "Elastic launched: $elastic (connect 127.0.0.1:5810, Ctrl+D for layout on port 5800)." }
else { Write-Output "Elastic not found. Install from https://github.com/Gold872/elastic-dashboard, connect 127.0.0.1:5810, Ctrl+D -> elastic-layout.json." }

if ($advScope) { Start-Process $advScope | Out-Null; Write-Output "AdvantageScope launched: $advScope (NT 127.0.0.1, open advantagescope-layout.json)." }
else { Write-Output "AdvantageScope not found. Install from https://github.com/Mechanical-Advantage/AdvantageScope, NT 127.0.0.1, open TitanRoboticsBuildSeason/advantagescope-layout.json." }
