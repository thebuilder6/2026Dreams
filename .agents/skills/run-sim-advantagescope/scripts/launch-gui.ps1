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

# Sim blocks: launch in a new window so dashboards can attach.
$simCmd = ".\gradlew simulateJava --offline"
if ($RealDs) { $simCmd += " -PrealDs" }
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd '$GradleRoot'; `$env:JAVA_HOME='$env:JAVA_HOME'; $simCmd" | Out-Null
Write-Output "simulateJava launched in a new window from $GradleRoot."

if ($elastic) { Start-Process $elastic | Out-Null; Write-Output "Elastic launched: $elastic (connect 127.0.0.1:5810, Ctrl+D for layout on port 5800)." }
else { Write-Output "Elastic not found. Install from https://github.com/Gold872/elastic-dashboard, connect 127.0.0.1:5810, Ctrl+D -> elastic-layout.json." }

if ($advScope) { Start-Process $advScope | Out-Null; Write-Output "AdvantageScope launched: $advScope (NT 127.0.0.1, open advantagescope-layout.json)." }
else { Write-Output "AdvantageScope not found. Install from https://github.com/Mechanical-Advantage/AdvantageScope, NT 127.0.0.1, open TitanRoboticsBuildSeason/advantagescope-layout.json." }
