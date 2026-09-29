# Tier 2 practice-match loop: starts simulateJava with console captured to
# TitanRoboticsBuildSeason/reports/, waits for the match, stops the sim, then
# runs review-practice-log.py. Stdlib python only; no ntcore needed.
param(
  [int]$MatchSec = 170,
  [string]$Drill = "Free Play Match",
  [int]$AutoStopSec = 0,
  [switch]$RealDs
)
$ErrorActionPreference = "Stop"
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
$GradleRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\..\..\..\TitanRoboticsBuildSeason"))
$ReportsDir = Join-Path $GradleRoot "reports"
New-Item -ItemType Directory -Force -Path $ReportsDir | Out-Null

$existing = Get-CimInstance Win32_Process -Filter "Name='java.exe'" |
  Where-Object { $_.CommandLine -like "*TitanRoboticsBuildSeason*" }
if ($existing) {
  Write-Output "A sim for this project looks already running:"
  foreach ($p in $existing) { Write-Output ("  pid=" + $p.ProcessId) }
  Write-Output "Stop it first (its console is not captured), then re-run this script."
  exit 2
}

$ts = Get-Date -Format "yyyyMMdd_HHmmss"
$log = Join-Path $ReportsDir ("sim-console-" + $ts + ".log")
$gradleArgs = ".\gradlew simulateJava --offline"
if ($RealDs) { $gradleArgs += " -PrealDs" }
$child = "cd '" + $GradleRoot + "'; `$env:JAVA_HOME='" + $env:JAVA_HOME + "'; `$env:PATH='" + $env:JAVA_HOME + "\bin;' + `$env:PATH; " + $gradleArgs + " 2>&1 | Tee-Object -FilePath '" + $log + "'"
$win = Start-Process powershell -ArgumentList "-NoExit", "-Command", $child -PassThru
Write-Output ("Sim started, console capturing to " + $log)
Write-Output "CHECKLIST before enabling:"
Write-Output "  1. Elastic Simulation tab: Opponent AI Active ON, Opponent Count 1-3."
Write-Output "  2. Set Simulation/DebugAI=true (Elastic or SimGUI NT view) so STUCK dumps print."
Write-Output ("  3. Drill: " + $Drill + ".")
if ($RealDs) {
  Write-Output "  3. Enable from the REAL Driver Station app (Practice mode) — full auto then teleop. Ignore the SimGUI DS."
} else {
  Write-Output "  3. Enable Teleoperated in SimGUI and play."
}
if ($AutoStopSec -gt 0) {
  Write-Output ("Auto-stop in " + $AutoStopSec + "s (sim startup eats the first ~60s).")
  Start-Sleep -Seconds $AutoStopSec
} else {
  Read-Host "Press Enter when the match is over (sim keeps running until reviewed)"
}
Write-Output "Stopping sim processes started for this run..."
Get-CimInstance Win32_Process -Filter "Name='java.exe'" |
  Where-Object { $_.CommandLine -like "*TitanRoboticsBuildSeason*" } |
  ForEach-Object { try { Stop-Process -Id $_.ProcessId -Force } catch { } }
try { Stop-Process -Id $win.Id -Force } catch { }
Write-Output "Running log review..."
python (Join-Path $PSScriptRoot "review-practice-log.py") --log $log --drill $Drill --report-dir $ReportsDir
