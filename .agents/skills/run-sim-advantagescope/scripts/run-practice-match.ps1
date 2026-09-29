# Tier 2 practice-match loop: starts simulateJava with console captured to
# TitanRoboticsBuildSeason/reports/, waits for the match, stops the sim, then
# runs review-practice-log.py. Stdlib python only; no ntcore needed.
#
# SCOPED SHUTDOWN
# ---------------
# This script used to stop the sim with:
#   Get-CimInstance Win32_Process -Filter "Name='java.exe'" |
#     Where-Object { $_.CommandLine -like "*TitanRoboticsBuildSeason*" } |
#     ForEach-Object { Stop-Process -Id $_.ProcessId -Force }
# That pattern cannot tell a leaked JVM from another agent's LIVE SimGUI or its
# in-flight `gradlew test` JVM -- every one of those matches the same wildcard.
# It was therefore a direct cause of agents interrupting each other. Shutdown is
# now scoped to the process tree this script started, resolved by parent PID.
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

Import-Module (Join-Path $GradleRoot "tools\lock\Lock.psm1") -Force

function Get-DescendantProcessIds {
  <#
    .SYNOPSIS
      Every transitive child of $RootPid, by ParentProcessId.
    .NOTES
      Captured at shutdown time, not launch time: the gradle and java processes
      for the sim do not exist yet when Start-Process returns, so a list
      captured up front would be empty and nothing would be stopped.
  #>
  param([int]$RootPid)
  $all = @(Get-CimInstance Win32_Process -ErrorAction SilentlyContinue |
           Select-Object ProcessId, ParentProcessId)
  $found = New-Object System.Collections.ArrayList
  $frontier = @($RootPid)
  while ($frontier.Count -gt 0) {
    $next = @()
    foreach ($p in $all) {
      $ppid = [int]$p.ParentProcessId
      $cpid = [int]$p.ProcessId
      if ($frontier -contains $ppid -and $cpid -ne $RootPid -and -not $found.Contains($cpid)) {
        [void]$found.Add($cpid)
        $next += $cpid
      }
    }
    $frontier = $next
  }
  return @($found)
}

# The sim owns NT4 5810, WebServer 5800 and CameraServer 1181-1182 for its whole
# run, and those are hardcoded WPILib constants with no offset switch (see the
# header of tools/score/sweep.ps1). So only one sim at a time, repo-wide. Take
# the lock before launching rather than scanning for processes afterwards.
$locked = @()
try {
  $locked = Enter-Lock -Resource sim-gui -Reason "practice match: $Drill" -TimeoutSec 300
} catch {
  Write-Error $_.Exception.Message
  exit 3
}

$win = $null
try {
  # Informational only. A sim running that did NOT come through this protocol
  # (a human, or an agent that skipped the lock) is still worth surfacing, but
  # it is not ours to stop and not a reason to abort.
  $stray = @(Get-CimInstance Win32_Process -Filter "Name='java.exe'" -ErrorAction SilentlyContinue |
             Where-Object { $_.CommandLine -like "*TitanRoboticsBuildSeason*" })
  if ($stray.Count -gt 0) {
    Write-Warning ("{0} java process(es) match this project but are not tracked by the sim-gui lock:" -f $stray.Count)
    foreach ($p in $stray) { Write-Warning ("  pid=" + $p.ProcessId) }
    Write-Warning "Leaving them alone. If one is yours and you need it gone, kill that PID directly."
  }

  $ts = Get-Date -Format "yyyyMMdd_HHmmss"
  $log = Join-Path $ReportsDir ("sim-console-" + $ts + ".log")
  $gradleArgs = ".\gradlew simulateJava --offline"
  if ($RealDs) { $gradleArgs += " -PrealDs" }
  $child = "cd '" + $GradleRoot + "'; `$env:JAVA_HOME='" + $env:JAVA_HOME + "'; `$env:PATH='" + $env:JAVA_HOME + "\bin;' + `$env:PATH; " + $gradleArgs + " 2>&1 | Tee-Object -FilePath '" + $log + "'"
  $win = Start-Process powershell -ArgumentList "-NoExit", "-Command", $child -PassThru
  Write-Output ("Sim started (host pid " + $win.Id + "), console capturing to " + $log)
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

  # ---- scoped shutdown: only the tree this script started ----
  $targets = @()
  if ($win) { $targets = @(Get-DescendantProcessIds -RootPid $win.Id) }
  if ($win) { $targets += $win.Id }
  Write-Output ("Stopping {0} process(es) started for this run (pids: {1})..." -f `
      $targets.Count, ($targets -join ','))
  # Deepest first, so a parent cannot outlive a child it is waiting on.
  foreach ($p in ($targets | Sort-Object -Descending)) {
    try { Stop-Process -Id $p -Force -ErrorAction Stop } catch { }
  }
  $win = $null

  Write-Output "Running log review..."
  python (Join-Path $PSScriptRoot "review-practice-log.py") --log $log --drill $Drill --report-dir $ReportsDir
} finally {
  if ($win) { try { Stop-Process -Id $win.Id -Force -ErrorAction SilentlyContinue } catch { } }
  if ($locked.Count -gt 0) { Exit-Lock -Resource $locked -Quiet }
}
