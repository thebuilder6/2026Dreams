# Pre-flight gate: fail loudly before a build/test/sweep instead of producing a
# number that looks clean but is not. Read-only: never takes a lock, never
# kills anything, never touches build/.
#
#   powershell -File tools/dev/check.ps1                                    # warn-only stamp
#   powershell -File tools/dev/check.ps1 -Resource gradle-build             # + probe the lock (exit 3 if held)
#   powershell -File tools/dev/check.ps1 -ForSweep                         # strict: dirty tree or dashboard = exit 1
#   powershell -File tools/dev/check.ps1 -FailOnDirty -FailOnDashboard
#
# Exit codes: 0 ok (warnings allowed), 1 environment fails, 3 lock held.
# PowerShell 5.1 only: no ternaries, no -Parallel.
param(
  [string[]]$Resource = @(),
  [string]$Reason = "",
  [int]$TimeoutSec = 0,
  [switch]$FailOnDirty,
  [switch]$FailOnDashboard,
  [switch]$ForSweep
)
$ErrorActionPreference = "Stop"
if ($ForSweep) { $FailOnDirty = $true; $FailOnDashboard = $true }

$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)   # TitanRoboticsBuildSeason
$repoRoot = Split-Path -Parent $root
Import-Module (Join-Path $root "tools\lock\Lock.psm1") -Force

$failures = @()
$warnings = @()

# --- 1. WPILib JDK ------------------------------------------------------------
$wantJdk = "C:\Users\Public\wpilib\2026\jdk"
$javaExe = $null
if ($env:JAVA_HOME) { $javaExe = Join-Path $env:JAVA_HOME "bin\java.exe" }
if (-not $env:JAVA_HOME) {
  $failures += "JAVA_HOME is unset. Run: `$env:JAVA_HOME = `"$wantJdk`"; `$env:PATH = `"`$env:JAVA_HOME\bin;`$env:PATH`""
} elseif (-not $javaExe -or -not (Test-Path $javaExe)) {
  $failures += "JAVA_HOME=$($env:JAVA_HOME) has no bin\java.exe. Expected the WPILib 2026 JDK at $wantJdk."
} elseif ($env:JAVA_HOME -ne $wantJdk) {
  $warnings += "JAVA_HOME=$($env:JAVA_HOME) is not the pinned WPILib JDK ($wantJdk); builds may fail with 'Unsupported class file major version'."
} else {
  Write-Output "[check] JDK ok: $env:JAVA_HOME"
}

# --- 2. Dirty-tree stamp ------------------------------------------------------
$dirty = @()
$untracked = @()
try {
  # -c core.autocrlf=false: read-only status output is identical, but without
  # the per-file CRLF warnings that are terminating errors under Stop.
  $porcelain = @(git -c core.autocrlf=false -C $repoRoot status --porcelain 2>$null)
  foreach ($ln in $porcelain) {
    if ($ln -match '^\?\?') { $untracked += $ln.Substring(3) } else { $dirty += $ln }
  }
} catch {
  $warnings += "git status failed ($($_.Exception.Message)); dirty stamp skipped."
}
if (($dirty.Count + $untracked.Count) -gt 0) {
  Write-Output ("[check] dirty tree: {0} modified + {1} untracked (next sweep/test runs dirty by definition)" -f $dirty.Count, $untracked.Count)
  $show = @($dirty | Select-Object -First 8)
  $show += @($untracked | Select-Object -First 4 | ForEach-Object { "?? $_" })
  foreach ($ln in $show) {
    Write-Output "         $ln"
  }
  if ($FailOnDirty) { $failures += "tree is dirty ($($dirty.Count) modified + $($untracked.Count) untracked). Commit/stash first, or re-run without -FailOnDirty for a warn-only stamp." }
} else {
  Write-Output "[check] tree clean"
}

# --- 3. Dashboard contamination ----------------------------------------------
$dash = @(Test-DashboardRunning)
if ($dash.Count -gt 0) {
  $who = (($dash | ForEach-Object { "$($_.ProcessName)#$($_.Id)" }) -join ', ')
  if ($FailOnDashboard) {
    $failures += "dashboard running ($who). It shares the robot NT namespace including the auto chooser; close it before sweeping (KNOWN_ISSUES.md section E)."
  } else {
    $warnings += "dashboard running ($who): fine for SimGUI, contaminates a score-rig sweep."
  }
} else {
  Write-Output "[check] no dashboard attached"
}

# --- 4. Lock probe (never waits beyond -TimeoutSec, default 0) ----------------
if ($Resource.Count -gt 0) {
  try {
    $null = Enter-Lock -Resource $Resource -TimeoutSec $TimeoutSec -Reason $Reason -Quiet
    Exit-Lock -Resource $Resource -Quiet
    Write-Output ("[check] lock free: {0}" -f ($Resource -join ','))
  } catch {
    [Console]::Error.WriteLine($_.Exception.Message)
    if ($_.Exception.Message -like "*TIMEOUT*") { exit 3 }
    $failures += "lock probe failed: $($_.Exception.Message)"
  }
} else {
  $rows = Get-LockStatus
  foreach ($r in $rows) {
    if ($r.State -ne 'free') { Write-Output ("[check] lock {0}: {1} by {2} ({3}s) {4}" -f $r.Resource, $r.State, $r.Owner, $r.HeldSec, $r.Reason) }
  }
}

foreach ($w in $warnings) { Write-Warning "[check] $w" }
if ($failures.Count -gt 0) {
  foreach ($f in $failures) { Write-Output "[check] FAIL: $f" }
  exit 1
}
Write-Output "[check] OK"
exit 0
