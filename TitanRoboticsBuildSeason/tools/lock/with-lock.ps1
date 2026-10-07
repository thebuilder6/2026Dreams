# Runs one command while holding one or more locks, releasing in a finally.
# This is how an orchestrator fans out a batch: take gradle-build ONCE for the
# whole batch and let workers run under it, instead of each worker contending.
#
#   powershell -File tools/lock/with-lock.ps1 -Resource gradle-build -Reason "full suite" -Command ".\gradlew test --offline --no-daemon"
#   powershell -File tools/lock/with-lock.ps1 -Resource sweep -Reason "seed sweep" -Command "powershell -File tools/score/sweep.ps1 -Seeds 7,42"
#
# -Command is one string (PowerShell 5.1 has no working argv passthrough for
# dash-prefixed child flags: they bind as this script's parameters instead).
# The lock is anchored on THIS script's PID, which stays alive for the whole
# child command, so the hold is real (unlike acquire-then-exit, which would go
# STALE immediately). Exit code is the child's exit code.
param(
  [Parameter(Mandatory = $true)][string[]]$Resource,
  [Parameter(Mandatory = $true)][string]$Command,
  [string]$Reason = "",
  [int]$TimeoutSec = -1
)
$ErrorActionPreference = "Stop"
Import-Module (Join-Path $PSScriptRoot "Lock.psm1") -Force

try {
  $null = Enter-Lock -Resource $Resource -TimeoutSec $TimeoutSec -Reason $Reason
} catch {
  [Console]::Error.WriteLine($_.Exception.Message)
  if ($_.Exception.Message -like "*TIMEOUT*") { exit 3 }
  exit 1
}

Write-Output ("[with-lock] running under '{0}': {1}" -f ($Resource -join ','), $Command)
try {
  Invoke-Expression $Command
  $code = $LASTEXITCODE
  if ($null -eq $code) { $code = 0 }
  exit $code
} finally {
  Exit-Lock -Resource $Resource -Quiet
}
