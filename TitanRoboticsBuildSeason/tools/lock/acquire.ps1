# Acquires one or more shared-resource locks. Waits, then fails loudly.
#
#   powershell -File tools/lock/acquire.ps1 -Resource gradle-build
#   powershell -File tools/lock/acquire.ps1 -Resource gradle-build -Reason "full suite"
#   powershell -File tools/lock/acquire.ps1 -Resource sim-gui -AnchorPid 12345
#   powershell -File tools/lock/acquire.ps1 -Resource gradle-build -TimeoutSec 0 -Quiet
#
# Exit codes: 0 acquired, 3 timed out, 1 bad usage.
# Wrap the protected work in try/finally and call release.ps1 in the finally.
param(
  [Parameter(Mandatory = $true)][string[]]$Resource,
  [int]$TimeoutSec = -1,
  [int[]]$AnchorPid = @(),
  [string]$Reason = "",
  [switch]$Quiet
)
$ErrorActionPreference = "Stop"
Import-Module (Join-Path $PSScriptRoot "Lock.psm1") -Force
$msg = $null
try {
  $null = Enter-Lock -Resource $Resource -TimeoutSec $TimeoutSec -AnchorPid $AnchorPid -Reason $Reason -Quiet:$Quiet
  exit 0
} catch {
  $msg = $_.Exception.Message
}
# Write to stderr directly rather than Write-Error: under $ErrorActionPreference
# = "Stop" a Write-Error inside the catch block re-throws and the exit code
# below never runs, which is how a timeout silently came back as exit 1.
[Console]::Error.WriteLine($msg)
if ($msg -like "*TIMEOUT*") { exit 3 }
exit 1
