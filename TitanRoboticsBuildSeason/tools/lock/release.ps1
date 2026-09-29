# Releases locks held by this owner. Safe to call for a lock already gone.
#
#   powershell -File tools/lock/release.ps1 -Resource gradle-build
#   powershell -File tools/lock/release.ps1 -Resource sim-gui,sweep
param(
  [Parameter(Mandatory = $true)][string[]]$Resource,
  [switch]$Quiet
)
$ErrorActionPreference = "Stop"
Import-Module (Join-Path $PSScriptRoot "Lock.psm1") -Force
Exit-Lock -Resource $Resource -Quiet:$Quiet
exit 0
