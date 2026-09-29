# Tier 0 headless gate: compile + sim scenario input tests. No display needed.
#
# Holds the gradle-build lock for the duration. Two agents running `gradlew test`
# against the same tree clobber each other's build/test-results/*.xml, and the
# "361 tests" baseline quoted in AGENTS.md / KNOWN_ISSUES.md is read out of those
# XMLs, so a clobber silently invalidates the number the whole repo cites.
param([int]$TimeoutSec = 900, [string]$Reason = "smoke-headless")
$ErrorActionPreference = "Stop"
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
$GradleRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\..\..\..\TitanRoboticsBuildSeason"))
Set-Location $GradleRoot

Import-Module (Join-Path $GradleRoot "tools\lock\Lock.psm1") -Force
$locked = @()
try {
  $locked = Enter-Lock -Resource gradle-build -Reason $Reason -TimeoutSec $TimeoutSec -Quiet
  .\gradlew compileJava --offline
  if ($LASTEXITCODE -ne 0) { throw "compileJava failed ($LASTEXITCODE)" }
  .\gradlew test --offline --tests "frc.robot.Sim.TrainingMatchScenarioTest" --tests "frc.robot.Sim.TrainingMatchScenarioApplicationTest"
  if ($LASTEXITCODE -ne 0) { throw "test failed ($LASTEXITCODE)" }
} catch {
  [Console]::Error.WriteLine($_.Exception.Message)
  exit 3
} finally {
  if ($locked.Count -gt 0) { Exit-Lock -Resource $locked -Quiet }
}
exit 0
