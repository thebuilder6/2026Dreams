# Tier 0 headless gate: compile + sim scenario input tests. No display needed.
$ErrorActionPreference = "Stop"
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
Set-Location ([System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\..\..\..\TitanRoboticsBuildSeason")))
.\gradlew compileJava --offline
.\gradlew test --offline --tests "frc.robot.Sim.TrainingMatchScenarioTest" --tests "frc.robot.Sim.TrainingMatchScenarioApplicationTest"
