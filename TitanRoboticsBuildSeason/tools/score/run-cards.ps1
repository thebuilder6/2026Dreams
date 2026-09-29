# Builds and runs tools/score/DecisionCards.java against the built fat jar.
#
#   powershell -File tools\score\run-cards.ps1
#   powershell -File tools\score\run-cards.ps1 -Out results\decision_cards.md
#
# The tool prints a decision card per situation (what the Jev engine chose, with a
# held-fuel sweep showing where the choice changes) and marks each card
# PASS / MISMATCH against the `expected` column in the TSV.
param(
  [string]$Tsv = "tools\score\decision_cards.tsv",
  [string]$Out = "results\decision_cards.md"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)   # TitanRoboticsBuildSeason
$jdk = "C:\Users\Public\wpilib\2026\jdk"
$env:JAVA_HOME = $jdk
$env:PATH = "$jdk\bin;$env:PATH"

$jar = Join-Path $root "build\libs\TitanRoboticsBuildSeason.jar"
$classes = Join-Path $root "build\decisioncards"
$src = Join-Path $PSScriptRoot "DecisionCards.java"
$jni = Join-Path $root "build\jni\release"

Push-Location $root
try {
  if (-not (Test-Path $jar)) {
    Write-Output "=== building project jar (DecisionCards links against it) ==="
    & ".\gradlew" jar --offline -q 2>&1 | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "gradlew jar failed" }
  }
  if (-not (Test-Path $jar)) { throw "no jar at $jar" }

  $class = Join-Path $classes "DecisionCards.class"
  $stale = (-not (Test-Path $class)) -or ((Get-Item $src).LastWriteTime -gt (Get-Item $class).LastWriteTime)
  if ($stale) {
    Write-Output "=== compiling DecisionCards.java ==="
    New-Item -ItemType Directory -Force -Path $classes | Out-Null
    & "$jdk\bin\javac.exe" -nowarn -encoding UTF-8 -cp $jar -d $classes $src
    if ($LASTEXITCODE -ne 0) { throw "javac failed" }
  }

  # WPILib's HAL_LoadExtension resolves through the OS loader (PATH), not the
  # JVM library path. Without this the tool dies with "Unable to find wpi driver
  # binary". Same reason dumpSimLaunch emits a pathPrefix for the sweep.
  if (Test-Path $jni) { $env:PATH = "$jni;$env:PATH" }

  New-Item -ItemType Directory -Force -Path (Join-Path $root "results") | Out-Null
  & "$jdk\bin\java.exe" "-Djava.library.path=$jni" -cp "$classes;$jar" DecisionCards $Tsv $Out
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
  Write-Output ""
  Write-Output "next: open $Out, fill in 'expected' in $Tsv for the UNREVIEWED cards, re-run"
} finally { Pop-Location }
