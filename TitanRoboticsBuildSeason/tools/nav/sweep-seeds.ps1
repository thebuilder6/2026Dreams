# Sweeps headless 3v3 matches across seeds and reports teleop throughput per bot.
# Usage: sweep-seeds.ps1 [-Seeds 7,11,2026] [-DurationSec 150] [-Tag label]
param(
  # Comma-separated so it binds cleanly from -File invocation on PowerShell 5.1
  # (a bare -Seeds 7,11,42 is parsed as a single string, not an int[]).
  [string]$Seeds = "7,11,42,101,2026,3141,7777,90210",
  [int]$DurationSec = 150,
  [int]$AutoSec = 15,
  [string]$Tag = "sweep"
)
$ErrorActionPreference = "Stop"
$seedList = $Seeds.Split(',') | ForEach-Object { [int]$_.Trim() } | Where-Object { $_ -gt 0 }
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"

$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)   # TitanRoboticsBuildSeason
$outDir = Join-Path $root "reports"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

# Serialize headless runs. Two concurrent drivers share the NetworkTables/CameraServer
# ports and the reports directory, so they interleave report files and a sweep can
# pick up another run's match. This already happened twice; refuse to start rather
# than produce silently mixed data.
$lock = Join-Path $root "build\headless-sweep.lock"
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $lock) | Out-Null
if (Test-Path $lock) {
  $held = Get-Content $lock -ErrorAction SilentlyContinue
  $ownerPid = if ($held) { [int]($held | Select-Object -First 1) } else { 0 }
  if ($ownerPid -gt 0 -and (Get-Process -Id $ownerPid -ErrorAction SilentlyContinue)) {
    Write-Output "REFUSING: another sweep is running (pid $ownerPid). Delete $lock if that is stale."
    exit 3
  }
  Write-Output "clearing stale lock from pid $ownerPid"
}
Set-Content -Path $lock -Value $PID
trap { Remove-Item $lock -ErrorAction SilentlyContinue }

$rows = @()
foreach ($s in $seedList) {
  Write-Output "=== seed $s ==="
  Push-Location $root
  try {
    # A failed seed must not abort the sweep; the row records ok=false and we move on.
    $ErrorActionPreference = "Continue"
    & ".\gradlew" simulateJavaRelease --offline -Pheadless "-Pseed=$s" `
        "-PdurationSec=$DurationSec" "-PautoSec=$AutoSec" "-PlogDir=logs" "-PreportDir=reports" 2>&1 |
      Select-String -Pattern "Headless\] FINAL|Headless\] report" | ForEach-Object { Write-Output "  $_" }
    $ErrorActionPreference = "Stop"
  } catch {
    Write-Output "  seed $s FAILED: $($_.Exception.Message)"
  } finally { Pop-Location }

  $report = Get-ChildItem (Join-Path $outDir "headless_match_seed${s}_*.md") |
            Sort-Object LastWriteTime | Select-Object -Last 1
  if (-not $report) { $rows += [pscustomobject]@{seed=$s; ok=$false}; continue }

  $c = Get-Content $report.FullName
  $num = { param($label) $line = $c | Select-String -SimpleMatch $label | Select-Object -First 1
           if (-not $line) { return 0 }
           ($line -split '\|')[1..2] | ForEach-Object { $_.Trim() } }

  $auto = (& $num "Auto fuel")       -join "/"
  $tele = (& $num "Teleop fuel")     -join "/"
  $waste= (& $num "Wasted fuel")
  $rows += [pscustomobject]@{
    seed    = $s
    ok      = $true
    report  = $report.Name
    auto    = $auto
    teleop  = $tele
    wasted  = ($waste -join "/")
    log     = (Get-ChildItem (Join-Path $root "logs") -Filter "headless_3v3_seed${s}_*.wpilog" |
               Sort-Object LastWriteTime | Select-Object -Last 1).Name
  }
}

$rows | Format-Table -AutoSize | Out-String | Write-Output
$rows | Export-Csv (Join-Path $outDir "seed_sweep_$Tag.csv") -NoTypeInformation
Remove-Item $lock -ErrorAction SilentlyContinue
Write-Output "wrote reports/seed_sweep_$Tag.csv"
