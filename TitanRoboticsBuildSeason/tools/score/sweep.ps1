# Sweeps headless 3v3 matches across a (variant x seed) grid, in parallel, into
# one JSONL file that tools/score/compare.py consumes.
#
#   powershell -File tools/score/sweep.ps1 -Seeds 7,11,42,101,500,1337,2026,9999
#   powershell -File tools/score/sweep.ps1 -Variants baseline,batch18 -Seeds 7,11,42
#   powershell -File tools/score/sweep.ps1 -Replicas 2 -Seeds 7,11,42   # noise floor
#
# Why raw `java` and not N Gradle invocations: a 150 s match costs ~2m48s wall,
# and each `gradlew simulateJavaRelease` adds JVM + Gradle startup and a daemon
# hop on top. The launch recipe (classpath, -Djava.library.path, main class)
# comes from `gradlew dumpSimLaunch`, which reflects over the real
# simulateJavaRelease JavaExec, so it cannot drift from what Gradle would run.
#
# Resumable: rows already in the JSONL for a (variant, seed, replica) are
# skipped, so an interrupted sweep can just be re-run.
param(
  [string]$Seeds = "7,11,42,101,500,1337,2026,9999",
  [string]$Variants = "baseline",
  [int]$Replicas = 1,
  [int]$DurationSec = 150,
  [int]$AutoSec = 15,
  [int]$FieldFuelCount = 108,
  [int]$MaxWorkers = 12,
  [string]$OutFile = "",
  [string]$Weights = "",
  [switch]$Fresh
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)   # TitanRoboticsBuildSeason
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$javaExe = Join-Path $env:JAVA_HOME "bin\java.exe"

$seedList = @($Seeds.Split(',') | ForEach-Object { [int]$_.Trim() } | Where-Object { $_ -gt 0 })
$variantList = @($Variants.Split(',') | ForEach-Object { $_.Trim() } | Where-Object { $_ })
if ($seedList.Count -eq 0) { throw "no valid seeds in '$Seeds'" }
if ($variantList.Count -eq 0) { throw "no valid variants in '$Variants'" }

if (-not $OutFile) { $OutFile = "results\sweep.jsonl" }
$outPath = if ([System.IO.Path]::IsPathRooted($OutFile)) { $OutFile } else { Join-Path $root $OutFile }
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $outPath) | Out-Null
if ($Fresh -and (Test-Path $outPath)) { Remove-Item -Force $outPath }

# --- launch recipe -----------------------------------------------------------
Push-Location $root
try {
  Write-Output "=== building launch recipe (dumpSimLaunch) ==="
  & ".\gradlew" dumpSimLaunch --offline -q 2>&1 | Out-Null
  if ($LASTEXITCODE -ne 0) { throw "dumpSimLaunch failed" }
  $cfgFile = Join-Path $root "build\simlaunch.properties"
  if (-not (Test-Path $cfgFile)) { throw "dumpSimLaunch produced no $cfgFile" }
} finally { Pop-Location }

$cfg = @{}
Get-Content $cfgFile | ForEach-Object {
  if ($_ -match '^([^#][^=]*)=(.*)$') { $cfg[$matches[1].Trim()] = $matches[2] }
}
foreach ($req in @('classpath', 'jvmArgs', 'mainClass')) {
  if (-not $cfg.ContainsKey($req)) { throw "launch recipe is missing '$req'" }
}
# The OS loader searches PATH, not the JVM library path. See the pathPrefix
# comment in build.gradle: without this every worker dies with
# "Unable to find wpi driver binary" then EXCEPTION_UNCAUGHT_CXX_EXCEPTION,
# even though wpiHal.dll / wpiHaljni.dll / halsim_*.dll are all present.
$jni = $cfg['pathPrefix']
if ($jni) { $env:PATH = "$jni;$env:PATH" }
Write-Output "=== recipe ok === main=$($cfg['mainClass'])"
Write-Output "=== jni=$jni"

# --- resumability: skip work already recorded --------------------------------
$done = @{}
if (Test-Path $outPath) {
  foreach ($ln in (Get-Content $outPath)) {
    if ($ln -notmatch '"seed":\s*(\d+)') { continue }
    $s = $matches[1]
    $v = if ($ln -match '"variant":\s*"([^"]*)"') { $matches[1] } else { "baseline" }
    $r = if ($ln -match '"replica":\s*(\d+)') { $matches[1] } else { "0" }
    $done["$v|$s|$r"] = $true
  }
  Write-Output "=== $($done.Count) existing row(s); resumable ==="
}

# --- build the job grid ------------------------------------------------------
$jobs = @()
foreach ($v in $variantList) {
  foreach ($s in $seedList) {
    for ($r = 0; $r -lt $Replicas; $r++) {
      if ($done.ContainsKey("$v|$s|$r")) { continue }
      $jobs += [pscustomobject]@{ variant = $v; seed = $s; replica = $r }
    }
  }
}
if ($jobs.Count -eq 0) { Write-Output "nothing to do (all rows already present)"; exit 0 }

$workers = [Math]::Min($MaxWorkers, $jobs.Count)
Write-Output "=== $($jobs.Count) match(es), $workers parallel, ${DurationSec}s scenario each ==="
Write-Output "=== variants: $($variantList -join ', ') ==="
Write-Output "=== output:   $outPath ==="

$logDir = Join-Path $root "logs\sweep"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null

# Execution uses Start-Process rather than runspaces or thread jobs:
#   * Start-ThreadJob needs PowerShell 6+; this has to run on 5.1.
#   * Start-Job spawns a process per worker, and a shared in-memory queue cannot
#     cross that process boundary.
#   * Start-Process is the lightest option and needs no shared state at all --
#     each match appends its own JSONL row, so the only coordination required is
#     "at most N at a time", which batching gives us.
$sw = [System.Diagnostics.Stopwatch]::StartNew()
$rows = @()
$pending = New-Object System.Collections.ArrayList
foreach ($j in $jobs) { $null = $pending.Add($j) }
$batchNo = 0

while ($pending.Count -gt 0) {
  $batchNo++
  $take = [Math]::Min($workers, $pending.Count)
  $batch = @()
  for ($i = 0; $i -lt $take; $i++) { $batch += $pending[0]; $pending.RemoveAt(0) }
  Write-Output "--- batch $batchNo : $($batch.Count) match(es) ---"

  $running = @()
  foreach ($job in $batch) {
    $tag = "$($job.variant)_seed$($job.seed)_r$($job.replica)"
    $stdout = Join-Path $logDir "$tag.out.log"
    $stderr = Join-Path $logDir "$tag.err.log"
    $argList = @(
      $cfg['jvmArgs'],
      "-Dfrc.headless=true",
      "-Dfrc.headless.seed=$($job.seed)",
      "-Dfrc.headless.durationSec=$DurationSec",
      "-Dfrc.headless.autoSec=$AutoSec",
      "-Dfrc.headless.fieldFuelCount=$FieldFuelCount",
      "-Dfrc.headless.logDir=$logDir",
      "-Dfrc.headless.resultJsonl=$outPath",
      "-Dfrc.headless.variant=$($job.variant)",
      "-Dfrc.headless.replica=$($job.replica)"
    )
    if ($Weights) { $argList += "-Dfrc.policyWeights=$Weights" }
    $argList += @("-cp", "`"$($cfg['classpath'])`"", $cfg['mainClass'])
    $p = Start-Process -FilePath $javaExe -ArgumentList $argList -WorkingDirectory $root `
                       -RedirectStandardOutput $stdout -RedirectStandardError $stderr `
                       -NoNewWindow -PassThru
    $running += [pscustomobject]@{ job = $job; proc = $p; out = $stdout; err = $stderr }
  }

  foreach ($r in $running) {
    $null = $r.proc.WaitForExit()
    $text = ""
    if (Test-Path $r.out) { $text = (Get-Content $r.out -Raw) }
    $final = $null
    if ($text) { $final = ($text -split "`n" | Select-String "Headless\] FINAL" | Select-Object -Last 1) }
    # Success is "the driver reached FINAL *and* a JSONL row exists for our
    # (variant, seed, replica) key". Deliberately not Process.ExitCode:
    # Start-Process -PassThru does not reliably populate ExitCode on Windows
    # PowerShell 5.1 (it comes back empty even for a clean run), and counting
    # total rows is racy with parallel appends. The unique key is neither.
    $key = "$($r.job.variant)|$($r.job.seed)|$($r.job.replica)"
    $rowSeen = $false
    if (Test-Path $outPath) {
      foreach ($ln in (Get-Content $outPath)) {
        if ($ln -match '"seed":\s*(\d+)' -and $matches[1] -eq "$($r.job.seed)") {
          $v = if ($ln -match '"variant":\s*"([^"]*)"') { $matches[1] } else { "baseline" }
          $rp = if ($ln -match '"replica":\s*(\d+)') { $matches[1] } else { "0" }
          if ("$v|$($r.job.seed)|$rp" -eq $key) { $rowSeen = $true; break }
        }
      }
    }
    $errText = ""
    if (Test-Path $r.err) { $errText = (Get-Content $r.err -Raw) }
    $errHits = ""
    if ($errText) {
      $errHits = (($errText -split "`n" | Where-Object { $_ -match 'Exception|UnsatisfiedLink|Unable to find' }) |
                  Select-Object -First 2) -join ' | '
    }
    $rows += [pscustomobject]@{
      variant = $r.job.variant; seed = $r.job.seed; replica = $r.job.replica
      ok = ($null -ne $final -and $rowSeen)
      summary = if ($final) { "$final".Trim() } else { "(no FINAL line)" }
      row = $rowSeen
      log = $r.out
      err = $errHits
    }
  }
  Write-Output "    batch $batchNo done ($([int]$sw.Elapsed.TotalSeconds)s elapsed)"
}
$sw.Stop()

# --- summary -----------------------------------------------------------------
$rows = @($rows | Sort-Object variant, seed, replica)
Write-Output ""
Write-Output "=== $($rows.Count) match(es) finished in $([int]$sw.Elapsed.TotalSeconds)s ==="
$rows | Format-Table -AutoSize variant, seed, replica, ok, row, summary | Out-String | Write-Output
$failed = @($rows | Where-Object { -not $_.ok })
if ($failed.Count -gt 0) {
  Write-Output "=== $($failed.Count) FAILED ==="
  foreach ($f in $failed) {
    Write-Output "  $($f.variant) seed$($f.seed) r$($f.replica)  log=$($f.log)"
    if ($f.err) { Write-Output "    $($f.err)" }
  }
  Write-Output "results file: $outPath"
  exit 1
}
Write-Output "all matches ok -> $outPath"
Write-Output "next:  python tools/score/compare.py --results `"$outPath`""
