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
#
# ISOLATION AND DEGRADATION
# -------------------------
# Parallel workers share one machine, and the ports they want are hardcoded
# constants in WPILib and CTRE with no system property or env var to offset them
# (verified against wpilibj/cameraserver/ntcore 2026.2.1 and wpiapi-java 26.1.0:
# CameraServer.kBasePort=1181, NetworkTableInstance.kDefaultPort3/4=1735/5810).
# So the rig cannot port-offset. It eliminates what it can and *fails loudly* on
# the rest, because the previous version recorded every one of these runs as a
# success: "ok" was only "a FINAL line exists and a JSONL row exists".
#
# The contended ports, split by whether losing the race can corrupt the number:
#
#   1735 / 5810  NT3 + NT4, NetworkTableInstance. A worker that loses the bind
#                falls back to ntcore CLIENT mode and can attach to a *sibling
#                worker's* NT -- two matches writing one namespace. This is the
#                one with a data path. Robot.robotInit() calls stopServer() when
#                headless and prints a marker; this script requires that marker.
#   5800-5805    Elastic WebServer + coprocessor PortForwarder. Also skipped when
#                headless. Desktop conveniences with no data path.
#   1181 / 1182  CameraServer MJPEG + HLS, created by PhotonCameraSim's
#                constructor via Sim/VisionSim. Nothing consumes the stream in a
#                headless match, and cscore logs the failure and continues, so the
#                photonvision NT table that VisionIOSim reads is unaffected.
#                Reported, not fatal.
#   1250         phoenix-diagnostics. No Java-side switch exists; it retries and
#                one worker wins. No match data flows through it. Reported, not
#                fatal.
#
# On top of ports, a CPU-starved worker does not fail -- it silently produces a
# different match. The archived 12-wide baseline had 27-53 WPILib overrun warnings
# per 150 s match including one robotPeriodic() epoch of 0.81 s, against 1-3 in a
# 2-wide batch, and all 16 rows were recorded ok. Loop timing is now measured
# in-process (Sim/LoopHealth) and stamped into each JSONL row, and this script
# fails the sweep when a row is degraded. See KNOWN_ISSUES.md.
param(
  [string]$Seeds = "7,11,42,101,500,1337,2026,9999",
  [string]$Variants = "baseline",
  [int]$Replicas = 1,
  [int]$DurationSec = 150,
  [int]$AutoSec = 15,
  [int]$FieldFuelCount = 108,
  [int]$MaxWorkers = 4,
  [string]$OutFile = "",
  [string]$Weights = "",
  [int]$MaxLoopOverruns = 8,
  [int]$MaxRobotPeriodicMs = 60,
  [switch]$AllowDegraded,
  [switch]$Force,
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

# Module import must precede the pre-flight below, which calls
# Test-DashboardRunning from here rather than repeating the process pattern.
Import-Module (Join-Path $root "tools\lock\Lock.psm1") -Force

if (-not $OutFile) { $OutFile = "results\sweep.jsonl" }
$outPath = if ([System.IO.Path]::IsPathRooted($OutFile)) { $OutFile } else { Join-Path $root $OutFile }
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $outPath) | Out-Null
if ($Fresh -and (Test-Path $outPath)) { Remove-Item -Force $outPath }

# --- pre-flight: no dashboard may be watching --------------------------------
# RobotBase calls NetworkTableInstance.startServer() before robotInit runs, so a
# worker listens on NT4 5810 for a moment no matter what we do afterwards. Any
# dashboard on this machine auto-connects to that port, and then it is reading and
# writing the worker's namespace -- including the "Auto Mission" chooser the robot
# reads at autonomousInit. That is silent, and it was live for the whole archived
# baseline: an elastic_dashboard process had been running since 10:52 and every
# worker logged "NT: CONNECTED NT4 client 'Elastic@1'".
#
# Robot.robotInit stops the server and calls startLocal() once headless, which
# closes the window quickly, but it cannot un-connect a client that already
# attached. So the rig refuses to start instead: a dashboard must be closed first.
if (-not $Force) {
  $dash = Test-DashboardRunning
  if ($dash.Count -gt 0) {
    Write-Output "=== REFUSING TO SWEEP: a dashboard is running ==="
    foreach ($d in $dash) {
      Write-Output "  pid=$($d.Id)  $($d.ProcessName)  started=$($d.StartTime)"
    }
    Write-Output ""
    Write-Output "A dashboard auto-connects to each worker's NT4 server on 5810 and then shares"
    Write-Output "its NetworkTables namespace, including the auto-mission chooser. Headless"
    Write-Output "matches have no dashboard. Close it and re-run, or pass -Force to override."
    exit 1
  }
}

# --- acquire the sweep resource ----------------------------------------------
# Taken AFTER pre-flight so a bad invocation leaves nothing behind, and BEFORE
# the gradle call at all, because `gradlew dumpSimLaunch` below is itself a
# gradle build and would otherwise collide with another agent's compile.
#
# The conflict with sim-gui is the whole point: a GUI sim holds NT4 5810 for its
# entire run, and RobotBase calls NetworkTableInstance.startServer() before
# robotInit gets the chance to stop it, so a rig worker and a live SimGUI would
# share a NetworkTables namespace. That contamination is silent and it has
# already invalidated a baseline once -- see KNOWN_ISSUES.md section E.
#
# Release is explicit on the normal exits below. On a crash the lock is left
# behind, and that is acceptable by design rather than an oversight: the lock is
# anchored on this script's PID, so a dead script's lock reads as STALE and the
# next agent reclaims it loudly rather than deadlocking.
try {
  $null = Enter-Lock -Resource sweep -Reason "sweep: $($variantList -join ',') x $($seedList.Count) seeds, ${MaxWorkers}-wide" -TimeoutSec 600
} catch {
  [Console]::Error.WriteLine($_.Exception.Message)
  exit 3
}

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
    if ($Weights) { $argList += "-Dfrc.jev.weights=$Weights" }
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
    $rowText = $null
    if (Test-Path $outPath) {
      foreach ($ln in (Get-Content $outPath)) {
        if ($ln -match '"seed":\s*(\d+)' -and $matches[1] -eq "$($r.job.seed)") {
          $v = if ($ln -match '"variant":\s*"([^"]*)"') { $matches[1] } else { "baseline" }
          $rp = if ($ln -match '"replica":\s*(\d+)') { $matches[1] } else { "0" }
          if ("$v|$($r.job.seed)|$rp" -eq $key) { $rowText = $ln; break }
        }
      }
    }
    $rowSeen = $null -ne $rowText

    # Read the health block the driver stamped into the row. -1 means the match
    # never armed LoopHealth, which is itself suspicious for a rig worker.
    $overruns = -1
    $maxMs = -1.0
    if ($rowText -and $rowText -match '"loopOverruns":(-?\d+)') { $overruns = [int]$matches[1] }
    if ($rowText -and $rowText -match '"maxRobotPeriodicMs":(-?[\d.]+)') { $maxMs = [double]$matches[1] }

    $errText = ""
    if (Test-Path $r.err) { $errText = (Get-Content $r.err -Raw) }
    $errHits = ""
    if ($errText) {
      $errHits = (($errText -split "`n" | Where-Object { $_ -match 'Exception|UnsatisfiedLink|Unable to find' }) |
                  Select-Object -First 2) -join ' | '
    }

    # --- isolation + degradation audit ---------------------------------------
    # $why holds reasons the row must not be compared as a clean sample. A match
    # that lost a *data-path* port, or that was starved on the CPU, produced a
    # number that reflects the machine rather than the robot.
    # $all is stdout+stderr together, because the isolation marker is a
    # System.out println while every library's bind complaint goes to stderr.
    $all = "$text`n$errText"
    $why = @()
    $ntStopped = $false
    if ($all) {
      $ntStopped = ($all -match 'NT server stopped')
      if (-not $ntStopped) {
        $why += "no '[Headless] NT server stopped' marker - the NT isolation did not run"
      }
      if ($all -match 'NT server stop FAILED') {
        $why += "NetworkTableInstance.stopServer() threw in this worker"
      }
      if ($all -match 'WebServer\] Notice: Elastic layout WebServer on port 5800 could not be started') {
        $why += "WebServer 5800 started in a headless worker (should be skipped)"
      }
      if ($all -match 'PortForwarder\] Notice') {
        $why += "coprocessor PortForwarder 5801-5805 ran in a headless worker (should be skipped)"
      }
      # An actual NT *client* attaching to a rig worker is real cross-talk: it
      # means something outside the match is reading or writing this worker's
      # namespace. NT4Publisher is not added in headless, so there is no legitimate
      # client -- a connection here means a dashboard was open during the sweep.
      if ($all -match 'NT: Got a NT4 connection|CONNECTED NT4 client') {
        $why += "an NT4 client (dashboard) attached to this rig worker - close Elastic/SimGUI before sweeping"
      }
    }
    if ($rowSeen) {
      if ($overruns -lt 0) {
        $why += "health block missing/unmeasured (schema or arming regression)"
      } else {
        if ($overruns -gt $MaxLoopOverruns) { $why += "loop overruns $overruns > $MaxLoopOverruns (CPU-starved)" }
        if ($maxMs -gt $MaxRobotPeriodicMs) { $why += "slowest robotPeriodic $([int]$maxMs)ms > ${MaxRobotPeriodicMs}ms" }
      }
    }
    # Benign, counted but not fatal: see the header comment. The NT bind line is
    # benign ONLY because we stop the server and add no client: RobotBase calls
    # startServer() before robotInit, so the losing workers log this during the
    # window before stopServer() runs. Nothing then needs the listener. Without
    # the marker, the same line means the isolation silently regressed and the
    # row is degraded above instead.
    $benign = 0
    $ntBind = 0
    if ($all) {
      $benign = ([regex]::Matches($all, 'bind\(\) to port 118[12] failed')).Count
      $benign += ([regex]::Matches($all, 'first attempt to open server failed at port 1250')).Count
      $ntBind = ([regex]::Matches($all, 'NT3 server socket error|NT4 server socket error')).Count
      if ($ntStopped) { $benign += $ntBind }
    }
    $wpilibOverrunMsgs = 0
    if ($all) { $wpilibOverrunMsgs = ([regex]::Matches($all, 'Loop time of .* overrun')).Count }

    $rows += [pscustomobject]@{
      variant = $r.job.variant; seed = $r.job.seed; replica = $r.job.replica
      ok = ($null -ne $final -and $rowSeen)
      degraded = ($why.Count -gt 0)
      why = ($why -join '; ')
      overruns = $overruns
      maxMs = [int]$maxMs
      benignPorts = $benign
      ntBinds = $ntBind
      wpilibOverruns = $wpilibOverrunMsgs
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
$rows | Format-Table -AutoSize variant, seed, replica, ok, degraded, overruns, maxMs, benignPorts, summary |
  Out-String | Write-Output

$totalBenign = ($rows | Measure-Object -Property benignPorts -Sum).Sum
if ($totalBenign -gt 0) {
  Write-Output "note: $totalBenign benign port conflict(s) across the batch, all on ports with no data"
  Write-Output "      path into a headless match: 1181/1182 (CameraServer MJPEG/HLS created by"
  Write-Output "      PhotonCameraSim via Sim/VisionSim), 1250 (phoenix-diagnostics, no Java-side switch),"
  Write-Output "      and 1735/5810 (NT) -- the last only because Robot.robotInit stops the NT server and"
  Write-Output "      adds no NT4Publisher once headless. A row where that did NOT happen is degraded above."
  Write-Output ""
}

$degraded = @($rows | Where-Object { $_.degraded })
if ($degraded.Count -gt 0) {
  Write-Output "=== $($degraded.Count) DEGRADED (ran, but the number is not a clean sample) ==="
  foreach ($d in $degraded) {
    Write-Output "  $($d.variant) seed$($d.seed) r$($d.replica): $($d.why)"
    Write-Output "    log=$($d.log)"
  }
  Write-Output ""
  Write-Output "Do NOT compare these against a clean baseline. Re-run at a lower -MaxWorkers, or pass"
  Write-Output "-AllowDegraded to accept the rows anyway (they stay flagged in the JSONL health block)."
  Write-Output "results file: $outPath"
  if (-not $AllowDegraded) { Exit-Lock -Resource sweep -Quiet; exit 1 }
}

$failed = @($rows | Where-Object { -not $_.ok })
if ($failed.Count -gt 0) {
  Write-Output "=== $($failed.Count) FAILED ==="
  foreach ($f in $failed) {
    Write-Output "  $($f.variant) seed$($f.seed) r$($f.replica)  log=$($f.log)"
    if ($f.err) { Write-Output "    $($f.err)" }
  }
  Write-Output "results file: $outPath"
  Exit-Lock -Resource sweep -Quiet
  exit 1
}
Exit-Lock -Resource sweep -Quiet
Write-Output "all matches ok and clean -> $outPath"
Write-Output "next:  python tools/score/compare.py --results `"$outPath`""
