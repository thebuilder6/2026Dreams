# Shared lock primitives so concurrent agents cannot corrupt each other's work.
#
# WHY THIS EXISTS
# ---------------
# Every agent in this repo shares ONE working tree, so these resources are
# genuinely single-tenant even though nothing enforces that:
#
#   build/ + .gradle/     Two `gradlew test` runs clobber each other's
#                         build/test-results/*.xml. The "360 / 361 tests, N
#                         failing" baseline quoted across AGENTS.md,
#                         KNOWN_ISSUES.md and docs/CHANGELOG.md is read out of
#                         those XMLs, so a clobber silently invalidates it.
#   NT4 5810 / WebServer
#   5800 / CameraServer
#   1181-1182            Hardcoded constants. CameraServer.kBasePort and
#                         NetworkTableInstance.kDefaultPort3/4 are public static
#                         final with no env override (verified against wpilibj
#                         2026.2.1 / wpiapi-java 26.1.0; see the header of
#                         tools/score/sweep.ps1 lines 17-50). One GUI sim, max.
#   CPU                  Silent. tools/score/sweep.ps1 measures the curve on
#                         this machine: 2- and 4-wide are clean, 6-wide is not.
#                         A concurrent compile does not fail a sweep, it makes
#                         the sweep produce a DIFFERENT match -- which is why
#                         Sim/LoopHealth exists and why sweep fails degraded
#                         rows rather than trusting them.
#
# The old recovery advice ("kill stray java processes", AGENTS.md) was itself
# the interrupt: it cannot distinguish a leaked JVM from another agent's live
# test run or SimGUI. Layer A removed the worst of that; this module is what
# stops the contention instead of cleaning it up afterwards.
#
# DESIGN NOTES
# ------------
# * PowerShell 5.1 only. This repo pins 5.1 (see the Start-Process note in
#   tools/score/sweep.ps1), so no ternaries, no -Parallel, no Start-ThreadJob.
# * Mutual exclusion is an exclusive file create, not a mutex: it is
#   inspectable with Get-Content, survives across shells, and does not need a
#   kernel object name that would collide with a stale kernel namespace.
# * Liveness is anchored on PID *plus* process start time. A bare PID is not
#   enough -- Windows recycles PIDs, and a dead sim whose PID got reassigned to
#   an unrelated process would look alive forever and deadlock the lock.
# * Liveness is "ANY anchor PID alive", because the caller may be a launcher
#   that exits while the work it started keeps running. launch-gui.ps1 uses
#   -NoExit in a detached window, so anchoring on the launcher's own $PID would
#   free the lock the moment the launcher returned. Callers pass the PID of the
#   thing they actually care about.
# * Stale locks are reaped, because agents die mid-task (see
#   .agents/teamwork/sentinel/BRIEFING.md: an orchestrator terminated on quota
#   exhaustion). A lock that outlives its owner is worse than no lock.

Set-StrictMode -Version Latest

# Which resources collide, and why. Conflicts are symmetric and listed on both
# sides deliberately -- this table is the single place that encodes the matrix,
# so `status.ps1` and `Enter-Lock` cannot disagree about it.
$script:ResourceTable = @{
    'gradle-build' = @{
        conflicts        = @('gradle-build')
        defaultTimeoutSec = 900
        description      = 'compileJava / test / dumpSimLaunch -- shared build/ and .gradle/ state'
    }
    'sim-gui'      = @{
        conflicts        = @('sim-gui', 'sweep')
        defaultTimeoutSec = 1800
        description      = 'simulateJava / SimGUI -- owns NT4 5810, WebServer 5800, CameraServer 1181-1182'
    }
    'sweep'        = @{
        conflicts        = @('sweep', 'sim-gui')
        defaultTimeoutSec = 3600
        description      = 'tools/score/sweep.ps1 -- parallel headless matches, gates on LoopHealth'
    }
    'deploy'       = @{
        conflicts        = @('deploy')
        defaultTimeoutSec = 300
        description      = 'gradlew deploy -- one RoboRIO target, last writer wins'
    }
}

$script:PollSec = 5

function Get-LockDirectory {
    <#
      .SYNOPSIS
        Resolves <gradle-root>/.locks, creating it if absent.
      .NOTES
        In-repo rather than LOCALAPPDATA so a human can go look at it, which
        matters when a run has to be diagnosed after the fact. It is
        gitignored. git clean -xdf wipes it, which is harmless: an absent lock
        directory reads as "nothing held" and everything re-acquires cleanly.
    #>
    [CmdletBinding()]
    param()
    $root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)   # TitanRoboticsBuildSeason
    $dir = Join-Path $root '.locks'
    if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Force -Path $dir | Out-Null }
    return $dir
}

function Get-OwnerId {
    <#
      .SYNOPSIS
        Best-effort identity for whoever is holding a lock.
      .NOTES
        This is diagnostic only. It must never be a reason a lock fails to
        acquire, so every branch has a non-empty fallback.
    #>
    [CmdletBinding()]
    param()
    if ($env:OPENCODE_SESSION_ID) { return "session:$($env:OPENCODE_SESSION_ID)" }
    if ($env:USERDOMAIN -and $env:USERNAME) { return "$($env:USERDOMAIN)\$($env:USERNAME)" }
    if ($env:USERNAME) { return "user:$($env:USERNAME)" }
    return "unknown-owner"
}

function Get-ProcessStartTicks {
    [CmdletBinding()]
    param([int]$Id)
    $p = Get-Process -Id $Id -ErrorAction SilentlyContinue
    if (-not $p) { return $null }
    try { return $p.StartTime.ToUniversalTime().Ticks } catch { return $null }
}

function Test-ProcessAlive {
    <#
      .SYNOPSIS
        True only if $Id is running AND was started at $StartTicks.
      .NOTES
        The start-time check is what makes PID recycling detectable. Start ticks
        are an integer round-trip, so there is no timestamp formatting to get
        subtly wrong across locales or DST.
    #>
    [CmdletBinding()]
    param([int]$Id, [string]$StartTicks)
    $ticks = Get-ProcessStartTicks -Id $Id
    if ($null -eq $ticks) { return $false }
    if ($StartTicks) {
        $expected = 0L
        if (-not [long]::TryParse($StartTicks, [ref]$expected)) { return $false }
        if ($ticks -ne $expected) { return $false }   # PID was recycled
    }
    return $true
}

function ConvertTo-LockRecord {
    [CmdletBinding()]
    param([string[]]$Lines)
    $rec = @{ pids = @(); startTicks = @() }
    foreach ($line in $Lines) {
        if ($line -match '^\s*([^=]+)=(.*)$') { $rec[$matches[1].Trim()] = $matches[2] }
    }
    if ($rec.ContainsKey('pids') -and $rec['pids'])       { $rec.pids       = $rec['pids'] -split ',' }
    if ($rec.ContainsKey('startTicks') -and $rec['startTicks']) { $rec.startTicks = $rec['startTicks'] -split ',' }
    return $rec
}

function Read-Lock {
    <#
      .SYNOPSIS
        Reads a lock file into a record, or $null if it does not exist.
    #>
    [CmdletBinding()]
    param([string]$Resource)
    $path = Join-Path (Get-LockDirectory) "$Resource.lock"
    if (-not (Test-Path $path)) { return $null }
    $rec = ConvertTo-LockRecord -Lines (Get-Content $path)
    $rec['path'] = $path
    $rec['ageSec'] = [int]((Get-Date) - (Get-Item $path).LastWriteTime).TotalSeconds
    return $rec
}

function Test-LockStale {
    <#
      .SYNOPSIS
        True when every anchor PID in a lock record is gone.
      .NOTES
        No time-based expiry on its own. A heartbeat older than $StaleSec is NOT
        sufficient evidence of death here, because the common case is a
        legitimately long run: a practice match or a 4-wide sweep holds its lock
        for many minutes with no shell left alive to touch a heartbeat. Killing
        those on a timer would be a self-inflicted interrupt, which is the exact
        failure this module exists to prevent. Dead PID is the only trustworthy
        signal; if an agent dies holding a lock, its PIDs are gone.
    #>
    [CmdletBinding()]
    param($Record)
    if ($null -eq $Record) { return $false }
    $pids = @($Record['pids'])
    if ($pids.Count -eq 0) { return $true }   # nothing anchors it: unusable
    $ticks = @($Record['startTicks'])
    for ($i = 0; $i -lt $pids.Count; $i++) {
        $st = $null
        if ($i -lt $ticks.Count) { $st = $ticks[$i] }
        $pidInt = 0
        if ([int]::TryParse($pids[$i], [ref]$pidInt)) {
            if (Test-ProcessAlive -Id $pidInt -StartTicks $st) { return $false }
        }
    }
    return $true
}

function Remove-StaleLock {
    <#
      .SYNOPSIS
        Deletes a lock whose owner is provably gone, loudly.
    #>
    [CmdletBinding()]
    param($Record, [string]$Resource)
    Write-Warning ("[lock] STALE: reclaiming '{0}' from {1} (pid {2}, held {3}s) -- owner process is gone." -f `
        $Resource, $Record['owner'], (@($Record['pids']) -join ','), $Record['ageSec'])
    Remove-Item $Record['path'] -Force -ErrorAction SilentlyContinue
}

function Test-LockOwnedByMe {
    <#
      .SYNOPSIS
        True when a lock record belongs to this owner and overlaps our anchors.
      .DESCRIPTION
        Both conditions are required. Owner id alone is too weak: every agent on
        this machine resolves to the same Windows account, so "same owner" is
        not evidence that it is the same agent. The PID overlap is what
        distinguishes a nested acquire by the same live process from a genuine
        second agent.
    #>
    [CmdletBinding()]
    param($Record, [int[]]$AnchorPid = @())
    if ($null -eq $Record) { return $false }
    if ((Get-OwnerId) -ne $Record['owner']) { return $false }
    $mine = if ($AnchorPid.Count -gt 0) { $AnchorPid } else { @($PID) }
    foreach ($p in @($Record['pids'])) {
        $pidInt = 0
        if ([int]::TryParse($p, [ref]$pidInt) -and ($mine -contains $pidInt)) { return $true }
    }
    return $false
}

function Enter-Lock {
    <#
      .SYNOPSIS
        Acquires one or more resources, waiting up to a timeout.
      .DESCRIPTION
        Waits with a visible heartbeat (one line per poll, never a silent hang),
        then throws naming the holder. Never auto-kills and never steals a live
        lock -- a wrong guess here corrupts a run, which is worse than waiting.
      .PARAMETER AnchorPid
        PIDs whose continued existence keeps the lock held. Defaults to the
        current process. Pass the PID of long-lived work you launched (a sim)
        rather than a launcher that returns immediately.
    #>
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string[]]$Resource,
        [int]$TimeoutSec = -1,
        [int[]]$AnchorPid = @(),
        [string]$Reason = "",
        [switch]$Quiet
    )

    $unknown = @($Resource | Where-Object { -not $script:ResourceTable.ContainsKey($_) })
    if ($unknown.Count -gt 0) {
        throw ("[lock] unknown resource(s): {0}. Known: {1}" -f `
            ($unknown -join ', '), (($script:ResourceTable.Keys | Sort-Object) -join ', '))
    }

    # Sorted + deduped so two callers asking for a pair in different orders
    # cannot deadlock against each other.
    $want = @($Resource | Sort-Object -Unique)
    if ($want.Count -eq 0) { return @() }

    if ($TimeoutSec -lt 0) {
        $max = 0
        foreach ($r in $want) {
            if ($script:ResourceTable[$r].defaultTimeoutSec -gt $max) {
                $max = $script:ResourceTable[$r].defaultTimeoutSec
            }
        }
        $TimeoutSec = $max
    }

    $anchors = if ($AnchorPid.Count -gt 0) { $AnchorPid } else { @($PID) }
    $ticks = @()
    foreach ($p in $anchors) {
        $t = Get-ProcessStartTicks -Id $p
        $ticks += $(if ($null -eq $t) { '' } else { "$t" })
    }

    $owner = Get-OwnerId
    $dir = Get-LockDirectory
    $acquired = @()
    $start = Get-Date
    $announced = $false

    try {
        while ($true) {
            # Everything that could possibly stand in our way.
            $blockers = @{}
            foreach ($r in $want) { $blockers[$r] = $true }
            foreach ($r in $want) {
                foreach ($c in $script:ResourceTable[$r].conflicts) { $blockers[$c] = $true }
            }

            # Reap provably-dead locks on any of them.
            foreach ($b in $blockers.Keys) {
                $rec = Read-Lock -Resource $b
                if ($rec -and (Test-LockStale -Record $rec)) { Remove-StaleLock -Record $rec -Resource $b }
            }

            # EXCLUSION. The reap pass above only cleans up dead owners; on its
            # own it provides no mutual exclusion at all, and a conflicting
            # resource that is genuinely held would simply be walked past. So
            # check explicitly for a live holder of anything we conflict with.
            $failed = $null
            $alreadyMine = @()
            foreach ($b in $blockers.Keys) {
                $rec = Read-Lock -Resource $b
                if (-not $rec) { continue }
                if (Test-LockStale -Record $rec) { continue }     # reaped above
                if (Test-LockOwnedByMe -Record $rec -AnchorPid $anchors) {
                    # Re-entrant: we already hold it. Not a conflict, and must
                    # not become a self-deadlock, since a lock is held for the
                    # duration of a script and scripts nest.
                    if ($want -contains $b) { $alreadyMine += $b }
                    continue
                }
                $failed = $b
                break
            }

            $taken = @()
            if ($null -eq $failed) {
                # Try to take all of them, or none.
                foreach ($r in $want) {
                    if ($alreadyMine -contains $r) { continue }
                    $path = Join-Path $dir "$r.lock"
                    try {
                        $fs = [System.IO.File]::Open($path, [System.IO.FileMode]::CreateNew,
                                                    [System.IO.FileAccess]::Write, [System.IO.FileShare]::None)
                    } catch [System.IO.IOException] {
                        $failed = $r
                        break
                    }
                    $body = @(
                        "resource=$r"
                        "owner=$owner"
                        "pids=$($anchors -join ',')"
                        "startTicks=$($ticks -join ',')"
                        "acquired=$((Get-Date).ToUniversalTime().ToString('o'))"
                        "heartbeat=$((Get-Date).ToUniversalTime().ToString('o'))"
                        "reason=$Reason"
                    ) -join "`n"
                    $bytes = [System.Text.Encoding]::UTF8.GetBytes($body)
                    $fs.Write($bytes, 0, $bytes.Length)
                    $fs.Close()
                    $taken += $r
                }
            }

            if ($null -eq $failed) {
                $acquired = @($taken + $alreadyMine | Sort-Object -Unique)
                if (-not $Quiet) {
                    foreach ($r in $taken) {
                        Write-Output ("[lock] acquired {0} (owner {1}, pid {2}){3}" -f `
                            $r, $owner, ($anchors -join ','), $(if ($Reason) { " -- $Reason" } else { "" }))
                    }
                }
                return $acquired
            }

            # Roll back a partial take so we never hold one resource while
            # waiting for another.
            foreach ($r in $taken) { Remove-Item (Join-Path $dir "$r.lock") -Force -ErrorAction SilentlyContinue }

            $held = (Get-Date) - $start
            if ($held.TotalSeconds -ge $TimeoutSec) {
                $rec = Read-Lock -Resource $failed
                $who = if ($rec) { "$($rec['owner']) (pid $(@($rec['pids']) -join ','), held $([int]$rec['ageSec'])s)" }
                       else { "an unknown process" }
                $why = if ($rec -and $rec['reason']) { " -- reason: $($rec['reason'])" } else { "" }
                # Built as an array of individually-formatted lines, then joined.
                # Do NOT concatenate strings and apply -f to the result: the
                # format operator binds looser than +, so only the last
                # fragment would ever be substituted and the rest would ship
                # literal "{0}" placeholders to the user.
                $msg = @(
                    ("[lock] TIMEOUT after {0}s waiting for '{1}'." -f [int]$TimeoutSec, $failed)
                    ("  held by : {0}{1}" -f $who, $why)
                    ("  inspect : powershell -File tools/lock/status.ps1")
                    "  This is a wait, not a corruption. Do NOT kill the holder -- that is the bug this"
                    "  module exists to prevent. Either wait, or run your work on a different resource."
                ) -join [Environment]::NewLine
                throw $msg
            }

            if (-not $Quiet -and (-not $announced -or [int]$held.TotalSeconds % 15 -eq 0)) {
                $rec = Read-Lock -Resource $failed
                $who = if ($rec) { "$($rec['owner']) pid $(@($rec['pids']) -join ',')" } else { "unknown" }
                Write-Output ("[lock] waiting {0}s for '{1}' (held by {2})..." -f `
                    [int]$held.TotalSeconds, $failed, $who)
                $announced = $true
            }

            # Sleep the remainder, not a full poll, so the timeout is honoured
            # to within a second instead of overshooting by up to PollSec.
            $remaining = $TimeoutSec - $held.TotalSeconds
            $nap = if ($remaining -lt $script:PollSec) { [Math]::Max(1, $remaining) } else { $script:PollSec }
            Start-Sleep -Seconds $nap
        }
    } catch {
        foreach ($r in $acquired) { Remove-Item (Join-Path $dir "$r.lock") -Force -ErrorAction SilentlyContinue }
        throw
    }
}

function Exit-Lock {
    <#
      .SYNOPSIS
        Releases resources this process holds. Leaves other owners' locks alone.
    #>
    [CmdletBinding()]
    param([Parameter(Mandatory = $true)][string[]]$Resource, [switch]$Quiet)
    $dir = Get-LockDirectory
    $owner = Get-OwnerId
    foreach ($r in $Resource) {
        $path = Join-Path $dir "$r.lock"
        if (-not (Test-Path $path)) { continue }
        $rec = ConvertTo-LockRecord -Lines (Get-Content $path)
        # Only ever delete our own. If a stale lock was legitimately reclaimed
        # and the old owner came back, it must not free the new holder.
        if ($rec['owner'] -ne $owner) {
            if (-not $Quiet) {
                Write-Warning ("[lock] not releasing '{0}': held by {1}, we are {2}." -f $r, $rec['owner'], $owner)
            }
            continue
        }
        Remove-Item $path -Force -ErrorAction SilentlyContinue
        if (-not $Quiet) { Write-Output "[lock] released $r" }
    }
}

function Get-LockStatus {
    <#
      .SYNOPSIS
        One row per known resource, held or not.
    #>
    [CmdletBinding()]
    param()
    $rows = @()
    foreach ($r in ($script:ResourceTable.Keys | Sort-Object)) {
        $rec = Read-Lock -Resource $r
        $rows += [pscustomobject]@{
            Resource  = $r
            State     = if (-not $rec) { 'free' }
                       elseif (Test-LockStale -Record $rec) { 'STALE (owner gone)' }
                       else { 'HELD' }
            Owner     = if ($rec) { $rec['owner'] } else { '' }
            Pids      = if ($rec) { (@($rec['pids']) -join ',') } else { '' }
            HeldSec   = if ($rec) { $rec['ageSec'] } else { 0 }
            Reason    = if ($rec -and $rec['reason']) { $rec['reason'] } else { '' }
            Conflicts = ($script:ResourceTable[$r].conflicts -join ',')
        }
    }
    return $rows
}

function Test-DashboardRunning {
    <#
      .SYNOPSIS
        Returns the Elastic / AdvantageScope / SimGUI processes that are up.
      .DESCRIPTION
        A live dashboard is not a port problem, it is a data problem.
        RobotBase calls NetworkTableInstance.startServer() BEFORE robotInit runs,
        so a headless rig worker listens on NT4 5810 for a window that robotInit
        can only close afterwards. Any dashboard on the machine auto-connects in
        that window and then shares the robot's whole NT namespace, including the
        Auto Mission chooser read at autonomousInit. That is silent contamination
        and it happened for real: an elastic_dashboard left running since 10:52
        auto-connected to every worker of the archived 12-way baseline. See
        KNOWN_ISSUES.md section E.
        SimGUI counts because it is an NT client too.
      .NOTES
        This is the single definition. tools/score/sweep.ps1 calls it rather than
        repeating the pattern, and status.ps1 surfaces it, so the rule cannot
        drift between the tool that enforces it and the tool that reports it.
    #>
    [CmdletBinding()]
    param()
    return @(Get-Process -ErrorAction SilentlyContinue |
        Where-Object { $_.ProcessName -match '^(elastic|Elastic|SimGUI|advantagescope)' })
}

Export-ModuleMember -Function Enter-Lock, Exit-Lock, Read-Lock, Test-LockStale, Get-LockStatus,
    Get-LockDirectory, Get-OwnerId, Test-DashboardRunning, Get-ProcessStartTicks, Test-ProcessAlive,
    Test-LockOwnedByMe
