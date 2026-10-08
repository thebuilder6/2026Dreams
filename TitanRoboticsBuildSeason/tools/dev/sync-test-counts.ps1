# Single source of truth for suite counts. Replaces the hand-synced numbers that
# drifted three times in the last 30 commits (217-from-stale-XML, wrong 342,
# 360-vs-361 artifact).
#
# Counts from Get-ChildItem -Recurse, never a PowerShell ** glob (which
# undercounts), and reads the measured totals out of build/test-results XML.
#
#   powershell -File tools/dev/sync-test-counts.ps1          # patch drifted files
#   powershell -File tools/dev/sync-test-counts.ps1 -Check  # CI mode: exit 1 on drift
#
# Patched targets (measured XML numbers + today's date):
#   AGENTS.md (root)                          "(N test files, M tests as of DATE ...)"
#   KNOWN_ISSUES.md (root) header             "DATE, offline, --no-daemon): N result files / M tests, `F FAILURES`"
#   docs/AUTONOMOUS_GUIDE.md verification     "(N result files / M tests, 0 failures, DATE)"
# PowerShell 5.1 only.
param([switch]$Check)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)   # TitanRoboticsBuildSeason
$repoRoot = Split-Path -Parent $root
$today = (Get-Date).ToString('yyyy-MM-dd')

# --- on-disk (Get-ChildItem -Recurse; ** globs undercount) --------------------
$testDir = Join-Path $root "src\test\java"
$testFiles = @(Get-ChildItem -Path $testDir -Recurse -Filter "*Test.java" | Where-Object { -not $_.PSIsContainer })
$diskFiles = $testFiles.Count
$diskTests = 0
foreach ($f in $testFiles) {
  $diskTests += @(Select-String -Path $f.FullName -Pattern '@Test\b' -SimpleMatch:$false).Count
}

# --- measured (XML, the only number that can claim green) ----------------------
$xmlDir = Join-Path $root "build\test-results\test"
$xmlFiles = @()
if (Test-Path $xmlDir) { $xmlFiles = @(Get-ChildItem -Path $xmlDir -Filter "*.xml" -File) }
if ($xmlFiles.Count -eq 0) {
  Write-Output "[counts] no XML in build/test-results/test -- run the suite first (under the gradle-build lock); nothing to sync against."
  exit 1
}
$xmlTests = 0; $xmlFail = 0; $xmlErr = 0; $xmlSkip = 0
foreach ($x in $xmlFiles) {
  try {
    [xml]$doc = Get-Content $x.FullName
    $ts = $doc.testsuite
    if ($ts) {
      $xmlTests += [int]$ts.tests; $xmlFail += [int]$ts.failures
      $xmlErr += [int]$ts.errors; $xmlSkip += [int]$ts.skipped
    }
  } catch { Write-Warning "[counts] unreadable XML: $($x.Name)" }
}
$bad = $xmlFail + $xmlErr
if ($xmlFiles.Count -ne $diskFiles) {
  Write-Output ("[counts] REFUSING: xml has {0} files but {1} *Test.java exist on disk." -f $xmlFiles.Count, $diskFiles)
  Write-Output "[counts] The results dir is partial: another agent's run is writing it now, or the last run died mid-write"
  Write-Output "[counts] (shared-tree clobber -- this is why test runs hold the gradle-build lock). Wait, then re-run."
  Write-Output "[counts] If a run just finished cleanly and this persists, the tree has fewer test classes than before; delete stale XML and re-run."
  exit 1
}
$verdict = if ($bad -eq 0) { "GREEN" } else { "RED" }
Write-Output ("[counts] disk: {0} files / {1} @Test | xml: {2} files / {3} tests ({4} fail, {5} err, {6} skip) -> {7}" -f `
  $diskFiles, $diskTests, $xmlFiles.Count, $xmlTests, $xmlFail, $xmlErr, $xmlSkip, $verdict)
if ($xmlTests -ne $diskTests) {
  Write-Warning ("[counts] xml tests ({0}) != on-disk @Test ({1}): parameterized/disabled tests or a stale results dir. The XML number is the measured one." -f $xmlTests, $diskTests)
}

$drift = 0
function Sync-File([string]$Path, [string]$Pattern, [string]$Replacement, [string]$Label) {
  $text = Get-Content $Path -Raw
  if ($text -match $Pattern) {
    $new = $text -replace $Pattern, $Replacement
    if ($new -ne $text) {
      if ($Check) { Write-Output "[counts] DRIFT: $Label"; $script:drift++ }
      else { Set-Content -Path $Path -Value $new -NoNewline; Write-Output "[counts] patched: $Label" }
    } else { Write-Output "[counts] ok: $Label" }
  } else { Write-Warning "[counts] pattern not found: $Label (file changed shape? update this script)" }
}

# AGENTS.md build line: measured numbers (it claims "green on clean re-run").
Sync-File (Join-Path $repoRoot "AGENTS.md") `
  '\(\d+ test files, \d+ tests as of \d{4}-\d{2}-\d{2}' `
  "($($xmlFiles.Count) test files, $xmlTests tests as of $today" `
  "AGENTS.md -> $($xmlFiles.Count) files / $xmlTests tests / $today"

# KNOWN_ISSUES.md header: rebuild only the leading stamp, history after it stays.
# Built by concatenation in single quotes so the .NET $1/$2/$4 group refs
# survive PowerShell interpolation (backtick-escaping them inline is fragile).
$kiReplacement = '${1}' + $today + '${2}' + $xmlFiles.Count + ' result files / ' + $xmlTests + ' tests, `' + $bad + ' FAILURES`' + '${4}' + $verdict
Sync-File (Join-Path $repoRoot "KNOWN_ISSUES.md") `
  '(\*\*Latest clean run \()\d{4}-\d{2}-\d{2}(, offline, `--no-daemon`\): )\d+ result files / \d+ tests, `(\d+) FAILURES`([^\*]*the suite is )[A-Z]+' `
  $kiReplacement `
  "KNOWN_ISSUES.md header -> $($xmlFiles.Count) / $xmlTests / $bad failures ($verdict)"

# Per-guide verification stamps of the same shape.
Sync-File (Join-Path $root "docs\AUTONOMOUS_GUIDE.md") `
  '\(\d+ result files / \d+ tests, \d+ failures, \d{4}-\d{2}-\d{2}\)' `
  "($($xmlFiles.Count) result files / $xmlTests tests, $bad failures, $today)" `
  "AUTONOMOUS_GUIDE.md verification stamp"

if ($Check -and $drift -gt 0) { Write-Output "[counts] $drift file(s) drifted; run without -Check to patch."; exit 1 }
if ($bad -gt 0) { Write-Output "[counts] suite is RED ($bad failures+errors); numbers synced but do not claim green."; exit 1 }
Write-Output "[counts] done"
exit 0
