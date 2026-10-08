# Single automated verifier. Replaces the 5-parallel-human-verifier fan-out that
# never closed (reviewer_m1_1/2 + challenger_m1_1/2 + auditor_m1_1, zero
# verdicts): one script owns the mechanical gates, one human owns judgment.
#
#   powershell -File tools/dev/verify.ps1                 # roadmap + counts + docs
#   powershell -File tools/dev/verify.ps1 -Results results\sweep.jsonl  # + rig schema gate
#
# Gates: (1) docs/nav/roadmap.html matches nav source, (2) suite counts in
# sync (measured XML), (3) docs-contract holds on the working tree,
# (4, optional) rig JSONL rows carry schemaVersion + health block.
# Exit 0 all PASS, 1 any FAIL. PowerShell 5.1 only.
param([string]$Results = "")
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)   # TitanRoboticsBuildSeason
$repoRoot = Split-Path -Parent $root
$pass = 0; $fail = 0
function Gate([string]$Name, [scriptblock]$Body) {
  Write-Output "--- gate: $Name ---"
  try { & $Body; Write-Output "[verify] PASS: $Name"; $script:pass++ }
  catch { Write-Output ("[verify] FAIL: {0}: {1}" -f $Name, $_.Exception.Message); $script:fail++ }
}

Gate "roadmap matches nav source" {
  $out = @(python (Join-Path $root "tools\nav\verify_roadmap.py") 2>&1)
  $out | Select-Object -Last 3 | ForEach-Object { Write-Output "    $_" }
  if ($LASTEXITCODE -ne 0) { throw "verify_roadmap.py exit $LASTEXITCODE" }
}
Gate "suite counts in sync" {
  & powershell -NoProfile -File (Join-Path $root "tools\dev\sync-test-counts.ps1") -Check | ForEach-Object { Write-Output "    $_" }
  if ($LASTEXITCODE -ne 0) { throw "counts gate failed -- see the [counts] lines above (drift, or a partial results dir from a concurrent run)" }
}
Gate "docs contract holds" {
  & powershell -NoProfile -File (Join-Path $root "tools\dev\check-docs.ps1") | ForEach-Object { Write-Output "    $_" }
  if ($LASTEXITCODE -ne 0) { throw "contract violation (see [docs] FAIL lines)" }
}
if ($Results) {
  Gate "rig JSONL schema + health" {
    $p = if ([System.IO.Path]::IsPathRooted($Results)) { $Results } else { Join-Path $root $Results }
    if (-not (Test-Path $p)) { throw "no such file: $p" }
    $n = 0; $noSchema = 0; $noHealth = 0
    foreach ($ln in (Get-Content $p)) {
      if (-not $ln.Trim()) { continue }
      $n++
      if ($ln -notmatch '"schemaVersion"') { $noSchema++ }
      if ($ln -notmatch '"loopOverruns"') { $noHealth++ }
    }
    Write-Output "    rows=$n missing-schema=$noSchema missing-health=$noHealth"
    if ($n -eq 0) { throw "empty results file" }
    if ($noSchema -gt 0 -or $noHealth -gt 0) { throw "$noSchema rows lack schemaVersion, $noHealth lack LoopHealth block (compare.py refuses v1)" }
  }
}
Write-Output ("=== verify: {0} PASS / {1} FAIL ===" -f $pass, $fail)
if ($fail -gt 0) { exit 1 }
exit 0
