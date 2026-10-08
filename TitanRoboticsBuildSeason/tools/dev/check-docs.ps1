# Docs-Contract gate (AGENTS.md strict contract): a behavior-affecting change must
# ship its docs in the same change, and a new guide is not done until INDEX
# lists it (the KNOWLEDGE_MODEL.md dangling-link lesson).
#
#   powershell -File tools/dev/check-docs.ps1                 # working tree vs HEAD
#   powershell -File tools/dev/check-docs.ps1 -Against HEAD~1 # last commit
#
# FAILS (exit 1) on: new guide with no INDEX row, new guide with bad
# frontmatter, behavior-affecting change with no CHANGELOG touch.
# WARNS (exit 0) on: test-only change (premise-first rule), no last_verified
# bump alongside a behavior change.
# PowerShell 5.1 only.
param([string]$Against = "")
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)   # TitanRoboticsBuildSeason
$repoRoot = Split-Path -Parent $root

function Invoke-RepoGit([string[]]$GitArgs) {
  # Read-only git plumbing with CRLF warnings off: this repo's autocrlf emits
  # a "LF will be replaced by CRLF" warning per touched file on stderr, and
  # under $ErrorActionPreference = "Stop" that warning is a terminating error.
  # -c core.autocrlf=false changes nothing about name/status output.
  return @(git -c core.autocrlf=false -C $repoRoot @GitArgs 2>$null)
}

if ($Against) { $changed = Invoke-RepoGit @('diff', '--name-only', $Against) }
else {
  $tracked = Invoke-RepoGit @('diff', '--name-only', 'HEAD')
  $untracked = Invoke-RepoGit @('ls-files', '--others', '--exclude-standard')
  $changed = @($tracked + $untracked | Sort-Object -Unique)
}
# Normalize to repo-root-relative forward slashes.
$changed = @($changed | ForEach-Object { $_ -replace '\\', '/' } | Where-Object { $_ })
if ($changed.Count -eq 0) { Write-Output "[docs] no changes; nothing to check."; exit 0 }

$fails = @()
$warns = @()
$pre = 'TitanRoboticsBuildSeason/'

function Is-New([string]$f) {
  # Untracked, or added in the diff (covers -Against ranges too).
  $u = (Invoke-RepoGit @('ls-files', '--others', '--exclude-standard')) -contains ($f -replace '/', '\')
  if ($u) { return $true }
  if ($Against) { $st = Invoke-RepoGit @('diff', '--name-status', $Against) }
  else { $st = Invoke-RepoGit @('status', '--porcelain') }
  foreach ($ln in $st) { if ($ln -match '^[A\?]' -and $ln -match [regex]::Escape($f)) { return $true } }
  return $false
}

# --- new guides need frontmatter + an INDEX row --------------------------------
$newGuides = @($changed | Where-Object { $_ -like "$pre*.md" -and $_ -notlike "*/CHANGELOG.md" -and $_ -notlike "*/INDEX.md" -and $_ -notlike "*/_TEMPLATE.md" -and (Is-New $_) })
$indexPath = Join-Path $root "docs\INDEX.md"
$indexText = ""
if (Test-Path $indexPath) { $indexText = Get-Content $indexPath -Raw }
foreach ($g in $newGuides) {
  $abs = Join-Path $repoRoot ($g -replace '/', '\')
  if (-not (Test-Path $abs)) { continue }
  $t = Get-Content $abs -Raw
  foreach ($k in @('title:', 'audience:', 'owner:', 'last_verified:', 'status:')) {
    if ($t -notmatch $k) { $fails += "$g : new guide missing frontmatter key '$k' (copy docs/_TEMPLATE.md)." }
  }
  $base = Split-Path $g -Leaf
  if ($indexText -notmatch [regex]::Escape($base)) { $fails += "$g : new guide has no row in docs/INDEX.md durable-guides table (contract rule 3)." }
}

# --- behavior-affecting edits need a CHANGELOG touch ---------------------------
$behaviorPrefixes = @(
  "$pre`src/main/java/", "$pre`build.gradle", "$pre`settings.gradle",
  "$pre`vendordeps/", "$pre`src/main/deploy/elastic-layout.json"
)
$behavior = @($changed | Where-Object { $hit = $_; @($behaviorPrefixes | Where-Object { $hit -like "$_*" }).Count -gt 0 })
$clChanged = @($changed | Where-Object { $_ -like "$pre*CHANGELOG.md" }).Count -gt 0
if ($behavior.Count -gt 0 -and -not $clChanged) {
  $fails += ("behavior-affecting change ({0} file(s), e.g. {1}) with no docs/CHANGELOG.md touch (contract rule 2)." -f $behavior.Count, $behavior[0])
}

# --- warnings -------------------------------------------------------------------
$testOnly = @($changed | Where-Object { $_ -like "$pre*src/test/*" }).Count -gt 0
if ($testOnly -and $behavior.Count -eq 0) {
  $warns += "test-only change: premise-first rule (AGENTS.md contract 5) -- confirm each failing test was re-run on the current binary before rewriting it."
}
if ($behavior.Count -gt 0 -and $clChanged) {
  $lv = @(Invoke-RepoGit @('diff', 'HEAD', '--', 'TitanRoboticsBuildSeason/docs/', 'AGENTS.md', 'KNOWN_ISSUES.md') | Select-String "last_verified")
  if ($lv.Count -eq 0) { $warns += "behavior change ships a CHANGELOG touch but no last_verified bump; confirm the owning guide was re-verified (contract rule 2)." }
}

foreach ($w in $warns) { Write-Warning "[docs] $w" }
if ($fails.Count -gt 0) { foreach ($f in $fails) { Write-Output "[docs] FAIL: $f" }; exit 1 }
Write-Output ("[docs] OK ({0} changed file(s), {1} behavior-affecting)" -f $changed.Count, $behavior.Count)
Write-Output "[docs] tip: found something surprising? File it in KNOWN_ISSUES.md ([OPEN] + repro) per AGENTS.md contract rule 6 -- the next agent cannot use what you only tell chat."
exit 0
