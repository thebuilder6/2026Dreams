# Reviews an AdvantageKit .wpilog with LogReview.java (wpiutil only, offline-safe).
# Usage: review-wpilog.ps1 <file.wpilog>
param([string]$Log = "")
$ErrorActionPreference = "Stop"
if ($Log -eq "") { Write-Output "usage: review-wpilog.ps1 <file.wpilog>"; exit 2 }
$jdk = "C:\Users\Public\wpilib\2026\jdk"
$jar = "C:\Users\Public\wpilib\2026\maven\edu\wpi\first\wpiutil\wpiutil-java\2026.2.1\wpiutil-java-2026.2.1.jar"
$src = Join-Path $PSScriptRoot "LogReview.java"
# Per-PID class output dir. This used to be a fixed "logreview-classes" in
# %TEMP%, shared by every invocation: two agents reviewing logs at the same time
# had one javac writing LogReview.class while the other's java was loading it.
$out = Join-Path ([System.IO.Path]::GetTempPath()) ("logreview-classes-" + $PID)
New-Item -ItemType Directory -Force -Path $out | Out-Null
try {
  $class = Join-Path $out "LogReview.class"
  if (-not (Test-Path $class) -or (Get-Item $src).LastWriteTime -gt (Get-Item $class).LastWriteTime) {
    & "$jdk\bin\javac.exe" -cp $jar -d $out $src
    if ($LASTEXITCODE -ne 0) { exit 1 }
  }
  & "$jdk\bin\java.exe" -cp ($out + ";" + $jar) LogReview $Log
} finally {
  # Own scratch dir, so cleaning it up cannot delete someone else's classes.
  Remove-Item $out -Recurse -Force -ErrorAction SilentlyContinue
}
