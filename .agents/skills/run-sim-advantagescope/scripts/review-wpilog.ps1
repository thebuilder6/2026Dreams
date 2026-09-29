# Reviews an AdvantageKit .wpilog with LogReview.java (wpiutil only, offline-safe).
# Usage: review-wpilog.ps1 <file.wpilog>
param([string]$Log = "")
$ErrorActionPreference = "Stop"
if ($Log -eq "") { Write-Output "usage: review-wpilog.ps1 <file.wpilog>"; exit 2 }
$jdk = "C:\Users\Public\wpilib\2026\jdk"
$jar = "C:\Users\Public\wpilib\2026\maven\edu\wpi\first\wpiutil\wpiutil-java\2026.2.1\wpiutil-java-2026.2.1.jar"
$src = Join-Path $PSScriptRoot "LogReview.java"
$out = Join-Path ([System.IO.Path]::GetTempPath()) "logreview-classes"
New-Item -ItemType Directory -Force -Path $out | Out-Null
$class = Join-Path $out "LogReview.class"
if (-not (Test-Path $class) -or (Get-Item $src).LastWriteTime -gt (Get-Item $class).LastWriteTime) {
  & "$jdk\bin\javac.exe" -cp $jar -d $out $src
  if ($LASTEXITCODE -ne 0) { exit 1 }
}
& "$jdk\bin\java.exe" -cp ($out + ";" + $jar) LogReview $Log
