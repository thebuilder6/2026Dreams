---
title: Repo Landing Page
audience: [human, ai]
owner: programming-leads
last_verified: 2026-09-29
status: authoritative
---

# 2026Dreams — FRC Team 8334 (2026 season)

GradleRIO robot project lives in `TitanRoboticsBuildSeason/` — run all Gradle commands from there, not the repo root.

Start here:

- `AGENTS.md` — build commands, generated-code ban, repo conventions (read first).
- `TitanRoboticsBuildSeason/docs/INDEX.md` — map of all durable guides (read second).
- `KNOWN_ISSUES.md` — open issues, roadmap, and test status (check before adding work).
- `TitanRoboticsBuildSeason/README.md` — platform quickstart (compile/sim/test/deploy).
- `TitanRoboticsBuildSeason/docs/CHANGELOG.md` — agent-maintained per-change log.

Quick build (Windows PowerShell, from `TitanRoboticsBuildSeason/`):

```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
.\gradlew compileJava --offline
.\gradlew test --offline --no-daemon
```
