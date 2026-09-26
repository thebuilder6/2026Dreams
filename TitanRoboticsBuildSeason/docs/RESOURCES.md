---
title: External Resources
audience: [human, ai]
owner: programming-leads
last_verified: 2026-09-26
status: authoritative
---

# External Resources (merged)

Single source for vendor/official docs. Merged from
`src/main/java/frc/robot/Resources.txt` (AI-focused) +
`docs_context/useful_documentation.txt` (creation links).
Check here before web-searching; prefer pinned vendor versions in `vendordeps/`.

## Primary (check first)

- https://docs.wpilib.org/ — WPILib 2026 (Java 17, GradleRIO 2026.2.1)
- https://docs.advantagekit.org/ — AdvantageKit IO abstraction, `@AutoLog`, replay
- https://docs.yagsl.com/ — YAGSL swerve (2026.1.14 pinned)
- https://yet-another-software-suite.github.io/YAGSL/javadocs/ — YAGSL API
- https://choreo.autos/ — Choreo trajectories
- https://api.typesafe.ai/docs — TypeSafe System One endpoint, request/response schema, and model discovery

## Vision / sim

- https://docs.photonvision.org/ — PhotonVision + sim
- https://docs.limelightvision.io/docs/docs-limelight/ — Limelight MegaTag2
- https://shenzhen-robotics-alliance.github.io/maple-sim/ — IronMaple physics

## Hardware / dashboard

- https://codedocs.revrobotics.com/ — REVLib / SparkMax
- https://frc-elastic.gitbook.io/docs — Elastic Dashboard widgets/layout
- https://yet-another-software-suite.github.io/YALL/ + `/javadocs/` — YALL

## Rules

- Don't bump `vendordeps/*.json` without checking Sim compat (see `AGENTS.md`).
- Add new links here when you consult a new doc; don't scatter URLs in code comments.
- `docs_context/useful_documentation.txt` and `Resources.txt` were superseded by
  this file and have since been deleted — add new links only here.
