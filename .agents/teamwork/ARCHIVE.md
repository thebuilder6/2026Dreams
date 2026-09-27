# .agents/teamwork — Archive Policy (2026-09-26)

Transient orchestration scratch. Not durable spec. AI: do not cite as authority;
durable knowledge lives in `TitanRoboticsBuildSeason/docs/INDEX.md` + root `AGENTS.md`.

## Keep (history)

- `ORIGINAL_REQUEST.md` — the Sep 25 overhaul request + acceptance criteria.
- `orchestrator_1/PROJECT.md` — feature inventory (20 items), milestones M1–M5.
- `orchestrator_1/plan.md` — phase plan.
- `worker_*/handoff.md`, `explorer_*/handoff.md`, `sentinel*/handoff.md` — outcome summaries only.

## Scratch (safe to prune after 30 days / on lead approval)

- `*/BRIEFING.md`, `*/DISPATCH.md`, `*/progress.md`, `*/GATE_STATUS.md`, `*/context.md`
- `explorer_docs_1/audit_raw.json` (~670 KB), `per_file_summary.json` (~128 KB),
  `type_catalog.json`, all `*.py` audit scripts (`audit_script.py`,
  `build_handoff_report.py`, `generate_per_file_report.py`, etc.)

## Rules

1. New agent runs write handoffs only; no new `BRIEFING/DISPATCH` sprawl.
2. Promote durable findings to `docs/CHANGELOG.md` + relevant guide, then delete scratch.
3. Never reference scratch JSONs/py as spec in code or docs.
