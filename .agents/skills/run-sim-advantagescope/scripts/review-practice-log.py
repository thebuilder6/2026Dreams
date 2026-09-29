#!/usr/bin/env python3
"""Review a captured simulateJava console log for errors and stuck-bot evidence.

Reads the stdout/stderr capture produced by run-practice-match.ps1 (or any
simulateJava console log) and reports:
  - Java exceptions / crashes (FAIL)
  - [AI DIAGNOSTIC] STUCK dumps (ATTENTION, Bot0 pipeline; needs
    Simulation/DebugAI=true during the run)
  - MapleSim brownout spam (ATTENTION; disableBatterySim should keep this at 0)
  - bind() to port 1181 (benign, counted separately, never a failure)

Writes a Markdown report next to the input log and prints a summary table.
Stdlib only.
"""

import argparse
import datetime
import os
import re
import sys

STUCK_RE = re.compile(
    r"\[AI DIAGNOSTIC\] STUCK: Mode: (\S+) Phase: (\S+)"
    r" Pose: \(([0-9.\-]+), ([0-9.\-]+)\)"
    r" Target: \(([0-9.\-]+), ([0-9.\-]+)\) Stall: ([0-9.]+)s"
)
HUBSEED_RE = re.compile(r"\[HubSchedule\] SHIFT 1 seed from AUTO: '(.)' inactive first")
BROWNOUT_RE = re.compile(r"brownout detected", re.IGNORECASE)
BIND1181_RE = re.compile(r"bind\(\) to port 1181 failed")
MISSION_DONE_RE = re.compile(r"AUTO MISSION DONE!!!! ENDED EARLY!!!!")
OVERRUN_RE = re.compile(r"Loop time of 0\.02s overrun")
SIMPERIODIC_RE = re.compile(r"simulationPeriodic\(\): ([0-9.]+)s")
CANSTALE_RE = re.compile(r"CAN frame not received/too-stale")
ERROR_RES = [
    re.compile(r"Exception in thread"),
    re.compile(r"NullPointerException"),
    re.compile(r"ArrayIndexOutOfBoundsException"),
    re.compile(r"EXCEPTION_ACCESS_VIOLATION"),
    re.compile(r"^\s*at frc\.robot\.", re.MULTILINE),
    re.compile(r"^Error at ", re.MULTILINE),
    re.compile(r"hs_err_pid\d+\.log"),
]


def read_text(path):
    with open(path, "rb") as f:
        raw = f.read()
    if raw.startswith(b"\xff\xfe") or raw.startswith(b"\xfe\xff"):
        return raw.decode("utf-16")
    if raw and raw.count(b"\x00") / len(raw) > 0.2:
        return raw.decode("utf-16-le", errors="replace")
    return raw.decode("utf-8", errors="replace")


def review(path):
    text = read_text(path)
    lines = text.splitlines()

    stuck = [m.groups() for m in STUCK_RE.finditer(text)]
    seeds = HUBSEED_RE.findall(text)
    brownouts = len(BROWNOUT_RE.findall(text))
    bind1181 = len(BIND1181_RE.findall(text))
    mission_done = len(MISSION_DONE_RE.findall(text))
    overruns = len(OVERRUN_RE.findall(text))
    sim_periodic_max = max(
        (float(m.group(1)) for m in SIMPERIODIC_RE.finditer(text)), default=0.0
    )
    can_stale = len(CANSTALE_RE.findall(text))
    errors = []
    for rx in ERROR_RES:
        for m in rx.finditer(text):
            lineno = text.count("\n", 0, m.start()) + 1
            line = lines[lineno - 1].strip()[:160]
            if MISSION_DONE_RE.search(line):
                continue  # normal auto-mission unwind, counted separately
            errors.append((lineno, line))

    if errors:
        verdict = "FAIL"
    elif not seeds:
        verdict = "ATTENTION"
    elif stuck or brownouts:
        verdict = "ATTENTION"
    else:
        verdict = "PASS"

    return {
        "file": os.path.basename(path),
        "total_lines": len(lines),
        "verdict": verdict,
        "matches_seen": len(seeds),
        "stuck": stuck,
        "brownouts": brownouts,
        "bind1181": bind1181,
        "mission_done": mission_done,
        "overruns": overruns,
        "sim_periodic_max": sim_periodic_max,
        "can_stale": can_stale,
        "errors": errors,
    }


def render_md(res, drill):
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    L = [
        "# Practice Match Log Review",
        f"**Reviewed**: {now}  ",
        f"**Console log**: `{res['file']}` ({res['total_lines']} lines)  ",
        f"**Drill**: {drill}  ",
        f"**Matches seen** (HubSchedule seeds): {res['matches_seen']}  ",
        f"**Verdict**: **{res['verdict']}**",
        "",
        "## Signals",
        "",
        "| Signal | Count | Status |",
        "| :--- | :--- | :--- |",
        f"| Java exceptions / crashes | {len(res['errors'])} | {'[FAIL]' if res['errors'] else '[PASS]'} |",
        f"| STUCK dumps (Bot0, needs DebugAI) | {len(res['stuck'])} | {'[ATTENTION]' if res['stuck'] else '[PASS]'} |",
        f"| Brownout spam lines | {res['brownouts']} | {'[ATTENTION]' if res['brownouts'] else '[PASS]'} |",
        f"| `bind() to port 1181` (benign) | {res['bind1181']} | [INFO] |",
        f"| Auto mission completions (benign unwind) | {res['mission_done']} | [INFO] |",
        f"| Main-loop overruns (20ms, sim perf noise) | {res['overruns']} | [INFO] |",
        f"| Worst `simulationPeriodic()` | {res['sim_periodic_max']:.3f}s | {'[ATTENTION]' if res['sim_periodic_max'] > 0.10 else '[INFO]'} |",
        f"| Stale CAN frames at startup (sim) | {res['can_stale']} | [INFO] |",
        "",
    ]
    if res["stuck"]:
        L += ["## Stuck events", "",
              "| Mode | Phase | Pose | Target | Stall |",
              "| :--- | :--- | :--- | :--- | :--- |"]
        for mode, phase, px, py, tx, ty, stall in res["stuck"]:
            L.append(f"| {mode} | {phase} | ({px}, {py}) | ({tx}, {ty}) | {stall}s |")
        L += ["",
              "Cluster near the opponent Hub/ramp footprint points at the known",
              "`DENY_SHOOTING_LANE` staging issue; trench-corridor clusters point at",
              "trench transit/yield work (see KNOWN_ISSUES.md).", ""]
    if res["errors"]:
        L += ["## Errors", ""]
        for lineno, line in res["errors"][:40]:
            L.append(f"- L{lineno}: `{line}`")
        L.append("")
    if not res["stuck"] and not res["errors"]:
        if res["matches_seen"] == 0:
            L += ["No teleop detected in this capture (no HubSchedule seed) —",
                  "the verdict is ATTENTION, not PASS. Enable Teleoperated and play",
                  "before stopping the sim.", ""]
        else:
            L += ["No stuck dumps and no exceptions in this capture.",
                  "If STUCK count is 0, confirm `Simulation/DebugAI` was true —",
                  "without it the Bot0 stuck pipeline stays silent.", ""]
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description="Review a simulateJava console capture.")
    ap.add_argument("--log", required=True, help="Console log file to review")
    ap.add_argument("--drill", default="Free Play Match", help="Drill/match label for the report")
    ap.add_argument("--report-dir", default=None,
                    help="Where to write the MD report (default: beside the log)")
    args = ap.parse_args()

    res = review(args.log)
    md = render_md(res, args.drill)
    out_dir = args.report_dir or os.path.dirname(os.path.abspath(args.log))
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = os.path.join(out_dir, f"practice_review_{stamp}.md")
    with open(out, "w", encoding="utf-8") as f:
        f.write(md)

    print(f"verdict={res['verdict']} lines={res['total_lines']} "
          f"errors={len(res['errors'])} stuck={len(res['stuck'])} "
          f"brownouts={res['brownouts']} bind1181={res['bind1181']} "
          f"missionDone={res['mission_done']} "
          f"overruns={res['overruns']} simPeriodicMax={res['sim_periodic_max']:.3f}s "
          f"canStale={res['can_stale']} matches={res['matches_seen']}")
    print(f"report={out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
