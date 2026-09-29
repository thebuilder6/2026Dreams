"""Verify docs/nav/roadmap.html against the real navigation source.

Run from the repo root:
    python TitanRoboticsBuildSeason/tools/nav/verify_roadmap.py

Fails (exit 1) on any transcription drift, so the saved map can never silently
outlive the geometry it documents.
"""

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]
NAV = ROOT / "TitanRoboticsBuildSeason/src/main/java/frc/robot/Navigation"
SP = NAV / "StaticPathfinder.java"
FM = NAV / "FieldMap.java"
HTML = ROOT / "TitanRoboticsBuildSeason/docs/nav/roadmap.html"

sp = SP.read_text(encoding="utf-8")
fm = FM.read_text(encoding="utf-8")
html = HTML.read_text(encoding="utf-8")

errors = []
checks = 0


def check(cond, msg):
    global checks
    checks += 1
    if not cond:
        errors.append(msg)


# ---- roadmap nodes ---------------------------------------------------------
src_nodes = {}
for m in re.finditer(
    r'NODES\.add\(new RoadmapNode\((\d+), "([^"]+)", ([\d.]+), ([\d.]+)\)\)', sp
):
    src_nodes[int(m.group(1))] = (m.group(2), float(m.group(3)), float(m.group(4)))

html_nodes = {}
for m in re.finditer(r'\[(\d+),"([^"]+)",([\d.]+),([\d.]+),"(\w+)"\]', html):
    html_nodes[int(m.group(1))] = (m.group(2), float(m.group(3)), float(m.group(4)))

check(len(src_nodes) == 34, f"source has {len(src_nodes)} nodes, expected 34")
check(len(html_nodes) == len(src_nodes),
      f"html has {len(html_nodes)} nodes, source has {len(src_nodes)}")
for i in sorted(set(src_nodes) | set(html_nodes)):
    if i not in src_nodes:
        errors.append(f"node {i} in html but not in StaticPathfinder")
    elif i not in html_nodes:
        errors.append(f"node {i} in StaticPathfinder but missing from html")
    else:
        sn, sx, sy = src_nodes[i]
        hn, hx, hy = html_nodes[i]
        check(sn == hn, f"node {i} name: source {sn!r} vs html {hn!r}")
        check(abs(sx - hx) < 1e-9 and abs(sy - hy) < 1e-9,
              f"node {i} ({sn}) pos: source ({sx}, {sy}) vs html ({hx}, {hy})")

# ---- roadmap edges ---------------------------------------------------------
consts = dict(re.findall(r"public static final int (N_[A-Z_0-9]+) = (\d+);", sp))
src_edges = set()
for m in re.finditer(r"connect\((N_[A-Z_0-9]+), (N_[A-Z_0-9]+)\)", sp):
    a, b = int(consts[m.group(1)]), int(consts[m.group(2)])
    src_edges.add((min(a, b), max(a, b)))

html_edges = set()
block = re.search(r"^const E = \[(.*?)^\];", html, re.S | re.M)
check(block is not None, "could not locate the E edge array in the html")
if block:
    for m in re.finditer(r"\[(\d+),(\d+)\]", block.group(1)):
        a, b = int(m.group(1)), int(m.group(2))
        html_edges.add((min(a, b), max(a, b)))

check(src_edges == html_edges,
      f"edges differ: only-source {sorted(src_edges - html_edges)}, "
      f"only-html {sorted(html_edges - src_edges)}")

# ---- FieldMap scalars quoted in the html ----------------------------------
scalars = {
    "FIELD_LENGTH": r"FIELD_LENGTH = ([\d.]+)",
    "FIELD_WIDTH": r"FIELD_WIDTH = ([\d.]+)",
    "ROBOT_RADIUS": r"ROBOT_RADIUS = ([\d.]+)",
    "TrenchWalls.BLUE_CENTER_X": r"BLUE_CENTER_X = ([\d.]+)",
    "TrenchWalls.SOUTH_CENTER_Y": r"SOUTH_CENTER_Y = ([\d.]+)",
    "TrenchWalls.WALL_X_LEN": r"WALL_X_LEN = ([\d.]+)",
    "TrenchWalls.WALL_Y_LEN": r"WALL_Y_LEN = ([\d.]+)",
    "Trenches.BLUE_TRENCH_MIN_X": r"BLUE_TRENCH_MIN_X = ([\d.]+)",
    "Trenches.BLUE_TRENCH_MAX_X": r"BLUE_TRENCH_MAX_X = ([\d.]+)",
    "Trenches.TOP_CORRIDOR_Y": r"TOP_CORRIDOR_Y = ([\d.]+)",
    "Trenches.BOT_CORRIDOR_Y": r"BOT_CORRIDOR_Y = ([\d.]+)",
    "Hubs.BLUE_HUB_X": r"BLUE_HUB_X = ([\d.]+)",
    "Hubs.HUB_Y": r"HUB_Y = ([\d.]+)",
    "Hubs.HUB_WIDTH": r"HUB_WIDTH = ([\d.]+)",
    "Ramps.RAMP_LENGTH_Y": r"RAMP_LENGTH_Y = ([\d.]+)",
}
values = {}
for name, pat in scalars.items():
    m = re.search(pat, fm)
    check(m is not None, f"could not find {name} in FieldMap.java")
    if m:
        values[name] = float(m.group(1))

FL = values.get("FIELD_LENGTH")
FW = values.get("FIELD_WIDTH")
check(
    re.search(r"const L = ([\d.]+), W = ([\d.]+), M = ([\d.]+);", html) is not None,
    "could not find the L/W/M header in the html",
)
hm = re.search(r"const L = ([\d.]+), W = ([\d.]+), M = ([\d.]+);", html)
if hm and FL and FW:
    check(abs(float(hm.group(1)) - FL) < 1e-9, f"html L {hm.group(1)} != FIELD_LENGTH {FL}")
    check(abs(float(hm.group(2)) - FW) < 1e-9, f"html W {hm.group(2)} != FIELD_WIDTH {FW}")
    check(
        abs(float(hm.group(3)) - values["ROBOT_RADIUS"]) < 1e-9,
        f"html M {hm.group(3)} != ROBOT_RADIUS {values['ROBOT_RADIUS']}",
    )

# ---- derived trench numbers quoted in the sidebar -------------------------
wall_hy = values["TrenchWalls.WALL_Y_LEN"] / 2.0
top_min_y = (FW - values["TrenchWalls.SOUTH_CENTER_Y"]) + wall_hy
bot_max_y = values["TrenchWalls.SOUTH_CENTER_Y"] - wall_hy
top_lane = values["Trenches.TOP_CORRIDOR_Y"]
bot_lane = values["Trenches.BOT_CORRIDOR_Y"]
margin = values["ROBOT_RADIUS"]
infl_top = top_min_y + margin
usable_top = (infl_top, FW - margin)
usable_bot = (margin, bot_max_y - margin)
width_top = usable_top[1] - usable_top[0]
clear_top = top_lane - infl_top
clear_bot = (bot_max_y - margin) - bot_lane
overlap = infl_top - top_min_y

for label, val in [
    ("TOP_TRENCH_MIN_Y", top_min_y),
    ("BOT_TRENCH_MAX_Y", bot_max_y),
    ("inflated wall face (top)", infl_top),
    ("lane->inflated wall (top)", clear_top),
    ("lane->inflated wall (bottom)", clear_bot),
    ("usable band width", width_top),
    ("overlap depth", overlap),
]:
    text = f"{val:.4f}"
    check(text in html, f"sidebar figure {label} = {text} is not present in the html")

# ---- report ----------------------------------------------------------------
print(f"source: {len(src_nodes)} nodes, {len(src_edges)} unique undirected edges")
print(f"html:   {len(html_nodes)} nodes, {len(html_edges)} unique undirected edges")
print(f"derived trench figures: usable band {width_top:.4f} m, "
      f"lane clearance {clear_top:.4f}/{clear_bot:.4f} m, overlap {overlap:.4f} m")
if errors:
    print(f"\nFAIL ({len(errors)} of {checks} checks)")
    for e in errors:
        print(f"  - {e}")
    sys.exit(1)
print(f"\nOK - {checks} checks passed; docs/nav/roadmap.html matches the navigation source")
