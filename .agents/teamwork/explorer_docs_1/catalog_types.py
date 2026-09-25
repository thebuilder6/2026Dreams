import os
import re
import json

CODEBASE_DIR = r"C:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason\src\main\java\frc\robot"

type_regex = re.compile(
    r'^(?P<modifiers>(?:(?:public|protected|private|static|abstract|final|sealed|non-sealed)\s+)*)'
    r'(?P<kind>@interface|class|interface|enum|record)\s+'
    r'(?P<name>[A-Za-z0-9_]+)'
    r'(?:<[^>]+>)?'
    r'(?:\s+extends\s+[A-Za-z0-9_.,\s<>]+)?'
    r'(?:\s+implements\s+[A-Za-z0-9_.,\s<>]+)?'
    r'\s*\{?'
)

all_types = []

for root, dirs, files in os.walk(CODEBASE_DIR):
    for f in sorted(files):
        if f.endswith('.java'):
            fpath = os.path.join(root, f)
            relpath = os.path.relpath(fpath, CODEBASE_DIR)
            with open(fpath, 'r', encoding='utf-8', errors='ignore') as fp:
                lines = fp.readlines()

            pkg = ""
            current_jd = []
            in_jd = False
            in_block = False

            for i, line in enumerate(lines):
                sline = line.strip()
                if sline.startswith("package "):
                    pkg = sline.replace("package ", "").rstrip(";").strip()
                    continue

                if sline.startswith("/**"):
                    current_jd = [sline]
                    if sline.endswith("*/") and len(sline) > 3:
                        in_jd = False
                    else:
                        in_jd = True
                    continue
                elif in_jd:
                    current_jd.append(sline)
                    if sline.endswith("*/"):
                        in_jd = False
                    continue
                elif sline.startswith("/*"):
                    in_block = True
                    if sline.endswith("*/") and len(sline) > 2:
                        in_block = False
                    continue
                elif in_block:
                    if sline.endswith("*/"):
                        in_block = False
                    continue
                elif sline.startswith("//") or sline.startswith("@") or not sline:
                    continue

                code_line = re.sub(r'//.*$', '', sline).strip()
                m = type_regex.match(code_line)
                if m:
                    kind = m.group('kind')
                    name = m.group('name')
                    mods = (m.group('modifiers') or '').strip()
                    has_jd = len(current_jd) > 0
                    jd_summary = ""
                    if has_jd:
                        # Extract first meaningful line
                        clean_lines = [l.strip('/* ').strip() for l in current_jd if l.strip('/* ').strip()]
                        jd_summary = clean_lines[0] if clean_lines else ""

                    all_types.append({
                        'package': pkg,
                        'file': relpath,
                        'line': i + 1,
                        'kind': kind,
                        'name': name,
                        'modifiers': mods,
                        'has_javadoc': has_jd,
                        'javadoc_summary': jd_summary
                    })
                    current_jd = []
                else:
                    current_jd = []

print(f"Total cataloged types: {len(all_types)}")
out_path = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\type_catalog.json"
with open(out_path, 'w', encoding='utf-8') as f:
    json.dump(all_types, f, indent=2)
print(f"Saved catalog to {out_path}")
