import json

data = json.load(open(r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\audit_raw.json", encoding="utf-8"))

def inspect_file(relpath_sub):
    for f in data:
        if relpath_sub.lower() in f['file'].lower():
            print(f"=== {f['file']} (Types: {len(f['types'])}, Methods: {len(f['methods'])}, Constants: {len(f['constants'])}) ===")
            print("--- TYPES ---")
            for t in f['types']:
                print(f"  Line {t['line']}: {t['modifiers']} {t['kind']} {t['name']} (JD: {t['has_javadoc']})")
            print("--- PUBLIC/PROTECTED METHODS ---")
            for m in f['methods']:
                if 'public' in m['modifiers'] or 'protected' in m['modifiers']:
                    print(f"  Line {m['line']}: {m['modifiers']} {m['return']} {m['name']}({m['params']}) [JD: {m['has_javadoc']}, Units: {m['has_units']}]")
            print("--- CONSTANTS ---")
            for c in f['constants']:
                print(f"  Line {c['line']}: {c['modifiers']} {c['type']} {c['name']} [JD: {c['has_javadoc']}, Units: {c['has_units']}]")
            print("--- SMELLS ---")
            for s in f['smells']:
                print(f"  Line {s['line']}: [{s['type']}] {s['message']}")
            print()

if __name__ == '__main__':
    import sys
    target = sys.argv[1] if len(sys.argv) > 1 else 'Subsystems\\Intake.java'
    inspect_file(target)
