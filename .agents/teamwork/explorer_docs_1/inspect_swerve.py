import json

raw_path = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\audit_raw.json"
with open(raw_path, 'r', encoding='utf-8') as f:
    data = json.load(f)

for f in data:
    if f['file'] == 'Subsystems\\SwerveBase.java':
        print(f"File: {f['file']}")
        print(f"Types: {len(f['types'])}")
        for t in f['types']:
            print(f"  {t['kind']} {t['name']} (JD: {t['has_javadoc']})")
        print(f"Methods: {len(f['methods'])}")
        no_jd = [m for m in f['methods'] if not m['has_javadoc'] and ('public' in m['modifiers'] or 'protected' in m['modifiers'])]
        with_jd = [m for m in f['methods'] if m['has_javadoc'] and ('public' in m['modifiers'] or 'protected' in m['modifiers'])]
        print(f"  Public/Protected Methods with JD: {len(with_jd)}, without JD: {len(no_jd)}")
        print("\nSample Methods WITHOUT Javadoc:")
        for m in no_jd[:15]:
            print(f"    Line {m['line']}: {m['modifiers']} {m['return']} {m['name']}({m['params']})")
        print("\nSample Methods WITH Javadoc:")
        for m in with_jd[:5]:
            print(f"    Line {m['line']}: {m['modifiers']} {m['return']} {m['name']}({m['params']})")
