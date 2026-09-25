import json
from collections import defaultdict

raw_path = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\audit_raw.json"
type_path = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\type_catalog.json"

with open(raw_path, 'r', encoding='utf-8') as f:
    files_data = json.load(f)

with open(type_path, 'r', encoding='utf-8') as f:
    types_data = json.load(f)

types_by_file = defaultdict(list)
for t in types_data:
    types_by_file[t['file']].append(t)

file_reports = []

for f in files_data:
    relpath = f['file']
    pkg = f['package']
    stats = f['javadoc_stats']
    smells = f['smells']

    types = types_by_file.get(relpath, [])
    missing_type_jd = [t['name'] for t in types if not t['has_javadoc']]

    pub_prot_methods = [m for m in f['methods'] if 'public' in m['modifiers'] or 'protected' in m['modifiers']]
    methods_missing_jd = [m for m in pub_prot_methods if not m['has_javadoc']]
    methods_missing_units = [m for m in pub_prot_methods if not m['has_units']]

    constants = f['constants']
    consts_missing_jd = [c for c in constants if not c['has_javadoc']]
    consts_missing_units = [c for c in constants if not c['has_units']]

    report = {
        'file': relpath,
        'package': pkg,
        'types_total': len(types),
        'types_with_jd': len(types) - len(missing_type_jd),
        'types_missing_jd': missing_type_jd,
        'methods_total': len(pub_prot_methods),
        'methods_with_jd': stats['methods_with_javadoc'],
        'methods_missing_jd_count': len(methods_missing_jd),
        'methods_missing_jd_names': [m['name'] for m in methods_missing_jd],
        'methods_missing_units_count': len(methods_missing_units),
        'constants_total': len(constants),
        'constants_with_jd': stats['constants_with_javadoc'],
        'constants_missing_jd_count': len(consts_missing_jd),
        'constants_missing_units_count': len(consts_missing_units),
        'constants_missing_units_names': [c['name'] for c in consts_missing_units],
        'smells_count': len(smells),
        'smells': smells
    }
    file_reports.append(report)

print(f"Generated reports for {len(file_reports)} files.")
with open(r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\per_file_summary.json", 'w', encoding='utf-8') as f:
    json.dump(file_reports, f, indent=2)
print("Saved per_file_summary.json")
