import json

data = json.load(open(r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\per_file_summary.json"))

print("=== TOP 25 FILES WITH MOST MISSING JAVADOC (METHODS + CONSTANTS) ===")
sorted_by_jd = sorted(data, key=lambda x: (x['methods_missing_jd_count'] + x['constants_missing_jd_count']), reverse=True)
for f in sorted_by_jd[:25]:
    print(f"{f['file']:<45} | Missing Meth JD: {f['methods_missing_jd_count']:>2}/{f['methods_total']:<2} | Missing Const JD: {f['constants_missing_jd_count']:>2}/{f['constants_total']:<2} | Missing Type JD: {len(f['types_missing_jd'])} | Smells: {f['smells_count']}")

print("\n=== TOP 20 FILES WITH MOST CODE SMELLS ===")
sorted_by_smells = sorted(data, key=lambda x: x['smells_count'], reverse=True)
for f in sorted_by_smells[:20]:
    print(f"{f['file']:<45} | Smells: {f['smells_count']:>2} | Methods missing JD: {f['methods_missing_jd_count']:>2} | Types missing JD: {len(f['types_missing_jd'])}")
