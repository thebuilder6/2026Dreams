import json
import os
from collections import defaultdict

raw_path = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\audit_raw.json"
with open(raw_path, 'r', encoding='utf-8') as f:
    data = json.load(f)

print(f"Total files: {len(data)}")

total_types = 0
types_with_jd = 0
total_pub_prot_methods = 0
methods_with_jd = 0
methods_with_units = 0
total_constants = 0
constants_with_jd = 0
constants_with_units = 0

all_smells = defaultdict(list)
files_without_type_jd = []
packages = defaultdict(list)

for file_info in data:
    relpath = file_info['file']
    pkg = file_info['package']
    packages[pkg].append(file_info)

    stats = file_info['javadoc_stats']
    total_types += stats['types_total']
    types_with_jd += stats['types_with_javadoc']
    total_pub_prot_methods += stats['methods_public_protected']
    methods_with_jd += stats['methods_with_javadoc']
    methods_with_units += stats['methods_with_units']
    total_constants += stats['constants_total']
    constants_with_jd += stats['constants_with_javadoc']
    constants_with_units += stats['constants_with_units']

    for t in file_info['types']:
        if not t['has_javadoc']:
            files_without_type_jd.append((relpath, t['name'], t['kind']))

    for s in file_info['smells']:
        all_smells[s['type']].append((relpath, s['line'], s['message']))

print("=== OVERALL JAVADOC STATS ===")
print(f"Types (classes/interfaces/enums): {total_types} total, {types_with_jd} with Javadoc ({types_with_jd/total_types*100:.1f}%), {total_types - types_with_jd} missing")
print(f"Public/Protected Methods: {total_pub_prot_methods} total, {methods_with_jd} with Javadoc ({methods_with_jd/total_pub_prot_methods*100:.1f}%), {methods_with_units} with Units ({methods_with_units/total_pub_prot_methods*100:.1f}%)")
print(f"Constants (static final): {total_constants} total, {constants_with_jd} with Javadoc ({constants_with_jd/total_constants*100:.1f}%), {constants_with_units} with Units ({constants_with_units/total_constants*100:.1f}%)")

print("\n=== CODE SMELLS SUMMARY ===")
for stype, occurrences in all_smells.items():
    print(f"{stype}: {len(occurrences)} occurrences across codebase")

print("\n=== CONSOLE IO OCCURRENCES ===")
for r, l, m in all_smells['CONSOLE_IO'][:25]:
    print(f"  {r}:{l} -> {m}")
if len(all_smells['CONSOLE_IO']) > 25:
    print(f"  ... and {len(all_smells['CONSOLE_IO']) - 25} more")

print("\n=== THREAD SLEEP OCCURRENCES ===")
for r, l, m in all_smells['THREAD_SLEEP']:
    print(f"  {r}:{l} -> {m}")

print("\n=== PRINT STACK TRACE OCCURRENCES ===")
for r, l, m in all_smells['PRINT_STACK_TRACE']:
    print(f"  {r}:{l} -> {m}")

print("\n=== PUBLIC MUTABLE FIELDS (first 25) ===")
for r, l, m in all_smells['PUBLIC_MUTABLE_FIELD'][:25]:
    print(f"  {r}:{l} -> {m}")
if len(all_smells['PUBLIC_MUTABLE_FIELD']) > 25:
    print(f"  ... and {len(all_smells['PUBLIC_MUTABLE_FIELD']) - 25} more")

# Package breakdown
print("\n=== PACKAGE BREAKDOWN ===")
for pkg, flist in sorted(packages.items()):
    pkg_types = sum(f['javadoc_stats']['types_total'] for f in flist)
    pkg_types_jd = sum(f['javadoc_stats']['types_with_javadoc'] for f in flist)
    pkg_methods = sum(f['javadoc_stats']['methods_public_protected'] for f in flist)
    pkg_methods_jd = sum(f['javadoc_stats']['methods_with_javadoc'] for f in flist)
    pkg_constants = sum(f['javadoc_stats']['constants_total'] for f in flist)
    pkg_const_jd = sum(f['javadoc_stats']['constants_with_javadoc'] for f in flist)
    print(f"Package: {pkg or '(default)'} ({len(flist)} files):")
    print(f"  Types: {pkg_types_jd}/{pkg_types} JD | Methods: {pkg_methods_jd}/{pkg_methods} JD | Consts: {pkg_const_jd}/{pkg_constants} JD")
