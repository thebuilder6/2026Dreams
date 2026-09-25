import json
import sys

data = json.load(open(r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\audit_raw.json", encoding="utf-8"))

pkg_filter = sys.argv[1] if len(sys.argv) > 1 else ""

for f in data:
    if f['package'].startswith(pkg_filter) or pkg_filter in f['file']:
        stats = f['javadoc_stats']
        print(f"{f['file']} (pkg: {f['package']}):")
        print(f"  Types: {stats['types_with_javadoc']}/{stats['types_total']} JD")
        print(f"  Methods: {stats['methods_with_javadoc']}/{stats['methods_public_protected']} JD ({stats['methods_with_units']} w/ units)")
        print(f"  Constants: {stats['constants_with_javadoc']}/{stats['constants_total']} JD ({stats['constants_with_units']} w/ units)")
        if f['smells']:
            print(f"  Smells: {len(f['smells'])} detected")
