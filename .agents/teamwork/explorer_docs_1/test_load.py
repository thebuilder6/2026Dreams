import json
from collections import defaultdict

raw_path = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\audit_raw.json"
type_path = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\type_catalog.json"
summary_path = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\per_file_summary.json"

with open(raw_path, 'r', encoding='utf-8') as f:
    raw_files = json.load(f)

with open(type_path, 'r', encoding='utf-8') as f:
    all_types = json.load(f)

with open(summary_path, 'r', encoding='utf-8') as f:
    file_summaries = json.load(f)

print(f"Loaded {len(raw_files)} files, {len(all_types)} types, {len(file_summaries)} summaries.")
