import os
import re
import json

CODEBASE_DIR = r"C:\Users\jumpi\Documents\Github\2026Dreams\TitanRoboticsBuildSeason\src\main\java\frc\robot"

UNIT_KEYWORDS = [
    'meter', 'metre', 'second', 'sec', 'radian', 'rad', 'degree', 'deg',
    'volt', 'amp', 'rpm', 'hz', 'hertz', 'kg', 'kilogram', 'pound', 'lb',
    'inch', 'ms', 'millisecond', 'fps', 'mps', 'joule', 'newton'
]

UNIT_SUFFIX_REGEX = re.compile(r'_(METERS?|M|SECS?|SECONDS?|RAD|RADIANS?|DEG|DEGREES?|VOLTS?|AMPS?|RPM|HZ|KG|LBS?|INCHES?|MS)(?:_PER_[A-Z0-9]+)?$', re.IGNORECASE)

def has_unit_in_name(name):
    return bool(UNIT_SUFFIX_REGEX.search(name))

def text_has_unit(text):
    if not text:
        return False
    lower = text.lower()
    return any(u in lower for u in UNIT_KEYWORDS)

class JavaFileAuditor:
    def __init__(self, filepath, relpath):
        self.filepath = filepath
        self.relpath = relpath
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            self.content = f.read()
        self.lines = self.content.splitlines()

    def audit(self):
        result = {
            'file': self.relpath,
            'package': '',
            'types': [],
            'methods': [],
            'constants': [],
            'smells': [],
            'javadoc_stats': {
                'types_total': 0,
                'types_with_javadoc': 0,
                'methods_total': 0,
                'methods_public_protected': 0,
                'methods_with_javadoc': 0,
                'methods_with_units': 0,
                'constants_total': 0,
                'constants_with_javadoc': 0,
                'constants_with_units': 0
            }
        }

        # 1. Package
        pkg_match = re.search(r'^\s*package\s+([a-zA-Z0-9_.]+);', self.content, re.MULTILINE)
        if pkg_match:
            result['package'] = pkg_match.group(1)

        # 2. Code smells check across whole file
        for idx, line in enumerate(self.lines):
            lineno = idx + 1
            # System.out / err / printStackTrace
            if re.search(r'System\.(out|err)\.print', line):
                result['smells'].append({
                    'line': lineno,
                    'type': 'CONSOLE_IO',
                    'message': line.strip()
                })
            if 'printStackTrace()' in line:
                result['smells'].append({
                    'line': lineno,
                    'type': 'PRINT_STACK_TRACE',
                    'message': line.strip()
                })
            if re.search(r'Thread\.sleep\(', line):
                result['smells'].append({
                    'line': lineno,
                    'type': 'THREAD_SLEEP',
                    'message': line.strip()
                })

        self.parse_declarations(result)
        return result

    def parse_declarations(self, result):
        lines = self.lines
        current_javadoc = []
        in_javadoc = False
        in_block_comment = False

        type_regex = re.compile(
            r'^(?P<modifiers>(?:(?:public|protected|private|static|abstract|final|sealed|non-sealed)\s+)*)'
            r'(?P<kind>class|interface|enum|record)\s+'
            r'(?P<name>[A-Za-z0-9_]+)'
            r'(?:<[^>]+>)?'
            r'(?:\s+extends\s+[A-Za-z0-9_.,\s<>]+)?'
            r'(?:\s+implements\s+[A-Za-z0-9_.,\s<>]+)?'
            r'\s*\{?'
        )

        method_regex = re.compile(
            r'^(?P<modifiers>(?:(?:public|protected|private|static|abstract|final|synchronized|default|native)\s+)*)'
            r'(?:<[^>]+>\s+)?'
            r'(?P<return>[A-Za-z0-9_<>,.\[\]]+)\s+'
            r'(?P<name>[A-Za-z0-9_]+)\s*'
            r'\((?P<params>[^)]*)\)'
            r'(?:\s*throws\s+[A-Za-z0-9_.,\s]+)?'
            r'\s*[{;]'
        )

        # Updated field regex
        field_regex = re.compile(
            r'^(?P<modifiers>(?:(?:public|protected|private|static|final|volatile|transient)\s+)+)'
            r'(?P<type>[A-Za-z0-9_<>,.\[\]]+)\s+'
            r'(?P<name>[A-Za-z0-9_]+)\s*'
            r'(?:=\s*.*)?;'
        )

        i = 0
        while i < len(lines):
            raw_line = lines[i]
            line = raw_line.strip()

            # Handle Javadoc collection
            if line.startswith('/**'):
                current_javadoc = [line]
                if line.endswith('*/') and len(line) > 3:
                    in_javadoc = False
                else:
                    in_javadoc = True
                i += 1
                continue
            elif in_javadoc:
                current_javadoc.append(line)
                if line.endswith('*/'):
                    in_javadoc = False
                i += 1
                continue
            elif line.startswith('/*'):
                in_block_comment = True
                if line.endswith('*/') and len(line) > 2:
                    in_block_comment = False
                i += 1
                continue
            elif in_block_comment:
                if line.endswith('*/'):
                    in_block_comment = False
                i += 1
                continue
            elif line.startswith('//'):
                i += 1
                continue
            elif line.startswith('@'):
                i += 1
                continue
            elif not line:
                i += 1
                continue

            # Strip inline comment for regex matching
            code_line = re.sub(r'//.*$', '', line).strip()
            inline_comment = line[len(code_line):].strip() if len(code_line) < len(line) else ""

            javadoc_text = "\n".join(current_javadoc) if current_javadoc else None
            has_jd = javadoc_text is not None and len(javadoc_text.strip()) > 0

            # Check Type declaration
            type_m = type_regex.match(code_line)
            if type_m:
                kind = type_m.group('kind')
                name = type_m.group('name')
                mods = (type_m.group('modifiers') or '').strip()
                result['types'].append({
                    'line': i + 1,
                    'kind': kind,
                    'name': name,
                    'modifiers': mods,
                    'has_javadoc': has_jd,
                    'javadoc': javadoc_text
                })
                result['javadoc_stats']['types_total'] += 1
                if has_jd:
                    result['javadoc_stats']['types_with_javadoc'] += 1
                current_javadoc = []
                i += 1
                continue

            # Check Method declaration
            meth_m = method_regex.match(code_line)
            if meth_m and not code_line.startswith('return ') and not code_line.startswith('throw ') and not code_line.startswith('new '):
                name = meth_m.group('name')
                ret = meth_m.group('return')
                mods = (meth_m.group('modifiers') or '').strip()
                params = meth_m.group('params').strip()

                if name not in ['if', 'for', 'while', 'switch', 'catch', 'synchronized']:
                    is_pub_prot = 'public' in mods or 'protected' in mods
                    has_unit = text_has_unit(javadoc_text) or has_unit_in_name(name) or text_has_unit(params) or text_has_unit(ret) or text_has_unit(inline_comment)

                    result['methods'].append({
                        'line': i + 1,
                        'name': name,
                        'return': ret,
                        'modifiers': mods,
                        'params': params,
                        'has_javadoc': has_jd,
                        'javadoc': javadoc_text,
                        'has_units': has_unit
                    })
                    result['javadoc_stats']['methods_total'] += 1
                    if is_pub_prot:
                        result['javadoc_stats']['methods_public_protected'] += 1
                        if has_jd:
                            result['javadoc_stats']['methods_with_javadoc'] += 1
                        if has_unit:
                            result['javadoc_stats']['methods_with_units'] += 1

                    current_javadoc = []
                    i += 1
                    continue

            # Check Field / Constant declaration
            field_m = field_regex.match(code_line)
            if field_m:
                mods = (field_m.group('modifiers') or '').strip()
                ftype = field_m.group('type')
                fname = field_m.group('name')

                is_constant = 'static' in mods and 'final' in mods
                is_public_mutable = 'public' in mods and 'final' not in mods

                has_unit = has_unit_in_name(fname) or text_has_unit(javadoc_text) or text_has_unit(inline_comment) or text_has_unit(ftype)

                if is_public_mutable:
                    result['smells'].append({
                        'line': i + 1,
                        'type': 'PUBLIC_MUTABLE_FIELD',
                        'message': f"Public mutable field '{fname}': {line}"
                    })

                if is_constant:
                    result['constants'].append({
                        'line': i + 1,
                        'name': fname,
                        'type': ftype,
                        'modifiers': mods,
                        'has_javadoc': has_jd,
                        'javadoc': javadoc_text,
                        'has_units': has_unit,
                        'inline_comment': inline_comment,
                        'raw': line
                    })
                    result['javadoc_stats']['constants_total'] += 1
                    if has_jd:
                        result['javadoc_stats']['constants_with_javadoc'] += 1
                    if has_unit:
                        result['javadoc_stats']['constants_with_units'] += 1

                current_javadoc = []
                i += 1
                continue

            current_javadoc = []
            i += 1

def run_full_audit():
    all_results = []
    for root, dirs, files in os.walk(CODEBASE_DIR):
        for f in files:
            if f.endswith('.java'):
                fpath = os.path.join(root, f)
                relpath = os.path.relpath(fpath, CODEBASE_DIR)
                auditor = JavaFileAuditor(fpath, relpath)
                res = auditor.audit()
                all_results.append(res)
    return all_results

if __name__ == '__main__':
    results = run_full_audit()
    print(f"Audited {len(results)} files.")
    output_path = r"C:\Users\jumpi\Documents\Github\2026Dreams\.agents\teamwork\explorer_docs_1\audit_raw.json"
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(results, f, indent=2)
    print(f"Saved to {output_path}")
