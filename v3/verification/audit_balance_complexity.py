"""Read-only RTL declaration ledger; declared register bits are not synthesized FF counts.

Writes only the requested audit JSON beside verification reports. This parser is
scoped to these scalar Verilog-2005 modules with literal numeric reg widths;
it fails rather than silently count unsupported declarations.
"""
from pathlib import Path
import hashlib
import json
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def registers(path):
    data = path.read_bytes()
    text = re.sub(r'/\*.*?\*/|//[^\n]*', '', data.decode('utf-8'), flags=re.S)
    header, body = text.split(');', 1)
    declarations = []
    for part in re.split(r'\boutput\s+', header)[1:]:
        if not part.startswith('reg'):
            continue
        declarations.append(part.strip().rstrip(','))
    declarations.extend(m.group(0).rstrip(';') for m in re.finditer(
        r'\breg\s+(?:signed\s+)?(?:\[[^\]]+\]\s*)?[^;]+;', body))
    found = []
    for declaration in declarations:
        match = re.fullmatch(r'reg\s+(?:signed\s+)?(?:\[(\d+):(\d+)\]\s*)?(.*)',
                             declaration, re.S)
        if not match:
            raise ValueError('Unsupported declaration: ' + declaration)
        width = abs(int(match[1]) - int(match[2])) + 1 if match[1] else 1
        for name in match[3].split(','):
            name = name.strip()
            if not re.fullmatch(r'[A-Za-z_]\w*', name):
                raise ValueError('Unsupported register name: ' + name)
            found.append({'name': name, 'bits': width})
    if len({entry['name'] for entry in found}) != len(found):
        raise ValueError('Duplicate register declarations: ' + str(path))
    # legal_state is fully assigned in always @*: Verilog reg syntax, no storage.
    combinational_names = {'legal_state'}
    sequential = [entry for entry in found if entry['name'] not in combinational_names]
    combinational = [entry for entry in found if entry['name'] in combinational_names]
    return {'path': path.relative_to(ROOT).as_posix(),
            'sha256': hashlib.sha256(data).hexdigest(),
            'lines': len(data.decode('utf-8').splitlines()),
            'register_variables': len(found),
            'declared_bits': sum(entry['bits'] for entry in found),
            'sequential_variables': len(sequential),
            'sequential_declared_bits': sum(entry['bits'] for entry in sequential),
            'combinational_reg_variables': combinational,
            'registers': found}


def main():
    sources = [HERE/'migration_baseline/balance_control.v']
    sources.extend(sorted((HERE/'migration_stages').glob('stage*_balance.v')))
    direct = HERE/'migration_stages/direct_balance.v'
    if direct.exists():
        sources.append(direct)
    sources.append(ROOT/'src/balance_control.v')
    report = {'scope': 'Scalar output/internal reg declarations, not synthesized cells; wire/localparam excluded',
              'versions': [registers(path) for path in sources]}
    target = HERE/'balance_complexity.json'
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    for item in report['versions']:
        print(item['path'], item['sequential_variables'], 'sequential variables',
              item['sequential_declared_bits'], 'sequential declared bits',
              len(item['combinational_reg_variables']), 'combinational reg declarations',
              item['lines'], 'lines')


if __name__ == '__main__':
    main()
