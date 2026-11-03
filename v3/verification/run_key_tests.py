"""II: full real 50MHz debounce at 1, 4 and 1000000 cycle thresholds; no count preloads."""
from pathlib import Path
import hashlib
import json
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PROJECT = ROOT.parents[1]
SOURCES = [ROOT/'src/key_filter.v', ROOT/'tb/tb_key_filter.v', Path(__file__).resolve()]
IVERILOG = 'D:/msys64/ucrt64/bin/iverilog.exe'
VVP = 'D:/msys64/ucrt64/bin/vvp.exe'


def hashes():
    return {p.relative_to(PROJECT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in SOURCES}


def main():
    before = hashes()
    report = {'status': 'running', 'scope': __doc__, 'source_sha256': before, 'tests': []}
    try:
        with tempfile.TemporaryDirectory(prefix='j280_key_') as build:
            for mode in ('g2005', 'g2012'):
                output = Path(build)/(mode+'.vvp')
                with (HERE/('key_compile_'+mode+'.log')).open('w', encoding='utf-8') as log:
                    subprocess.run([IVERILOG, '-'+mode, '-Wall', '-s', 'tb_key_filter',
                                    '-o', str(output), *map(str, SOURCES[:-1])],
                                   stdout=log, stderr=subprocess.STDOUT, check=True, timeout=60)
                path = HERE/('key_simulate_'+mode+'.log')
                with path.open('w', encoding='utf-8') as log:
                    subprocess.run([VVP, str(output)], stdout=log, stderr=subprocess.STDOUT,
                                   check=True, timeout=120)
                text = path.read_text(encoding='utf-8').strip()
                if 'PASS key_filter' not in text and 'PASS KEY' not in text:
                    raise RuntimeError('Missing key test pass marker: '+text)
                report['tests'].append({'mode': mode, 'summary': text, 'exit': 0})
        report['source_hashes_unchanged_during_run'] = before == hashes()
        if not report['source_hashes_unchanged_during_run']:
            raise RuntimeError('Source changed during test')
        report['status'] = 'passed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = str(error)
        raise
    finally:
        (HERE/'key_test_results.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
