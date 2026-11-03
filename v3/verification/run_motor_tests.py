"""Known-input motor equivalence and independent math/timing checks; temp binaries."""
from pathlib import Path
import hashlib
import json
import re
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PROJECT = ROOT.parents[1]
IVERILOG = Path('D:/msys64/ucrt64/bin/iverilog.exe')
VVP = Path('D:/msys64/ucrt64/bin/vvp.exe')
SOURCES = [ROOT/'src/motro.v', ROOT/'tb/tb_motro.v',
           PROJECT/'v2.2/src/motor_drv.v', Path(__file__).resolve()]


def hashes():
    return {p.relative_to(PROJECT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in SOURCES}


def command(args, log):
    with log.open('w', encoding='utf-8') as stream:
        result = subprocess.run([str(x) for x in args], cwd=ROOT,
                                stdout=stream, stderr=stream, timeout=180)
    if result.returncode:
        raise RuntimeError(f'Exit {result.returncode}: {log}')
    return result.returncode


def main():
    for path in SOURCES + [IVERILOG, VVP]:
        if not path.is_file():
            raise FileNotFoundError(path)
    before = hashes()
    build = Path(tempfile.mkdtemp(prefix='j280_v3_motor_'))
    report = {'status': 'running', 'level': 'II digital simulation',
              'scope': 'Known-input pin-by-pin comparison against old motor_drv plus independent '
                       'full signed16 arithmetic, cycle model, stable pulse counts, boundary/clear tests. '
                       'Old unknown normalized_width default checked; no general X equivalence, '
                       'synthesis, timing closure or hardware claim.',
              'period_cycles': 5000, 'source_sha256': before,
              'temporary_build_directory': str(build), 'tests': []}
    try:
        for mode in ('g2005', 'g2012'):
            image = build/f'motor_{mode}.vvp'
            compile_log = HERE/f'motor_compile_{mode}.log'
            sim_log = HERE/f'motor_simulate_{mode}.log'
            cc = command([IVERILOG, '-'+mode, '-Wall', '-s', 'tb_motro', '-o', image,
                          *SOURCES[:3]], compile_log)
            sc = command([VVP, image], sim_log)
            marker = re.search(r'^PASS MOTOR .+$', sim_log.read_text(encoding='utf-8'), re.M)
            if not marker:
                raise RuntimeError(f'Missing PASS marker: {sim_log}')
            report['tests'].append({'mode': mode, 'compile_exit': cc, 'simulate_exit': sc,
                                    'summary': marker[0], 'compile_log': compile_log.name,
                                    'simulate_log': sim_log.name})
        report['source_hashes_unchanged_during_run'] = before == hashes()
        if not report['source_hashes_unchanged_during_run']:
            raise RuntimeError('Source changed during final verification')
        report['status'] = 'passed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = str(error)
        raise
    finally:
        (HERE/'motor_test_results.json').write_text(
            json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
        print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
