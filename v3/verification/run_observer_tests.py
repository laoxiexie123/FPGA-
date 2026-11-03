"""Test merged ADC observation logic in isolation using continuously forced mean/freshness.
This checks observation arithmetic/stages against legacy RTL, not the acquisition chain.
"""
from pathlib import Path
import hashlib
import json
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PROJECT = ROOT.parents[1]
IVERILOG = Path('D:/msys64/ucrt64/bin/iverilog.exe')
VVP = Path('D:/msys64/ucrt64/bin/vvp.exe')
SOURCES = [ROOT/'src/adc_pend.v', ROOT/'tb/tb_pendulum_observer.v',
           PROJECT/'v2.2/src/swing_observer.v', Path(__file__).resolve()]

def hashes():
    return {p.relative_to(PROJECT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in SOURCES}

def run(args, log):
    with log.open('w', encoding='utf-8') as stream:
        code = subprocess.run([str(a) for a in args], stdout=stream, stderr=stream,
                              cwd=ROOT, timeout=180).returncode
    if code:
        raise RuntimeError(f'Exit {code}: {log}')
    return code

def main():
    before = hashes()
    build = Path(tempfile.mkdtemp(prefix='j280_v3_observer_'))
    report = {'status': 'running', 'level': 'II digital simulation',
              'scope': 'Merged ADC observation block, mean/freshness continuously forced; not full acquisition. Known-input legacy cycle comparison plus independent integer model; '
                       'directed compressed transaction ticks, no hardware/timing claim.',
              'source_sha256': before, 'tests': [], 'temporary_build_directory': str(build)}
    try:
        for mode in ('g2005', 'g2012'):
            image = build/f'observer_{mode}.vvp'
            compile_log = HERE/f'observer_compile_{mode}.log'
            sim_log = HERE/f'observer_simulate_{mode}.log'
            cc = run([IVERILOG, '-'+mode, '-Wall', '-s', 'tb_pendulum_observer', '-o',
                      image, *SOURCES[:3]], compile_log)
            sc = run([VVP, image], sim_log)
            summary = next((line for line in sim_log.read_text(encoding='utf-8').splitlines()
                            if line.startswith('PASS OBSERVER')), None)
            if not summary:
                raise RuntimeError(f'Missing PASS: {sim_log}')
            report['tests'].append({'mode': mode, 'compile_exit': cc, 'simulate_exit': sc,
                                    'summary': summary, 'compile_log': compile_log.name,
                                    'simulate_log': sim_log.name})
        report['source_hashes_unchanged_during_run'] = before == hashes()
        if not report['source_hashes_unchanged_during_run']:
            raise RuntimeError('Source changed during verification')
        report['status'] = 'passed'
    except Exception as error:
        report.update(status='failed', error=str(error))
        raise
    finally:
        (HERE/'observer_test_results.json').write_text(
            json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
        print(json.dumps(report, indent=2, ensure_ascii=False))

if __name__ == '__main__':
    main()
