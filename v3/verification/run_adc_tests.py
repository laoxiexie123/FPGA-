"""Run real-period ADC model and old-chain comparisons; build artifacts stay in temp."""
from pathlib import Path
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PROJECT = ROOT.parents[1]
PYTHON = Path(sys.executable)
IVERILOG = Path('D:/msys64/ucrt64/bin/iverilog.exe')
VVP = Path('D:/msys64/ucrt64/bin/vvp.exe')
SOURCES = [ROOT/'src/adc_pend.v', ROOT/'tb/tb_adc_pend.v', ROOT/'tb/tb_adc_freshness.v',
           PROJECT/'v2.2/src/adc_pend.v', PROJECT/'v2.2/src/dual_pd_core.v',
           Path(__file__).resolve()]


def hashes():
    return {p.relative_to(PROJECT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in SOURCES}


def command(args, log):
    with log.open('w', encoding='utf-8') as stream:
        result = subprocess.run([str(x) for x in args], stdout=stream, stderr=stream,
                                cwd=ROOT, timeout=180)
    if result.returncode:
        raise RuntimeError(f'Command failed ({result.returncode}), see {log}')
    return result.returncode


def main():
    for p in SOURCES + [IVERILOG, VVP]:
        if not p.is_file():
            raise FileNotFoundError(p)
    before = hashes()
    build = Path(tempfile.mkdtemp(prefix='j280_v3_adc_'))
    old = (PROJECT/'v2.2/src/adc_pend.v').read_text(encoding='utf-8-sig')
    renamed, count = re.subn(r'\bmodule\s+adc_pend\b', 'module old_adc_pend', old)
    assert count == 1
    old_file = build/'old_adc_pend.v'
    old_file.write_text(renamed, encoding='utf-8')
    report = {
        'status': 'running', 'level': 'II digital simulation',
        'scope': 'Independent numeric/cycle model and old ADC plus full-core normal filter path. '
                 'No core fault-hold coupling equivalence, ADC board timing, synthesis or hardware claim.',
        'sample_period_cycles': 9375, 'group_samples': 8, 'filter_alpha': 128,
        'source_sha256': before, 'temporary_build_directory': str(build), 'tests': []}
    try:
        for mode in ('g2005', 'g2012'):
            image = build/f'adc_{mode}.vvp'
            compile_log = HERE/f'compile_{mode}.log'
            sim_log = HERE/f'simulate_{mode}.log'
            compile_code = command([IVERILOG, '-'+mode, '-Wall', '-s', 'tb_adc_pend',
                                    '-o', image, ROOT/'src/adc_pend.v', old_file,
                                    PROJECT/'v2.2/src/dual_pd_core.v', ROOT/'tb/tb_adc_pend.v'],
                                   compile_log)
            simulate_code = command([VVP, image], sim_log)
            text = sim_log.read_text(encoding='utf-8')
            marker = re.search(r'^PASS ADC .+$', text, re.M)
            if not marker:
                raise RuntimeError(f'Missing PASS marker: {sim_log}')
            report['tests'].append({'mode': mode, 'compile_exit': compile_code,
                                    'simulate_exit': simulate_code, 'summary': marker[0],
                                    'compile_log': compile_log.name, 'simulate_log': sim_log.name})
        for mode in ('g2005', 'g2012'):
            image = build/f'freshness_{mode}.vvp'
            compile_log = HERE/f'freshness_compile_{mode}.log'
            sim_log = HERE/f'freshness_simulate_{mode}.log'
            command([IVERILOG, '-'+mode, '-Wall', '-s', 'tb_adc_freshness', '-o', image,
                     ROOT/'src/adc_pend.v', ROOT/'tb/tb_adc_freshness.v'], compile_log)
            command([VVP, image], sim_log)
            if 'PASS ADC_FRESHNESS' not in sim_log.read_text(encoding='utf-8'):
                raise RuntimeError('Missing freshness PASS')
            report['tests'].append({'mode': mode, 'name': 'freshness', 'compile_exit': 0,
                                    'simulate_exit': 0, 'simulate_log': sim_log.name})
        report['source_hashes_unchanged_during_run'] = before == hashes()
        if not report['source_hashes_unchanged_during_run']:
            raise RuntimeError('Sources changed during verification')
        report['status'] = 'passed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = str(error)
        raise
    finally:
        (HERE/'adc_test_results.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
