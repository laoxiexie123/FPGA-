"""Encoder cycle/model/old-chain verification. Compiled images stay in temp."""
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
SOURCES = [ROOT/'src/encoder.v', ROOT/'tb/tb_encoder.v',
           PROJECT/'v2.2/src/encoder_guard.v', PROJECT/'v2.2/src/top.v',
           Path(__file__).resolve()]


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


def check_old_speed_contract():
    # The TB shadow is a documented literal copy. Fail if old arithmetic/priority changes.
    text = SOURCES[3].read_text(encoding='utf-8-sig')
    required = [r'speed_difference_next\s*\*\s*40\x27sd100',
                r'else if \(zero_cmd_next\)', r'else if \(tick_10ms_q\)',
                r'speed_product_next > 40\x27sd2147483647',
                r'speed_product_next < -40\x27sd2147483648',
                r'arm_speed_q <= speed_product_next\[31:0\]']
    for pattern in required:
        if not re.search(pattern, text):
            raise ValueError('Old top speed contract changed: ' + pattern)


def main():
    for path in SOURCES + [IVERILOG, VVP]:
        if not path.is_file():
            raise FileNotFoundError(path)
    check_old_speed_contract()
    before = hashes()
    build = Path(tempfile.mkdtemp(prefix='j280_v3_encoder_'))
    report = {'status': 'running', 'level': 'II digital simulation',
              'scope': 'Known-input independent phase/cycle/signed64 speed model, old encoder_guard '
                       'and literal old top speed shadow bound by source hashes. Directed ticks '
                       'and three actual 500000-cycle windows; no hardware/CDC/implementation claim.',
              'debounce_cycles': 4, 'natural_window_cycles': 500000,
              'source_sha256': before, 'temporary_build_directory': str(build), 'tests': []}
    try:
        for mode in ('g2005', 'g2012'):
            image = build/f'encoder_{mode}.vvp'
            compile_log = HERE/f'encoder_compile_{mode}.log'
            sim_log = HERE/f'encoder_simulate_{mode}.log'
            cc = command([IVERILOG, '-'+mode, '-Wall', '-s', 'tb_encoder', '-o', image,
                          *SOURCES[:3]], compile_log)
            sc = command([VVP, image], sim_log)
            marker = re.search(r'^PASS ENCODER .+$', sim_log.read_text(encoding='utf-8'), re.M)
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
        (HERE/'encoder_test_results.json').write_text(
            json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
        print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
