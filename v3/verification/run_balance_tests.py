"""II digital simulation: fixed balance controller versus complete legacy core.

Legacy ADC owns filtering and feeds identical Q8/x4 to the new external-ADC
interface on each edge. Default PD (legacy Ki=0, external reference disabled) retained control outputs
(motor command, commit, busy, fault, reason) and commit timing are compared each cycle.
Independent angle checks and position cadence/cancellation checks read existing algorithm state only in the testbench;
no diagnostic RTL ports, shadow registers or hardware sequence counters are retained.
Position integral and external Q16 reference have been intentionally removed from stage3 onward.
External ADC range faults are tested separately: legacy raw/alpha/ADC-busy
fault coupling is intentionally not claimed as part of the migrated controller.
"""
from pathlib import Path
import hashlib
import os
import json
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
PROJECT = ROOT.parents[1]
IVERILOG = Path('D:/msys64/ucrt64/bin/iverilog.exe')
VVP = Path('D:/msys64/ucrt64/bin/vvp.exe')
STAGE = os.environ.get('BALANCE_STAGE', '')
PREFIX = ('migration_stages/'+STAGE+'_') if STAGE else ''
SOURCE = Path(os.environ.get('BALANCE_SOURCE', str(ROOT/'src/balance_control.v'))).resolve()
SOURCES = [SOURCE, ROOT/'tb/tb_balance_control.v',
           PROJECT/'v2.2/src/dual_pd_core.v', Path(__file__).resolve()]

def hashes():
    return {p.relative_to(PROJECT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in SOURCES}

def main():
    before = hashes()
    report = {'status': 'running', 'scope': __doc__, 'stage': STAGE, 'source_sha256': before,
              'tests': [], 'verification_level': 'II; no synthesis, timing closure or board claim'}
    try:
        with tempfile.TemporaryDirectory(prefix='j280_balance_') as build:
            for mode in ('g2005', 'g2012'):
                output = Path(build)/('balance_'+mode+'.vvp')
                with (HERE/(PREFIX+'balance_compile_'+mode+'.log')).open('w', encoding='utf-8') as log:
                    subprocess.run([str(IVERILOG), '-'+mode, '-Wall', '-s', 'tb_balance_control',
                                    '-o', str(output), *(['-DDIRECT_BALANCE'] if 'wait_position_weight' in SOURCE.read_text(encoding='utf-8') else []), *map(str, SOURCES[:3])],
                                   stdout=log, stderr=subprocess.STDOUT, check=True)
                log_path = HERE/(PREFIX+'balance_simulate_'+mode+'.log')
                with log_path.open('w', encoding='utf-8') as log:
                    subprocess.run([str(VVP), str(output)], stdout=log,
                                   stderr=subprocess.STDOUT, check=True, timeout=120)
                summary = log_path.read_text(encoding='utf-8').strip()
                if 'PASS BALANCE' not in summary:
                    raise RuntimeError('missing PASS BALANCE marker')
                report['tests'].append({'mode': mode, 'summary': summary, 'exit': 0})
        report['source_hashes_unchanged_during_run'] = before == hashes()
        if not report['source_hashes_unchanged_during_run']:
            raise RuntimeError('sources changed during regression')
        report['status'] = 'passed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = str(error)
        raise
    finally:
        (HERE/(PREFIX+'balance_test_results.json')).write_text(
            json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(report, ensure_ascii=False, indent=2))

if __name__ == '__main__':
    main()
