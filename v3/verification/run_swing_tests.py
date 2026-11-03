"""Fixed mode3 supervisor normal-path equivalence and intentional policy changes.
Ticks are accelerated test enables, not wall-clock milliseconds; blend retains
all 32768 original Bresenham increments. Counter deposits test timeout bounds.
Compatibility differences: explicit start rising edge, failure/stop -> idle,
latched failure until legal fresh start, safety input, reset boost/brake_last.
Capture readiness now includes the old top's one-clock registered raw/filtered
ADC error checks. A separately registered integer model drives the old shadow;
inclusive +/-380 boundaries, full 13-bit difference extremes and reset/latency
are checked before the original pulse and handover regressions. Origin session
checks deposit count=24999998 and exercise the two terminal edges, one-clock
seed, ready suppression, direct stop/fault gating and asynchronous reset. The
full 25000000-cycle half-second interval is not simulated by this unit test.
"""
from pathlib import Path
import hashlib,json,subprocess,tempfile
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
PROJECT=ROOT.parents[1]
SOURCES=[ROOT/'src/swing_control.v',ROOT/'tb/tb_swing_control.v',
         PROJECT/'v2.2/src/swing_supervisor.v',Path(__file__).resolve()]
def hashes():
    return {str(p.relative_to(PROJECT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCES}
def main():
    before=hashes(); build=Path(tempfile.mkdtemp(prefix='j280_swing_'))
    report={'status':'running','source_sha256':before,'tests':[],
      'scope':'II accelerated-enable simulation, old mode3 normal path and independent registered ADC capture readiness including +/-380 and signed13 extremes; origin delay terminal edges with counter deposit, single-cycle seed and cancellation gating (not full 25000000-cycle delay); explicit start/failure differences intentional. No hardware proof.',
      'compatibility_differences':['rising-edge explicit start; held start cannot retry',
       'stop/failure idle; failure retained until new legal edge',
       'boost_ms_q and brake_last_q reset repaired; boost timer crosses balance',
       'only mode3 fixed constants; no cfg, diagnostics or direct-balance mode']}
    try:
        for mode in ('g2005','g2012'):
            image=build/(mode+'.vvp')
            for stage,cmd in [('compile',['D:/msys64/ucrt64/bin/iverilog.exe','-'+mode,'-Wall','-s','tb_swing_control','-o',str(image),*map(str,SOURCES[:3])]),
                              ('simulate',['D:/msys64/ucrt64/bin/vvp.exe',str(image)])]:
                log=HERE/f'swing_{stage}_{mode}.log'
                with log.open('w',encoding='utf-8') as f:
                    result=subprocess.run(cmd,stdout=f,stderr=f,timeout=120)
                if result.returncode:raise RuntimeError(f'{mode} {stage}: {log}')
            summary=(HERE/f'swing_simulate_{mode}.log').read_text()
            assert 'PASS SWING' in summary
            report['tests'].append({'mode':mode,'summary':summary.strip()})
        assert hashes()==before
        report['source_hashes_unchanged_during_run']=True
        report['status']='passed'
    finally:
        (HERE/'swing_test_results.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
if __name__=='__main__':main()
