"""Real 50MHz ADC/observer/control integration smoke with the actual balance PD."""
from pathlib import Path
import subprocess, tempfile, hashlib, json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
PROJECT=ROOT.parents[1]
SOURCES=[ROOT/'src'/n for n in ('adc_pend.v','swing_control.v','balance_control.v')]+[ROOT/'tb/tb_pendulum_startup.v',Path(__file__).resolve()]
def hashes():
    return {p.relative_to(PROJECT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCES}
def main():
    before=hashes();build=Path(tempfile.mkdtemp(prefix='j280_startup_'))
    report={'status':'running','scope':'Real 50MHz ADC and four-mode startup integration with actual balance PD; separate balance differential tests verify numeric behavior. Origin delay preloaded near terminal count: tests last two cycles and real PD origin seeding, not full 0.5s wait. No board or complete safety-system claim','source_sha256':before,'tests':[]}
    try:
        for mode in ('g2005','g2012'):
            image=build/(mode+'.vvp')
            commands=[['D:/msys64/ucrt64/bin/iverilog.exe','-'+mode,'-Wall','-s','tb_pendulum_startup','-o',str(image),*[str(p) for p in SOURCES[:-1]]],['D:/msys64/ucrt64/bin/vvp.exe',str(image)]]
            for stage,cmd in zip(('compile','simulate'),commands):
                log=HERE/f'startup_{stage}_{mode}.log'
                with log.open('w',encoding='utf-8') as stream:
                    code=subprocess.run(cmd,stdout=stream,stderr=stream,timeout=180).returncode
                if code:raise RuntimeError(f'{stage} {mode} failed: {log}')
            text=log.read_text(encoding='utf-8')
            if 'PASS STARTUP' not in text:raise RuntimeError('missing PASS')
            report['tests'].append({'mode':mode,'summary':text.strip(),'exit':0})
        report['source_hashes_unchanged_during_run']=before==hashes()
        if not report['source_hashes_unchanged_during_run']:raise RuntimeError('source changed')
        report['status']='passed'
    except Exception as error:
        report['status']='failed';report['error']=str(error);raise
    finally:
        (HERE/'startup_test_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
