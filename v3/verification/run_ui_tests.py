"""II: actual UI, EC11, LCD, bridge and V3 controls. Accelerated ms ticks,
ADC sample terminal counts, one real physical EC press debounce terminal,
LCD delays and origin counter are preloaded; divider is cleared before each
accelerated tick to prevent extra real-time tick insertion. UI states are never forced.
V3.2 covers position-mode toggling, signed130 steps, +1040 clockwise full circle,
busy rejection, endpoint hold, fault cancellation and calibration.
No board timing, physical motion or full real-time debounce claim.
"""
from pathlib import Path
import hashlib,json,subprocess,tempfile
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
SOURCES=[ROOT/'src'/n for n in ('top.v','ec11_input.v','ui_controller.v','ui_control_bridge.v','local_startup_profile.v','ui_lcd.v','adc_pend.v','encoder.v','swing_control.v','balance_control.v','motro.v','position_reference.v')]
SOURCES += [ROOT/'tb/tb_ui_top.v',ROOT/'src/ui_font.vh',Path(__file__).resolve()]
def hashes():
 return {str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCES}
def main():
 before=hashes();report={'status':'running','scope':__doc__,'source_sha256':before,'tests':[]}
 try:
  with tempfile.TemporaryDirectory(prefix='v32ui_') as tmp:
   for mode in ('g2005','g2012'):
    image=Path(tmp)/(mode+'.vvp')
    with (HERE/('ui_compile_'+mode+'.log')).open('w',encoding='utf-8') as log:
     subprocess.run(['D:/msys64/ucrt64/bin/iverilog.exe','-'+mode,'-Wall','-I',str(ROOT/'src'),'-s','tb_ui_top','-o',str(image),*map(str,SOURCES[:-3]),str(ROOT/'tb/tb_ui_top.v')],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
    path=HERE/('ui_simulate_'+mode+'.log')
    with path.open('w',encoding='utf-8') as log:
     subprocess.run(['D:/msys64/ucrt64/bin/vvp.exe',str(image)],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=300)
    text=path.read_text(encoding='utf-8');assert 'PASS UI_TOP' in text,text
    report['tests'].append({'mode':mode,'exit':0,'summary':text.strip()})
  report['source_hashes_unchanged_during_run']=before==hashes()
  assert report['source_hashes_unchanged_during_run']
  report['status']='passed'
 except Exception as error:
  report['status']='failed';report['error']=str(error);raise
 finally:
  (HERE/'ui_test_results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
  print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
