"""Offscreen Qt/WebChannel check; never opens COM or sends motor commands."""
import json
import os
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
os.environ['QT_QPA_PLATFORM']='offscreen'
os.environ['QTWEBENGINE_CHROMIUM_FLAGS']='--disable-gpu'
os.environ['QTWEBENGINE_DISABLE_SANDBOX']='1'
from PySide6.QtCore import QTimer,QSettings
from PySide6.QtWidgets import QApplication
from imu_desktop.main import MainWindow

out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
QSettings.setDefaultFormat(QSettings.IniFormat)
QSettings.setPath(QSettings.IniFormat,QSettings.UserScope,str(out/'settings'))
app=QApplication([])
window=MainWindow('motion.html');window.resize(1400,1000);window.show()
results={}
window.page.loadingChanged.connect(lambda info:results.update(load_status=info.status().name,url=info.url().toString()))
def fail():
    results['timeout']=True
    finish()
def finish():
    (out/'ui-smoke.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    window.bridge.shutdown();app.exit(1 if results.get('timeout') or not results.get('passed') else 0)
def inspect(value):
    results.update(value)
    results['passed']=(value['cards']==7 and value['joints']==3 and value['options']==6 and value['blank']>=2
                       and value['errors']==[] and value['measured']==False)
    window.grab().save(str(out/'motion-ui.png'))
    code="""JSON.stringify((()=>{const times=[];for(let i=0;i<400;i++){
      const batch=['shoulder','elbow','wrist'].map(j=>({kind:'sample',count:i*3,row:{joint:j,t_ms:i*33,calibration_id:1,measured:true,
        angles:{shoulder_plane:20,shoulder_elevation:45,shoulder_axial:10,elbow_flexion:50,elbow_supination:15,wrist_flexion:5,wrist_radial:10},
        quality:{shoulder_plane:'valid',shoulder_elevation:'valid',shoulder_axial:'valid',elbow_flexion:'valid',elbow_supination:'valid',wrist_flexion:'valid',wrist_radial:'valid'}}}));
      const t=performance.now();window.handleMotionBatch(batch);times.push((performance.now()-t)*1000);
    }times.sort((a,b)=>a-b);return {synthetic:true,batches:400,p50_us:times[200],p95_us:times[380],p99_us:times[396]};})())"""
    window.page.runJavaScript(code,0,bench_done)
def bench_done(value):
    results['ui_batch_benchmark']=json.loads(value)
    finish()
def inject():
    window.page.runJavaScript("window.__testErrors=[];window.addEventListener('error',e=>window.__testErrors.push(e.message));")
    window.bridge._run_js('window.handleMotionBatch',[{'kind':'sample','count':0,'row':{
        'joint':'shoulder','t_ms':1000,'calibration_id':0,'angles':{'shoulder_plane':None,'shoulder_elevation':0,'shoulder_axial':None},
        'quality':{'shoulder_plane':'near_singularity','shoulder_elevation':'calibration_required','shoulder_axial':'near_singularity'},'measured':False}}])
    QTimer.singleShot(300,lambda:window.page.runJavaScript("JSON.stringify({cards:document.querySelectorAll('.reading').length,joints:document.querySelectorAll('.joint-card').length,options:document.querySelectorAll('#maneuver option').length,blank:[...document.querySelectorAll('.reading strong')].filter(x=>x.textContent==='—').length,errors:window.__testErrors,measured:document.querySelector('.reading').classList.contains('invalid')===false})",0,lambda value:inspect(json.loads(value))))
window.page.loadFinished.connect(lambda ok:(results.update(load_finished_ok=bool(ok)),QTimer.singleShot(1500,inject)))
QTimer.singleShot(15000,fail)
sys.exit(app.exec())
