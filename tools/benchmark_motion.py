"""Synthetic wire load benchmark; virtual duration is not hardware observation."""
import argparse
import ctypes
import json
import math
import statistics
import struct
import time
from pathlib import Path
from imu_desktop.core.motion import MotionSession
from imu_desktop.core.protocol import StreamDecoder,encode_frame
from imu_desktop.core.upper_limb import MANEUVERS
from imu_desktop.core.math3d import IDENTITY,axis_angle,multiply

def percentile(a,p):
    a=sorted(a);return a[round((len(a)-1)*p)] if a else None
def summarize(a):return dict(p50_us=percentile(a,.5),p95_us=percentile(a,.95),p99_us=percentile(a,.99))
def rss():
    class Counters(ctypes.Structure):
        _fields_=[('cb',ctypes.c_ulong),('faults',ctypes.c_ulong)]+[(name,ctypes.c_size_t) for name in ('peak','working','pmin','pmax','npmin','npmax','page','peakpage')]
    c=Counters();c.cb=ctypes.sizeof(c)
    current=ctypes.windll.kernel32.GetCurrentProcess
    current.restype=ctypes.c_void_p
    query=ctypes.windll.psapi.GetProcessMemoryInfo
    query.argtypes=[ctypes.c_void_p,ctypes.POINTER(Counters),ctypes.c_ulong]
    if not query(current(),ctypes.byref(c),c.cb):
        raise ctypes.WinError()
    return c.working
def wire(node,t,seq,moving=False):
    p=bytearray(64)
    struct.pack_into('<BBBBIII',p,0,1,9,node,3,12,seq,seq)
    q=IDENTITY
    if moving and node>=2:
        phase=seq*.025
        q=axis_angle((0,0,1),40+20*math.sin(phase))
        if node>=3:
            q=multiply(multiply(q,axis_angle((0,0,1),50+25*math.sin(phase*.7))),axis_angle((0,1,0),20*math.sin(phase*.8)))
        if node==4:
            q=multiply(multiply(q,axis_angle((0,0,1),15*math.sin(phase*.9))),axis_angle((1,0,0),-10*math.sin(phase)))
    struct.pack_into('<Q4hBB2xI',p,16,t,*(round(v*16384) for v in q),255,2,1000)
    struct.pack_into('<IIiIII',p,40,t&0xffffffff,t&0xffffffff,0,30,77,1000)
    return encode_frame(p,(t+node*1000)&0xffffffff)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('output');ap.add_argument('--ticks',type=int,default=120000);ap.add_argument('--moving',action='store_true');args=ap.parse_args()
    s=MotionSession();s.engine.sensor_to_segment={n:IDENTITY for n in range(1,5)}
    s.engine.calibration_kind='functional';s.engine.validation={m:{'accepted':True,'synthetic':True} for m in MANEUVERS}
    s.save_change({'synthetic':True})
    decoder=StreamDecoder();stages={n:[] for n in ('decode','sync','kinematics','recording','process_frame')}
    def instrument(obj,name,stage):
        fn=getattr(obj,name)
        def run(*a,**kw):
            start=time.perf_counter_ns();r=fn(*a,**kw)
            if len(stages[stage])<20000:stages[stage].append((time.perf_counter_ns()-start)/1000)
            return r
        setattr(obj,name,run)
    instrument(decoder,'feed','decode')
    for sync in [s.full,*s.sync.values()]:instrument(sync,'add','sync')
    instrument(s.engine,'compute_detailed','kinematics')
    instrument(s.rows,'append','recording');instrument(s.raw,'append','recording')
    instrument(s,'process_frame','process_frame')
    memory=[];start=time.perf_counter();cpu=time.process_time()
    for seq in range(args.ticks):
        t=1000000+seq*10000
        for node in range(1,5):
            for sample in decoder.feed(wire(node,t,seq,args.moving)):
                s.record(sample.record);s.ingest(sample)
        if seq in (10000,args.ticks//2,args.ticks-1):memory.append({'tick':seq,'rss_bytes':rss()})
        if seq and seq%30000==0:print(json.dumps({'ticks':seq,'elapsed_s':round(time.perf_counter()-start,2)}),flush=True)
    result={'synthetic':True,'moving':args.moving,'virtual_duration_s':args.ticks/100,'wall_s':time.perf_counter()-start,
            'cpu_s':time.process_time()-cpu,'packets':len(s.raw),'rows':len(s.rows),
            'stages':{k:summarize(v) for k,v in stages.items()},'rss':memory,
            'queue':'synchronous buffered file writes; no writer queue',
            'coverage':s.coverage.snapshot(),'hardware_tested':False}
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    (out/'benchmark.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    s.export(out/'synthetic-motion.csv')
    print(json.dumps({k:v for k,v in result.items() if k!='coverage'},indent=2),flush=True)
    s.close()
if __name__=='__main__':main()
