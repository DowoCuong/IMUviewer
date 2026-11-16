"""Live/replay motion pipeline using existing decoder and bounded pair alignment."""
from __future__ import annotations
import copy
import csv
import io
import json
import math
import os
import tempfile
import threading
import zipfile
from collections import Counter, deque
from pathlib import Path
from .alignment import SampleSynchronizer
from .angles import JointKinematics, JOINTS as LOWER_JOINTS, COMPONENTS
from .calibration import neutral_calibration, functional_calibration
from .math3d import conjugate, multiply, normalize, matrix
from .recording import RowLog, Coverage
from .upper_limb import UpperLimbKinematics, JOINTS as UPPER_JOINTS, COLUMNS, DIAGNOSTIC, decompose
from .upper_limb_calibration import reference, capture, accepted, MANEUVERS
from .serial import SerialReader
from .radio_bench import RadioBench


class MotionSession:
    def __init__(self, emit=lambda _:None):
        self.emit = emit
        self.lock = threading.RLock()
        self.model_id, self.side = 'upper_limb_v1', 'right'
        self.engine = UpperLimbKinematics()
        self.rows, self.raw = RowLog(), RowLog()
        self.coverage = Coverage()
        self.profiles = []
        self.calibration_id = 0
        self.origin = None
        self.latest = {}
        self.sensor_identities={}
        self.profile_identities={}
        self.sessions = {}
        self.domain = None
        self.counts = Counter()
        self.recent = deque(maxlen=350)
        self.collecting = None
        self.frames = []
        self.capture_windows = []
        self.capture_start = None
        self.running = False
        self.closed = False
        self.error = None
        self.epoch_broken=False
        self.last_health = {}
        self.radio=RadioBench()
        self.node_status={}
        self.validated_lower = set()
        self.fitted_lower = set()
        self.lower_fit_windows = {}
        self.last_processed = {}
        self.reset_sync()
        self.reader = SerialReader(lambda _:None, self.status, on_record=self.record,
            on_health=self.health, sample_handler=self.ingest)
        self.latency_source='not_applied'

    @property
    def joints(self):
        return UPPER_JOINTS if self.model_id=='upper_limb_v1' else LOWER_JOINTS

    @property
    def columns(self):
        return list(COLUMNS+DIAGNOSTIC) if self.model_id=='upper_limb_v1' else [f'{j}_{c}' for j in LOWER_JOINTS for c in COMPONENTS]

    def reset_sync(self):
        self.sync = {j:SampleSynchronizer(pair) for j,pair in self.joints.items()}
        self.full = SampleSynchronizer()
        self.recent.clear()

    def configure(self, model_id, side):
        if self.running or self.reader.is_running:
            raise ValueError('Dừng phiên trước khi đổi mô hình')
        if model_id not in ('upper_limb_v1','lower_limb_v1') or side not in ('left','right'):
            raise ValueError('Mô hình hoặc bên đo không hợp lệ')
        if model_id=='lower_limb_v1' and side!='right':
            raise ValueError('Chi dưới hiện giữ quy ước đã bàn giao; chưa kiểm chứng chuyển đổi bên trái')
        self.model_id,self.side = model_id,side
        self.engine = UpperLimbKinematics(side) if model_id=='upper_limb_v1' else JointKinematics()
        self.reader.latency_us={}; self.latency_source='not_applied'
        self.rows.close(); self.raw.close()
        self.rows,self.raw=RowLog(),RowLog()
        self.coverage=Coverage(); self.profiles=[]; self.calibration_id=0
        self.origin=None; self.latest={}; self.sessions={}; self.domain=None; self.sensor_identities={}; self.profile_identities={}
        self.epoch_broken=False
        self.counts=Counter(); self.error=None; self.collecting=None; self.frames=[]
        self.capture_windows=[]; self.capture_start=None
        self.validated_lower=set()
        self.radio=RadioBench(); self.node_status={}
        self.fitted_lower=set(); self.lower_fit_windows={}; self.last_processed={}
        self.reset_sync()

    def start(self, port, baud=115200):
        if not port:
            raise ValueError('Chọn đúng cổng hub trước khi kết nối')
        self.reader.mode='binary'
        self.running=True
        ok,message,_=self.reader.start(port,baud)
        if not ok:
            self.running=False
            raise ValueError(message)

    def status(self, message):
        self.emit({'kind':'status','message':message})

    def health(self, health):
        self.last_health=dict(health,calibration=dict(self.node_status),radio=self.radio.snapshot())
        # No new samples: never present cached angles as current measurements.
        if not any(health.get('nodes',{}).values()):
            self.latest={}
        self.emit({'kind':'health','health':self.last_health})

    def record(self, record):
        with self.lock:
            try:
                self.raw.append(record)
            except (OSError,ValueError) as exc:
                self.error='Lỗi ghi raw: '+str(exc)
                self.running=False; self.reader._stop_event.set()
                raise
            self.radio.add(record)
            if 'calibration' in record and not record.get('epoch_stale'):
                self.node_status[str(record['node'])]=record['calibration']

    def ingest(self, sample):
        with self.lock:
            if self.error:
                raise OSError(self.error)
            if sample.record.get('epoch_stale'): self.counts['old_epoch']+=1; return
            if 'sensor_epoch' in sample.record: self.sensor_identities[str(sample.node)]=[sample.session,sample.record['sensor_epoch']]
            expected=self.profile_identities.get(str(sample.node))
            if expected and self.sensor_identities.get(str(sample.node))!=expected:
                self.engine.reset();self.reset_sync();self.latest={};self.epoch_broken=True;self.running=False
                self.profile_identities={}
                self.status('Profile thuộc epoch khác; tạo phiên mới và hiệu chuẩn lại')
            if sample.record.get('epoch_changed'):
                self.counts['sensor_epoch_changes']+=1
                self.engine.reset(); self.reset_sync(); self.latest={}
                self.validated_lower.clear(); self.collecting=None; self.frames=[]
                self.running=False; self.epoch_broken=True
                self.status('Fusion đổi epoch: bắt đầu phiên mới và hiệu chuẩn lại; raw vẫn được thu')
            if self.epoch_broken: return
            if sample.time_us is None:
                self.counts['transport_only_or_unmapped']+=1
                return
            if ((sample.node in self.sessions and self.sessions[sample.node]!=sample.session)
                    or (self.domain is not None and self.domain!=sample.domain)):
                self.engine.reset(); self.reset_sync(); self.latest={}
                self.validated_lower.clear(); self.collecting=None; self.frames=[]
                self.running=False
                self.error='Thiết bị/clock reset: xuất phiên hiện tại rồi bắt đầu phiên mới và hiệu chuẩn lại'
                self.reader._stop_event.set()
                self.status(self.error)
                return
            self.sessions[sample.node]=sample.session; self.domain=sample.domain
            if self.origin is None:
                self.origin=math.ceil(sample.time_us/10000)*10000
            if self.coverage.first is None:
                self.coverage.first=self.origin
            self.coverage.last=max(self.coverage.last or self.origin, math.floor(sample.time_us/10000)*10000)
            if not sample.usable:
                self.counts['unusable_node_'+str(sample.node)]+=1
                for joint,pair in self.joints.items():
                    if sample.node in pair:
                        if self.latest.pop(joint,None) is not None:
                            self.emit({'kind':'invalid','joint':joint,'reason':'calibration_required'})
            for frame in self.full.add(sample):
                self.recent.append(frame)
                if self.collecting:
                    if len(self.frames)>=6000:
                        self.end_capture_window(); self.collecting=None; self.frames=[]
                        self.status('Ghi hiệu chuẩn quá 60 giây; thực hiện lại')
                    else:
                        self.frames.append(frame)
            for joint,sync in self.sync.items():
                for frame in sync.add(sample):
                    self.process_frame(joint,frame)
            for joint,data in list(self.latest.items()):
                if sample.time_us-data['time_us']>30000:
                    del self.latest[joint]
                    self.emit({'kind':'invalid','joint':joint,'reason':'stale'})

    def ready(self):
        if self.model_id=='upper_limb_v1':
            return accepted(self.engine)
        return self.engine.calibration_kind in ('functional','measured_profile') and self.validated_lower==set(LOWER_JOINTS)

    def process_frame(self, joint, frame):
        if frame.time_us <= self.last_processed.get(joint, -math.inf):
            return
        self.last_processed[joint] = frame.time_us
        if self.model_id=='upper_limb_v1':
            result=self.engine.compute_detailed(frame.orientations)
            prefix=joint+'_'
            angles={k:v for k,v in result['angles'].items() if k.startswith(prefix)}
            quality={k:result['quality'][k] for k in angles}
            relative=result['relative'][joint]
            condition=result['conditioning'][joint]
        else:
            # Same operations/signs as legacy compute, on a single valid pair.
            parent,child=self.joints[joint]
            segments={n:normalize(multiply(q,self.engine.sensor_to_segment.get(n,(1.,0.,0.,0.))))
                      for n,q in frame.orientations.items()}
            q=normalize(multiply(conjugate(segments[parent]),segments[child]))
            basis=self.engine.joint_frames[joint]
            relative=normalize(multiply(multiply(conjugate(basis),q),basis))
            values,condition=decompose(relative,'ZXY',1e-6)
            angles={f'{joint}_{c}':None if v is None else v*s for c,v,s in zip(COMPONENTS,values,self.engine.signs[joint])}
            quality={k:'near_singularity' if v is None else 'valid' for k,v in angles.items()}
        measured=self.ready() and self.collecting is None
        if not measured:
            quality={k:'calibration_required' if v is not None else quality[k] for k,v in angles.items()}
        row={'time_us':frame.time_us,'t_ms':(frame.time_us-self.origin)/1000,
             'model_id':self.model_id,'side':self.side,'joint':joint,'calibration_id':self.calibration_id,
             'angles':angles,'quality':quality,'relative':list(relative),'conditioning':condition,
             'clock_uncertainty_us':frame.clock_uncertainty_us,'interpolation_gap_us':frame.interpolation_gap_us,
             'sample_time_basis':frame.time_basis,'measured':measured}
        self.latest[joint]=row
        if measured:
            try:
                self.rows.append(row)
            except (OSError,ValueError) as exc:
                self.error='Lỗi ghi dữ liệu: '+str(exc)
                self.reader._stop_event.set(); self.running=False
                raise
            self.coverage.add(frame.time_us,angles,quality)
        self.emit({'kind':'sample','row':row,'count':len(self.rows)})

    def capture_neutral(self):
        with self.lock:
            candidate=copy.deepcopy(self.engine)
            report=reference(candidate,list(self.recent)) if self.model_id=='upper_limb_v1' else neutral_calibration(candidate,list(self.recent))
            self.engine=candidate; self.validated_lower.clear()
            self.fitted_lower.clear(); self.lower_fit_windows.clear()
            self.end_capture_window(); self.collecting=None; self.frames=[]
            self.save_change(report)
            return report

    def functional(self, name, finish=False, validate=False):
        allowed=MANEUVERS if self.model_id=='upper_limb_v1' else LOWER_JOINTS
        with self.lock:
            if name not in allowed or not self.engine.calibrated:
                raise ValueError('Lấy mốc chuẩn và chọn đúng chuyển động')
            if not finish:
                if self.collecting:
                    raise ValueError('Hoàn tất chuyển động đang ghi trước')
                self.collecting=(name,validate); self.frames=[]
                self.capture_start=len(self.raw)
                return {'collecting':name,'validation':validate}
            if self.collecting!=(name,validate):
                raise ValueError('Chưa bắt đầu cửa sổ ghi này')
            frames=self.frames; self.frames=[]; self.collecting=None
            self.end_capture_window()
            candidate=copy.deepcopy(self.engine)
            if self.model_id=='upper_limb_v1':
                report=capture(candidate,name,frames,validate)
            else:
                original=copy.deepcopy(candidate)
                if validate and (not frames or name not in self.lower_fit_windows or frames[0].time_us<=self.lower_fit_windows[name][1]):
                    raise ValueError('Kiểm tra phải dùng đoạn chuyển động mới sau lần khớp trục')
                report=functional_calibration(candidate,name,frames)
                if validate:
                    from .math3d import distance_deg
                    report['frame_repeat_error_deg']=distance_deg(original.joint_frames[name],candidate.joint_frames[name])
                    report['accepted']=report['frame_repeat_error_deg']<=5.
                    candidate=original
                    if report['accepted']:
                        self.validated_lower.add(name)
                    else:
                        self.validated_lower.discard(name)
                else:
                    self.validated_lower.discard(name)
                    self.fitted_lower.add(name)
                    self.lower_fit_windows[name]=(frames[0].time_us,frames[-1].time_us)
                    if self.fitted_lower>=set(LOWER_JOINTS):
                        candidate.calibration_kind='functional'
            self.engine=candidate
            self.save_change(report)
            return report

    def save_change(self, report):
        self.calibration_id+=1
        self.latest={}
        self.reset_sync()
        self.profiles.append({'id':self.calibration_id,'after_record':len(self.raw),
                             'profile':self.profile(),'report':report})

    def profile(self):
        p=self.engine.profile()
        if self.model_id=='lower_limb_v1':
            p.update(schema_version=1,model_id=self.model_id,side=self.side,
                node_mapping={'1':'pelvis','2':'thigh','3':'shank','4':'foot'},
                quaternion_convention='Hamilton_wxyz_qAB_maps_B_to_A',
                segment_frames=p['axes'],angle_conventions={'all':'intrinsic_ZXY'},
                reference_pose='legacy_upright',calibration_report={},
                validated_lower=sorted(self.validated_lower))
        return dict(p,sensor_identities=self.reader_identity(),calibration_id=self.calibration_id,
                    latency_parameters={'node_us':dict(self.reader.latency_us),'source':self.latency_source})

    def load_profile(self, p, save=True):
        if p.get('sensor_identities') and self.sensor_identities and p['sensor_identities']!=self.sensor_identities:
            raise ValueError('Profile thuộc epoch cảm biến khác; hiệu chuẩn lại')
        if self.collecting:
            raise ValueError('Hoàn tất lần ghi hiệu chuẩn trước khi mở profile')
        self.profile_identities=copy.deepcopy(p.get('sensor_identities',{}))
        if self.model_id=='upper_limb_v1':
            candidate=UpperLimbKinematics(self.side); candidate.load_profile(p)
        else:
            if p.get('model_id','lower_limb_v1')!='lower_limb_v1' or p.get('side',self.side)!=self.side:
                raise ValueError('Profile khác mô hình')
            candidate=JointKinematics(); candidate.load_profile(p)
        latency={int(k):float(v) for k,v in p.get('latency_parameters',{}).get('node_us',p.get('latency_us',{})).items()}
        if any(k not in range(1,5) or not math.isfinite(v) or abs(v)>100000 for k,v in latency.items()):
            raise ValueError('Hiệu chỉnh độ trễ không hợp lệ')
        self.engine=candidate; self.reader.latency_us=latency
        self.latency_source=str(p.get('latency_parameters',{}).get('source','legacy_unverified'))
        self.validated_lower=set(p.get('validated_lower',[]))
        if save:
            self.save_change({'imported':True})
        else:
            self.calibration_id=int(p.get('calibration_id',0)); self.latest={}; self.reset_sync()

    def reader_identity(self):
        return dict(self.sensor_identities)

    def metadata(self):
        from .quality import POLICY
        return {'schema_version':1,'model_id':self.model_id,'side':self.side,
            'strict_quality':self.reader.strict_quality,'quality_policy':POLICY,'sensor_identities':self.reader_identity(),
            'quantitative_stopped_reason':'sensor_epoch_changed' if self.epoch_broken else None,
            'time_origin_us':self.origin,'profiles':self.profiles,'coverage':self.coverage.snapshot(),
            'capture_counts':dict(self.counts),'health':self.last_health,'error':self.error,
            'radio_bench':self.radio.snapshot(),
            'capture_windows':list(self.capture_windows),
            'rows':len(self.rows),'measurement_filter_hz':0,'protocol':'binary',
            'accuracy':'Software validation only; in-vivo RMSE <1 degree is unproven',
            'time_basis':'Mapped I2C completion, not verified BNO fusion epoch'}

    def export(self, destination):
        if self.reader.is_running:
            raise ValueError('Dừng thu nhận trước khi xuất để cố định dữ liệu và metadata')
        self.end_capture_window()
        destination=Path(destination)
        destination.parent.mkdir(parents=True,exist_ok=True)
        fields=['t_ms','joint',*self.columns,'model_id','side','calibration_id','qw','qx','qy','qz',
                'quality_json','clock_uncertainty_us','interpolation_gap_us','sample_time_basis']
        temps=[]
        try:
            for suffix in ('.csv','.zip'):
                f=tempfile.NamedTemporaryFile(dir=destination.parent,suffix=suffix,delete=False); f.close(); temps.append(Path(f.name))
            with temps[0].open('w',encoding='utf-8-sig',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=fields); writer.writeheader()
                for r in self.rows:
                    row={k:r[k] for k in ('t_ms','joint','model_id','side','calibration_id','clock_uncertainty_us','interpolation_gap_us','sample_time_basis')}
                    row.update(r['angles']); row.update(zip(('qw','qx','qy','qz'),r['relative']))
                    row['quality_json']=json.dumps(r['quality'],separators=(',',':'))
                    writer.writerow(row)
            with zipfile.ZipFile(temps[1],'w',zipfile.ZIP_DEFLATED) as z:
                z.write(temps[0],'angles.csv')
                z.writestr('metadata.json',json.dumps(self.metadata(),indent=2,allow_nan=False))
                for name,log in (('raw.jsonl',self.raw),('results.jsonl',self.rows)):
                    with z.open(name,'w') as stream:
                        for row in log:
                            stream.write((json.dumps(row,allow_nan=False,separators=(',',':'))+'\n').encode())
            os.replace(temps[1],destination.with_suffix('.session.zip'))
            os.replace(temps[0],destination)
        finally:
            for p in temps:
                p.unlink(missing_ok=True)
        return self.metadata()

    def stop(self):
        self.reader.stop(); self.running=False; self.latest={}
        self.end_capture_window(); self.collecting=None; self.frames=[]

    def end_capture_window(self):
        if self.capture_start is not None:
            self.capture_windows.append({'start_record':self.capture_start,'end_record':len(self.raw)})
            self.capture_start=None

    def close(self):
        if self.closed:
            return
        self.stop(); self.rows.close(); self.raw.close(); self.closed=True
