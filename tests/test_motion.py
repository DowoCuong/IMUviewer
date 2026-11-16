"""Synthetic verification; no hardware accuracy claim."""
import copy
import csv
import json
import math
import random
import tempfile
import zipfile
from pathlib import Path
import pytest
from imu_desktop.core.math3d import IDENTITY, axis_angle, multiply, conjugate, distance_deg, from_matrix
from imu_desktop.core.upper_limb import UpperLimbKinematics, decompose, MANEUVERS
from imu_desktop.core.upper_limb_calibration import capture, reference, accepted
from imu_desktop.core.alignment import JointFrame
from imu_desktop.core.motion import MotionSession
from imu_desktop.core.recording import RowLog
from imu_desktop.core.replay_motion import replay
from imu_desktop.core.protocol import StreamDecoder
from test_protocol_alignment import wire
from test_replay import quality_payload


def product(*qs):
    q=IDENTITY
    for v in qs:q=multiply(q,v)
    return q


def ready_engine(side='right'):
    e=UpperLimbKinematics(side)
    e.sensor_to_segment={n:IDENTITY for n in range(1,5)}
    e.calibration_kind='functional'
    e.validation={m:{'accepted':True,'synthetic':True} for m in MANEUVERS}
    return e


def calibration_frames(name,start=0, amplitude=35):
    from imu_desktop.core.upper_limb import JOINTS
    joint,axis=MANEUVERS[name]
    axis={'x':(1,0,0),'y':(0,1,0),'z':(0,0,1)}[axis]
    sign=-1 if name in ('shoulder_abduction','elbow_supination','wrist_radial') else 1
    frames=[]
    for i in range(301):
        q={n:IDENTITY for n in range(1,5)}
        moving=axis_angle(axis,sign*amplitude*math.sin(i/300*2*math.pi))
        for n in range(JOINTS[joint][1],5):q[n]=moving
        frames.append(JointFrame(start+i*10000,q,20,10000,'synthetic'))
    return frames


@pytest.mark.parametrize('sequence', ['YXY','ZXY'])
def test_decomposition_against_independent_matrix_oracle(sequence):
    rng=random.Random(811)
    def mat(axis,a):
        c,s=math.cos(math.radians(a)),math.sin(math.radians(a))
        return {'X':((1,0,0),(0,c,-s),(0,s,c)),
                'Y':((c,0,s),(0,1,0),(-s,0,c)),
                'Z':((c,-s,0),(s,c,0),(0,0,1))}[axis]
    def mm(a,b):return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)) for i in range(3))
    for _ in range(200):
        a,c=rng.uniform(-179,179),rng.uniform(-179,179)
        b=rng.uniform(5,175) if sequence=='YXY' else rng.uniform(-85,85)
        expected=mm(mm(mat(sequence[0],a),mat(sequence[1],b)),mat(sequence[2],c))
        actual,condition=decompose(from_matrix(expected),sequence)
        assert actual==pytest.approx((a,b,c),abs=1e-8)


@pytest.mark.parametrize('angle',[0,.1,179.9,180])
def test_shoulder_singular_retains_elevation_and_quaternion(angle):
    e=ready_engine()
    r=e.compute_detailed({1:IDENTITY,2:axis_angle((1,0,0),angle)})
    assert r['angles']['shoulder_plane'] is None
    assert r['angles']['shoulder_axial'] is None
    assert r['angles']['shoulder_elevation']==pytest.approx(angle,abs=1e-6)
    assert len(r['relative']['shoulder'])==4


@pytest.mark.parametrize('side,rotation,radial',[('right',-1,-1),('left',1,1)])
def test_side_specific_signs_and_common_world_rotation(side,rotation,radial):
    e=ready_engine(side)
    q={1:axis_angle((1,0,0),30),2:axis_angle((1,0,0),60)}
    q[3]=product(q[2],axis_angle((0,0,1),40),axis_angle((0,1,0),rotation*25))
    q[4]=product(q[3],axis_angle((1,0,0),radial*15))
    a=e.compute_detailed(q)
    assert a['angles']['elbow_flexion']==pytest.approx(40)
    assert a['angles']['elbow_supination']==pytest.approx(25)
    assert a['angles']['wrist_radial']==pytest.approx(15)
    common=product(axis_angle((1,0,0),53),axis_angle((0,1,0),-71))
    for transformed in ({n:product(common,v) for n,v in q.items()}, {n:tuple(-x for x in v) for n,v in q.items()}):
        b=e.compute_detailed(transformed)
        for key,value in a['angles'].items():
            assert b['angles'][key]==pytest.approx(value,abs=1e-6) if value is not None else b['angles'][key] is None


def test_mounting_offsets_and_independent_joint_frames():
    e=ready_engine()
    e.sensor_to_segment={n:axis_angle((1,2,n),n*17) for n in range(1,5)}
    q={n:axis_angle((1,n,2),n*8) for n in range(1,5)}
    sensors={n:product(q[n],conjugate(e.sensor_to_segment[n])) for n in q}
    expected=ready_engine().compute_detailed(q)
    actual=e.compute_detailed(sensors)
    assert actual['angles']==pytest.approx(expected['angles'])
    e.parent_frames['elbow']=axis_angle((1,0,0),20)
    e.child_frames['elbow']=axis_angle((0,0,1),10)
    expected=product(conjugate(e.parent_frames['elbow']),conjugate(q[2]),q[3],e.child_frames['elbow'])
    assert distance_deg(actual['relative']['elbow'],expected)>1
    assert distance_deg(e.compute_detailed(sensors)['relative']['elbow'],expected)<1e-6


@pytest.mark.parametrize('q',[(0,0,0,0),(math.nan,0,0,1),(1,2,3)])
def test_invalid_quaternion_rejected(q):
    with pytest.raises(ValueError):ready_engine().compute_detailed({1:q,2:IDENTITY})


def test_profile_roundtrip_rejects_model_side_and_convention():
    e=ready_engine();p=e.profile();e.load_profile(p)
    for key,value in [('model_id','lower_limb_v1'),('side','left'),('schema_version',99),('singularity_threshold',float('nan')),('angle_conventions',{})]:
        invalid=copy.deepcopy(p);invalid[key]=value
        with pytest.raises(ValueError):e.load_profile(invalid)


def test_functional_frames_and_held_out_validation():
    e=ready_engine();e.validation={};e.calibration_report={}
    for i,m in enumerate(MANEUVERS):capture(e,m,calibration_frames(m,i*4000000))
    assert e.calibration_kind=='functional'
    assert not accepted(e)
    for i,m in enumerate(MANEUVERS):
        report=capture(e,m,calibration_frames(m,30000000+i*4000000),validate=True)
        assert report['accepted']
    assert accepted(e)
    assert distance_deg(e.parent_frames['elbow'],IDENTITY)<1e-6
    capture(e,'elbow_flexion',calibration_frames('elbow_flexion',70000000))
    assert not accepted(e)


def test_calibration_rejects_weak_gap_and_reused_window():
    e=ready_engine()
    for frames in (calibration_frames('elbow_flexion',amplitude=2),calibration_frames('elbow_flexion')[:20]):
        with pytest.raises(ValueError):capture(e,'elbow_flexion',frames)
    frames=calibration_frames('elbow_flexion')
    capture(e,'elbow_flexion',frames)
    with pytest.raises(ValueError):capture(e,'elbow_flexion',frames,validate=True)
    broken=calibration_frames('elbow_flexion');del broken[30:40]
    with pytest.raises(ValueError):capture(e,'elbow_flexion',broken)


def test_disk_log_repeated_iteration_and_bounded_state():
    log=RowLog()
    try:
        for i in range(10000):log.append({'t':i,'x':-i})
        assert len(log)==10000
        assert sum(r['x'] for r in log)==-49995000
        assert sum(1 for _ in log)==10000
        assert not any(isinstance(v,list) for v in vars(log).values())
    finally:log.close()


def make_session():
    s=MotionSession();s.engine=ready_engine();s.save_change({'synthetic':True})
    return s


def feed(s,decoder,node,t,seq,q=IDENTITY,calibration=255):
    payload=bytearray(quality_payload(node,t,offset_us=node*100000,seq=seq,q=q))
    payload[32]=calibration
    payload[33]=2 if calibration==255 else 1
    for sample in decoder.feed(wire(payload)):
        s.record(sample.record);s.ingest(sample)


def test_partial_pair_calibration_drop_duplicate_and_reset():
    s=make_session();d=StreamDecoder()
    try:
        for seq in range(12):
            for node in range(1,5):feed(s,d,node,1000000+seq*10000,seq)
        count=len(s.rows)
        feed(s,d,4,1110000,11)
        assert len(s.rows)==count
        for seq in range(12,24):
            for node in (1,2,3):feed(s,d,node,1000000+seq*10000,seq)
        assert 'elbow' in s.latest and 'wrist' not in s.latest
        feed(s,d,3,1240000,24,calibration=243)
        assert 'elbow' not in s.latest
        meta=s.metadata();assert meta['coverage']['valid_fraction']['wrist_flexion']<1
        sample=d.feed(wire(quality_payload(1,1250000,seq=25)))[0]
        from dataclasses import replace
        s.ingest(replace(sample,session=sample.session+1))
        assert s.error and not s.engine.calibrated and s.latest=={}
    finally:s.close()


def test_export_replay_identical_and_empty_csv_fields(tmp_path):
    s=make_session();d=StreamDecoder()
    try:
        for seq in range(41):
            for node in range(1,5):feed(s,d,node,1000000+seq*10000,seq)
        dest=tmp_path/'live.csv';s.export(dest)
        with dest.open(encoding='utf-8-sig',newline='') as stream:
            first=next(csv.DictReader(stream))
            assert first['shoulder_plane']=='' and first['elbow_flexion']==''
        result=replay(dest.with_suffix('.session.zip'),tmp_path/'replay.csv')
        try:
            assert list(result.rows)==list(s.rows)
            assert result.coverage.snapshot()==s.coverage.snapshot()
        finally:result.close()
    finally:s.close()


def test_writer_failure_visible(tmp_path):
    s=make_session();d=StreamDecoder()
    try:
        s.rows.file.close()
        with pytest.raises((ValueError,OSError)):
            for node in (1,2):feed(s,d,node,1000000,0)
        assert s.error and s.reader._stop_event.is_set()
    finally:s.close()


def test_lower_motion_pair_matches_legacy_without_changing_gait():
    s=MotionSession()
    try:
        s.configure('lower_limb_v1','right')
        s.engine.sensor_to_segment={n:IDENTITY for n in range(1,5)}
        s.engine.calibration_kind='functional';s.validated_lower={'hip','knee','ankle'}
        q={1:IDENTITY,2:axis_angle((0,0,1),20),3:axis_angle((0,0,1),-10),4:axis_angle((0,0,1),5)}
        expected=s.engine.compute(q)
        for joint,pair in s.joints.items():
            s.origin=0
            s.process_frame(joint,JointFrame(10000,{n:q[n] for n in pair},20,0,'test'))
        for row in s.rows:
            for key,value in row['angles'].items():assert value==pytest.approx(expected[key])
    finally:s.close()


def test_replay_preserves_capture_exclusion_and_profile_transition(tmp_path):
    s=make_session();d=StreamDecoder()
    try:
        for seq in range(30):
            if seq==5:
                s.functional('elbow_flexion')
            if seq==15:
                s.end_capture_window();s.collecting=None;s.frames=[]
                s.save_change({'synthetic_profile_transition':True})
            for node in range(1,5):feed(s,d,node,1000000+seq*10000,seq)
        dest=tmp_path/'capture.csv';s.export(dest)
        other=replay(dest.with_suffix('.session.zip'))
        try:assert list(other.rows)==list(s.rows)
        finally:other.close()
    finally:s.close()


def test_type10_never_produces_motion_rows():
    from test_radio_bench import packet
    from imu_desktop.core.protocol import encode_frame
    s=make_session();d=StreamDecoder()
    try:
        for n in (3,4):
            for sample in d.feed(encode_frame(packet(node=n))):s.record(sample.record);s.ingest(sample)
        assert len(s.rows)==0 and s.latest=={}
    finally:s.close()


def test_profile_change_and_writer_error_do_not_silently_continue():
    s=make_session()
    try:
        s.functional('wrist_flexion')
        with pytest.raises(ValueError):s.load_profile(s.profile())
        with pytest.raises(ValueError):s.functional('wrist_radial')
    finally:s.close()


def test_exact_error_metrics_and_missing_reference(tmp_path):
    from tools.summarize_motion_errors import summarize
    p=tmp_path/'errors.csv';p.write_text('error_deg\n-2\n-1\n0\n1\n2\n""\n')
    m=summarize(p)
    assert m['rmse_deg']==pytest.approx(math.sqrt(2))
    assert m['bias_deg']==0 and m['mae_deg']==1.2 and m['p95_abs_deg']==2
    assert m['valid_fraction_of_input_rows']==pytest.approx(5/6)
    p.write_text('error_deg\n""\n')
    assert summarize(p)['rmse_deg'] is None
    p.write_text('error_deg,reference_speed\n-2,20\n2,200\n,20\n')
    bins=summarize(p,speed_column='reference_speed')['speed_bins']
    assert bins['0_to_30']['rmse_deg']==2 and bins['0_to_30']['valid_fraction_of_input_rows']==.5
    assert bins['above_180']['bias_deg']==2


def test_bad_raw_writer_stops_and_uncalibrated_node_invalidates_ui():
    events=[];s=make_session();s.emit=events.append;d=StreamDecoder()
    try:
        for node in range(1,5):feed(s,d,node,1000000,0)
        feed(s,d,4,1010000,1,calibration=243)
        assert any(e.get('kind')=='invalid' and e['joint']=='wrist' for e in events)
        s.raw.file.close()
        with pytest.raises(ValueError):s.record({'test':True})
        assert s.error and s.reader._stop_event.is_set()
    finally:s.close()


def test_nearly_collinear_directions_rejected_and_validation_can_fail():
    e=ready_engine();e.calibration_report={};e.validation={}
    capture(e,'wrist_flexion',calibration_frames('wrist_flexion'))
    bad=calibration_frames('wrist_flexion',4000000)
    with pytest.raises(ValueError):capture(e,'wrist_radial',bad)
    capture(e,'elbow_flexion',calibration_frames('elbow_flexion'))
    bad=calibration_frames('elbow_flexion',10000000)
    tilt=axis_angle((1,0,0),20)
    for f in bad:
        for n in (3,4):f.orientations[n]=product(tilt,f.orientations[n],conjugate(tilt))
    assert not capture(e,'elbow_flexion',bad,validate=True)['accepted']


def test_actual_serial_callback_path_and_latency_replay(tmp_path):
    import threading
    from test_replay import ReplayPort
    s=make_session()
    try:
        p=s.profile();p['latency_parameters']['node_us']={'1':1000,'2':2000,'3':1500,'4':3000}
        s.load_profile(p)
        data=bytearray()
        for seq in range(30):
            for n in range(1,5):data.extend(wire(quality_payload(n,1000000+seq*10000,seq=seq,offset_us=n*100000)))
        stop=threading.Event()
        port=ReplayPort(data,stop)
        s.reader._read_loop('SYNTHETIC_NO_COM',port,stop)
        assert port.closed and len(s.rows)>0 and len(s.raw)==120
        dest=tmp_path/'callback.csv';s.export(dest)
        other=replay(dest.with_suffix('.session.zip'))
        try:assert list(other.rows)==list(s.rows)
        finally:other.close()
    finally:s.close()
