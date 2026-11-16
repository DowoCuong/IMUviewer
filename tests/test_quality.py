import json,struct,zipfile
from dataclasses import replace
import pytest
from imu_desktop.core.protocol import StreamDecoder,encode_frame
from imu_desktop.core.alignment import SampleSynchronizer
from imu_desktop.core.motion import MotionSession
from imu_desktop.core.quality import EpochTracker,usable_at

def packet(node=1,t=10000,epoch=1,cal=255,state=2,ages=(0,0),valid=True,session=42,kind=11):
    p=bytearray(64)
    struct.pack_into('<BBBBIII',p,0,1,kind,node,3 if valid else 2,session,t//10000,t//10000)
    struct.pack_into('<Q4hBBBBI',p,16,t,16384,0,0,0,cal,state,*ages,epoch)
    struct.pack_into('<IIiIII',p,40,t,t,0,16,99,1000)
    return encode_frame(bytes(p),t+300)

def health(epoch=1):
    p=bytearray(64);struct.pack_into('<BBBBIII',p,0,1,12,1,2,42,0,0)
    struct.pack_into('<QIH10B',p,16,10000,epoch,0x0314,1,0,255,2,1,1,5,0,12,0)
    return encode_frame(p)

def test_type11_strict_quality_age_unknown_and_invalid_placeholder():
    d=StreamDecoder(strict_quality=True)
    s=d.feed(packet())[0];assert s.usable and s.record['sensor_epoch']==1
    assert not d.feed(packet(t=20000,ages=(100,100)))[0].usable # conservative hub transit
    assert not d.feed(packet(t=30000,ages=(255,0)))[0].usable
    s=d.feed(packet(t=0,valid=False))[0];assert not s.usable and s.time_us is None
    with pytest.raises(ValueError): d._binary(bytes(64),0)

def test_epoch_detected_before_usable_and_late_packets_cannot_rollback():
    d=StreamDecoder(strict_quality=True)
    d.feed(packet(epoch=1))
    s=d.feed(health(epoch=2))[0];assert s.record['epoch_changed'] and s.time_us is None
    s=d.feed(packet(epoch=1,t=20000))[0];assert not s.usable and s.record['epoch_stale']
    s=d.feed(packet(epoch=2,t=30000))[0];assert s.usable and not s.record['epoch_changed']

def test_epoch_wrap_and_retired_boot():
    d=StreamDecoder(); d.feed(packet(epoch=0xffffffff))
    assert d.feed(packet(epoch=0,t=20000))[0].record['epoch_changed']
    d.feed(packet(epoch=1,session=43,t=30000))
    assert d.feed(packet(epoch=0,session=42,t=40000))[0].record['epoch_stale']

def test_legacy_compatibility_explicit_and_identical_quaternions_are_reads():
    p=bytearray(64);struct.pack_into('<BBBBIII',p,0,1,9,1,3,42,1,1)
    struct.pack_into('<Q4hBB2xI',p,16,10000,16384,0,0,0,255,2,0)
    struct.pack_into('<IIiIII',p,40,10000,10000,0,16,99,1000)
    assert StreamDecoder().feed(encode_frame(p,10300))[0].usable
    assert not StreamDecoder(strict_quality=True).feed(encode_frame(p,10300))[0].usable
    d=StreamDecoder(strict_quality=True);sync=SampleSynchronizer((1,))
    assert len(sync.add(d.feed(packet(t=10000))[0]))==1
    assert len(sync.add(d.feed(packet(t=20000))[0]))==1

def test_sync_quality_age_expires_and_epoch_clears_before_no_time():
    d=StreamDecoder();s=d.feed(packet(ages=(90,90)))[0]
    assert usable_at(s,20000) and not usable_at(s,21000)
    sync=SampleSynchronizer((1,));sync.add(s);generation=sync.generation
    sync.add(d.feed(health(2))[0]);assert sync.generation>generation and not sync.buffers[1]

def test_motion_epoch_stops_quantitative_but_keeps_raw(tmp_path):
    m=MotionSession();d=StreamDecoder(strict_quality=True)
    try:
        for wire in (packet(),health(2),packet(epoch=1,t=20000),packet(epoch=2,t=30000)):
            for s in d.feed(wire):m.record(s.record);m.ingest(s)
        assert m.epoch_broken and not m.running and len(m.raw)==4 and len(m.rows)==0
        assert not m.reader._stop_event.is_set()
        assert m.metadata()['sensor_identities']['1']==[42,2]
    finally:m.close()

def test_all_three_joint_engines_get_identical_valid_orientations():
    # Identity input and clock grid are unchanged; quality metadata gates, never modifies q.
    d=StreamDecoder(strict_quality=True);sync=SampleSynchronizer()
    frames=[]
    for t in range(10000,110000,10000):
        for node in range(1,5):frames+=sync.add(d.feed(packet(node,t))[0])
    assert len(frames)==10 and all(all(q==(1.,0.,0.,0.) for q in f.orientations.values()) for f in frames)

def test_strict_live_replay_including_health_cal_drop_and_epoch(tmp_path):
    from test_motion import ready_engine
    from imu_desktop.core.replay_motion import replay
    m=MotionSession();m.reader.strict_quality=True;m.engine=ready_engine();m.save_change({'synthetic':True})
    d=StreamDecoder(strict_quality=True)
    try:
        for i in range(30):
            for node in range(1,5):
                for s in d.feed(packet(node,10000+i*10000,cal=0xf3 if i==15 and node==2 else 255)):
                    m.record(s.record);m.ingest(s)
        for wire in (health(2),packet(epoch=1,t=400000),packet(epoch=2,t=410000)):
            for s in d.feed(wire):m.record(s.record);m.ingest(s)
        m.export(tmp_path/'live.csv');r=replay(tmp_path/'live.session.zip')
        try:
            assert list(m.rows)==list(r.rows) and len(m.rows)>0
            assert r.epoch_broken and r.metadata()['quantitative_stopped_reason']=='sensor_epoch_changed'
        finally:r.close()
    finally:m.close()

def test_bench_epoch_invalidates_zero_without_motor_commands():
    from imu_desktop.core.bench import BenchController
    b=BenchController(lambda _:None)
    try:
        sent=[];b.motor.send=sent.append;b.calibration={'example':True};b.recording=True
        b._epoch_reset()
        assert b.calibration is None and not b.recording and not sent and not b.raw.closed
    finally:b.close()

def test_profile_loaded_before_data_cannot_bind_to_another_epoch():
    from test_motion import ready_engine
    m=MotionSession();p=ready_engine().profile();p['sensor_identities']={'1':[42,1]}
    try:
        m.load_profile(p,save=False)
        s=StreamDecoder(strict_quality=True).feed(packet(epoch=2))[0]
        m.record(s.record);m.ingest(s)
        assert m.epoch_broken and len(m.rows)==0
    finally:m.close()

def test_hub_usb_health_cannot_create_sensor_epoch_or_measurement():
    p=bytearray(health()[10:74]);p[2]=0;p[3]=0;p[39]=3
    struct.pack_into('<6I',p,40,1,2,3,0,0,0)
    s=StreamDecoder(strict_quality=True).feed(encode_frame(p))[0]
    assert s.record['hub_health'] and 'sensor_epoch' not in s.record and not s.usable
    assert s.record['health_counters'][:3]==[1,2,3]

@pytest.mark.parametrize('field,value',[(0,2),(1,99),(2,0),(3,0x83),(33,4)])
def test_malformed_new_packet_rejected(field,value):
    p=bytearray(packet()[10:74]);p[field]=value
    assert not StreamDecoder(strict_quality=True).feed(encode_frame(p))


def test_hub_pages_retained_separately_without_changing_motion_or_rate():
    from imu_desktop.core.radio_bench import RadioBench
    a=MotionSession(); b=MotionSession(); d=StreamDecoder(strict_quality=True); radio=RadioBench()
    try:
        for i in range(30):
            for node in range(1,5):
                wire=packet(node,10000+i*10000)
                for sample in d.feed(wire):
                    a.ingest(sample); b.ingest(sample); radio.add(sample.record)
            for page in (2,3):
                p=bytearray(health()[10:74]);p[2]=0;p[3]=0;p[39]=page
                struct.pack_into('<6I',p,40,page,2,3,4,5,0)
                s=d.feed(encode_frame(p))[0]
                b.ingest(s);radio.add(s.record)
                assert not s.usable and 'sensor_epoch' not in s.record
        assert list(a.rows)==list(b.rows) and not b.epoch_broken
        snapshot=radio.snapshot()
        assert set(snapshot['hub_quality_pages'])=={'2','3'}
        assert snapshot['hub_quality_pages']['2']['health_counters'][0]==2
        assert all(n['packets']==30 and n['sequence_gaps']==0 for n in snapshot['nodes'])
        for field,value in ((2,5),(3,2),(39,1)):
            invalid=bytearray(p);invalid[field]=value
            assert not d.feed(encode_frame(invalid))
        assert not d.feed(encode_frame(p[:-1]))
    finally:a.close();b.close()
