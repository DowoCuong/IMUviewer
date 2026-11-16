import json
from pathlib import Path
import struct
import tempfile
import unittest
import zipfile

from imu_desktop.core.alignment import SampleSynchronizer
from imu_desktop.core.bench import BenchController
from imu_desktop.core.protocol import StreamDecoder, encode_frame
from imu_desktop.core.radio_bench import RadioBench


def packet(node=3, seq=0, t=17000, session=1, hub_session=77):
    p=bytearray(64)
    struct.pack_into('<BBBBIII',p,0,1,10,node,2,session,seq,0)
    struct.pack_into('<Q4I',p,16,t,t,t,t+20,t-7000)
    struct.pack_into('<IIiIII',p,40,t-7000,t-7000,0,10,hub_session,0x54534554)
    return p


def measurement(node, seq, t):
    p=bytearray(64)
    struct.pack_into('<BBBBIII',p,0,1,9,node,3,123,seq,seq)
    struct.pack_into('<Q4hBB2xI',p,16,t,16384,0,0,0,255,2,1000)
    struct.pack_into('<IIiIII',p,40,t,t,0,10,77,1000)
    return p


class RadioBenchTests(unittest.TestCase):
    def test_test_packet_fragmentation_never_produces_quaternion(self):
        decoder=StreamDecoder(); result=[]
        for byte in encode_frame(packet(),17400): result.extend(decoder.feed(bytes([byte])))
        self.assertEqual(len(result),1)
        sample=result[0]
        self.assertIsNone(sample.quaternion); self.assertIsNone(sample.time_us)
        self.assertFalse(sample.usable); self.assertTrue(sample.record['synthetic'])
        self.assertEqual(sample.record['tx_start_local_us'],17020)
        self.assertFalse(any(decoder.counts.values()))

    def test_malformed_synthetic_packets_fail_closed(self):
        for offset,fmt,value in ((2,'B',1),(3,'B',3),(12,'I',1),(48,'i',201),(52,'I',51),(60,'I',0)):
            p=packet(); struct.pack_into('<'+fmt,p,offset,value)
            decoder=StreamDecoder()
            self.assertEqual(decoder.feed(encode_frame(p)),[])
            self.assertEqual(decoder.counts['invalid_records'],1)

    def test_mixed_four_node_stream_leaves_two_imu_frames_unchanged(self):
        plain=SampleSynchronizer((1,2)); mixed=SampleSynchronizer((1,2))
        wrong_pair=SampleSynchronizer((3,4)); expected=[]; actual=[]
        decoder=StreamDecoder()
        for seq in range(10):
            t=100000+seq*10000
            for node in (1,2,3,4):
                p=measurement(node,seq,t) if node<3 else packet(node,seq,t)
                sample=decoder.feed(encode_frame(p,t+400))[0]
                actual.extend(mixed.add(sample))
                if node<3: expected.extend(plain.add(sample))
                else: self.assertEqual(wrong_pair.add(sample),[])
        self.assertEqual(actual,expected); self.assertGreater(len(actual),5)
        self.assertTrue(all(set(f.orientations)=={1,2} for f in actual))

    def test_gaps_wrap_duplicates_and_reboots_use_hub_time_not_usb_time(self):
        bench=RadioBench()
        def add(seq,event,session=1):
            bench.add(dict(type=9,node=1,session=session,hub_session=77,tx_seq=seq,
                           receiver_event_us_low32=event&0xffffffff,host_monotonic_s=5.0))
        add(0xffffffff,0xfffff000); add(0,0xfffff000+10000)
        add(2,0xfffff000+30000); add(2,0xfffff000+30000); add(1,0xfffff000+20000)
        add(0,100,session=2)
        s=bench.snapshot(now=5.1)['nodes'][0]
        self.assertEqual(s['sequence_gaps'],1); self.assertEqual(s['epochs'],2)
        self.assertEqual(s['duplicates'],1); self.assertEqual(s['out_of_order'],1)
        self.assertAlmostEqual(s['received_hz'],2e6/30000)
        self.assertTrue(s['receiving'])
        self.assertFalse(bench.snapshot(now=6)['nodes'][0]['receiving'])

    def test_test_tdma_diagnostics_and_rate(self):
        bench=RadioBench(); decoder=StreamDecoder()
        for i in range(4):
            t=17000+i*10000
            row=decoder.feed(encode_frame(packet(seq=i,t=t),t+400))[0].record
            bench.add(dict(row,host_monotonic_s=5.0))
        s=bench.snapshot(now=5)['nodes'][2]
        self.assertEqual(s['received_hz'],100); self.assertEqual(s['test_packets'],4)
        self.assertEqual(s['max_tx_start_lateness_us'],20)
        self.assertEqual(s['max_rx_end_offset_us'],400); self.assertEqual(s['rx_end_outside_slot'],0)

    def test_controller_export_keeps_transport_separate_from_rmse(self):
        updates=[]; bench=BenchController(updates.append)
        try:
            record=StreamDecoder().feed(encode_frame(packet(),17400))[0].record
            bench._log('imu',record)
            self.assertEqual(bench.n,0); self.assertEqual(bench.rows,[])
            bench._radio_health({'decoder':{'crc_errors':2}})
            self.assertEqual(updates[-1]['kind'],'radio')
            bench.rows=[dict(imu_deg=0,encoder=0),dict(imu_deg=0,encoder=0)]
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/'bench.csv'; bench.export(path)
                with zipfile.ZipFile(path.with_suffix('.session.zip')) as z:
                    radio=json.loads(z.read('radio_bench.json'))
                    self.assertEqual(radio['nodes'][2]['test_packets'],1)
                    self.assertEqual(radio['decoder']['crc_errors'],2)
                    self.assertIn(b'"synthetic": true',z.read('raw.jsonl'))
                    self.assertEqual(json.loads(z.read('metadata.json'))['metrics']['rmse_deg'],0)
        finally: bench.close()


if __name__=='__main__': unittest.main()
