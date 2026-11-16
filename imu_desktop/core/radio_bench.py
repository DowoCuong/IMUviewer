"""Bounded per-node statistics from the hub stream, separate from angle scoring.

Rates use hub RX-END time, never USB delivery spacing. Gaps are missing
submission identities between endpoints, not independently proven RF loss.
"""
import time
from .protocol import signed_delta


class RadioBench:
    def __init__(self):
        self.nodes = {}
        self.decoder = {}
        self.health_nodes={}
        self.hub_pages={}

    def add(self, record):
        kind, node = record.get('type'), record.get('node')
        if kind==12:
            if record.get('epoch_stale'): return
            if node==0:
                if self.hub_pages and next(iter(self.hub_pages.values())).get('session')!=record.get('session'):
                    self.hub_pages.clear()
                self.hub_pages[str(record['health_page'])]=dict(record)
            self.health_nodes[node]=dict(record)
            return
        if kind not in (1, 2, 9, 10, 11) or node not in (1, 2, 3, 4):
            return
        now = record.get('host_monotonic_s', time.monotonic())
        s = self.nodes.setdefault(node, dict(packets=0, test_packets=0, timed_packets=0,
            sequence_gaps=0, duplicates=0, out_of_order=0, epochs=0, intervals=0,
            elapsed_us=0, min_interval_us=None, max_interval_us=None,
            max_tx_start_lateness_us=None, min_rx_end_offset_us=None,
            max_rx_end_offset_us=None, rx_end_outside_slot=0))
        s['packets'] += 1
        s['test_packets'] += kind == 10
        s['timed_packets'] += kind in (9,11)
        s['last_host_s'] = now
        if kind in (1, 2, 9,11):
            s['calibration_raw'] = record.get('calibration')
            s['sensor_state'] = record.get('state')
        else:
            s['calibration_raw'] = s['sensor_state'] = None
        # A new hub or node session starts a new interval; never bridge reboots.
        identity = (record['session'], record.get('hub_session'))
        seq, event = record['tx_seq'], record['receiver_event_us_low32']
        if s.get('identity') != identity:
            s.update(identity=identity, last_seq=seq, last_event=event)
            s['epochs'] += 1
        else:
            delta = signed_delta(seq, s['last_seq'])
            if delta == 0:
                s['duplicates'] += 1
                return
            if delta < 0:
                s['out_of_order'] += 1
                return
            s['sequence_gaps'] += delta-1
            dt = signed_delta(event, s['last_event'])
            if dt > 0:
                s['intervals'] += 1
                s['elapsed_us'] += dt
                for key, fn in (('min_interval_us', min), ('max_interval_us', max)):
                    s[key] = dt if s[key] is None else fn(s[key], dt)
            s.update(last_seq=seq, last_event=event)
        if kind == 10:
            late = signed_delta(record['tx_start_local_us'], record['scheduled_local_us'])
            end_offset = signed_delta(event, record['scheduled_hub_us'])
            s['max_tx_start_lateness_us'] = late if s['max_tx_start_lateness_us'] is None else max(s['max_tx_start_lateness_us'], late)
            for key, fn in (('min_rx_end_offset_us', min), ('max_rx_end_offset_us', max)):
                s[key] = end_offset if s[key] is None else fn(s[key], end_offset)
            s['rx_end_outside_slot'] += not 0 <= end_offset < 1000

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        nodes = []
        for node in range(1, 5):
            s = self.nodes.get(node, {})
            row = {k: v for k, v in s.items() if k not in ('identity', 'last_seq', 'last_event', 'last_host_s')}
            health=self.health_nodes.get(node,{})
            age=now-health.get('host_monotonic_s',now)
            row.update(quality=health if 0<=age<2 else {},health_receiving=bool(health) and 0<=age<2)
            row.update(node=node, receiving=bool(s) and 0 <= now-s.get('last_host_s', now) < .5,
                       received_hz=s['intervals']*1e6/s['elapsed_us'] if s.get('elapsed_us') else None)
            nodes.append(row)
        return dict(scope='since_connection', source='hub_USB_RX_END', nodes=nodes,hub_quality=self.health_nodes.get(0,{}),hub_quality_pages=dict(self.hub_pages),
                    decoder=dict(self.decoder), expected_packets_per_second_per_node=100,
                    note='Gaps include RF/USB/queue/write losses; skips and endpoint losses are not counted. '
                         'Synthetic TDMA timing covers received packets only, not BNO055 I2C or CPU load. '
                         'RX-END minus scheduled slot is not one-way radio latency.')
