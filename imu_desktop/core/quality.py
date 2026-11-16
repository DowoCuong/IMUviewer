"""Shared quality/identity contract; no fitting or angle correction."""
from .protocol import signed_delta
TTL_MS=100
POLICY=dict(calibration_hz=20,quality_ttl_ms=TTL_MS,recovery_good_observations=2,
            time_basis='hub_read_completion',thresholds_hardware_validated=False)

class EpochTracker:
    def __init__(self):
        self.current={}; self.retired={}
    def observe(self,sample):
        r=sample.record
        if 'sensor_epoch' not in r: return False,False
        node=sample.node; key=(sample.session,r['sensor_epoch'])
        previous=self.current.get(node)
        retired=self.retired.setdefault(node,[])
        if previous is not None and key[0]!=previous[0] and key[0] in retired: return False,True
        if previous is not None and key[0]==previous[0] and signed_delta(key[1],previous[1])<0: return False,True
        changed=previous is not None and key!=previous
        if changed and key[0]!=previous[0]:
            retired.append(previous[0]); del retired[:-8]
        self.current[node]=key
        return changed,False

def usable_at(sample,time_us):
    if not sample.usable: return False
    r=sample.record
    if r.get('type')!=11: return True
    age=max(r['cal_age_ms'],r['health_age_ms'])
    elapsed=max(0.,time_us-sample.time_us)/1000
    return age+elapsed<=TTL_MS
