"""Replay exported raw using exactly the live MotionSession pipeline; no COM."""
import argparse
import json
import zipfile
from pathlib import Path
from dataclasses import replace
from .motion import MotionSession
from .protocol import StreamDecoder, encode_frame


def replay(bundle, destination=None):
    session=MotionSession()
    try:
        with zipfile.ZipFile(bundle) as z:
            meta=json.loads(z.read('metadata.json'))
            session.configure(meta['model_id'],meta['side'])
            decoder=StreamDecoder(meta.get('protocol','binary'),meta.get('strict_quality',False))
            session.reader.strict_quality=meta.get('strict_quality',False)
            events=iter(meta.get('profiles',[])); event=next(events,None)
            windows=meta.get('capture_windows',[])
            window_index=0
            with z.open('raw.jsonl') as raw:
                for index,line in enumerate(raw):
                    while event is not None and event['after_record']<=index:
                        session.collecting=None
                        session.load_profile(event['profile'],save=False)
                        session.profiles.append(event)
                        event=next(events,None)
                    while window_index<len(windows) and index>=windows[window_index]['end_record']:
                        window_index+=1
                    suppressed=(window_index<len(windows) and windows[window_index]['start_record']<=index<windows[window_index]['end_record'])
                    session.collecting=('replay_capture',False) if suppressed else None
                    record=json.loads(line)
                    session.record(record)
                    if 'payload_hex' in record:
                        data=encode_frame(bytes.fromhex(record['payload_hex']),record.get('receiver_event_us_low32',0))
                    elif 'raw_line' in record:
                        data=(record['raw_line']+'\n').encode('ascii')
                    else:
                        raise ValueError('Raw thiếu payload hoặc raw_line; không tái tạo mẫu đo')
                    for sample in decoder.feed(data):
                        if sample.time_us is not None:
                            sample=replace(sample,time_us=sample.time_us-session.reader.latency_us.get(sample.node,0.))
                        session.ingest(sample)
            while event is not None:
                session.collecting=None
                session.load_profile(event['profile'],save=False)
                session.profiles.append(event); event=next(events,None)
        session.collecting=None
        session.capture_windows=windows
        if destination is not None:
            session.export(Path(destination))
        return session
    except Exception:
        session.close()
        raise


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('bundle'); parser.add_argument('destination')
    args=parser.parse_args()
    result=replay(args.bundle,args.destination)
    print(json.dumps(result.metadata(),ensure_ascii=False,indent=2))
    result.close()
