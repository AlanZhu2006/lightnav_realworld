"""Bounded live-camera display worker, independent of model ingress.

Only the newest pending frame is retained. Original source header stamps are used
for freshness; waiting or encoding never refreshes a source timestamp.
"""
from collections import deque
import threading
import time


class LatestFramePreview:
    def __init__(self,publish,*,hz=15.,max_age_s=.3):
        if hz<=0 or max_age_s<=0:
            raise ValueError("rate and source age must be positive")
        self.publish,self.period,self.max_age_s=publish,1./hz,max_age_s
        self.condition=threading.Condition()
        self.pending=None
        self.stopped=False
        self.last_source_ns=0
        self.counts=dict(published=0,replaced=0,source_age_dropped=0,errors=0)
        self.times=deque(maxlen=150)
        self.last_publish_source_ns=None
        self.last_encode_ms=None
        self.worker=threading.Thread(target=self.run,name="imagegoal-live-rgb-preview",daemon=True)
        self.worker.start()

    def submit(self,frame,source_ns):
        with self.condition:
            if self.stopped or source_ns<=self.last_source_ns:
                return
            age=(time.time_ns()-source_ns)/1e9
            if age<-.05 or age>self.max_age_s:
                self.counts["source_age_dropped"]+=1
                return
            self.last_source_ns=source_ns
            if self.pending is not None:
                self.counts['replaced']+=1
            self.pending=(frame,source_ns)
            self.condition.notify()

    def run(self):
        next_at=0.
        while True:
            with self.condition:
                if self.stopped:
                    return
                if self.pending is None:
                    self.condition.wait()
                    continue
                delay=next_at-time.monotonic()
                if delay>0:
                    self.condition.wait(delay)
                    continue
                frame,source_ns=self.pending
                self.pending=None
                next_at=time.monotonic()+self.period
            age=(time.time_ns()-source_ns)/1e9
            if age<-.05 or age>self.max_age_s:
                with self.condition:
                    self.counts['source_age_dropped']+=1
                continue
            started=time.monotonic()
            try:
                published=self.publish(frame)
                with self.condition:
                    if published is False:
                        self.counts['source_age_dropped']+=1
                    else:
                        self.counts['published']+=1
                        self.times.append(time.monotonic())
                        self.last_publish_source_ns=source_ns
                        self.last_encode_ms=(time.monotonic()-started)*1000
            except Exception:
                with self.condition:
                    self.counts['errors']+=1

    def snapshot(self):
        with self.condition:
            recent=[t for t in self.times if time.monotonic()-t<=10.]
            return {**self.counts,'fps':(len(recent)-1)/(recent[-1]-recent[0]) if len(recent)>1 else 0.,
                    'source_timestamp_ns':self.last_publish_source_ns,
                    'source_age_s':(time.time_ns()-self.last_publish_source_ns)/1e9 if self.last_publish_source_ns else None,
                    'encode_ms':self.last_encode_ms,'max_pending_frames':1}

    def close(self):
        with self.condition:
            self.stopped=True
            self.pending=None
            self.condition.notify_all()
        self.worker.join(timeout=2)


def decode_compressed_rgb(data):
    """Decode a bounded camera JPEG for the display worker, never in a callback."""
    import io
    from PIL import Image
    if not data or len(data)>4*1024*1024:
        raise ValueError("compressed preview must be a JPEG at most 4 MiB")
    with Image.open(io.BytesIO(bytes(data))) as source:
        if source.format!="JPEG" or source.width*source.height>4096*2160:
            raise ValueError("unsupported compressed camera preview")
        return source.convert("RGB")
