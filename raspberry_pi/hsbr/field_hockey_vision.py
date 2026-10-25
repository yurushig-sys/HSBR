"""One background detector/encoder shared by autonomy and all browser viewers."""
import threading
import time


class FieldHockeyVisionWorker(object):
    def __init__(self,camera,detector,encode,period=.1,clock=time.monotonic):
        self.camera,self.detector,self.encode=camera,detector,encode
        self.period,self.clock=period,clock
        self.condition=threading.Condition()
        self.stop_event=threading.Event()
        self.frame_id=0
        self.camera_frame_id=-1
        self.jpeg=None
        self.last_error=''
        self.thread=None

    def start(self):
        self.thread=threading.Thread(target=self._loop,name='field-hockey-vision')
        self.thread.daemon=True
        self.thread.start()

    def step(self):
        # Reuse the camera manager's existing public frame-id API so an old
        # cached frame is not repeatedly labelled as a new observation.
        jpeg,ident=self.camera.get_jpeg(self.camera_frame_id)
        if jpeg is None or ident==self.camera_frame_id:
            return False
        observed=self.clock()
        frame=self.camera.get_frame()
        if frame is None:
            return False
        annotated=self.detector.process(frame,observed_at=observed)
        self.camera_frame_id=ident
        encoded=self.encode(annotated)
        with self.condition:
            self.jpeg=encoded
            self.frame_id+=1
            self.last_error=''
            self.condition.notify_all()
        return True

    def _loop(self):
        while not self.stop_event.is_set():
            began=self.clock()
            try:
                self.step()
            except Exception as exc:
                self.last_error=str(exc)
            self.stop_event.wait(max(0.,self.period-(self.clock()-began)))

    def get_jpeg(self,previous):
        with self.condition:
            if (self.jpeg is None or previous==self.frame_id) and not self.stop_event.is_set():
                self.condition.wait(timeout=.25)
            return self.jpeg,self.frame_id

    def stop(self):
        self.stop_event.set()
        with self.condition:self.condition.notify_all()
        if self.thread:self.thread.join(timeout=.3)
