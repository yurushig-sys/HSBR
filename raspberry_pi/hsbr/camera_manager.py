import cv2
import threading
import time


class CameraManager:
    def __init__(
        self,
        device=0,
        width=640,
        height=480,
        stream_fps=10,
        jpeg_quality=75
    ):
        self.device = device
        self.width = width
        self.height = height
        self.stream_fps = stream_fps
        self.jpeg_quality = jpeg_quality

        self.cap = None
        self.running = False
        self.thread = None

        self.condition = threading.Condition()

        self.latest_frame = None
        self.latest_jpeg = None
        self.frame_id = 0
        self.actual_fps = 0.0

    def start(self):
        if self.running:
            return

        self.cap = cv2.VideoCapture(self.device)

        if not self.cap.isOpened():
            raise RuntimeError(
                "Camera /dev/video{} could not be opened".format(
                    self.device
                )
            )

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        self.running = True

        self.thread = threading.Thread(
            target=self._capture_loop,
            daemon=True
        )
        self.thread.start()

    def _capture_loop(self):
        jpeg_interval = 1.0 / self.stream_fps
        last_jpeg_time = 0.0

        fps_start = time.time()
        fps_count = 0

        while self.running:
            ret, frame = self.cap.read()

            if not ret:
                time.sleep(0.05)
                continue

            with self.condition:
                self.latest_frame = frame

            now = time.time()

            # Web配信用JPEGは指定fpsだけ生成
            if now - last_jpeg_time >= jpeg_interval:
                ok, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [
                        int(cv2.IMWRITE_JPEG_QUALITY),
                        self.jpeg_quality
                    ]
                )

                if ok:
                    with self.condition:
                        self.latest_jpeg = encoded.tobytes()
                        self.frame_id += 1
                        self.condition.notify_all()

                    fps_count += 1
                    last_jpeg_time = now

            elapsed = now - fps_start

            if elapsed >= 2.0:
                self.actual_fps = fps_count / elapsed
                fps_count = 0
                fps_start = now

    def get_frame(self):
        with self.condition:
            if self.latest_frame is None:
                return None

            return self.latest_frame.copy()

    def get_jpeg(self, last_frame_id=-1, timeout=2.0):
        with self.condition:
            self.condition.wait_for(
                lambda: (
                    self.frame_id != last_frame_id
                    or not self.running
                ),
                timeout=timeout
            )

            return self.latest_jpeg, self.frame_id

    def get_status(self):
        return {
            "running": self.running,
            "width": self.width,
            "height": self.height,
            "stream_fps": self.stream_fps,
            "actual_fps": round(self.actual_fps, 1),
            "frame_id": self.frame_id
        }

    def stop(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=2.0)

        if self.cap is not None:
            self.cap.release()

        self.cap = None
