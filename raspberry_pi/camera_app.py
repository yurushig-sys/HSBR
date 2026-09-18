from flask import Flask, Response, render_template, jsonify
from hsbr.camera_manager import CameraManager
import atexit

app = Flask(__name__)

camera = CameraManager(
    device=0,
    width=640,
    height=480,
    stream_fps=10,
    jpeg_quality=75
)

camera.start()


def generate_mjpeg():
    last_frame_id = -1

    while True:
        jpeg, frame_id = camera.get_jpeg(last_frame_id)

        if jpeg is None:
            continue

        if frame_id == last_frame_id:
            continue

        last_frame_id = frame_id

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + jpeg
            + b"\r\n"
        )


@app.route("/")
def index():
    return render_template("camera.html")


@app.route("/camera")
def camera_page():
    return render_template("camera.html")


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/api/camera/status")
def camera_status():
    return jsonify(camera.get_status())


def cleanup():
    camera.stop()


atexit.register(cleanup)


if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5001,
        threaded=True,
        debug=False
    )
