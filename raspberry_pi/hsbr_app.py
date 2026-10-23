#!/usr/bin/env python3
# HSBR Web Remote
# Python 3.7 compatible
#
# Browser -> Raspberry Pi (Flask) -> USB Serial -> ESP32
#
# ESP32 commands used:
#   zj <speed> <steer>
#   zu1   Stand Up
#   zu0   Stand Down / disable balance control
#   zh    Halt movement flags
#
# Serial parser on ESP32 terminates commands with '\n'.

import atexit
import math
from datetime import datetime
import os
import threading
import time

#from flask import Flask, jsonify, render_template, request
from flask import Flask, Response, jsonify, render_template, request
from hsbr.camera_manager import CameraManager
from hsbr.field_hockey import FieldHockeyDetector
from hsbr.field_hockey_motion import FieldHockeyAutoController
import cv2
import serial
import subprocess

SERIAL_PORT = os.environ.get("HSBR_SERIAL", "/dev/ttyUSB0")
SERIAL_BAUD = int(os.environ.get("HSBR_BAUD", "115200"))

# Server-side safety limits.
# The browser UI uses smaller limits by default.
SERVER_MAX_SPEED = 50.0
SERVER_MAX_STEER = 30.0

WEB_HOST = "0.0.0.0"
#WEB_PORT = int(os.environ.get("HSBR_WEB_PORT", "8080"))
WEB_PORT = int(os.environ.get("HSBR_MAIN_WEB_PORT", "5002"))

# Logs are stored beside this program in raspberry_pi/logs/ by default.
APP_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.environ.get("HSBR_LOG_DIR", os.path.join(APP_DIR, "logs"))


def clamp(value, low, high):
    return max(low, min(high, value))


class ESP32Link(object):
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None
        self.lock = threading.Lock()
        self.running = True
        self.last_rx = ""
        self.last_error = ""
        self.last_tx = ""
        self.telemetry_lock = threading.Lock()
        self.telemetry = {
            "enable_control": None,
            "move_flag": None,
            "rotate_flag": None,
            "continuous_flag": None,
            "moving_flag": None,
            "param_index": None,
            "battery_voltage": None,
            "status_at": 0.0,
            "data_at": 0.0,
            "generation": 0,
            "od_at": 0.0,
            "radius_cap_at": 0.0
        }

        # Serial log state. Only this process owns /dev/ttyUSB0;
        # miniterm/other serial monitors must remain stopped.
        self.log_lock = threading.Lock()
        self.log_fp = None
        self.log_path = ""
        self.log_started_monotonic = 0.0

        # Buffer bytes until a complete newline-terminated ESP32 line arrives.
        # This prevents timeout-fragmented reads from becoming broken log lines.
        self.rx_buffer = bytearray()

        self._command_id = 0
        self._command_results = {}
        self._last_device_ms = None
        self.reader_thread = threading.Thread(target=self._reader_loop, name="esp32-rx")
        self.reader_thread.daemon = True
        self.reader_thread.start()

    def _close_unlocked(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        with self.telemetry_lock:
            self.telemetry['od_at']=0.0

    def _ensure_open_unlocked(self):
        if self.ser is not None and self.ser.is_open:
            return

        self._close_unlocked()
        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=0.01,
            write_timeout=0.05,
            exclusive=True
        )
        with self.telemetry_lock:
            generation=self.telemetry.get("generation",0)+1
            self.telemetry.update(generation=generation, od_at=0.0, radius_cap_at=0.0,
                                  status_at=0.0, data_at=0.0, enable_control=None,
                                  battery_voltage=None)
            self._last_device_ms=None
            self._command_results.clear()
        # Keep pySerial's default DTR/RTS settings, as in the successful miniterm test.
        # Do not reset ESP32 input/output buffers here.
        self.last_error = ""

    def _log_record(self, direction, text):
        with self.log_lock:
            if self.log_fp is None:
                return

            elapsed = time.monotonic() - self.log_started_monotonic
            wall = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self.log_fp.write("{:.3f}\t{}\t{}\t{}\n".format(
                elapsed, wall, direction, text
            ))
            # Flush every line so a sudden stop still leaves a useful file.
            self.log_fp.flush()

    def start_log(self):
        with self.log_lock:
            if self.log_fp is not None:
                return self.log_path

            if not os.path.isdir(LOG_DIR):
                os.makedirs(LOG_DIR)

            filename = "hsbr_log_{}.txt".format(
                datetime.now().strftime("%Y%m%d_%H%M%S")
            )
            self.log_path = os.path.join(LOG_DIR, filename)
            self.log_fp = open(self.log_path, "w", buffering=1)
            self.log_started_monotonic = time.monotonic()
            self.log_fp.write("# HSBR Web Remote serial log\n")
            self.log_fp.write("# started={}\n".format(datetime.now().isoformat()))
            self.log_fp.write("# serial={} baud={}\n".format(self.port, self.baud))
            self.log_fp.write("# columns: elapsed_s  wall_time  dir  data\n")
            self.log_fp.flush()
            return self.log_path

    def stop_log(self):
        with self.log_lock:
            path = self.log_path
            if self.log_fp is not None:
                try:
                    elapsed = time.monotonic() - self.log_started_monotonic
                    wall = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    self.log_fp.write("{:.3f}\t{}\tEVENT\tLOG STOP\n".format(
                        elapsed, wall
                    ))
                    self.log_fp.flush()
                    self.log_fp.close()
                except Exception:
                    pass
                self.log_fp = None
            return path

    def is_logging(self):
        with self.log_lock:
            return self.log_fp is not None

    def send(self, command):
        # ESP32 parseSerial() requires '\n'.
        line = command.rstrip("\r\n") + "\n"
        data = line.encode("ascii")

        with self.lock:
            try:
                self._ensure_open_unlocked()
                self.ser.write(data)
                self.ser.flush()
                self.last_tx = command
            except Exception as exc:
                self.last_error = str(exc)
                self._close_unlocked()
                raise

        # Record exactly which command caused the following motion response.
        self._log_record("TX", command)

    def send_many(self, commands):
        for command in commands:
            self.send(command)

    def _handle_rx_line(self, raw_line):
        # Strip CR left by CRLF, while preserving the rest of the ESP32 line.
        if raw_line.endswith(b"\r"):
            raw_line = raw_line[:-1]

        text = raw_line.decode("utf-8", errors="replace")
        self.last_rx = text
        self._log_record("RX", text)
        if self._handle_remote_line(text):
            return
        fields = text.split()
        try:
            if len(fields) >= 7 and fields[0] == "Status":
                with self.telemetry_lock:
                    self.telemetry.update({
                        "enable_control": int(float(fields[1])),
                        "move_flag": int(float(fields[2])),
                        "rotate_flag": int(float(fields[3])),
                        "continuous_flag": int(float(fields[4])),
                        "moving_flag": int(float(fields[5])),
                        "param_index": int(float(fields[6])),
                        "status_at": time.monotonic()
                    })
            elif len(fields) >= 8 and fields[0] == "Data":
                with self.telemetry_lock:
                    self.telemetry.update({
                        "battery_voltage": float(fields[7]),
                        "data_at": time.monotonic()
                    })
        except (TypeError, ValueError):
            # Keep the serial reader alive if a diagnostic line is malformed.
            pass

    def next_command_id(self):
        with self.telemetry_lock:
            self._command_id += 1
            self._command_results.pop(0,None)
            return self._command_id

    def get_command_result(self, ident):
        with self.telemetry_lock:
            return self._command_results.pop(ident,None) or self._command_results.pop(0,None)

    def _handle_remote_line(self, text):
        f=text.split(',')
        now=time.monotonic()
        try:
            if len(f)==2 and f[0]=='CAP' and f[1]=='FH_RADIUS_V1':
                with self.telemetry_lock:
                    self.telemetry['radius_cap_at']=now
                return True
            if len(f)==7 and f[0]=='OD':
                ms=int(f[1]); d,v,angle=map(float,f[2:5])
                upright,active=map(int,f[5:7])
                if not all(math.isfinite(x) for x in (d,v,angle)):
                    return True
                with self.telemetry_lock:
                    if self._last_device_ms is not None:
                        delta=(ms-self._last_device_ms)&0xffffffff
                        if delta==0:
                            return True
                        if delta>10000:
                            self.telemetry['generation']+=1
                            self.telemetry['radius_cap_at']=0.
                    self._last_device_ms=ms
                    self.telemetry.update(device_ms=ms,distance_mm=d,velocity_mm_s=v,
                                          angle_deg=angle,enable_control=upright,
                                          remote_active=active,od_at=now)
                return True
            if len(f)==5 and f[0]=='ACK':
                ident=int(f[1]); ms=int(f[2]); d,angle=map(float,f[3:5])
                if not all(math.isfinite(x) for x in (d,angle)):
                    return True
                with self.telemetry_lock:
                    self._command_results[ident]=dict(device_ms=ms,distance_mm=d,
                                                       angle_deg=angle,ack_at=now)
                    if len(self._command_results)>100:
                        self._command_results.pop(next(iter(self._command_results)))
                return True
            if len(f)>=3 and f[0]=='ERR':
                with self.telemetry_lock:
                    self._command_results[int(f[1])]=dict(error=','.join(f[2:]))
                return True
        except (ValueError,TypeError):
            return True
        return False

    def get_telemetry(self):
        with self.telemetry_lock:
            return dict(self.telemetry)

    def _reader_loop(self):
        while self.running:
            try:
                with self.lock:
                    self._ensure_open_unlocked()
                    ser = self.ser

                # Read whatever is available, but keep partial data in rx_buffer
                # until ESP32 sends a newline.
                raw = ser.read(ser.in_waiting or 1)
                if raw:
                    self.rx_buffer.extend(raw)

                    while True:
                        newline = self.rx_buffer.find(b"\n")
                        if newline < 0:
                            break

                        raw_line = bytes(self.rx_buffer[:newline])
                        del self.rx_buffer[:newline + 1]
                        self._handle_rx_line(raw_line)

            except Exception as exc:
                self.last_error = str(exc)
                self._log_record("ERROR", self.last_error)
                with self.lock:
                    self._close_unlocked()
                self.rx_buffer = bytearray()
                time.sleep(0.5)

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def shutdown(self):
        self.running = False
        try:
            # Best effort: zero the remote command before shutting down.
            self.send_many(["zh", "zV 0"])
        except Exception:
            pass

        self.stop_log()

        with self.lock:
            self._close_unlocked()


#app = Flask(__name__)
#link = ESP32Link(SERIAL_PORT, SERIAL_BAUD)
app = Flask(__name__)

camera = CameraManager(
    device=0,
    width=640,
    height=480,
    stream_fps=10,
    jpeg_quality=75
)
camera.start()

field_hockey = FieldHockeyDetector()

link = ESP32Link(SERIAL_PORT, SERIAL_BAUD)
field_hockey_auto = FieldHockeyAutoController(link, field_hockey)

current_speed = 0.0
current_steer = 0.0
state_lock = threading.Lock()


@app.before_request
def protect_automatic_motion():
    if request.path=="/api/shutdown" and request.method=="POST":
        data=request.get_json(silent=True) or {}
        if data.get("confirm")=="shutdown":
            field_hockey_auto.stop("シャットダウン要求")
    if not field_hockey_auto.get_status()["running"]:
        return None
    if request.path=="/api/action" and request.method=="POST":
        data=request.get_json(silent=True) or {}
        if data.get("action") in ("stop","stand_down"):
            field_hockey_auto.stop()
            if data.get("action")=="stand_down":
                with field_hockey_auto.command_lock:
                    link.send_many(["zh","zV 0","zu0"])
            return jsonify(ok=True,action=data["action"])
    if request.path in ("/api/drive","/api/action","/api/log",
                         "/api/apps/field-hockey/command") and request.method=="POST":
        return jsonify(ok=False,error="自動動作中です。停止してから操作してください"),409
    return None


@app.route("/")
def index():
    return render_template("hsbr.html") #("remote.html")

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


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


def generate_field_hockey_mjpeg():
    """Detection-only stream; no motion commands are sent here."""
    while True:
        frame = camera.get_frame()
        if frame is None:
            time.sleep(0.05)
            continue

        annotated = field_hockey.process(frame)
        ok, encoded = cv2.imencode(
            ".jpg",
            annotated,
            [int(cv2.IMWRITE_JPEG_QUALITY), 75]
        )
        if ok:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + encoded.tobytes()
                + b"\r\n"
            )

        # Keep the initial detector load modest on Raspberry Pi 4.
        time.sleep(0.1)


@app.route("/field_hockey_feed")
def field_hockey_feed():
    return Response(
        generate_field_hockey_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/api/apps/field-hockey/status")
def api_field_hockey_status():
    status = field_hockey.get_status()
    status["automation"] = field_hockey_auto.get_status()
    if status["automation"]["running"] and status["automation"].get("frozen_route"):
        status["route"]=status["automation"]["frozen_route"]
    return jsonify(status)


@app.route("/api/apps/field-hockey/auto", methods=["POST"])
def api_field_hockey_auto():
    data = request.get_json(silent=True) or {}
    action = data.get("action", "")
    if action == "start":
        ok, message = field_hockey_auto.start(
            hit_enabled=data.get("hit_enabled") is True,
            stroke_seconds=data.get("stroke_seconds"))
        return jsonify(ok=ok, message=message), (200 if ok else 409)
    if action == "stop":
        field_hockey_auto.stop()
        return jsonify(ok=True, message="停止しました")
    return jsonify(ok=False, error="unknown action"), 400


@app.route("/api/apps/field-hockey/command", methods=["POST"])
def api_field_hockey_command():
    data = request.get_json(silent=True) or {}
    command = str(data.get("command", "")).strip()

    if not command:
        return jsonify(ok=False, error="command is empty"), 400
    if len(command) > 120 or "\n" in command or "\r" in command:
        return jsonify(ok=False, error="invalid command length/line break"), 400

    parts = command.split()
    token = parts[0]
    if (len(token) < 2 or len(token) > 3 or token[0] != "z" or
            not token[1:].isalnum()):
        return jsonify(ok=False, error="only HSBR z commands are allowed"), 400

    allowed_parameter_chars = set("0123456789+-. ")
    parameter_text = command[len(token):]
    if any(ch not in allowed_parameter_chars for ch in parameter_text):
        return jsonify(ok=False, error="parameters must be numeric"), 400

    automation = field_hockey_auto.get_status()
    if automation.get("running"):
        return jsonify(
            ok=False,
            error="stop automatic operation before manual command"
        ), 409

    try:
        link.send(command)
        time.sleep(0.15)
    except Exception as exc:
        return jsonify(ok=False, error=str(exc)), 503

    return jsonify(
        ok=True,
        command=command,
        last_rx=link.last_rx
    )


@app.route("/camera")
def camera_page():
    return render_template("camera.html")


@app.route("/api/camera/status")
def api_camera_status():
    return jsonify(camera.get_status())


@app.route("/api/drive", methods=["POST"])
def api_drive():
    global current_speed, current_steer

    data = request.get_json(silent=True) or {}

    try:
        speed = float(data.get("speed", 0.0))
        steer = float(data.get("steer", 0.0))
    except (TypeError, ValueError):
        return jsonify(ok=False, error="speed/steer must be numbers"), 400

    speed = clamp(speed, -SERVER_MAX_SPEED, SERVER_MAX_SPEED)
    steer = clamp(steer, -SERVER_MAX_STEER, SERVER_MAX_STEER)

    try:
        link.send("zj {:.1f} {:.1f}".format(speed, steer))
    except Exception as exc:
        return jsonify(ok=False, error=str(exc)), 503

    with state_lock:
        current_speed = speed
        current_steer = steer

    return jsonify(ok=True, speed=speed, steer=steer)


@app.route("/api/action", methods=["POST"])
def api_action():
    global current_speed, current_steer

    data = request.get_json(silent=True) or {}
    action = data.get("action", "")

    try:
        if action == "stand_up":
            # Make sure drive demand is zero before stand-up sequence.
            link.send_many([
                "zj 0 0",
                "zu1"
            ])

        elif action == "stand_down":
            # Zero remote demand first, then disable balance control.
            link.send_many([
                "zj 0 0",
                "zu0"
            ])

        elif action == "stop":
            # zj clears joystick demand.
            # zh also clears the other movement/turn flags.
            link.send_many([
                "zj 0 0",
                "zh"
            ])

        else:
            return jsonify(ok=False, error="unknown action"), 400

    except Exception as exc:
        return jsonify(ok=False, error=str(exc)), 503

    with state_lock:
        current_speed = 0.0
        current_steer = 0.0

    return jsonify(ok=True, action=action)


@app.route("/api/log", methods=["POST"])
def api_log():
    data = request.get_json(silent=True) or {}
    action = data.get("action", "")

    try:
        if action == "start":
            path = link.start_log()
            link.send("ze1")
            return jsonify(ok=True, logging=True, log_path=path)

        if action == "stop":
            link.send("ze0")
            path = link.stop_log()
            return jsonify(ok=True, logging=False, log_path=path)

        return jsonify(ok=False, error="unknown log action"), 400

    except Exception as exc:
        return jsonify(ok=False, error=str(exc)), 500

@app.route("/api/shutdown", methods=["POST"])
def api_shutdown():
    data = request.get_json(silent=True) or {}

    # Browser側の確認だけに頼らず、サーバ側でも確認する
    if data.get("confirm") != "shutdown":
        return jsonify(ok=False, error="confirmation required"), 400

    def do_shutdown():
        # HTTP応答をブラウザへ返す時間を確保
        time.sleep(1.0)

        # ESP32を安全側へ
        try:
            link.send_many([
                "zh",
                "zV 0",
                "zu0"
            ])
        except Exception:
            pass

        # ログが動いていれば閉じる
        try:
            link.stop_log()
        except Exception:
            pass

        time.sleep(1.0)

        # Raspberry Pi shutdown
        subprocess.Popen([
            "sudo", "-n",
            "/sbin/shutdown", "-h", "now"
        ])

    t = threading.Thread(target=do_shutdown, name="pi-shutdown")
    t.daemon = True
    t.start()

    return jsonify(ok=True)

@app.route("/api/status")
def api_status():
    with state_lock:
        speed = current_speed
        steer = current_steer

    return jsonify(
        ok=True,
        serial_open=link.is_open(),
        serial_port=SERIAL_PORT,
        speed=speed,
        steer=steer,
        last_tx=link.last_tx,
        last_rx=link.last_rx,
        last_error=link.last_error,
        logging=link.is_logging(),
        log_path=link.log_path
    )


@atexit.register
def cleanup():
    field_hockey_auto.stop("アプリ終了")
    link.shutdown()
    camera.stop()

if __name__ == "__main__":
    print("HSBR Main App") #("HSBR Web Remote")
    print("Serial: {} @ {}".format(SERIAL_PORT, SERIAL_BAUD))
    print("Web:    http://0.0.0.0:{}/".format(WEB_PORT))
    print("Camera: http://0.0.0.0:{}/camera".format(WEB_PORT))
    print("Logs:   {}".format(LOG_DIR))
    print("Stop any miniterm/serial monitor before starting this program.")

    # Important: no Flask reloader.
    # The reloader would start this process twice and contend for /dev/ttyUSB0.
    app.run(
        host=WEB_HOST,
        port=WEB_PORT,
        debug=False,
        threaded=True,
        use_reloader=False
    )
