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
import os
import threading
import time

from flask import Flask, jsonify, render_template, request
import serial


SERIAL_PORT = os.environ.get("HSBR_SERIAL", "/dev/ttyUSB0")
SERIAL_BAUD = int(os.environ.get("HSBR_BAUD", "115200"))

# Server-side safety limits.
# The browser UI uses smaller limits by default.
SERVER_MAX_SPEED = 30.0
SERVER_MAX_STEER = 30.0

WEB_HOST = "0.0.0.0"
WEB_PORT = int(os.environ.get("HSBR_WEB_PORT", "8080"))


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

        self.reader_thread = threading.Thread(target=self._reader_loop)
        self.reader_thread.daemon = True
        self.reader_thread.start()

    def _close_unlocked(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _ensure_open_unlocked(self):
        if self.ser is not None and self.ser.is_open:
            return

        self._close_unlocked()
        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=0.05,
            write_timeout=0.2
        )
        # Do not reset ESP32 input/output buffers here.
        # We want to avoid throwing away useful startup/status messages.
        self.last_error = ""

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

    def send_many(self, commands):
        for command in commands:
            self.send(command)

    def _reader_loop(self):
        while self.running:
            try:
                with self.lock:
                    self._ensure_open_unlocked()
                    ser = self.ser

                raw = ser.readline()
                if raw:
                    self.last_rx = raw.decode("utf-8", errors="replace").strip()

            except Exception as exc:
                self.last_error = str(exc)
                with self.lock:
                    self._close_unlocked()
                time.sleep(0.5)

    def is_open(self):
        return self.ser is not None and self.ser.is_open

    def shutdown(self):
        self.running = False
        try:
            # Best effort: zero the remote command before shutting down.
            self.send("zj 0 0")
        except Exception:
            pass

        with self.lock:
            self._close_unlocked()


app = Flask(__name__)
link = ESP32Link(SERIAL_PORT, SERIAL_BAUD)

current_speed = 0.0
current_steer = 0.0
state_lock = threading.Lock()


@app.route("/")
def index():
    return render_template("remote.html")


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
        last_error=link.last_error
    )


@atexit.register
def cleanup():
    link.shutdown()


if __name__ == "__main__":
    print("HSBR Web Remote")
    print("Serial: {} @ {}".format(SERIAL_PORT, SERIAL_BAUD))
    print("Web:    http://0.0.0.0:{}/".format(WEB_PORT))
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
