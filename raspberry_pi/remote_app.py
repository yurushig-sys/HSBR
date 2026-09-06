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
from datetime import datetime
import os
import threading
import time

from flask import Flask, jsonify, render_template, request
import serial


SERIAL_PORT = os.environ.get("HSBR_SERIAL", "/dev/ttyUSB0")
SERIAL_BAUD = int(os.environ.get("HSBR_BAUD", "115200"))

# Server-side safety limits.
# The browser UI uses smaller limits by default.
SERVER_MAX_SPEED = 50.0
SERVER_MAX_STEER = 30.0

WEB_HOST = "0.0.0.0"
WEB_PORT = int(os.environ.get("HSBR_WEB_PORT", "8080"))

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

        # Serial log state. Only this process owns /dev/ttyUSB0;
        # miniterm/other serial monitors must remain stopped.
        self.log_lock = threading.Lock()
        self.log_fp = None
        self.log_path = ""
        self.log_started_monotonic = 0.0

        # Buffer bytes until a complete newline-terminated ESP32 line arrives.
        # This prevents timeout-fragmented reads from becoming broken log lines.
        self.rx_buffer = bytearray()

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
            self.send("zj 0 0")
        except Exception:
            pass

        self.stop_log()

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
    link.shutdown()


if __name__ == "__main__":
    print("HSBR Web Remote")
    print("Serial: {} @ {}".format(SERIAL_PORT, SERIAL_BAUD))
    print("Web:    http://0.0.0.0:{}/".format(WEB_PORT))
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
