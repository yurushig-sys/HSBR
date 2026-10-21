#!/usr/bin/env python3
"""HSBR: Pi owns distance/angle targets; ESP32 executes immediate commands.
Python 3.7+, pyserial. No camera, Flask, automatic stand-up or hitting.
"""
import argparse
import csv
import math
import sys
import time


def prediction(distance, velocity, received_at, now, feedback_ms, command_ms):
    # ESP timestamps are logged, NOT compared directly with the Pi clock.
    # Fixed feedback/command estimates must be calibrated on the actual link.
    horizon = max(0.0, now - received_at) + (feedback_ms + command_ms) / 1000.0
    return distance + max(0.0, velocity) * horizon


class Link:
    def __init__(self, serial_port, writer, timeout=0.2, clock=time.monotonic):
        self.serial = serial_port
        self.writer = writer
        self.clock = clock
        self.timeout = timeout
        self.buffer = bytearray()
        self.latest = None
        self.acks = {}
        self.errors = {}
        self.command_id = 0
        self.running = False
        self.last_heartbeat = 0.0
        self.last_device_ms = None

    def log(self, kind, **kw):
        row = dict(kind=kind, host_s="%.6f" % self.clock())
        row.update(kw)
        self.writer.writerow(row)

    def send(self, line):
        payload = (line + "\n").encode("ascii")
        if self.serial.write(payload) != len(payload):
            raise RuntimeError("Incomplete serial write")

    def ingest(self, raw):
        try:
            fields = raw.decode("ascii").strip().split(",")
            now = self.clock()
            if fields[0] == "OD" and len(fields) == 7:
                device_ms = int(fields[1])
                d, v, angle = map(float, fields[2:5])
                upright, active = map(int, fields[5:7])
                if not all(math.isfinite(x) for x in (d, v, angle)):
                    raise RuntimeError("Non-finite telemetry")
                if self.last_device_ms is not None:
                    delta = (device_ms - self.last_device_ms) & 0xffffffff
                    if delta == 0:
                        return
                    if delta > 10000:
                        raise RuntimeError("ESP32 restart or telemetry clock discontinuity")
                self.last_device_ms = device_ms
                self.latest = dict(device_ms=device_ms, distance_mm=d,
                                   velocity_mm_s=v, angle_deg=angle,
                                   upright=upright, active=active, received=now)
                self.log("OD", device_ms=device_ms, distance_mm=d,
                         velocity_mm_s=v, angle_deg=angle,
                         upright=upright, active=active)
            elif fields[0] == "ACK" and len(fields) == 5:
                ident = int(fields[1])
                self.acks[ident] = dict(device_ms=int(fields[2]),
                                        distance_mm=float(fields[3]),
                                        angle_deg=float(fields[4]), received=now)
            elif fields[0] == "ERR" and len(fields) >= 3:
                self.errors[int(fields[1])] = ",".join(fields[2:])
        except (ValueError, UnicodeError):
            # Human-readable legacy logging is allowed on the same channel.
            return

    def pump(self):
        now = self.clock()
        if self.running and now - self.last_heartbeat >= 0.1:
            self.send("zK")
            self.last_heartbeat = now
        n = min(self.serial.in_waiting, 4096)
        chunk = self.serial.read(n if n else 1)
        self.buffer.extend(chunk)
        while b"\n" in self.buffer:
            raw, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            self.ingest(raw)
        if len(self.buffer) > 4096:
            raise RuntimeError("Serial framing lost")
        if self.running:
            if self.latest is None or self.clock() - self.latest["received"] > self.timeout:
                raise RuntimeError("Telemetry timeout; stop requested")
            if not self.latest["upright"]:
                raise RuntimeError("Robot no longer upright")

    def motion(self, speed, steer):
        self.command_id += 1
        ident = self.command_id
        sent = self.clock()
        self.send("zR %d %.2f %.2f" % (ident, speed, steer))
        self.running = True
        self.last_heartbeat = sent
        self.log("COMMAND", command_id=ident, speed_input=speed, steer_input=steer)
        # Do not resend uncertain commands; fail closed on missing ACK.
        while self.clock() - sent < 0.25:
            self.pump()
            if ident in self.errors or 0 in self.errors:
                raise RuntimeError("ESP32 rejected command: " +
                                   self.errors.get(ident, self.errors.get(0, "")))
            if ident in self.acks:
                ack = self.acks.pop(ident)
                rtt = (ack["received"] - sent) * 1000
                self.log("ACK", command_id=ident, device_ms=ack["device_ms"],
                         distance_mm=ack["distance_mm"], angle_deg=ack["angle_deg"],
                         rtt_ms="%.3f" % rtt)
                print("ACK %d: distance %.1f mm, angle %.1f deg, RTT %.1f ms" %
                      (ident, ack["distance_mm"], ack["angle_deg"], rtt))
                return ack
        raise RuntimeError("No command acknowledgement within 250 ms")

    def stop(self):
        self.running = False
        self.send("zh")
        self.log("STOP")


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True, help="e.g. /dev/ttyUSB0")
    p.add_argument("--distance", type=float, default=500, help="straight before turn, mm")
    p.add_argument("--turn", choices=("right", "left"), default="right")
    p.add_argument("--angle", type=float, default=90, help="positive turn magnitude, deg")
    p.add_argument("--after-mm", type=float, default=300, help="straight after turn, mm")
    p.add_argument("--speed", type=float, default=30, help="speedInput, NOT mm/s")
    p.add_argument("--steer", type=float, default=15, help="absolute steerInput while moving")
    p.add_argument("--lead-mm", type=float, default=0, help="extra early turn correction, mm")
    p.add_argument("--feedback-delay-ms", type=float, default=10, help="ASSUMED sample-to-Pi delay")
    p.add_argument("--command-delay-ms", type=float, default=10, help="ASSUMED Pi-to-application delay")
    p.add_argument("--turn-lead-deg", type=float, default=0, help="extra early end of steering, deg")
    p.add_argument("--timeout", type=float, default=0.2, help="telemetry silence limit, seconds")
    p.add_argument("--max-seconds", type=float, default=20, help="overall motion time limit")
    p.add_argument("--csv", default="hsbr_distance_log.csv")
    a = p.parse_args()
    for key, value in vars(a).items():
        if isinstance(value, float) and not math.isfinite(value):
            p.error("non-finite argument: " + key)
    if not (50 <= a.distance <= 3000 and 50 <= a.after_mm <= 3000 and
            5 <= a.angle <= 180 and 15 <= a.speed <= 60 and 1 <= a.steer <= 60):
        p.error("distance/after 50..3000, angle 5..180, speed 15..60, steer 1..60")
    if not (0 <= a.feedback_delay_ms <= 100 and 0 <= a.command_delay_ms <= 100 and
            abs(a.lead_mm) < a.distance and 0 <= a.turn_lead_deg < a.angle and
            0.05 <= a.timeout <= 0.4 and 1 <= a.max_seconds <= 120):
        p.error("invalid timing/lead/timeout argument")
    return a


def experiment(link, a):
    link.send("ze0")
    link.send("zV 20")
    deadline = link.clock() + 3
    while link.clock() < deadline:
        link.pump()
        if link.latest is not None:
            break
    if link.latest is None:
        raise RuntimeError("No OD telemetry; check the firmware and port")
    if not link.latest["upright"]:
        raise RuntimeError("Stand robot upright BEFORE starting this script")
    started = link.clock()
    ack = link.motion(a.speed, 0)
    origin = ack["distance_mm"]
    target = origin + a.distance
    turn_sign = 1 if a.turn == "right" else -1
    steer = -turn_sign * a.steer  # original theta/steer convention
    state = "STRAIGHT"
    last_print = 0.0
    while True:
        link.pump()
        now = link.clock()
        if now - started > a.max_seconds:
            raise RuntimeError("Overall motion time limit exceeded")
        t = link.latest
        # An inactive sample can predate an ACK; only check samples after it.
        device_delta = (t["device_ms"] - ack["device_ms"]) & 0xffffffff
        if 0 < device_delta < 0x80000000:
            if not t["active"]:
                raise RuntimeError("ESP32 remote mode ended (watchdog/reset/interruption)")
        predicted = prediction(t["distance_mm"], t["velocity_mm_s"],
                               t["received"], now, a.feedback_delay_ms,
                               a.command_delay_ms)
        if now - last_print >= 0.25:
            print("%s d=%.1f mm v=%.1f mm/s theta=%.1f deg" %
                  (state, t["distance_mm"]-origin, t["velocity_mm_s"], t["angle_deg"]))
            last_print = now
        if state == "STRAIGHT" and predicted >= target - a.lead_mm:
            link.log("TURN_TRIGGER", distance_mm=t["distance_mm"],
                     predicted_mm=predicted, target_mm=target)
            ack = link.motion(a.speed, steer)
            turn_start = ack["angle_deg"]
            state = "TURN"
        elif state == "TURN":
            turned = turn_sign * (t["angle_deg"] - turn_start)
            if turned >= a.angle - a.turn_lead_deg:
                ack = link.motion(a.speed, 0)
                target = ack["distance_mm"] + a.after_mm
                state = "AFTER"
        elif state == "AFTER" and predicted >= target:
            link.stop()
            print("Stop requested. Final coasting distance is intentionally uncontrolled.")
            # Capture coasting, without renewing any motion command.
            until = link.clock()+1
            while link.clock() < until:
                link.pump()
            return


def main():
    a = arguments()
    import serial
    fields = ["kind", "host_s", "device_ms", "distance_mm", "velocity_mm_s",
              "angle_deg", "upright", "active", "command_id", "speed_input",
              "steer_input", "rtt_ms", "predicted_mm", "target_mm"]
    # Set control lines before opening; some USB adapters can still reset ESP32.
    port = serial.Serial(port=None, baudrate=115200, timeout=0.005,
                         write_timeout=0.05, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = a.port
    link = None
    try:
        port.open()
        time.sleep(2)  # allow boot if opening the USB port resets ESP32
        port.reset_input_buffer()
        with open(a.csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fields)
            writer.writeheader()
            link = Link(port, writer, a.timeout)
            try:
                experiment(link, a)
            finally:
                # Best effort immediate stop, firmware watchdog is the fallback.
                try:
                    link.stop()
                    link.send("zV 0")
                except Exception as e:
                    print("Stop send failed: %s; firmware watchdog should stop motion" % e,
                          file=sys.stderr)
    finally:
        port.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted; stop requested", file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print("Experiment stopped: %s" % exc, file=sys.stderr)
        sys.exit(1)
