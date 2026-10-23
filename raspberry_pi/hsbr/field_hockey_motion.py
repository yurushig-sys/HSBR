"""Pi-supervised initial pivot -> tangent straight -> radius arc -> hit/halt.
Python 3.7. The app's ESP32Link is the ONLY serial owner.
Angles: + left, - right, matching the verified odometry convention.
"""
import collections
import math
import os
import subprocess
import threading
import time


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def projected_distance(t, now, feedback_s, command_s):
    return t['distance_mm'] + max(0., t['velocity_mm_s']) * (
        max(0., now-t['od_at']) + feedback_s + command_s)


class Cancelled(RuntimeError):
    pass


class FieldHockeyAutoController(object):
    WARNING_VOLTAGE = 11.0
    CRITICAL_VOLTAGE = 10.5
    CRITICAL_SECONDS = 2.0
    STICK_BACK = 50
    STICK_HIT = 150
    STICK_NEUTRAL = 90

    def __init__(self, link, detector, monitor=True, clock=time.monotonic):
        self.link, self.detector, self.clock = link, detector, clock
        self.lock = threading.RLock()
        self.command_lock = threading.RLock()
        self.stop_event = threading.Event()
        self.running = False
        self.state, self.message = 'idle', '停止中'
        self.last_command = ''
        self.warning = False
        self.shutdown_started = False
        self.critical_since = None
        self.frozen_route = None
        self.session_generation = None
        self.latest_ack = None
        self.last_heartbeat = 0.
        self.session_motion = False
        self.session_started = False
        self.max_seconds = 30.
        self.speed = float(os.environ.get('HSBR_FH_SPEED', '30'))
        self.pivot_steer = float(os.environ.get('HSBR_FH_PIVOT_STEER', '20'))
        self.feedback_s = float(os.environ.get('HSBR_FH_FEEDBACK_MS', '10'))/1000.
        self.command_s = float(os.environ.get('HSBR_FH_COMMAND_MS', '10'))/1000.
        self.stroke_s = float(os.environ.get('HSBR_FH_STROKE_SECONDS', '0.18'))
        self.end_lead_deg = float(os.environ.get('HSBR_FH_END_LEAD_DEG', '0'))
        self.STICK_BACK = int(os.environ.get('HSBR_FH_STICK_BACK', '50'))
        self.STICK_HIT = int(os.environ.get('HSBR_FH_STICK_HIT', '150'))
        self.STICK_NEUTRAL = int(os.environ.get('HSBR_FH_STICK_NEUTRAL', '90'))
        self.hit_enabled = False
        if monitor:
            thread = threading.Thread(target=self._monitor_loop, name='fh-voltage')
            thread.daemon = True
            thread.start()

    def _set_state(self, state, message):
        with self.lock:
            self.state, self.message = state, message
        self.link._log_record('FH', state + ' ' + message)

    def get_status(self):
        t = self.link.get_telemetry()
        with self.lock:
            return dict(running=self.running, state=self.state, message=self.message,
                        last_command=self.last_command,
                        battery_voltage=t.get('battery_voltage'), battery_warning=self.warning,
                        shutdown_started=self.shutdown_started,
                        enable_control=t.get('enable_control'), continuous_flag=t.get('continuous_flag'),
                        hit_enabled=self.hit_enabled, frozen_route=self.frozen_route,
                        distance_mm=t.get('distance_mm'), angle_deg=t.get('angle_deg'),
                        stroke_seconds=self.stroke_s)

    def start(self, hit_enabled=False, stroke_seconds=None):
        if stroke_seconds is not None:
            try:
                candidate=float(stroke_seconds)
            except (TypeError,ValueError):
                return False,'振り下ろし時間が不正です'
            if not finite(candidate) or not .02<=candidate<=1:
                return False,'振り下ろし時間は0.02～1秒で指定してください'
            with self.lock:
                if self.running:
                    return False,'すでに動作中です'
                self.stroke_s=candidate
        parameters = (self.speed, self.pivot_steer, self.feedback_s, self.command_s,
                      self.stroke_s, self.end_lead_deg)
        if (not all(finite(v) for v in parameters) or not 15 <= self.speed <= 60 or
                not 1 <= self.pivot_steer <= 60 or not 0 <= self.feedback_s <= .1 or
                not 0 <= self.command_s <= .1 or not .02 <= self.stroke_s <= 1 or
                not 0 <= self.end_lead_deg <= 10 or
                any(not 0 <= v <= 180 for v in (self.STICK_BACK,self.STICK_HIT,self.STICK_NEUTRAL))):
            return False, '走行／打撃設定が範囲外です'
        with self.lock:
            if self.running or self.shutdown_started:
                return False, '動作中またはシャットダウン中です'
            self.stop_event.clear()
            self.running = True
            self.hit_enabled = bool(hit_enabled)
            self.frozen_route = None
            self.state, self.message = 'starting', '単発動作を開始します'
        worker = threading.Thread(target=self._run_once, name='field-hockey-auto')
        worker.daemon = True
        worker.start()
        return True, '開始しました'

    def _send(self, command):
        with self.command_lock:
            if self.stop_event.is_set():
                raise Cancelled('停止しました')
            self.link.send(command)
            with self.lock:
                self.last_command = command

    def _halt(self):
        with self.command_lock:
            self.link.send('zh')
            self.session_motion = False

    def stop(self, message='操作者が停止しました'):
        # Do not clear running until the worker has actually exited.
        self.stop_event.set()
        with self.command_lock:
            try:
                self.link.send('zh')
            except Exception:
                pass
            self.session_motion = False
        self._set_state('stopping' if self.running else 'stopped', message)

    def _check(self):
        if self.stop_event.is_set():
            raise Cancelled('停止しました')
        t = self.link.get_telemetry()
        if self.session_generation is not None and t.get('generation') != self.session_generation:
            raise RuntimeError('ESP32接続が変わりました。再開せず停止します')
        return t

    def _tick(self, motion=True):
        t = self._check()
        now = self.clock()
        if not t.get('od_at') or now-t['od_at'] > .2:
            raise RuntimeError('距離情報が200ms以上途絶えました')
        if t.get('enable_control') != 1:
            raise RuntimeError('倒立制御が解除されました')
        if any(not finite(t.get(k)) for k in ('distance_mm','velocity_mm_s','angle_deg')):
            raise RuntimeError('距離・速度・角度が不正です')
        if motion and self.session_motion:
            if self.latest_ack is not None:
                delta=(t['device_ms']-self.latest_ack['device_ms']) & 0xffffffff
                if 0 < delta < 0x80000000 and not t.get('remote_active'):
                    raise RuntimeError('ESP32が走行指令を解除しました')
            if now-self.last_heartbeat >= .1:
                self._send('zK')
                self.last_heartbeat = now
        return t

    def _wait(self, seconds, telemetry=False):
        end=self.clock()+seconds
        while self.clock()<end:
            self._tick(motion=self.session_motion) if telemetry else self._check()
            if self.stop_event.wait(.005):
                raise Cancelled('停止しました')

    def _motion(self, speed, steer=0, radius=None):
        ident=self.link.next_command_id()
        sent=self.clock()
        command=('zR {} {:.2f} {:.2f}'.format(ident,speed,steer) if radius is None else
                 'zC {} {:.2f} {:.2f}'.format(ident,speed,radius))
        self._send(command)
        self.session_motion=True
        self.last_heartbeat=sent
        while self.clock()-sent<.25:
            self._tick()
            result=self.link.get_command_result(ident)
            if result:
                if result.get('error'):
                    raise RuntimeError('ESP32: '+result['error'])
                self.latest_ack=result
                self.link._log_record('FH_ACK','id={} RTT_ms={:.2f} d={} th={}'.format(
                    ident,(result['ack_at']-sent)*1000,result['distance_mm'],result['angle_deg']))
                return result
            if self.stop_event.wait(.003):
                raise Cancelled('停止しました')
        raise RuntimeError('走行指令のACKが250ms以内に返りません')

    def _standing(self):
        self._set_state('stand_check','起立状態を確認しています')
        self._send('zV 0')
        self._send('zd0')
        self._wait(.2)
        t=self._check()
        if t.get('enable_control') != 1:
            self._send('zu1')
        until=self.clock()+12
        stable=None
        while self.clock()<until:
            self._send('zd0')
            self._wait(.15)
            t=self._check()
            if t.get('enable_control') == 1 and self.clock()-t.get('status_at',0)<.5:
                stable=stable or self.clock()
                if self.clock()-stable>=1.:
                    return
            else:
                stable=None
        raise RuntimeError('起立を確認できません')

    def _capture_route(self):
        self._set_state('searching','ゴール・ボールを測定し、軌道を確定します')
        until=self.clock()+8
        last_stamp=None
        samples=[]
        while self.clock()<until:
            self._check()
            status=self.detector.get_status()
            stamp=status.get('processed_at',0)
            route=status.get('route') or {}
            if (stamp != last_stamp and abs(time.time()-stamp)<1. and route.get('available')):
                last_stamp=stamp
                if samples and (abs(route['hit_x_mm']-samples[0]['hit_x_mm'])>40 or
                                abs(route['hit_y_mm']-samples[0]['hit_y_mm'])>40 or
                                abs(route['first_turn_deg']-samples[0]['first_turn_deg'])>5 or
                                abs(route['arc_deg']-samples[0]['arc_deg'])>5):
                    samples=[]
                samples.append(dict(route))
                if len(samples)>=5:
                    keys=('first_turn_deg','straight_mm','orbit_radius_mm','arc_deg',
                          'hit_heading_deg','hit_x_mm','hit_y_mm')
                    if any(not finite(route.get(k)) for k in keys):
                        raise RuntimeError('軌道に不正な数値があります')
                    if (not 60 <= route['orbit_radius_mm'] <= 5000 or
                            not 0 <= route['straight_mm'] <= 5000 or
                            not .1 <= abs(route['arc_deg']) <= 270):
                        raise RuntimeError('軌道が実行範囲外です（円弧0.1～270度、直進5m以内）')
                    self.frozen_route=dict(route)
                    return self.frozen_route
            self._wait(.03)
        raise RuntimeError('新しい安定した軌道を取得できません。検出画面を開いて確認してください')

    def _begin_remote(self):
        self._send('ze0')
        before=self.clock()
        self._send('zP')
        while self.clock()-before<1.:
            self._check()
            if self.link.get_telemetry().get('radius_cap_at',0)>=before:
                break
            self._wait(.01)
        else:
            raise RuntimeError('半径指定ファームウェア未確認。ESP32へ今回の2ファイルを書き込んでください')
        before=self.clock()
        self._send('zV 20')
        self.session_started=True
        while self.clock()-before<1.:
            t=self._check()
            if t.get('od_at',0)>=before:
                self._tick(motion=False)
                return
            self._wait(.005)
        raise RuntimeError('OD距離情報を受信できません')

    def _pivot(self, angle):
        if abs(angle)<.5:
            return
        self._set_state('initial_turn','軌道の接線方向へ初期旋回しています')
        sign=1 if angle>0 else -1
        ack=self._motion(0,-sign*self.pivot_steer)
        target=abs(angle)
        until=self.clock()+10
        while self.clock()<until:
            t=self._tick()
            if sign*(t['angle_deg']-ack['angle_deg'])>=max(0,target-1.):
                # Immediately start the tangent straight; no stop/settle phase.
                return
            self._wait(.005,telemetry=True)
        raise RuntimeError('初期旋回が時間内に終わりません')

    def _execute(self, route):
        if self.hit_enabled:
            self._send('zss {}'.format(self.STICK_BACK))
            self._wait(.5,telemetry=True)
        deadline=self.clock()+self.max_seconds
        self._pivot(route['first_turn_deg'])
        self._set_state('straight','接点まで直進しています')
        start=self._motion(self.speed)
        target=start['distance_mm']+route['straight_mm']
        fired=False
        arc_length=route['orbit_radius_mm']*abs(math.radians(route['arc_deg']))
        def fire(t, eta, remaining_deg=None):
            nonlocal fired
            if self.hit_enabled:
                self._send('zss {}'.format(self.STICK_HIT))
            fired=True
            self.link._log_record('FH_HIT','enabled={} eta={:.3f} remaining_deg={} d={}'.format(
                self.hit_enabled,eta,remaining_deg,t['distance_mm']))
            self._set_state('hitting' if self.hit_enabled else 'orbit',
                            '振り下ろし指令済み' if self.hit_enabled else '打撃予定時刻を記録（走行のみ）')
        while True:
            t=self._tick()
            if self.clock()>deadline:
                raise RuntimeError('移動時間を超えました')
            predicted=projected_distance(t,self.clock(),self.feedback_s,self.command_s)
            # A short final arc may take less time than the stroke itself.
            # In that case start the swing during the tangent straight, so
            # the lowest point still coincides with the end of the final arc.
            if not fired and t['velocity_mm_s']>20:
                eta=(max(0.,target-predicted)+arc_length)/t['velocity_mm_s']
                if eta<=self.stroke_s:
                    fire(t,eta)
            if predicted>=target:
                break
            self._wait(.005,telemetry=True)
        sign=1 if route['arc_deg']>0 else -1
        self._set_state('orbit','半径指定で回り込み中。終点で打撃・停止します')
        ack=self._motion(self.speed,radius=sign*route['orbit_radius_mm'])
        arc=abs(route['arc_deg'])
        history=collections.deque()
        while True:
            t=self._tick()
            now=self.clock()
            if now>deadline:
                raise RuntimeError('回り込み時間を超えました')
            if not history or history[-1][0] != t['device_ms']:
                history.append((t['device_ms'], t['angle_deg']))
            while len(history)>2 and ((history[-1][0]-history[0][0]) & 0xffffffff)>100:
                history.popleft()
            omega=0.
            if len(history)>1:
                dt=((history[-1][0]-history[0][0]) & 0xffffffff)/1000.
                if dt>0:
                    omega=max(0.,sign*(history[-1][1]-history[0][1])/dt)
            progress=sign*(t['angle_deg']-ack['angle_deg'])
            current_progress=progress+omega*(max(0.,now-t['od_at'])+self.feedback_s)
            remaining=max(0.,arc-current_progress)
            # Time to geometric end of the circle. Angle rate incorporates
            # actual turning motion instead of assuming a fixed steer/radius map.
            eta=remaining/omega if omega>1 else float('inf')
            if not fired and eta<=self.stroke_s+self.command_s:
                fire(t,eta,remaining)
            if current_progress+omega*self.command_s>=arc-self.end_lead_deg:
                self._halt()  # no after-hit powered straight segment
                if self.hit_enabled and not fired:
                    raise RuntimeError('打撃の先行タイミングを取得できませんでした')
                break
            self._wait(.005,telemetry=True)
        self._set_state('coasting','円弧終点で停止指令。惰性走行中です')
        self._wait(max(.6, self.stroke_s+.2),telemetry=True)
        if self.hit_enabled:
            self._send('zss {}'.format(self.STICK_NEUTRAL))
        self._set_state('completed','単発ヒット完了' if self.hit_enabled else '打撃なしの軌道確認完了')

    def _run_once(self):
        try:
            self.session_generation=self.link.get_telemetry().get('generation')
            self.latest_ack=None
            self.session_started=False
            self.session_motion=False
            self._standing()
            self._send('zd1')
            self._wait(.3)
            voltage=self.link.get_telemetry().get('battery_voltage')
            if not finite(voltage):
                raise RuntimeError('バッテリ電圧を取得できません')
            if voltage<=self.CRITICAL_VOLTAGE:
                raise RuntimeError('10.5V以下です。走行を開始しません')
            route=self._capture_route()
            self._begin_remote()
            self._execute(route)
        except Cancelled:
            self._set_state('stopped','停止しました')
        except Exception as exc:
            self._set_state('error',str(exc))
        finally:
            with self.command_lock:
                for command in ('zh','zV 0'):
                    try:
                        self.link.send(command)
                    except Exception:
                        pass
                self.session_motion=False
            with self.lock:
                self.running=False
                self.session_started=False

    def _critical_shutdown(self, voltage):
        with self.lock:
            if self.shutdown_started:
                return
            self.shutdown_started=True
        self.stop('低電圧 {:.2f}V：停止してシャットダウンします'.format(voltage))
        with self.command_lock:
            try:
                self.link.send_many(['zh','zV 0','zss {}'.format(self.STICK_NEUTRAL),'zu0'])
            except Exception:
                pass
        time.sleep(1)
        self.link.stop_log()
        subprocess.Popen(['sudo','-n','/sbin/shutdown','-h','now'])

    def _monitor_loop(self):
        while True:
            try:
                self.link.send('zd1')
                time.sleep(.3)
                t=self.link.get_telemetry()
                voltage=t.get('battery_voltage')
                if finite(voltage) and self.clock()-t.get('data_at',0)<2:
                    self.warning=voltage<=self.WARNING_VOLTAGE
                    if self.running and voltage<=self.CRITICAL_VOLTAGE:
                        self.critical_since=self.critical_since or self.clock()
                        if self.clock()-self.critical_since>=self.CRITICAL_SECONDS:
                            self._critical_shutdown(voltage)
                    else:
                        self.critical_since=None
            except Exception:
                pass
            time.sleep(.7)
