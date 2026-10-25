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
from hsbr.field_hockey_search import PoseTracker, GoalFrame, plan_from_world


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def projected_distance(t, now, feedback_s, command_s):
    return t['distance_mm'] + max(0., t['velocity_mm_s']) * (
        max(0., now-t['od_at']) + feedback_s + command_s)


class Cancelled(RuntimeError):
    pass


class Fallen(RuntimeError):
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
        self.speed = float(os.environ.get('HSBR_FH_SPEED', '40'))
        self.pivot_steer = float(os.environ.get('HSBR_FH_PIVOT_STEER', '20'))
        self.feedback_s = float(os.environ.get('HSBR_FH_FEEDBACK_MS', '10'))/1000.
        self.command_s = float(os.environ.get('HSBR_FH_COMMAND_MS', '10'))/1000.
        self.stroke_s = float(os.environ.get('HSBR_FH_STROKE_SECONDS', '0.18'))
        self.end_lead_deg = float(os.environ.get('HSBR_FH_END_LEAD_DEG', '0'))
        self.STICK_BACK = int(os.environ.get('HSBR_FH_STICK_BACK', '50'))
        self.STICK_HIT = int(os.environ.get('HSBR_FH_STICK_HIT', '150'))
        self.STICK_NEUTRAL = int(os.environ.get('HSBR_FH_STICK_NEUTRAL', '90'))
        self.return_delay_s = float(os.environ.get('HSBR_FH_RETURN_DELAY_SECONDS', '1.5'))
        self.return_speed = float(os.environ.get('HSBR_FH_RETURN_SPEED', '20'))
        self.return_steer = float(os.environ.get('HSBR_FH_RETURN_STEER', '12'))
        self.return_tolerance_mm = 50.
        self.position_valid = False
        self.recovery_active = False
        self.hit_enabled = False
        self.pose = None
        self.goal_frame = None
        self.goal_world = None
        self.ball_world = None
        self.map_state = None
        self.armed_at = None
        self.search_steer = float(os.environ.get('HSBR_FH_SEARCH_STEER', '12'))
        self.search_timeout = float(os.environ.get('HSBR_FH_SEARCH_SECONDS', '20'))
        self.camera_lag_s = float(os.environ.get('HSBR_FH_CAMERA_LAG_MS', '100'))/1000.
        self.search_settle_s = float(os.environ.get('HSBR_FH_SEARCH_SETTLE_MS', '120'))/1000.
        self.raise_s = float(os.environ.get('HSBR_FH_RAISE_SECONDS', '0.5'))
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
                        stroke_seconds=self.stroke_s, map_state=self.map_state,
                        speed_input=self.speed, position_valid=self.position_valid,
                        return_delay_seconds=self.return_delay_s)

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
                      self.stroke_s, self.end_lead_deg, self.search_steer,
                      self.search_timeout, self.camera_lag_s, self.search_settle_s, self.raise_s,
                      self.return_delay_s, self.return_speed, self.return_steer)
        if (not all(finite(v) for v in parameters) or not 15 <= self.speed <= 60 or
                not 1 <= self.pivot_steer <= 60 or not 0 <= self.feedback_s <= .1 or
                not 0 <= self.command_s <= .1 or not .02 <= self.stroke_s <= 1 or
                not 0 <= self.end_lead_deg <= 10 or not 1 <= self.search_steer <= 30 or
                not 5 <= self.search_timeout <= 60 or not 0 <= self.camera_lag_s <= .5 or
                not 0 <= self.search_settle_s <= .5 or not 0 <= self.raise_s <= 2 or
                not 1 <= self.return_delay_s <= 2 or not 5 <= self.return_speed <= 30 or
                not 1 <= self.return_steer <= 20 or
                any(not 0 <= v <= 180 for v in (self.STICK_BACK,self.STICK_HIT,self.STICK_NEUTRAL))):
            return False, '走行／打撃設定が範囲外です'
        with self.lock:
            if self.running or self.shutdown_started:
                return False, '動作中またはシャットダウン中です'
            self.stop_event.clear()
            self.running = True
            self.hit_enabled = bool(hit_enabled)
            self.frozen_route = None
            self.map_state = None
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
                if self.recovery_active:
                    self.link.send('zV 0')
                    self.link.send('zu0')
                    self.recovery_active=False
            except Exception:
                pass
            self.session_motion = False
        self._set_state('stopping' if self.running else 'stopped', message)

    def _check(self):
        if self.stop_event.is_set():
            raise Cancelled('停止しました')
        t = self.link.get_telemetry()
        if self.session_generation is not None and t.get('generation') != self.session_generation:
            self.position_valid=False
            raise RuntimeError('ESP32接続が変わりました。再開せず停止します')
        return t

    def _tick(self, motion=True):
        t = self._check()
        now = self.clock()
        if not t.get('od_at') or now-t['od_at'] > .2:
            self.position_valid=False
            raise RuntimeError('距離情報が200ms以上途絶えました')
        if t.get('enable_control') != 1:
            self.position_valid=False
            if self.pose is not None:
                raise Fallen('走行中に転倒しました')
            raise RuntimeError('起立していません。起立してから開始してください')
        if any(not finite(t.get(k)) for k in ('distance_mm','velocity_mm_s','angle_deg')):
            self.position_valid=False
            raise RuntimeError('距離・速度・角度が不正です')
        if motion and self.session_motion:
            if self.latest_ack is not None:
                delta=(t['device_ms']-self.latest_ack['device_ms']) & 0xffffffff
                if 0 < delta < 0x80000000 and not t.get('remote_active'):
                    raise RuntimeError('ESP32が走行指令を解除しました')
            if now-self.last_heartbeat >= .1:
                self._send('zK')
                self.last_heartbeat = now
        if self.pose is not None:
            self.pose.update(t)
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
        self.latest_ack=None # prior command may have been intentionally halted
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

    @staticmethod
    def _valid_target(target):
        return (target.get('found') and
                finite(target.get('robot_x_mm')) and finite(target.get('robot_y_mm')))

    def _acquire_target(self, key, label):
        self._set_state('search_goal' if key=='green' else 'search_ball', label+'を探しています')
        deadline=self.clock()+self.search_timeout
        initial_wait=self.clock()+.15
        scan_origin=self.pose.heading
        scanning=False
        refining=False
        refine_after=0.
        last_stamp=None
        last_fresh=self.clock()
        while self.clock()<deadline:
            self._tick()
            now=self.clock()
            status=self.detector.get_status()
            observed=status.get('observed_at',0.)
            target=status.get(key) or {}
            fresh=(finite(observed) and observed>0 and 0<=now-observed<.3)
            valid=fresh and self._valid_target(target)
            new=fresh and observed!=last_stamp
            if new:
                last_stamp=observed
                last_fresh=now
            if now-last_fresh>.6:
                raise RuntimeError('新しいカメラ画像が600ms以上届いていません')
            if refining:
                # A moving detection triggers a stop, then a fresh measurement
                # after the image pipeline and measured yaw have settled.
                if (new and valid and observed>=refine_after and
                        abs(self.pose.angular_speed)<=5.):
                    return self.pose.target_world(target,observed-self.camera_lag_s)
                if now>=refine_after+.6:
                    refining=False
            elif scanning:
                if new and valid:
                    self._motion(0,0)
                    scanning=False
                    refining=True
                    refine_after=self.clock()+self.camera_lag_s+self.search_settle_s
                    self._set_state('measure_goal' if key=='green' else 'measure_ball',
                                    label+'を検出。新しい画像で座標を確定します')
            elif valid:
                return self.pose.target_world(target,observed-self.camera_lag_s)
            elif (new or now>=initial_wait):
                self._motion(0,-self.search_steer) # leftward scan: increasing theta
                scanning=True
                self._set_state('search_goal' if key=='green' else 'search_ball',
                                label+'を旋回して探索しています')
            if abs(math.degrees(self.pose.heading-scan_origin))>=390:
                raise RuntimeError(label+'を一周探索しても測距できませんでした')
            self._wait(.005,telemetry=True)
        raise RuntimeError(label+'の探索が時間内に完了しませんでした')

    def _capture_route(self):
        self.goal_world=self._acquire_target('green','ゴール')
        self.goal_frame=GoalFrame(self.goal_world)
        self.map_state=self.goal_frame.describe(self.pose)
        self.link._log_record('FH_MAP','goal='+repr(self.map_state))
        self.ball_world=self._acquire_target('red','ボール')
        self.map_state=self.goal_frame.describe(self.pose,self.ball_world)
        self.link._log_record('FH_MAP','ball='+repr(self.map_state))
        self._set_state('planning','ゴール基準座標から軌道を計算しています')
        began=self.clock()
        route=plan_from_world(self.detector,self.pose,self.goal_world,self.ball_world)
        keys=('first_turn_deg','straight_mm','orbit_radius_mm','arc_deg',
              'hit_heading_deg','hit_x_mm','hit_y_mm')
        if any(not finite(route.get(k)) for k in keys):
            raise RuntimeError('軌道に不正な数値があります')
        if (not 60 <= route['orbit_radius_mm'] <= 5000 or
                not 0 <= route['straight_mm'] <= 5000 or
                not .1 <= abs(route['arc_deg']) <= 270):
            raise RuntimeError('軌道が実行範囲外です（円弧0.1～270度、直進5m以内）')
        route['goal_frame']=dict(self.map_state)
        self.frozen_route=route
        self.link._log_record('FH_PLAN','seconds={:.6f} route={}'.format(
            self.clock()-began,route['command']))
        return route

    def _begin_remote(self):
        # No stand-up command, standing wait or pose-stability wait on start.
        # Reuse capabilities only within the same serial connection generation.
        self._send('ze0')
        if not self.link.get_telemetry().get('radius_cap_at'):
            before=self.clock()
            self._send('zP')
            while self.clock()-before<1.:
                t=self._check()
                if t.get('radius_cap_at',0)>=before:
                    break
                self._wait(.005)
            else:
                raise RuntimeError('半径指定ファームウェアを確認できません')
        before=self.clock()
        self._send('zV 20')
        self.session_started=True
        while self.clock()-before<1.:
            t=self._check()
            if t.get('od_at',0)>=before:
                t=self._tick(motion=False)
                self.pose=PoseTracker(t)
                self.position_valid=True
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
        # The stick has been raising in parallel with acquisition/pivot.
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
                if self.armed_at is None or self.clock()-self.armed_at<self.raise_s:
                    raise RuntimeError('振り上げ準備時間が不足しています。ボールから距離を取って再実行してください')
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
        self._wait(self.return_delay_s,telemetry=True)
        if self.hit_enabled:
            self._send('zss {}'.format(self.STICK_NEUTRAL))
        self._return_home()
        self._set_state('completed','開始位置へ帰還しました（車輪距離による推定）')

    @staticmethod
    def _angle_error(target, actual):
        return (target-actual+math.pi) % (2*math.pi)-math.pi

    def _update_map(self):
        if self.goal_frame is not None and self.pose is not None:
            self.map_state=self.goal_frame.describe(self.pose,self.ball_world)

    def _settle(self, minimum=.3):
        """Track coasting as well as powered motion; never assume zh stops at once."""
        began=self.clock()
        stable=None
        while self.clock()-began<3.:
            t=self._tick(motion=False)
            self._update_map()
            if abs(t['velocity_mm_s'])<30 and abs(self.pose.angular_speed)<5:
                if stable is None:stable=self.clock()
                if self.clock()-began>=minimum and self.clock()-stable>=.25:return
            else:stable=None
            self._wait(.005,telemetry=True)
        raise RuntimeError('惰性走行が収まりません。帰還を停止します')

    def _turn_home(self, heading):
        # Shortest in-place turn, at lower steering input than the hitting run.
        # A second pass corrects residual rotation after releasing steering.
        for attempt in range(2):
            error=self._angle_error(heading,self.pose.heading)
            if abs(math.degrees(error))<=3:return
            sign=1 if error>0 else -1
            self._motion(0,-sign*self.return_steer)
            until=self.clock()+15.
            while True:
                self._tick()
                error=self._angle_error(heading,self.pose.heading)
                lead=math.radians(abs(self.pose.angular_speed))*(self.feedback_s+self.command_s+.02)
                if sign*error<=max(math.radians(1),lead):break
                if self.clock()>until:raise RuntimeError('帰還旋回が時間内に終わりません')
                self._wait(.005,telemetry=True)
            self._halt()
            self._settle()
        if abs(math.degrees(self._angle_error(heading,self.pose.heading)))>5:
            raise RuntimeError('帰還方向を合わせられません。その場で待機します')

    def _return_home(self):
        if not self.position_valid:
            raise RuntimeError('開始位置の座標を信頼できません。その場で待機します')
        self._set_state('returning','低速で開始位置へ戻ります')
        self._settle()
        began=self.clock()
        for attempt in range(3):
            self._tick()
            remaining=math.hypot(self.pose.x,self.pose.y)
            if remaining<=self.return_tolerance_mm:break
            self._turn_home(math.atan2(-self.pose.y,-self.pose.x))
            # In-place balance can translate the body: recompute the distance.
            remaining=math.hypot(self.pose.x,self.pose.y)
            if remaining<=self.return_tolerance_mm:break
            start=self._motion(self.return_speed)
            target=start['distance_mm']+remaining
            while True:
                t=self._tick()
                self._update_map()
                if projected_distance(t,self.clock(),self.feedback_s,self.command_s)>=target:break
                if self.clock()-began>60:
                    raise RuntimeError('帰還が時間内に終わりません。その場で待機します')
                self._wait(.005,telemetry=True)
            self._halt()
            self._settle(.8)
        if math.hypot(self.pose.x,self.pose.y)>self.return_tolerance_mm:
            raise RuntimeError('開始位置へ収束しません。その場で待機します')
        self._set_state('return_heading','開始時の向きに戻しています')
        self._turn_home(0.)
        self._update_map()
        error=math.hypot(self.pose.x,self.pose.y)
        self.link._log_record('FH_HOME','estimated_error_mm={:.1f}'.format(error))
        if error>self.return_tolerance_mm:
            raise RuntimeError('方向合わせ後に開始位置からずれました。その場で待機します')

    def _recover_fall(self):
        # ESP32 odometry_init() runs on both fall and re-enable. Wheel motion
        # while horizontal also is not ground travel. Position is untrustworthy.
        self.position_valid=False
        self.map_state=None
        self.pose=None
        self.latest_ack=None
        self._halt()
        self._set_state('fallen','転倒を検出。座標を無効にし、自動起立を準備します')
        self._wait(1.) # allow the body to finish falling; STOP remains active
        self._send('zV 0') # firmware rejects legacy zu1 inside a remote session
        self.session_started=False
        self._send('zss {}'.format(self.STICK_NEUTRAL))
        self.recovery_active=True
        self._send('zu1') # one attempt only; do not resume the interrupted hit
        self._send('zV 20') # retain serial ownership/OD while ESP32 stands itself
        self.session_started=True
        self._set_state('recovering','自動起立中。起立後はその場で待機します')
        began=self.clock()
        stable=None
        while self.clock()-began<8.:
            t=self._check()
            now=self.clock()
            if not t.get('od_at') or now-t['od_at']>.2:
                raise RuntimeError('起立中に距離情報が途絶えました。その場で待機します')
            if (not finite(t.get('battery_voltage')) or now-t.get('data_at',0)>2 or
                    t['battery_voltage']<=self.CRITICAL_VOLTAGE):
                raise RuntimeError('起立中の電圧が不足または不明です')
            if t.get('enable_control')==1:
                if stable is None:stable=now
                # ESP32 stand_up takes >2 seconds; an early transient upright
                # sample must not finish recovery or permit any driving.
                if now-began>=3 and now-stable>=1:
                    self._halt()
                    self.recovery_active=False
                    self._set_state('waiting','起立しました。転倒で座標が無効のため、その場で待機します')
                    return
            else:stable=None
            self._wait(.01)
        # Abort the stand-up state machine before reporting failure. zu0 requests
        # a controlled stand-down; it never starts another stand-up attempt.
        raise RuntimeError('自動起立を確認できませんでした。その場で待機します')

    def _run_once(self):
        try:
            self.session_generation=self.link.get_telemetry().get('generation')
            self.latest_ack=None
            self.session_started=False
            self.session_motion=False
            self.pose=None
            self.position_valid=False
            self.goal_frame=None
            self.goal_world=self.ball_world=None
            self.armed_at=None
            self._begin_remote()
            if self.hit_enabled:
                self._send('zss {}'.format(self.STICK_BACK))
                self.armed_at=self.clock()
            # Usually the background voltage monitor already has a fresh value.
            t=self.link.get_telemetry()
            if not finite(t.get('battery_voltage')) or self.clock()-t.get('data_at',0)>2:
                self._send('zd1')
                until=self.clock()+.2
                while self.clock()<until:
                    t=self.link.get_telemetry()
                    if finite(t.get('battery_voltage')) and self.clock()-t.get('data_at',0)<2:
                        break
                    self._wait(.005,telemetry=True)
            if not finite(t.get('battery_voltage')) or self.clock()-t.get('data_at',0)>2:
                raise RuntimeError('新しいバッテリ電圧を取得できません')
            if t['battery_voltage']<=self.CRITICAL_VOLTAGE:
                raise RuntimeError('10.5V以下です。走行を開始しません')
            route=self._capture_route()
            self._execute(route)
        except Fallen:
            try:
                self._recover_fall()
            except Cancelled:
                self._set_state('stopped','停止しました')
            except Exception as exc:
                self._set_state('error',str(exc))
        except Cancelled:
            self._set_state('stopped','停止しました')
        except Exception as exc:
            self._set_state('error',str(exc))
        finally:
            with self.command_lock:
                if self.recovery_active:
                    # Cancel an incomplete stand-up even on STOP, telemetry loss
                    # or a voltage error. Never leave its servo routine running.
                    for command in ('zV 0','zu0'):
                        try:self.link.send(command)
                        except Exception:pass
                    self.recovery_active=False
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
