"""Geometry for goal-first, ball-second acquisition. No OpenCV/serial imports.
World origin = robot at first OD of this run; world heading = its initial heading.
Goal frame: origin unchanged, +X toward recorded goal, +Y to its left.
Camera/body coordinates retain +X forward, +Y left.
"""
import bisect
import collections
import math


def rotate(point, angle):
    c,s=math.cos(angle),math.sin(angle)
    return c*point[0]-s*point[1], s*point[0]+c*point[1]


def finite_point(p):
    return p is not None and len(p)==2 and all(math.isfinite(v) for v in p)


class PoseTracker(object):
    def __init__(self, telemetry):
        self.origin_angle=math.radians(telemetry['angle_deg'])
        self.x=self.y=self.heading=0.
        self.distance=telemetry['distance_mm']
        self.device_ms=telemetry['device_ms']
        self.received=telemetry['od_at']
        self.history=collections.deque([(self.received,0.,0.,0.)],maxlen=2000)
        self.angular_speed=0.

    def update(self,t):
        if t['device_ms']==self.device_ms:
            return
        delta=(t['device_ms']-self.device_ms)&0xffffffff
        if delta>=0x80000000:
            raise RuntimeError('ESP32時刻が戻りました')
        h=math.radians(t['angle_deg'])-self.origin_angle
        dh=h-self.heading
        ds=t['distance_mm']-self.distance
        # Exact constant-curvature chord from arc distance and heading delta.
        half=dh/2
        chord=ds*(math.sin(half)/half if abs(half)>1e-8 else 1.)
        self.x+=chord*math.cos(self.heading+half)
        self.y+=chord*math.sin(self.heading+half)
        self.angular_speed=math.degrees(dh)/(delta/1000.) if delta else 0.
        self.heading=h;self.distance=t['distance_mm'];self.device_ms=t['device_ms']
        self.received=t['od_at']
        self.history.append((self.received,self.x,self.y,h))

    def at(self,when):
        history=list(self.history)
        if when<=history[0][0]:return history[0][1:]
        if when>=history[-1][0]:return history[-1][1:]
        i=bisect.bisect_right([p[0] for p in history],when)
        a,b=history[i-1],history[i]
        f=(when-a[0])/(b[0]-a[0]) if b[0]!=a[0] else 1.
        return tuple(a[j]+f*(b[j]-a[j]) for j in (1,2,3))

    def target_world(self,target,when):
        body=(target.get('robot_x_mm'),target.get('robot_y_mm'))
        if any(v is None or not math.isfinite(v) for v in body):
            raise RuntimeError('対象の座標を算出できません')
        x,y,h=self.at(when)
        dx,dy=rotate(body,h)
        return x+dx,y+dy

    def body_point(self,world):
        return rotate((world[0]-self.x,world[1]-self.y),-self.heading)


class GoalFrame(object):
    def __init__(self,goal_world):
        if not finite_point(goal_world) or math.hypot(*goal_world)<100:
            raise RuntimeError('ゴール座標を定義できません')
        self.goal=tuple(goal_world)
        self.angle=math.atan2(goal_world[1],goal_world[0])
        self.goal_distance=math.hypot(*goal_world)

    def point(self,world):return rotate(world,-self.angle)

    def describe(self,pose,ball=None):
        robot=self.point((pose.x,pose.y))
        out=dict(frame='start_origin_goal_x_left_y',goal_x_mm=self.goal_distance,
                 goal_y_mm=0.,robot_x_mm=robot[0],robot_y_mm=robot[1],
                 robot_heading_deg=math.degrees(pose.heading-self.angle))
        if ball is not None:
            out['ball_x_mm'],out['ball_y_mm']=self.point(ball)
        return out


def plan_from_world(detector,pose,goal_world,ball_world):
    # The planner accepts signed body coordinates, including negative X.
    # No requirement that ball and goal are visible in the same camera image.
    bx,by=pose.body_point(ball_world);gx,gy=pose.body_point(goal_world)
    target=lambda x,y:dict(found=True,robot_x_mm=x,robot_y_mm=y)
    route=detector._plan_route(target(bx,by),target(gx,gy))
    if not route.get('available'):
        raise RuntimeError(route.get('reason') or '軌道を計算できません')
    return route
