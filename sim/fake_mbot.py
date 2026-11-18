"""A small 2D simulator that stands in for mbot_bridge.api.MBot.

It lets hit_the_spot.py and bug_navigation.py run on a laptop without the
robot. The world is a set of axis-aligned rectangular obstacles; the lidar is
simulated by ray casting. time.sleep() and time.time() are patched to use a
simulated clock so runs finish in a fraction of a second of wall time.

Two modes:
  * ideal      - perfect motors, perfect lidar, no latency.
  * realistic  - modelled on logs from a real MBot-Omni: three omni wheels that
                 do not turn at all below a minimum speed and under-deliver
                 above it, a noisy ~8 Hz lidar with a blind zone close to the
                 robot, and a delay on every bridge call.
"""
import math
import random
import time as _time

_sim_clock = [0.0]
_robots = []


def _fake_sleep(secs):
    """Advance the simulated clock and move every robot forward in time."""
    for r in _robots:
        r._integrate(secs)
    _sim_clock[0] += secs


def _fake_time():
    return _sim_clock[0]


def install_fake_time():
    _time.sleep = _fake_sleep
    _time.time = _fake_time


def reset_clock():
    _sim_clock[0] = 0.0
    del _robots[:]


class Rect:
    def __init__(self, xmin, ymin, xmax, ymax):
        self.xmin, self.ymin, self.xmax, self.ymax = xmin, ymin, xmax, ymax

    def contains(self, x, y):
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax

    def ray_hit(self, ox, oy, dx, dy):
        """Distance along ray (ox,oy)+t(dx,dy) to this box, or inf."""
        tmin, tmax = 0.0, float("inf")
        for o, d, lo, hi in ((ox, dx, self.xmin, self.xmax), (oy, dy, self.ymin, self.ymax)):
            if abs(d) < 1e-12:
                if o < lo or o > hi:
                    return float("inf")
            else:
                t1, t2 = (lo - o) / d, (hi - o) / d
                if t1 > t2:
                    t1, t2 = t2, t1
                tmin, tmax = max(tmin, t1), min(tmax, t2)
                if tmin > tmax:
                    return float("inf")
        return tmin if tmin > 0 else float("inf")


# Omni (kiwi) drive geometry: three wheels 120 degrees apart.
_WHEEL_ANGLES = [math.radians(a) for a in (60.0, 180.0, 300.0)]
_WHEEL_BASE_RADIUS = 0.10  # Distance from robot centre to each wheel (m).


class FakeMBot:
    """Drop-in replacement for mbot_bridge.api.MBot."""

    def __init__(self, obstacles=(), num_rays=360, max_range=8.0, realistic=False, seed=1,
                 wheel_deadband=0.15, wheel_gain=0.6, lidar_period=0.125, lidar_noise=0.005,
                 lidar_min_range=0.12, call_latency=0.015):
        self.obstacles = list(obstacles)
        self.num_rays = num_rays
        self.max_range = max_range
        self.realistic = realistic
        self.wheel_deadband = wheel_deadband if realistic else 0.0
        self.wheel_gain = wheel_gain if realistic else 1.0
        self.lidar_period = lidar_period if realistic else 0.0
        self.lidar_noise = lidar_noise if realistic else 0.0
        self.lidar_min_range = lidar_min_range if realistic else 0.0
        self.call_latency = call_latency if realistic else 0.0
        self.rng = random.Random(seed)
        self.x = self.y = self.theta = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.path = [(0.0, 0.0)]
        self.collided = False
        self.min_clearance = float("inf")
        self.radius = 0.12  # Robot footprint radius used for collision checks.
        self._scan = None
        self._scan_time = -1e9
        _robots.append(self)

    def _latency(self):
        if self.call_latency > 0:
            _fake_sleep(self.call_latency)

    # --- publishers ---
    def drive(self, vx, vy, wz):
        self._latency()
        self.vx, self.vy, self.wz = float(vx), float(vy), float(wz)

    def stop(self):
        self.drive(0, 0, 0)

    def reset_odometry(self):
        self.x = self.y = self.theta = 0.0

    # --- subscribers ---
    def read_odometry(self):
        self._latency()
        return [self.x, self.y, self.theta]

    def read_slam_pose(self):
        return self.read_odometry()

    def read_lidar(self):
        self._latency()
        # A real lidar only produces a new scan every so often; in between,
        # the bridge returns the same (stale) scan.
        if self._scan is not None and _sim_clock[0] - self._scan_time < self.lidar_period:
            return list(self._scan[0]), list(self._scan[1])
        ranges, thetas = [], []
        for i in range(self.num_rays):
            th = 2 * math.pi * i / self.num_rays          # robot frame, CCW
            wth = self.theta + th
            dx, dy = math.cos(wth), math.sin(wth)
            d = min((ob.ray_hit(self.x, self.y, dx, dy) for ob in self.obstacles), default=float("inf"))
            if d > self.max_range:
                d = 0.0  # out of range -> invalid, like the real sensor
            else:
                if self.lidar_noise > 0:
                    d += self.rng.gauss(0.0, self.lidar_noise)
                if d < self.lidar_min_range:
                    d = 0.0  # too close for the sensor to measure
            ranges.append(d)
            thetas.append(th)
        self._scan = (ranges, thetas)
        self._scan_time = _sim_clock[0]
        return list(ranges), list(thetas)

    # --- physics ---
    def _actual_velocity(self):
        """Body velocity the robot really achieves for the current command."""
        if not self.realistic:
            return self.vx, self.vy, self.wz
        # Command -> wheel speeds -> dead zone and weak tracking -> body velocity.
        ax = ay = aw = 0.0
        for a in _WHEEL_ANGLES:
            tx, ty = -math.sin(a), math.cos(a)
            s = tx * self.vx + ty * self.vy + _WHEEL_BASE_RADIUS * self.wz
            s = 0.0 if abs(s) < self.wheel_deadband else s * self.wheel_gain
            ax += s * tx
            ay += s * ty
            aw += s
        return (2.0 / 3.0) * ax, (2.0 / 3.0) * ay, aw / (3.0 * _WHEEL_BASE_RADIUS)

    def _integrate(self, dt):
        steps = max(1, int(math.ceil(dt / 0.01)))
        h = dt / steps
        for _ in range(steps):
            vx, vy, wz = self._actual_velocity()
            c, s = math.cos(self.theta), math.sin(self.theta)
            nx = self.x + (c * vx - s * vy) * h
            ny = self.y + (s * vx + c * vy) * h
            self.theta = math.atan2(math.sin(self.theta + wz * h), math.cos(self.theta + wz * h))
            blocked = False
            for ob in self.obstacles:
                if Rect(ob.xmin - self.radius, ob.ymin - self.radius,
                        ob.xmax + self.radius, ob.ymax + self.radius).contains(nx, ny):
                    blocked = True
            if blocked:
                self.collided = True
                continue  # Blocked: don't move into the obstacle.
            self.x, self.y = nx, ny
        self.path.append((self.x, self.y))
        for ob in self.obstacles:
            cx = min(max(self.x, ob.xmin), ob.xmax)
            cy = min(max(self.y, ob.ymin), ob.ymax)
            self.min_clearance = min(self.min_clearance, math.hypot(self.x - cx, self.y - cy) - self.radius)
