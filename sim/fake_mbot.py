"""A tiny 2D simulator that stands in for mbot_bridge.api.MBot.

It lets hit_the_spot.py and bug_navigation.py run on a laptop without the
robot. The world is a set of axis-aligned rectangular obstacles; the lidar
is simulated by ray casting. time.sleep() is patched to advance a simulated
clock (and integrate the robot's motion) so runs finish in a fraction of a
second of wall time.
"""
import math
import time as _time

_sim_clock = [0.0]
_robots = []
_real_sleep = _time.sleep


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


class FakeMBot:
    """Drop-in replacement for mbot_bridge.api.MBot."""

    def __init__(self, obstacles=(), num_rays=360, max_range=8.0, noise=0.0, log_path=True):
        self.obstacles = list(obstacles)
        self.num_rays = num_rays
        self.max_range = max_range
        self.noise = noise
        self.x = self.y = self.theta = 0.0
        self.vx = self.vy = self.wz = 0.0
        self.path = [(0.0, 0.0)] if log_path else None
        self.collided = False
        self.radius = 0.12  # MBot footprint radius used for collision checks.
        _robots.append(self)

    # --- publishers ---
    def drive(self, vx, vy, wz):
        self.vx, self.vy, self.wz = float(vx), float(vy), float(wz)

    def stop(self):
        self.drive(0, 0, 0)

    def reset_odometry(self):
        self.x = self.y = self.theta = 0.0

    # --- subscribers ---
    def read_odometry(self):
        return [self.x, self.y, self.theta]

    def read_slam_pose(self):
        return self.read_odometry()

    def read_lidar(self):
        ranges, thetas = [], []
        for i in range(self.num_rays):
            th = 2 * math.pi * i / self.num_rays          # robot frame, CCW
            wth = self.theta + th
            dx, dy = math.cos(wth), math.sin(wth)
            d = min((ob.ray_hit(self.x, self.y, dx, dy) for ob in self.obstacles), default=float("inf"))
            if d > self.max_range:
                d = 0.0  # out of range -> invalid, like the real sensor
            ranges.append(d)
            thetas.append(th)
        return ranges, thetas

    # --- physics ---
    def _integrate(self, dt):
        # Robot-frame velocity -> world frame.
        c, s = math.cos(self.theta), math.sin(self.theta)
        wx = c * self.vx - s * self.vy
        wy = s * self.vx + c * self.vy
        nx, ny = self.x + wx * dt, self.y + wy * dt
        self.theta = math.atan2(math.sin(self.theta + self.wz * dt), math.cos(self.theta + self.wz * dt))
        for ob in self.obstacles:
            # Inflate the box by the robot radius for the collision test.
            if Rect(ob.xmin - self.radius, ob.ymin - self.radius,
                    ob.xmax + self.radius, ob.ymax + self.radius).contains(nx, ny):
                self.collided = True
                return  # Blocked: don't move into the obstacle.
        self.x, self.y = nx, ny
        if self.path is not None:
            self.path.append((self.x, self.y))
