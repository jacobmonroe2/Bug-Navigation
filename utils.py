"""Shared helpers for Project 2: Bug Navigation.

These mirror the helper functions provided in the C++ template
(wrapAngle, sleepFor, findMinDistInSlice) plus a few small utilities reused
from the Project 1 wall follower.
"""
import math
import time


def sleep_for(secs):
    """Sleep for the given number of seconds."""
    time.sleep(secs)


def wrap_angle(angle):
    """Wrap an angle (radians) into the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, low, high):
    """Clamp value into [low, high]."""
    return max(low, min(high, value))


def apply_min_speed(v, min_speed):
    """Bump very small (but non-zero) commands up to min_speed, keeping sign.

    The MBot motors stall on tiny velocity commands, so if we want to move at
    all we have to ask for at least min_speed.
    """
    if 0 < abs(v) < min_speed:
        return min_speed if v > 0 else -min_speed
    return v


def find_min_ray_in_slice(ranges, thetas, target_angle, slice_size, min_range=0.0):
    """Find the shortest valid ray inside a slice of the Lidar scan.

    The slice is centred on target_angle (robot frame, radians) and has a
    total angular width of slice_size radians, i.e. it contains every ray
    whose angle is within +/- slice_size / 2 of target_angle. Rays with a
    range <= 0 (or <= min_range) are invalid and are ignored.

    Args:
        ranges (list): Length of each Lidar ray (meters).
        thetas (list): Angle of each Lidar ray (radians, robot frame).
        target_angle (float): Centre of the slice (radians, robot frame).
        slice_size (float): Total angular width of the slice (radians).
        min_range (float): Rays this short or shorter are treated as invalid.

    Returns:
        int: Index of the shortest valid ray in the slice, or -1 if the
             slice contains no valid rays.
    """
    half = slice_size / 2.0
    min_idx = -1
    min_dist = float("inf")
    for i, (dist, theta) in enumerate(zip(ranges, thetas)):
        if dist <= min_range:
            continue  # Invalid reading.
        if abs(wrap_angle(theta - target_angle)) > half:
            continue  # Outside the slice.
        if dist < min_dist:
            min_dist = dist
            min_idx = i
    return min_idx


def find_min_dist(ranges, thetas):
    """Length and angle of the shortest valid ray in the whole scan.

    Reused from the Project 1 wall follower.

    Returns:
        tuple: (distance, angle), or (None, None) if the scan had no valid rays.
    """
    min_dist, min_angle = None, None
    for dist, angle in zip(ranges, thetas):
        if dist <= 0:
            continue
        if min_dist is None or dist < min_dist:
            min_dist = dist
            min_angle = angle
    return min_dist, min_angle


def world_to_robot(dx, dy, theta):
    """Rotate a world-frame vector into the robot frame.

    The robot is at heading theta (radians) in the world frame. The returned
    vector is expressed with x forward and y to the robot's left.
    """
    c, s = math.cos(theta), math.sin(theta)
    rx = c * dx + s * dy
    ry = -s * dx + c * dy
    return rx, ry


def format_pose(pose):
    """Pretty-print an [x, y, theta] pose."""
    if not pose or len(pose) < 3:
        return "[no odometry]"
    return f"x={pose[0]:+.3f} m  y={pose[1]:+.3f} m  theta={pose[2]:+.3f} rad ({math.degrees(pose[2]):+.1f} deg)"


class StallBooster:
    """Turns the command up when the robot is told to move but is not moving.

    The MBot's motors do not turn at all below some minimum command, and that
    minimum differs from robot to robot, with battery level, and with the
    floor. Rather than guess it, watch the odometry: if we have been
    commanding motion for check_time seconds and the pose has barely changed,
    multiply the command by `step`, and keep doing so (up to max_boost) until
    the robot moves. The boost is kept for the rest of the run, since the
    stall speed is a property of the robot, not of the moment.

    Usage, once per control iteration:
        boost = booster.update(pose, was_commanding_motion, now)
        robot.drive(vx * boost, vy * boost, wz)
    """

    def __init__(self, check_time=0.4, min_speed=0.04, min_rate=0.15, step=1.25, max_boost=2.5,
                 watch_position=True, watch_heading=False):
        self.check_time = check_time      # How long with no progress counts as a stall (s).
        self.min_speed = min_speed        # Slower than this is "not moving" (m/s).
        self.min_rate = min_rate          # Slower than this is "not turning" (rad/s).
        self.step = step                  # Multiply the boost by this on every stall.
        self.max_boost = max_boost        # Never boost beyond this factor.
        self.watch_position = watch_position
        self.watch_heading = watch_heading
        self.boost = 1.0
        self.stalled_for = 0.0            # Seconds of continuous stall so far.
        self._ref = None                  # Pose at the last sign of progress.
        self._ref_time = None

    def reset(self):
        """Forget the reference pose (call when the robot is told to stop)."""
        self._ref = None
        self._ref_time = None
        self.stalled_for = 0.0

    def update(self, pose, commanding_motion, now):
        """Return the boost factor to apply to this iteration's command."""
        if not commanding_motion:
            self.reset()
            return self.boost
        if self._ref is None:
            self._ref, self._ref_time = (pose[0], pose[1], pose[2]), now
            return self.boost

        elapsed = now - self._ref_time
        if elapsed < self.check_time:
            return self.boost

        moved = math.hypot(pose[0] - self._ref[0], pose[1] - self._ref[1])
        turned = abs(wrap_angle(pose[2] - self._ref[2]))
        progressed = ((self.watch_position and moved > self.min_speed * elapsed) or
                      (self.watch_heading and turned > self.min_rate * elapsed))
        if progressed:
            self.stalled_for = 0.0
        else:
            self.stalled_for += elapsed
            self.boost = min(self.max_boost, self.boost * self.step)
        self._ref, self._ref_time = (pose[0], pose[1], pose[2]), now
        return self.boost
