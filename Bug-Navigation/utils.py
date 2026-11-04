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


def find_min_ray_in_slice(ranges, thetas, target_angle, slice_size):
    """Find the shortest valid ray inside a slice of the Lidar scan.

    The slice is centred on target_angle (robot frame, radians) and has a
    total angular width of slice_size radians, i.e. it contains every ray
    whose angle is within +/- slice_size / 2 of target_angle. Rays with a
    range <= 0 are invalid and are ignored.

    Args:
        ranges (list): Length of each Lidar ray (meters).
        thetas (list): Angle of each Lidar ray (radians, robot frame).
        target_angle (float): Centre of the slice (radians, robot frame).
        slice_size (float): Total angular width of the slice (radians).

    Returns:
        int: Index of the shortest valid ray in the slice, or -1 if the
             slice contains no valid rays.
    """
    half = slice_size / 2.0
    min_idx = -1
    min_dist = float("inf")
    for i, (dist, theta) in enumerate(zip(ranges, thetas)):
        if dist <= 0:
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
