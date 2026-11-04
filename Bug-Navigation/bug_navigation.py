"""Project 2 (P2.3 & P2.4): Bug navigation for the MBot-Omni.

Drives to a user-supplied goal pose while avoiding obstacles using a Bug 2
style algorithm:

  * DRIVE_TO_GOAL : head straight for the goal along the m-line.
  * FOLLOW_WALL   : an obstacle blocks the goal direction, so follow its
                    boundary (Project 1 wall follower) until the robot is
                    back on the m-line, closer to the goal than where it hit
                    the obstacle, with a clear path toward the goal.
  * TURN_TO_GOAL  : at the goal position, rotate to the goal heading.
  * AT_GOAL       : done.

The current state is printed every iteration so staff can verify transitions.
"""
import sys
import math
import time
import numpy as np
from mbot_bridge.api import MBot

from utils import (wrap_angle, clamp, apply_min_speed, find_min_ray_in_slice,
                   find_min_dist, world_to_robot, format_pose)

# ---------------------------------------------------------------------------
# States (tracked as integers, as in class)
# ---------------------------------------------------------------------------
STATE_DRIVE_TO_GOAL = 0
STATE_FOLLOW_WALL = 1
STATE_TURN_TO_GOAL = 2
STATE_AT_GOAL = 3
STATE_NAMES = {
    STATE_DRIVE_TO_GOAL: "DRIVE_TO_GOAL",
    STATE_FOLLOW_WALL: "FOLLOW_WALL",
    STATE_TURN_TO_GOAL: "TURN_TO_GOAL",
    STATE_AT_GOAL: "AT_GOAL",
}

# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
LOOP_PERIOD = 0.05        # Control loop period (s).
TIMEOUT = 180.0           # Give up after this many seconds.

# Goal reaching.
POS_TOLERANCE = 0.06      # Position tolerance (m).
ANG_TOLERANCE = 0.08      # Heading tolerance (rad).
KP_LIN = 1.0              # Gain on position error.
KP_ANG = 1.5              # Gain on heading error.
MAX_LIN_SPEED = 0.35      # Max translational speed (m/s).
MIN_LIN_SPEED = 0.08      # Motors stall below this (m/s).
MAX_ANG_SPEED = 1.5       # Max rotation speed (rad/s).
MIN_ANG_SPEED = 0.25      # Motors stall below this (rad/s).

# Obstacle detection: minimum ray in a slice of the scan centred on the goal.
GOAL_SLICE = math.radians(70)   # Total width of the slice (rad).
OBSTACLE_DIST = 0.45            # Blocked if the min ray in the slice is shorter (m).
CLEAR_DIST = 0.70               # Need this much room toward the goal to leave the wall (m).

# Wall following (reused from the Project 1 wall follower).
WALL_SETPOINT = 0.35            # Distance to keep from the wall (m).
KP_WALL = 1.2                   # Gain on wall-distance error.
FOLLOW_SPEED = 0.25             # Tangential speed along the wall (m/s).
MAX_WALL_CORRECTION = 0.20      # Cap on the radial correction speed (m/s).
FORWARD_SLICE = math.radians(60)  # Slice checked in the direction of travel (rad).
SLOW_DOWN_DIST = 0.6            # Slow the tangential speed when a wall is ahead (m).

# Leaving the wall (Bug 2).
MLINE_TOL = 0.12                # How close to the m-line counts as "on" it (m).
LEAVE_MARGIN = 0.10             # Must be this much closer to goal than the hit point (m).
CLEAR_ITERS_TO_LEAVE = 8        # Fallback: leave after this many consecutive clear iterations.


class BugNavigator:
    """Bug 2 navigation state machine."""

    def __init__(self, robot, goal_x, goal_y, goal_theta):
        self.robot = robot
        self.goal = (goal_x, goal_y, wrap_angle(goal_theta))
        self.state = STATE_DRIVE_TO_GOAL
        self.iteration = 0

        # m-line: from the start pose to the goal. Set from the first odometry read.
        self.start = None

        # Bookkeeping for FOLLOW_WALL.
        self.hit_dist = None          # Distance to goal when we hit the obstacle.
        self.follow_side = +1         # +1: wall on robot's right; -1: wall on left.
        self.clear_iters = 0          # Consecutive iterations with a clear goal direction.
        self.left_mline = False       # Have we left the m-line since hitting the wall?

    # ----------------------------------------------------------- geometry ---
    def goal_errors(self, pose):
        """Distance to goal, goal direction in robot frame, heading error."""
        x, y, theta = pose
        dx, dy = self.goal[0] - x, self.goal[1] - y
        dist = math.hypot(dx, dy)
        goal_angle_robot = wrap_angle(math.atan2(dy, dx) - theta)
        heading_err = wrap_angle(self.goal[2] - theta)
        return dist, goal_angle_robot, heading_err

    def dist_to_mline(self, pose):
        """Perpendicular distance from the robot to the start->goal line."""
        sx, sy = self.start
        gx, gy = self.goal[0], self.goal[1]
        lx, ly = gx - sx, gy - sy
        length = math.hypot(lx, ly)
        if length < 1e-6:
            return 0.0
        # |cross(line, point - start)| / |line|
        return abs(lx * (pose[1] - sy) - ly * (pose[0] - sx)) / length

    def goal_direction_min_dist(self, ranges, thetas, goal_angle_robot):
        """Min lidar distance in the slice toward the goal (inf if none)."""
        idx = find_min_ray_in_slice(ranges, thetas, goal_angle_robot, GOAL_SLICE)
        if idx < 0:
            return float("inf"), None
        return ranges[idx], thetas[idx]

    # -------------------------------------------------------------- states ---
    def step_drive_to_goal(self, pose, ranges, thetas):
        dist, goal_angle, _ = self.goal_errors(pose)
        min_dist, min_angle = self.goal_direction_min_dist(ranges, thetas, goal_angle)

        # Reached the goal position: switch to aligning the heading.
        if dist < POS_TOLERANCE:
            self.state = STATE_TURN_TO_GOAL
            return 0.0, 0.0, 0.0

        # An obstacle blocks the goal direction (and it is closer than the
        # goal itself): record the hit point and start following the wall.
        if min_dist < OBSTACLE_DIST and min_dist < dist:
            self.hit_dist = dist
            self.clear_iters = 0
            self.left_mline = False
            # Go around on the side with more room: if the closest obstacle
            # ray is to the right of the goal direction, keep the wall on the
            # right and travel left (and vice versa).
            rel = wrap_angle(min_angle - goal_angle)
            self.follow_side = +1 if rel <= 0 else -1
            self.state = STATE_FOLLOW_WALL
            return 0.0, 0.0, 0.0

        # Proportional control straight toward the goal (robot frame).
        speed = clamp(KP_LIN * dist, MIN_LIN_SPEED, MAX_LIN_SPEED)
        # Slow down as we approach something in front of us.
        if min_dist < SLOW_DOWN_DIST:
            speed = min(speed, max(MIN_LIN_SPEED, FOLLOW_SPEED * min_dist / SLOW_DOWN_DIST))
        vx = speed * math.cos(goal_angle)
        vy = speed * math.sin(goal_angle)
        return vx, vy, 0.0

    def step_follow_wall(self, pose, ranges, thetas):
        dist, goal_angle, _ = self.goal_errors(pose)
        wall_dist, wall_angle = find_min_dist(ranges, thetas)
        goal_min_dist, _ = self.goal_direction_min_dist(ranges, thetas, goal_angle)

        # ---- Leave conditions (Bug 2) ----
        on_mline = self.dist_to_mline(pose) < MLINE_TOL
        if not on_mline:
            self.left_mline = True
        closer = dist < self.hit_dist - LEAVE_MARGIN
        goal_clear = goal_min_dist > min(CLEAR_DIST, dist)
        self.clear_iters = self.clear_iters + 1 if (goal_clear and closer) else 0

        if closer and goal_clear and (on_mline and self.left_mline):
            self.state = STATE_DRIVE_TO_GOAL          # Back on the m-line.
            return 0.0, 0.0, 0.0
        if self.clear_iters >= CLEAR_ITERS_TO_LEAVE:
            self.state = STATE_DRIVE_TO_GOAL          # Fallback: clear shot at the goal.
            return 0.0, 0.0, 0.0

        if wall_dist is None:
            # Lost the wall entirely; drift toward the goal to find it again.
            self.state = STATE_DRIVE_TO_GOAL
            return 0.0, 0.0, 0.0

        # ---- Wall following (Project 1 controller) ----
        # Direction of travel is tangent to the wall, keeping the wall on the
        # chosen side. Radial P-control holds the setpoint distance.
        tangent = wall_angle + self.follow_side * math.pi / 2.0
        error = wall_dist - WALL_SETPOINT
        radial = clamp(KP_WALL * error, -MAX_WALL_CORRECTION, MAX_WALL_CORRECTION)

        # If something is ahead along the tangent, slow down so the nearest
        # ray smoothly becomes the new wall at inside corners.
        idx = find_min_ray_in_slice(ranges, thetas, tangent, FORWARD_SLICE)
        ahead = ranges[idx] if idx >= 0 else float("inf")
        speed = FOLLOW_SPEED
        if ahead < SLOW_DOWN_DIST:
            speed = max(MIN_LIN_SPEED, FOLLOW_SPEED * ahead / SLOW_DOWN_DIST)

        vx = speed * math.cos(tangent) + radial * math.cos(wall_angle)
        vy = speed * math.sin(tangent) + radial * math.sin(wall_angle)
        return vx, vy, 0.0

    def step_turn_to_goal(self, pose, ranges, thetas):
        dist, _, heading_err = self.goal_errors(pose)
        if abs(heading_err) < ANG_TOLERANCE:
            self.state = STATE_AT_GOAL
            return 0.0, 0.0, 0.0
        if dist > 2 * POS_TOLERANCE:
            # Drifted off the goal while turning; go back and fix position.
            self.state = STATE_DRIVE_TO_GOAL
            return 0.0, 0.0, 0.0
        wz = clamp(KP_ANG * heading_err, -MAX_ANG_SPEED, MAX_ANG_SPEED)
        wz = apply_min_speed(wz, MIN_ANG_SPEED)
        return 0.0, 0.0, wz

    # ---------------------------------------------------------------- run ---
    def run(self):
        start_time = time.time()
        pose = []
        while self.state != STATE_AT_GOAL:
            self.iteration += 1
            pose = self.robot.read_odometry()
            ranges, thetas = self.robot.read_lidar()
            if len(pose) < 3 or len(ranges) == 0:
                print(f"[{self.iteration:5d}] waiting for odometry/lidar...")
                self.robot.drive(0, 0, 0)
                time.sleep(LOOP_PERIOD)
                continue
            if self.start is None:
                self.start = (pose[0], pose[1])

            if self.state == STATE_DRIVE_TO_GOAL:
                vx, vy, wz = self.step_drive_to_goal(pose, ranges, thetas)
            elif self.state == STATE_FOLLOW_WALL:
                vx, vy, wz = self.step_follow_wall(pose, ranges, thetas)
            elif self.state == STATE_TURN_TO_GOAL:
                vx, vy, wz = self.step_turn_to_goal(pose, ranges, thetas)
            else:
                vx, vy, wz = 0.0, 0.0, 0.0

            # (P2.3) Report the state every iteration.
            dist, goal_angle, heading_err = self.goal_errors(pose)
            goal_min_dist, _ = self.goal_direction_min_dist(ranges, thetas, goal_angle)
            print(f"[{self.iteration:5d}] STATE {self.state} {STATE_NAMES[self.state]:<13} | "
                  f"{format_pose(pose)} | dist to goal {dist:.3f} m | "
                  f"min ray toward goal {goal_min_dist:.2f} m | "
                  f"cmd vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}")

            self.robot.drive(vx, vy, wz)

            if time.time() - start_time > TIMEOUT:
                print("WARNING: timed out before reaching the goal.")
                break
            time.sleep(LOOP_PERIOD)

        self.robot.stop()
        return pose


def ask_goal():
    """(P2.4) Ask the user for a goal pose, or take it from the command line."""
    args = sys.argv[1:]
    if len(args) == 3:
        return float(args[0]), float(args[1]), float(args[2])
    print("Enter the goal pose in the odometry frame (robot starts at 0, 0, 0).")
    goal_x = float(input("  goal x (m): "))
    goal_y = float(input("  goal y (m): "))
    goal_theta = float(input("  goal theta (rad): "))
    return goal_x, goal_y, goal_theta


if __name__ == "__main__":
    robot = MBot()

    # Reset the robot odometry to zero at the beginning of the run.
    robot.reset_odometry()
    time.sleep(0.5)

    goal_x, goal_y, goal_theta = ask_goal()
    print(f"\nNavigating to goal x={goal_x:+.3f}  y={goal_y:+.3f}  theta={goal_theta:+.3f}\n")

    try:
        nav = BugNavigator(robot, goal_x, goal_y, goal_theta)
        nav.run()
        print(f"\n[{nav.iteration:5d}] STATE {nav.state} {STATE_NAMES[nav.state]}: goal reached.")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Stop the robot before exiting.
        for _ in range(3):
            robot.stop()
            time.sleep(0.05)
        # Print out the robot's final odometry before exiting.
        print("\nFinal odometry:", format_pose(robot.read_odometry()))
