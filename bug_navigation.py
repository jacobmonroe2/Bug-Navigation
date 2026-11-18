"""Project 2 (P2.3 & P2.4): Bug navigation for the MBot-Omni.

Drives to a user-supplied goal pose while avoiding obstacles using a Bug 2
style algorithm:

  * DRIVE_TO_GOAL    : head straight for the goal along the m-line.
  * FOLLOW_WALL      : an obstacle blocks the goal direction, so follow its
                       boundary (Project 1 wall follower) until the robot is
                       back on the m-line, closer to the goal than where it hit
                       the obstacle, with a clear path toward the goal.
  * TURN_TO_GOAL     : at the goal position, rotate to the goal heading.
  * AT_GOAL          : done.
  * GOAL_UNREACHABLE : followed the obstacle all the way around without
                       finding a way to the goal, so give up.

The current state is printed every iteration so staff can verify transitions.
"""
import sys
import math
import time
import numpy as np
from mbot_bridge.api import MBot

from utils import (wrap_angle, clamp, find_min_ray_in_slice, format_pose, StallBooster)

# ---------------------------------------------------------------------------
# States (tracked as integers, as in class)
# ---------------------------------------------------------------------------
STATE_DRIVE_TO_GOAL = 0
STATE_FOLLOW_WALL = 1
STATE_TURN_TO_GOAL = 2
STATE_AT_GOAL = 3
STATE_GOAL_UNREACHABLE = 4
STATE_NAMES = {
    STATE_DRIVE_TO_GOAL: "DRIVE_TO_GOAL",
    STATE_FOLLOW_WALL: "FOLLOW_WALL",
    STATE_TURN_TO_GOAL: "TURN_TO_GOAL",
    STATE_AT_GOAL: "AT_GOAL",
    STATE_GOAL_UNREACHABLE: "GOAL_UNREACHABLE",
}
TERMINAL_STATES = (STATE_AT_GOAL, STATE_GOAL_UNREACHABLE)

# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
LOOP_PERIOD = 0.05        # Control loop period (s).
TIMEOUT = 180.0           # Give up after this many seconds.

# Goal reaching.
POS_TOLERANCE = 0.08      # Position tolerance (m).
ANG_TOLERANCE = 0.10      # Heading tolerance (rad).

# Speeds. On the real robot the wheels do not turn at all for commands much
# below ~0.25 m/s, so the robot either cruises or stops: it never crawls.
# If it still does not move, the StallBooster turns the command up further.
CRUISE_SPEED = 0.35       # Normal translational speed (m/s).
MIN_LIN_SPEED = 0.25      # Never command a slower (non-zero) translation (m/s).
HARD_MAX_LIN_SPEED = 0.55 # Absolute cap, even when boosted (m/s).
KP_LIN = 1.5              # Gain on position error.
KP_ANG = 2.0              # Gain on heading error.
MAX_ANG_SPEED = 1.5       # Max rotation speed before boosting (rad/s).
MIN_ANG_SPEED = 0.6       # Never command a slower (non-zero) rotation (rad/s).
HARD_MAX_ANG_SPEED = 3.0  # Absolute cap, even when boosted (rad/s).

# Heading hold: keep facing the same way while translating, so uneven wheel
# response does not slowly spin the robot.
KP_HEADING_HOLD = 1.5
MAX_HOLD_RATE = 0.6       # rad/s
HOLD_DEADBAND = 0.06      # rad

# Lidar.
MIN_VALID_RANGE = 0.05    # Rays shorter than this are junk (m).

# Obstacle detection: minimum ray in a slice of the scan centred on the goal.
GOAL_SLICE = math.radians(70)   # Total width of the slice (rad).
OBSTACLE_DIST = 0.40            # Blocked if the min ray in the slice is shorter (m).
CLEAR_DIST = 0.60               # Need this much room toward the goal to leave the wall (m).

# Wall following (reused from the Project 1 wall follower).
WALL_SETPOINT = 0.27            # Distance to keep from the wall (m).
KP_WALL = 1.5                   # Gain on wall-distance error.
FOLLOW_SPEED = 0.30             # Tangential speed along the wall (m/s).
MAX_WALL_CORRECTION = 0.20      # Cap on the radial correction speed (m/s).
MIN_PASSABLE_GAP = 0.45         # Things closer together than this count as one obstacle (m).
CLUSTER_RANGE = 2.0             # Only rays shorter than this are used to outline the obstacle (m).
SEED_MAX_DIST = 0.5             # The obstacle must still be within this of where it was (m).
MAX_CLUSTER_POINTS = 400        # Subsample denser scans to keep the loop fast.
LOST_WALL_DIST = 1.0            # The obstacle is gone if nothing is within this range (m).

# Safety bubble: back away from anything this close, whatever the state.
SAFE_DIST = 0.17                # (m)
K_SAFE = 4.0                    # Push-away gain.
MAX_SAFE_PUSH = 0.25            # (m/s)

# Following the m-line. The robot aims at a point this far ahead of it on the
# start->goal line. Off the line (after going around something) it curves back
# onto the line and then drives along it, instead of cutting diagonally.
LOOKAHEAD = 0.10                # (m)

# Leaving the wall (Bug 2).
MLINE_TOL = 0.12                # How close to the m-line counts as "on" it (m).
LEAVE_MARGIN = 0.10             # Must be this much closer to goal than the hit point (m).
CLEAR_TIME_TO_LEAVE = 0.8       # Fallback: leave after a clear view of the goal for this long (s).
KEEP_SIDE_TIME = 1.5            # Blocked again this soon after leaving = same obstacle course, keep direction (s).

# Giving up / getting unstuck.
LOOP_ARM_DIST = 0.50            # Loop detection starts once this far from the hit point (m).
LOOP_CLOSE_DIST = 0.20          # Back this close to the hit point = went all the way around (m).
STUCK_TIME = 4.0                # Not moving for this long while following = try the other way (s).


class BugNavigator:
    """Bug 2 navigation state machine."""

    def __init__(self, robot, goal_x, goal_y, goal_theta):
        self.robot = robot
        self.goal = (goal_x, goal_y, wrap_angle(goal_theta))
        self.state = STATE_DRIVE_TO_GOAL
        self.iteration = 0
        self.now = 0.0            # Seconds since the run started.
        self.note = ""            # Extra text for this iteration's printout.

        # m-line: from the start pose to the goal. Set from the first odometry read.
        self.start = None
        self.hold_theta = None    # Heading to hold while translating.

        # Bookkeeping for FOLLOW_WALL.
        self.hit_dist = None      # Distance to goal when we hit the obstacle.
        self.hit_point = None     # Where we hit it (for loop detection).
        self.loop_armed = False   # True once we have moved away from the hit point.
        self.follow_side = +1     # +1: wall on robot's right; -1: wall on left.
        self.wall_point = None    # World-frame closest point on the obstacle we are following.
        self.clear_since = None   # Time the goal direction became clear, or None.
        self.left_mline = False   # Have we left the m-line since hitting the wall?
        self.left_wall_time = None  # When we last stopped following a wall.

        # Stall handling (see utils.StallBooster).
        self.move_booster = StallBooster(watch_position=True, watch_heading=False)
        self.turn_booster = StallBooster(watch_position=False, watch_heading=True, max_boost=4.0)
        self.was_translating = False
        self.was_turning = False

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

    def mline_heading(self, pose):
        """Direction to drive (robot frame) to travel along the start->goal line.

        Aims at the point LOOKAHEAD meters further along the m-line than the
        robot is. On the line, that is straight toward the goal. Off the line,
        it points back onto it, so after going around an obstacle the robot
        returns to its original path instead of heading diagonally to the goal.
        """
        sx, sy = self.start
        gx, gy = self.goal[0], self.goal[1]
        lx, ly = gx - sx, gy - sy
        length = math.hypot(lx, ly)
        tx, ty = gx, gy
        if length > 1e-6:
            ux, uy = lx / length, ly / length
            along = (pose[0] - sx) * ux + (pose[1] - sy) * uy + LOOKAHEAD
            if along < length:
                tx, ty = sx + ux * along, sy + uy * along
        return wrap_angle(math.atan2(ty - pose[1], tx - pose[0]) - pose[2])

    def goal_direction_min_dist(self, ranges, thetas, goal_angle_robot):
        """Min lidar distance in the slice toward the goal (inf if none)."""
        idx = find_min_ray_in_slice(ranges, thetas, goal_angle_robot, GOAL_SLICE, MIN_VALID_RANGE)
        if idx < 0:
            return float("inf"), None
        return ranges[idx], thetas[idx]

    def find_wall(self, pose, ranges, thetas):
        """Closest point on the obstacle being followed: (distance, angle) or None.

        Simply using the closest ray in the whole scan does not work in a
        cluttered room: as soon as some other object is nearer than the one
        being followed, the "wall" jumps to it, the direction of travel
        reverses, and the robot shuttles back and forth between the two.

        Instead the scan is grouped into obstacles. Two lidar points belong to
        the same obstacle if they are closer together than MIN_PASSABLE_GAP
        (the robot could not fit between them anyway), directly or through a
        chain of other points. Starting from where the wall was last
        iteration, the whole obstacle is collected and its closest point is
        the wall. Anything separated from it by a passable gap is ignored.
        """
        r = np.asarray(ranges, dtype=float)
        th = np.asarray(thetas, dtype=float)
        keep = np.nonzero((r > MIN_VALID_RANGE) & (r < CLUSTER_RANGE))[0]
        if keep.size == 0:
            return None
        if keep.size > MAX_CLUSTER_POINTS:
            keep = keep[::int(math.ceil(keep.size / MAX_CLUSTER_POINTS))]
        r, th = r[keep], th[keep]
        px, py = r * np.cos(th), r * np.sin(th)            # Robot frame.

        # Seed: the lidar point nearest to where the wall was last time.
        seed = None
        if self.wall_point is not None:
            dx, dy = self.wall_point[0] - pose[0], self.wall_point[1] - pose[1]
            c, s = math.cos(pose[2]), math.sin(pose[2])
            wx, wy = c * dx + s * dy, -s * dx + c * dy     # World -> robot frame.
            d2 = (px - wx) ** 2 + (py - wy) ** 2
            j = int(np.argmin(d2))
            if d2[j] < SEED_MAX_DIST ** 2:
                seed = j
        if seed is None:
            seed = int(np.argmin(r))                       # Lost it: take the nearest thing.

        # Grow the obstacle outward from the seed (single-linkage clustering).
        linked = ((px[:, None] - px[None, :]) ** 2 + (py[:, None] - py[None, :]) ** 2) < MIN_PASSABLE_GAP ** 2
        member = np.zeros(r.size, dtype=bool)
        member[seed] = True
        frontier = np.array([seed])
        while frontier.size:
            new = linked[frontier].any(axis=0) & ~member
            member |= new
            frontier = np.nonzero(new)[0]

        members = np.nonzero(member)[0]
        k = int(members[np.argmin(r[members])])
        c, s = math.cos(pose[2]), math.sin(pose[2])
        self.wall_point = (pose[0] + c * px[k] - s * py[k], pose[1] + s * px[k] + c * py[k])
        return float(r[k]), float(th[k])

    # -------------------------------------------------------------- states ---
    def step_drive_to_goal(self, pose, ranges, thetas):
        dist, _, _ = self.goal_errors(pose)
        goal_angle = self.mline_heading(pose)     # Travel along the m-line.
        min_dist, min_angle = self.goal_direction_min_dist(ranges, thetas, goal_angle)

        # Reached the goal position: switch to aligning the heading.
        if dist < POS_TOLERANCE:
            self.state = STATE_TURN_TO_GOAL
            self.hold_theta = self.goal[2]
            return 0.0, 0.0, 0.0

        # An obstacle blocks the goal direction (and it is closer than the
        # goal itself): record the hit point and start following the wall.
        if min_dist < OBSTACLE_DIST and min_dist < dist:
            self.hit_dist = dist
            self.hit_point = (pose[0], pose[1])
            self.loop_armed = False
            self.clear_since = None
            self.left_mline = False
            bearing = min_angle + pose[2]
            self.wall_point = (pose[0] + min_dist * math.cos(bearing), pose[1] + min_dist * math.sin(bearing))
            # Go around on the side with more room: if the closest obstacle
            # ray is to the right of the goal direction, keep the wall on the
            # right and travel left (and vice versa). If we only just left
            # another obstacle, keep going the same way round instead, so the
            # robot cannot bounce back and forth between two obstacles.
            just_left = self.left_wall_time is not None and self.now - self.left_wall_time < KEEP_SIDE_TIME
            if not just_left:
                rel = wrap_angle(min_angle - goal_angle)
                self.follow_side = +1 if rel <= 0 else -1
            self.state = STATE_FOLLOW_WALL
            return 0.0, 0.0, 0.0

        # Drive along the m-line toward the goal (robot frame). Speed is
        # proportional to the distance but never below the speed the motors
        # actually move at.
        speed = clamp(KP_LIN * dist, MIN_LIN_SPEED, CRUISE_SPEED)
        return speed * math.cos(goal_angle), speed * math.sin(goal_angle), 0.0

    def step_follow_wall(self, pose, ranges, thetas):
        dist, _, _ = self.goal_errors(pose)
        goal_angle = self.mline_heading(pose)     # Where we would drive if we left now.
        goal_min_dist, _ = self.goal_direction_min_dist(ranges, thetas, goal_angle)

        # ---- Leave conditions (Bug 2) ----
        on_mline = self.dist_to_mline(pose) < MLINE_TOL
        if not on_mline:
            self.left_mline = True
        closer = dist < self.hit_dist - LEAVE_MARGIN
        goal_clear = goal_min_dist > min(CLEAR_DIST, dist)              # Wide open toward the goal.
        if goal_clear and closer:
            if self.clear_since is None:
                self.clear_since = self.now
        else:
            self.clear_since = None

        # Back on the m-line, closer than where we hit: leave (Bug 2). If
        # another obstacle is in the way here, DRIVE_TO_GOAL will see it on
        # the next iteration and start following that one from this new,
        # closer hit point.
        if closer and on_mline and self.left_mline:
            self.state = STATE_DRIVE_TO_GOAL
            self.left_wall_time = self.now
            return 0.0, 0.0, 0.0
        if self.clear_since is not None and self.now - self.clear_since >= CLEAR_TIME_TO_LEAVE:
            self.state = STATE_DRIVE_TO_GOAL          # Fallback: clear shot at the goal.
            self.left_wall_time = self.now
            return 0.0, 0.0, 0.0

        # ---- Went all the way around without finding a way through ----
        from_hit = math.hypot(pose[0] - self.hit_point[0], pose[1] - self.hit_point[1])
        if from_hit > LOOP_ARM_DIST:
            self.loop_armed = True
        elif self.loop_armed and from_hit < LOOP_CLOSE_DIST:
            self.state = STATE_GOAL_UNREACHABLE
            return 0.0, 0.0, 0.0

        # ---- Not moving for a long time: try going around the other way ----
        if self.move_booster.stalled_for >= STUCK_TIME:
            self.follow_side = -self.follow_side
            self.move_booster.reset()
            self.hit_point = (pose[0], pose[1])
            self.loop_armed = False
            self.note = "stuck, reversing direction around the obstacle"

        wall = self.find_wall(pose, ranges, thetas)
        if wall is None or wall[0] > LOST_WALL_DIST:
            # The obstacle is gone (it moved away, or was a bad reading).
            self.state = STATE_DRIVE_TO_GOAL
            return 0.0, 0.0, 0.0
        wall_dist, wall_angle = wall

        # ---- Wall following (Project 1 controller) ----
        # Travel tangent to the wall, keeping the wall on the chosen side,
        # with proportional control on the distance to the wall.
        tangent = wall_angle + self.follow_side * math.pi / 2.0
        radial = clamp(KP_WALL * (wall_dist - WALL_SETPOINT), -MAX_WALL_CORRECTION, MAX_WALL_CORRECTION)
        vx = FOLLOW_SPEED * math.cos(tangent) + radial * math.cos(wall_angle)
        vy = FOLLOW_SPEED * math.sin(tangent) + radial * math.sin(wall_angle)
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
        if abs(wz) < MIN_ANG_SPEED:
            wz = math.copysign(MIN_ANG_SPEED, heading_err)
        return 0.0, 0.0, wz

    # ------------------------------------------------- command shaping ---
    def shape_command(self, vx, vy, wz, pose, ranges, thetas):
        """Turn the state's desired velocity into what is actually sent.

        Adds the safety bubble and heading hold, enforces the minimum speed
        the motors respond to, and applies the stall boost.
        """
        translating = math.hypot(vx, vy) > 1e-6
        turning = (not translating) and abs(wz) > 1e-6
        move_boost = self.move_booster.update(pose, translating and self.was_translating, self.now)
        turn_boost = self.turn_booster.update(pose, turning and self.was_turning, self.now)
        self.was_translating, self.was_turning = translating, turning

        if translating:
            # Safety bubble: never drive toward something that is very close.
            idx = find_min_ray_in_slice(ranges, thetas, 0.0, 2.0 * math.pi + 0.01, MIN_VALID_RANGE)
            if idx >= 0 and ranges[idx] < SAFE_DIST:
                ux, uy = math.cos(thetas[idx]), math.sin(thetas[idx])
                toward = vx * ux + vy * uy
                if toward > 0:                       # Remove the part heading into it...
                    vx, vy = vx - toward * ux, vy - toward * uy
                push = min(MAX_SAFE_PUSH, K_SAFE * (SAFE_DIST - ranges[idx]))
                vx, vy = vx - push * ux, vy - push * uy   # ...and back away.

            speed = math.hypot(vx, vy)
            if speed > 1e-6:
                target = clamp(speed, MIN_LIN_SPEED, CRUISE_SPEED) * move_boost
                target = min(target, HARD_MAX_LIN_SPEED)
                vx, vy = vx / speed * target, vy / speed * target

            # Heading hold.
            err = wrap_angle(self.hold_theta - pose[2])
            wz = 0.0 if abs(err) < HOLD_DEADBAND else clamp(KP_HEADING_HOLD * err, -MAX_HOLD_RATE, MAX_HOLD_RATE)
            if move_boost > 1.0:
                self.note = (self.note + "; " if self.note else "") + f"stall boost x{move_boost:.2f}"
        elif turning:
            wz = clamp(wz * turn_boost, -HARD_MAX_ANG_SPEED, HARD_MAX_ANG_SPEED)
            if turn_boost > 1.0:
                self.note = (self.note + "; " if self.note else "") + f"stall boost x{turn_boost:.2f}"
        return vx, vy, wz

    # ---------------------------------------------------------------- run ---
    def run(self):
        start_time = time.time()
        pose = []
        while self.state not in TERMINAL_STATES:
            self.iteration += 1
            self.now = time.time() - start_time
            self.note = ""
            pose = self.robot.read_odometry()
            ranges, thetas = self.robot.read_lidar()
            if len(pose) < 3 or len(ranges) == 0:
                print(f"[{self.iteration:5d}] waiting for odometry/lidar...")
                self.robot.drive(0, 0, 0)
                time.sleep(LOOP_PERIOD)
                continue
            if self.start is None:
                self.start = (pose[0], pose[1])
                self.hold_theta = pose[2]

            if self.state == STATE_DRIVE_TO_GOAL:
                vx, vy, wz = self.step_drive_to_goal(pose, ranges, thetas)
            elif self.state == STATE_FOLLOW_WALL:
                vx, vy, wz = self.step_follow_wall(pose, ranges, thetas)
            elif self.state == STATE_TURN_TO_GOAL:
                vx, vy, wz = self.step_turn_to_goal(pose, ranges, thetas)
            else:
                vx, vy, wz = 0.0, 0.0, 0.0
            vx, vy, wz = self.shape_command(vx, vy, wz, pose, ranges, thetas)

            # (P2.3) Report the state every iteration.
            dist, goal_angle, heading_err = self.goal_errors(pose)
            goal_min_dist, _ = self.goal_direction_min_dist(ranges, thetas, goal_angle)
            print(f"[{self.iteration:5d} t={self.now:6.1f}s] STATE {self.state} {STATE_NAMES[self.state]:<16} | "
                  f"{format_pose(pose)} | dist to goal {dist:.3f} m | "
                  f"min ray toward goal {goal_min_dist:.2f} m | "
                  f"cmd vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}"
                  + (f" | {self.note}" if self.note else ""))

            self.robot.drive(vx, vy, wz)

            if self.now > TIMEOUT:
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
    print("Enter the goal pose. The robot starts at x=0, y=0 facing along +x.")
    print("x is meters FORWARD, y is meters to the LEFT, theta is the final heading in radians.")
    print("Example for a small room: x = 1.5, y = 0, theta = 0")
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
    if math.hypot(goal_x, goal_y) > 10.0:
        print(f"NOTE: that goal is {math.hypot(goal_x, goal_y):.0f} meters away. Goals are in meters from where the")
        print("robot starts. If the goal is outside the room the robot can never reach it.\n")

    try:
        nav = BugNavigator(robot, goal_x, goal_y, goal_theta)
        nav.run()
        if nav.state == STATE_AT_GOAL:
            print(f"\n[{nav.iteration:5d}] STATE {nav.state} {STATE_NAMES[nav.state]}: goal reached.")
        elif nav.state == STATE_GOAL_UNREACHABLE:
            print(f"\n[{nav.iteration:5d}] STATE {nav.state} {STATE_NAMES[nav.state]}: went all the way around the "
                  "obstacle without finding a path. The goal cannot be reached.")
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Stop the robot before exiting.
        for _ in range(3):
            robot.stop()
            time.sleep(0.05)
        # Print out the robot's final odometry before exiting.
        print("\nFinal odometry:", format_pose(robot.read_odometry()))
