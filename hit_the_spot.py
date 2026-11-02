import time
import math
import numpy as np
from mbot_bridge.api import MBot

from utils import wrap_angle, clamp, apply_min_speed, world_to_robot, format_pose

# ---------------------------------------------------------------------------
# Controller parameters
# ---------------------------------------------------------------------------
KP_LIN = 1.0           # Proportional gain on position error (1/s).
KP_ANG = 1.5           # Proportional gain on heading error (1/s).
MAX_LIN_SPEED = 0.4    # Max translational speed (m/s).
MAX_ANG_SPEED = 1.5    # Max rotational speed (rad/s).
MIN_LIN_SPEED = 0.08   # Smallest command the motors actually move at (m/s).
MIN_ANG_SPEED = 0.25   # Smallest rotation command the motors move at (rad/s).
POS_TOLERANCE = 0.05   # "Reached" when within this distance of the goal (m).
ANG_TOLERANCE = 0.08   # "Reached" when within this heading error (rad), ~4.5 deg.
LOOP_PERIOD = 0.05     # Control loop period (s).
TIMEOUT = 60.0         # Give up on a goal after this many seconds.


def compute_velocity(pose, goal_x, goal_y, goal_theta):
    """Compute robot-frame velocity (vx, vy, wz) that moves pose toward the goal.

    Returns:
        tuple: (vx, vy, wz, dist_error, angle_error)
    """
    x, y, theta = pose

    # Position error in the world (odometry) frame.
    dx = goal_x - x
    dy = goal_y - y
    dist = math.hypot(dx, dy)

    # Heading error, wrapped to [-pi, pi] so we always turn the short way.
    angle_error = wrap_angle(goal_theta - theta)

    # The MBot is omnidirectional, so we can translate straight toward the
    # goal while rotating at the same time. drive() expects robot-frame
    # velocities, so rotate the world-frame error into the robot frame.
    ex, ey = world_to_robot(dx, dy, theta)

    # Proportional control on position: command a speed proportional to the
    # distance, pointing along the error vector, with a speed cap.
    speed = clamp(KP_LIN * dist, 0.0, MAX_LIN_SPEED)
    if dist < POS_TOLERANCE:
        speed = 0.0
    else:
        speed = max(speed, MIN_LIN_SPEED)
    if dist > 1e-6:
        vx = speed * ex / dist
        vy = speed * ey / dist
    else:
        vx, vy = 0.0, 0.0

    # Proportional control on heading.
    wz = clamp(KP_ANG * angle_error, -MAX_ANG_SPEED, MAX_ANG_SPEED)
    if abs(angle_error) < ANG_TOLERANCE:
        wz = 0.0
    else:
        wz = apply_min_speed(wz, MIN_ANG_SPEED)

    return vx, vy, wz, dist, angle_error


def drive_to_pose(goal_x, goal_y, goal_theta, robot):
    """(P2.1) Drive the robot to the goal pose [goal_x, goal_y, goal_theta].

    Compares the robot's odometry pose with the goal pose every iteration and
    commands a velocity proportional to the difference. Stops once both the
    position and heading errors are inside their tolerances.

    HINT: Use robot.read_odometry() to get a list of length 3 containing the
    robot's odometry [x, y, theta]. The odometry frame is the pose the robot
    is in at the start of this script.

    Returns:
        list: The robot's odometry pose when the goal was reached.
    """
    goal_theta = wrap_angle(goal_theta)
    print(f"\n==> Driving to goal: x={goal_x:+.3f}  y={goal_y:+.3f}  theta={goal_theta:+.3f}")

    start_time = time.time()
    pose = robot.read_odometry()
    while True:
        pose = robot.read_odometry()
        if len(pose) < 3:
            # Odometry not available this iteration; hold still and retry.
            robot.drive(0, 0, 0)
            time.sleep(LOOP_PERIOD)
            continue

        vx, vy, wz, dist, angle_error = compute_velocity(pose, goal_x, goal_y, goal_theta)

        # Goal reached when both position and heading are within tolerance.
        if dist < POS_TOLERANCE and abs(angle_error) < ANG_TOLERANCE:
            robot.stop()
            break

        if time.time() - start_time > TIMEOUT:
            print("    WARNING: timed out before reaching the goal.")
            robot.stop()
            break

        robot.drive(vx, vy, wz)
        print(f"    {format_pose(pose)} | dist err {dist:.3f} m | ang err {angle_error:+.3f} rad "
              f"| cmd vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}", end="\r")
        time.sleep(LOOP_PERIOD)

    print()  # End the progress line.
    print(f"<== Reached goal. Pose: {format_pose(pose)}")
    return pose


if __name__ == "__main__":
    robot = MBot()

    # Goals the robot should visit in order, as (x, y, theta) in the odometry
    # frame (meters, meters, radians). Course staff can edit this list or pass
    # goals on the command line as x y theta triples, e.g.
    #   python3 hit_the_spot.py 1 0 0  1 1 1.57  0 0 0
    import sys
    args = sys.argv[1:]
    if len(args) >= 3 and len(args) % 3 == 0:
        goals = [(float(args[i]), float(args[i + 1]), float(args[i + 2])) for i in range(0, len(args), 3)]
    else:
        goals = [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, math.pi / 2),
            (0.0, 1.0, math.pi),
            (0.0, 0.0, 0.0),
        ]
    PAUSE_AT_GOAL = 2.0  # Seconds to hold at each goal so staff can read the pose.

    # Reset the robot odometry to zero at the beginning of the run.
    robot.reset_odometry()
    time.sleep(0.5)  # Give the odometry reset a moment to take effect.

    try:
        for i, (gx, gy, gth) in enumerate(goals):
            print(f"\n######## Goal {i + 1} of {len(goals)} ########")
            drive_to_pose(gx, gy, gth, robot)
            time.sleep(PAUSE_AT_GOAL)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Stop the robot before exiting. Send it a few times in case a message
        # is dropped.
        for _ in range(3):
            robot.stop()
            time.sleep(0.05)

        # Print out the robot's final odometry before exiting.
        print("\nFinal odometry:", format_pose(robot.read_odometry()))
