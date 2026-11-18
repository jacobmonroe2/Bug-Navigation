"""Run hit_the_spot / bug_navigation against the simulator.

Usage (from the repo root):
    python3 sim/run_sim.py all                        # every scenario, ideal + realistic robot
    python3 sim/run_sim.py hit [--real]               # drive through the default goals
    python3 sim/run_sim.py bug <world> x y th [--real]  # one bug navigation run, full printout

Worlds: empty, box, wall, two_boxes, corridor, u_trap, small_room, narrow_gap, blocked_door, cluttered

--real uses a robot modelled on the real MBot-Omni (wheels that stall at low
speed, noisy slow lidar, bridge latency). Without it the robot is ideal.
"""
import os
import sys
import math
import types
import io
import contextlib
import importlib

HERE = os.path.dirname(os.path.abspath(__file__))
# The code under test. Override with BUGNAV_CODE_DIR to test another copy.
ROOT = os.environ.get("BUGNAV_CODE_DIR", os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)

import fake_mbot  # noqa: E402
from fake_mbot import FakeMBot, Rect  # noqa: E402

# Install a fake mbot_bridge package before the robot code imports it.
pkg = types.ModuleType("mbot_bridge")
api = types.ModuleType("mbot_bridge.api")
pkg.api = api
sys.modules["mbot_bridge"] = pkg
sys.modules["mbot_bridge.api"] = api
fake_mbot.install_fake_time()


def room(xmin, ymin, xmax, ymax, t=0.1):
    """Four walls enclosing a rectangular room."""
    return [Rect(xmin - t, ymin - t, xmax + t, ymin), Rect(xmin - t, ymax, xmax + t, ymax + t),
            Rect(xmin - t, ymin, xmin, ymax), Rect(xmax, ymin, xmax + t, ymax)]


WORLDS = {
    "empty": [],
    # A box directly between the start and a goal at (2.5, 0).
    "box": [Rect(1.0, -0.4, 1.5, 0.4)],
    # A long wall the robot must go around the end of.
    "wall": [Rect(1.2, -1.5, 1.3, 0.6)],
    # Two staggered boxes.
    "two_boxes": [Rect(0.8, -0.6, 1.2, 0.3), Rect(1.8, -0.2, 2.2, 0.9)],
    # A corridor with an opening the robot must find.
    "corridor": [Rect(1.0, -2.0, 1.1, -0.3), Rect(1.0, 0.3, 1.1, 2.0), Rect(2.0, -0.5, 2.1, 2.0)],
    # A U-shaped trap opening away from the goal (classic Bug test).
    "u_trap": [Rect(0.9, -0.8, 1.0, 0.8), Rect(1.0, 0.7, 1.8, 0.8), Rect(1.0, -0.8, 1.8, -0.7)],
    # A 2.4 m x 2.4 m room with a box between the start and the goal.
    "small_room": room(-0.6, -1.2, 1.8, 1.2) + [Rect(0.5, -0.25, 0.75, 0.25)],
    # A small room split by a wall with a 0.55 m doorway off to one side.
    "narrow_gap": room(-0.6, -1.2, 1.9, 1.2) + [Rect(0.8, -1.2, 0.9, 0.25), Rect(0.8, 0.8, 0.9, 1.2)],
    # The same room, but the doorway is only 0.30 m wide: too narrow to fit through.
    "blocked_door": room(-0.6, -1.2, 1.9, 1.2) + [Rect(0.8, -1.2, 0.9, 0.25), Rect(0.8, 0.55, 0.9, 1.2)],
    # A small room with several things lying around.
    "cluttered": room(-0.6, -1.3, 2.2, 1.3) + [Rect(0.45, -0.15, 0.65, 0.35), Rect(1.1, -0.9, 1.3, -0.25),
                                               Rect(1.15, 0.3, 1.4, 0.55), Rect(0.2, 0.85, 0.5, 1.3)],
}

# (world, goal x, goal y, goal theta, expected final state name)
SCENARIOS = [
    ("empty", 2.0, 1.0, 1.0, "AT_GOAL"),
    ("box", 2.5, 0.0, 0.0, "AT_GOAL"),
    ("wall", 2.5, 0.0, 1.57, "AT_GOAL"),
    ("two_boxes", 3.0, 0.3, 0.0, "AT_GOAL"),
    ("corridor", 3.0, 0.0, 0.0, "AT_GOAL"),
    ("u_trap", 2.5, 0.0, 0.0, "AT_GOAL"),
    ("small_room", 1.3, 0.0, 0.0, "AT_GOAL"),
    ("narrow_gap", 1.4, 0.0, 1.57, "AT_GOAL"),
    ("cluttered", 1.8, -0.2, 0.0, "AT_GOAL"),
    # A goal on the far side of the room wall can never be reached.
    ("small_room", 5.0, 0.0, 0.0, "GOAL_UNREACHABLE"),
    ("blocked_door", 1.4, 0.0, 0.0, "GOAL_UNREACHABLE"),
]


def run_bug(world, gx, gy, gth, realistic=False, verbose=True):
    fake_mbot.reset_clock()
    robot = FakeMBot(WORLDS[world], realistic=realistic)
    api.MBot = lambda: robot
    import bug_navigation
    importlib.reload(bug_navigation)
    nav = bug_navigation.BugNavigator(robot, gx, gy, gth)
    out = io.StringIO()
    with contextlib.redirect_stdout(sys.stdout if verbose else out):
        pose = nav.run()
    dist = math.hypot(gx - pose[0], gy - pose[1])
    ang = abs(math.atan2(math.sin(gth - pose[2]), math.cos(gth - pose[2])))
    state_name = bug_navigation.STATE_NAMES[nav.state]
    return dict(nav=nav, robot=robot, dist=dist, ang=ang, state=state_name, time=fake_mbot._sim_clock[0])


def run_hit(realistic=False, verbose=True):
    fake_mbot.reset_clock()
    robot = FakeMBot([], realistic=realistic)
    api.MBot = lambda: robot
    import hit_the_spot
    importlib.reload(hit_the_spot)
    goals = [(1.0, 0.0, 0.0), (1.0, 1.0, math.pi / 2), (0.0, 1.0, math.pi), (0.0, 0.0, 0.0)]
    results = []
    for g in goals:
        t0 = fake_mbot._sim_clock[0]
        with contextlib.redirect_stdout(sys.stdout if verbose else io.StringIO()):
            pose = hit_the_spot.drive_to_pose(*g, robot)
        d = math.hypot(g[0] - pose[0], g[1] - pose[1])
        a = abs(math.atan2(math.sin(g[2] - pose[2]), math.cos(g[2] - pose[2])))
        results.append((g, pose, d, a, fake_mbot._sim_clock[0] - t0))
    return results


def plot(robot, world, gx, gy, path):
    """Save an ASCII map of the run (no matplotlib needed)."""
    W, H, scale = 70, 32, 0.1
    xmin, ymin = -0.9, -1.6
    grid = [[" "] * W for _ in range(H)]
    for ob in WORLDS[world]:
        for r in range(H):
            for c in range(W):
                x, y = xmin + c * scale, ymin + (H - 1 - r) * scale
                if ob.contains(x, y):
                    grid[r][c] = "#"
    for (x, y) in robot.path:
        c, r = int(round((x - xmin) / scale)), H - 1 - int(round((y - ymin) / scale))
        if 0 <= r < H and 0 <= c < W and grid[r][c] == " ":
            grid[r][c] = "."
    for (x, y, ch) in ((0, 0, "S"), (gx, gy, "G")):
        c, r = int(round((x - xmin) / scale)), H - 1 - int(round((y - ymin) / scale))
        if 0 <= r < H and 0 <= c < W:
            grid[r][c] = ch
    with open(path, "w") as f:
        f.write(f"world={world} goal=({gx},{gy})  S=start G=goal #=obstacle .=path\n")
        f.write("\n".join("".join(row) for row in grid) + "\n")


def main():
    args = [a for a in sys.argv[1:] if a != "--real"]
    realistic = "--real" in sys.argv
    mode = args[0] if args else "all"
    if mode == "hit":
        for g, pose, d, a, t in run_hit(realistic, verbose=True):
            print(f"goal {g} -> pose {[round(p, 3) for p in pose]}  pos err {d:.3f}  ang err {a:.3f}  {t:.1f} s")
        return 0
    if mode == "bug":
        world, gx, gy, gth = args[1], float(args[2]), float(args[3]), float(args[4])
        r = run_bug(world, gx, gy, gth, realistic, verbose=True)
        print(f"\nRESULT: final state {r['state']}  pos err {r['dist']:.3f} m  ang err {r['ang']:.3f} rad  "
              f"{r['time']:.1f} s  collided={r['robot'].collided}")
        return 0

    all_ok = True
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    for realistic in (False, True):
        label = "REALISTIC robot (stalling wheels, noisy lidar, latency)" if realistic else "IDEAL robot"
        print(f"\n===== {label} =====")
        print("hit_the_spot:")
        for g, pose, d, a, t in run_hit(realistic, verbose=False):
            ok = d < 0.12 and a < 0.15
            all_ok &= ok
            print(f"  {'PASS' if ok else 'FAIL'} goal ({g[0]:.1f}, {g[1]:.1f}, {g[2]:.2f}) -> "
                  f"pos err {d:.3f} m, ang err {a:.3f} rad, {t:.1f} s")
        print("bug_navigation:")
        for world, gx, gy, gth, expect in SCENARIOS:
            r = run_bug(world, gx, gy, gth, realistic, verbose=False)
            ok = r["state"] == expect and not r["robot"].collided
            if expect == "AT_GOAL":
                ok = ok and r["dist"] < 0.12 and r["ang"] < 0.15
            all_ok &= ok
            tag = f"{world}{'_real' if realistic else ''}{'_unreachable' if expect != 'AT_GOAL' else ''}"
            plot(r["robot"], world, gx, gy, os.path.join(HERE, "out", f"{tag}.txt"))
            print(f"  {'PASS' if ok else 'FAIL'} {world:<10} goal ({gx},{gy},{gth}) -> {r['state']:<16} "
                  f"pos err {r['dist']:.3f} m, {r['time']:5.1f} s, closest approach "
                  f"{r['robot'].min_clearance * 100:4.0f} cm, collided={r['robot'].collided}")
    print("\nALL PASS" if all_ok else "\nSOME FAILED")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
