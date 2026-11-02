"""Run hit_the_spot / bug_navigation against the fake simulator.

Usage (from the repo root):
    python3 sim/run_sim.py hit                  # drive through the default goals
    python3 sim/run_sim.py bug <world> x y th   # bug navigation in a test world
    python3 sim/run_sim.py all                  # run every scenario, report pass/fail

Worlds: empty, box, wall, two_boxes, corridor, u_trap
"""
import os
import sys
import math
import types
import io
import contextlib

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)

import fake_mbot  # noqa: E402
from fake_mbot import FakeMBot, Rect  # noqa: E402

# Install a fake mbot_bridge package before the student code imports it.
pkg = types.ModuleType("mbot_bridge")
api = types.ModuleType("mbot_bridge.api")
pkg.api = api
sys.modules["mbot_bridge"] = pkg
sys.modules["mbot_bridge.api"] = api
fake_mbot.install_fake_time()

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
}


def run_bug(world, gx, gy, gth, verbose=True):
    robot = FakeMBot(WORLDS[world])
    api.MBot = lambda: robot
    import importlib
    import bug_navigation
    importlib.reload(bug_navigation)
    nav = bug_navigation.BugNavigator(robot, gx, gy, gth)
    out = io.StringIO()
    with contextlib.redirect_stdout(out if not verbose else sys.stdout):
        pose = nav.run()
    states_seen = sorted(set(int(line.split("STATE ")[1].split()[0]) for line in out.getvalue().splitlines()
                             if "STATE " in line)) if not verbose else None
    dist = math.hypot(gx - pose[0], gy - pose[1])
    ang = abs(math.atan2(math.sin(gth - pose[2]), math.cos(gth - pose[2])))
    ok = (nav.state == bug_navigation.STATE_AT_GOAL and dist < 0.1 and ang < 0.15 and not robot.collided)
    return ok, nav, robot, dist, ang, states_seen


def run_hit(verbose=True):
    robot = FakeMBot([])
    api.MBot = lambda: robot
    import hit_the_spot
    goals = [(1.0, 0.0, 0.0), (1.0, 1.0, math.pi / 2), (0.0, 1.0, math.pi), (0.0, 0.0, 0.0)]
    results = []
    for g in goals:
        with contextlib.redirect_stdout(sys.stdout if verbose else io.StringIO()):
            pose = hit_the_spot.drive_to_pose(*g, robot)
        d = math.hypot(g[0] - pose[0], g[1] - pose[1])
        a = abs(math.atan2(math.sin(g[2] - pose[2]), math.cos(g[2] - pose[2])))
        results.append((g, pose, d, a))
    return results


def plot(robot, world, gx, gy, path):
    """Save an ASCII map of the run (no matplotlib needed)."""
    W, H, scale = 70, 30, 0.1
    xmin, ymin = -0.5, -1.5
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


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    if mode == "hit":
        for g, pose, d, a in run_hit(verbose=True):
            print(f"goal {g} -> pose {[round(p, 3) for p in pose]}  pos err {d:.3f}  ang err {a:.3f}")
    elif mode == "bug":
        world, gx, gy, gth = sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
        ok, nav, robot, d, a, _ = run_bug(world, gx, gy, gth, verbose=True)
        print(f"\nRESULT: {'PASS' if ok else 'FAIL'}  pos err {d:.3f} m  ang err {a:.3f} rad  "
              f"iterations {nav.iteration}  collided={robot.collided}")
    else:
        scenarios = [("empty", 2.0, 1.0, 1.0), ("box", 2.5, 0.0, 0.0), ("wall", 2.5, 0.0, 1.57),
                     ("two_boxes", 3.0, 0.3, 0.0), ("corridor", 3.0, 0.0, 0.0), ("u_trap", 2.5, 0.0, 0.0)]
        all_ok = True
        print("hit_the_spot:")
        for g, pose, d, a in run_hit(verbose=False):
            ok = d < 0.1 and a < 0.15
            all_ok &= ok
            print(f"  {'PASS' if ok else 'FAIL'} goal {g} -> pos err {d:.3f} m, ang err {a:.3f} rad")
        print("bug_navigation:")
        os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
        for world, gx, gy, gth in scenarios:
            fake_mbot._sim_clock[0] = 0.0
            ok, nav, robot, d, a, states = run_bug(world, gx, gy, gth, verbose=False)
            all_ok &= ok
            plot(robot, world, gx, gy, os.path.join(HERE, "out", f"{world}.txt"))
            print(f"  {'PASS' if ok else 'FAIL'} {world:<10} goal ({gx},{gy},{gth}) -> pos err {d:.3f} m, "
                  f"ang err {a:.3f} rad, {nav.iteration} iters, states {states}, collided={robot.collided}")
        print("\nALL PASS" if all_ok else "\nSOME FAILED")
        sys.exit(0 if all_ok else 1)
