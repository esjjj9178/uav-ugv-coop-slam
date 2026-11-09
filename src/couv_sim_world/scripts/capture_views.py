#!/usr/bin/env python3
"""Render reference views of canopy_world with the same ogre2 pipeline the robots'
cameras use, and save them to docs/:

    docs/view_top.png    straight down from 26 m
    docs/view_ugv.png    UGV start pose, camera height 0.5 m, facing +x (D435 HFOV 87 deg)
    docs/view_uav.png    UAV above the start, altitude 5 m, facing +x, pitched down

A temporary copy of the world with static camera sensors (<save enabled>) is
run headless; the image topics are subscribed (gz only renders cameras that have
subscribers) until every camera has saved a few frames. Only docs/ is modified.

Usage: python3 scripts/capture_views.py [--frames 4] [--timeout 120]
"""
import argparse
import glob
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_world  # noqa: E402
import layout as L  # noqa: E402

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
D435_HFOV = math.radians(87)

VIEWS = {
    # name: (pose x y z roll pitch yaw, hfov, width, height)
    "view_top": ((0, 0, 26, 0, math.pi / 2, math.pi / 2), 0.80, 1024, 1024),
    "view_ugv": ((L.START_XY[0], L.START_XY[1], 0.5, 0, 0, 0), D435_HFOV, 1280, 720),
    "view_uav": ((L.START_XY[0], L.START_XY[1], L.DRONE_ALTITUDE, 0, 0.45, 0), D435_HFOV, 1280, 720),
}


def camera_model(name, pose, hfov, w, h, out_dir):
    return f"""    <model name="{name}_cam">
      <static>true</static>
      <pose>{gen_world.vec(*pose)}</pose>
      <link name="link">
        <sensor name="{name}" type="camera">
          <always_on>true</always_on>
          <topic>capture/{name}/image</topic>
          <update_rate>2</update_rate>
          <camera>
            <horizontal_fov>{hfov:.4f}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>100</far></clip>
            <save enabled="true"><path>{out_dir}/{name}</path></save>
          </camera>
        </sensor>
      </link>
    </model>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=int, default=4, help="frames to wait for per camera (last one is kept)")
    ap.add_argument("--timeout", type=float, default=120.0, help="seconds")
    args = ap.parse_args()

    tmp = tempfile.mkdtemp(prefix="couv_capture_")
    extra = "".join(camera_model(n, *v, tmp) for n, v in VIEWS.items())
    world = os.path.join(tmp, "capture_world.sdf")
    open(world, "w").write(gen_world.world_sdf(extra))

    env = dict(os.environ)
    models = os.path.join(PKG_DIR, "models")
    env["GZ_SIM_RESOURCE_PATH"] = models + (":" + env["GZ_SIM_RESOURCE_PATH"] if env.get("GZ_SIM_RESOURCE_PATH") else "")
    cmd = ["gz", "sim", "-s", "-r", world]
    print("running:", " ".join(cmd))
    procs = [subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL)]
    procs += [subprocess.Popen(["gz", "topic", "-e", "-t", f"capture/{n}/image"], env=env,
                               stdout=subprocess.DEVNULL) for n in VIEWS]

    def counts():
        return {n: len(glob.glob(os.path.join(tmp, n, "*.png"))) for n in VIEWS}

    t0 = time.time()
    try:
        while time.time() - t0 < args.timeout and min(counts().values()) < args.frames:
            if procs[0].poll() is not None:
                break
            time.sleep(1.0)
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                p.kill()
    time.sleep(0.5)

    docs = os.path.join(PKG_DIR, "docs")
    os.makedirs(docs, exist_ok=True)
    ok = True
    for name in VIEWS:
        frames = sorted(glob.glob(os.path.join(tmp, name, "*.png")), key=os.path.getmtime)
        if not frames:
            print(f"  {name}: NO FRAMES SAVED")
            ok = False
            continue
        shutil.copy(frames[-1], os.path.join(docs, f"{name}.png"))
        print(f"  docs/{name}.png  ({len(frames)} frames rendered)")
    if ok:
        shutil.rmtree(tmp, ignore_errors=True)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
