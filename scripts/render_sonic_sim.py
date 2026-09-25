#!/usr/bin/env python3
"""Offline mp4 render of the SONIC MuJoCo sim2sim (watch the sim).

In-process driver: builds the deployment from the sim entry, steps the 50 Hz
control loop manually (fixedpos entry gate -> sonic_clip), and renders frames
with mujoco.Renderer through a pelvis-tracking side camera. Like
verify_sonic_sim.py it bypasses the PLNJ operator ingress; it is a
visualization aid, not joystick-chain validation. Requires the
optional sim and inference extras plus imageio-ffmpeg (dev tool only; not a
package dependency):

  uv pip install --python .venv/bin/python "mujoco>=3,<4" "onnxruntime>=1.16,<1.24" imageio-ffmpeg

Usage:
  scripts/render_sonic_sim.py --clip 001_walk_front_slow.npz --out /tmp/walk.mp4
  scripts/render_sonic_sim.py --clip BMD_0319_stand.npz --seconds 12 --out /tmp/stand.mp4
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys

import numpy as np

# Headless Linux defaults to EGL offscreen rendering (no X needed).
if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

ROOT = Path(__file__).resolve().parents[1]

from cadence.deployment import load_run_config, prepare_deployment
from cadence.plugins import ControlFrame
from cadence.runtime import RuntimeConfig, RuntimeInput, RuntimeKernel
from cadence.runtime.loop import execute_cycle

from cadence.sonic.contract import isaaclab_to_cadence
from cadence.sonic.reference import SonicClip

DEFAULT_ENTRY = ROOT / "configs/entry/a3/mock/entry_a3_sonic_sim.yaml"


def _load_encoder(out, width, height, fps):
    try:
        import imageio_ffmpeg
    except ImportError as exc:
        raise SystemExit(
            "imageio-ffmpeg is required for rendering: "
            "uv pip install --python .venv/bin/python imageio-ffmpeg"
        ) from exc
    if width % 2 or height % 2:
        raise SystemExit("width/height must be even for h264 yuv420p")
    writer = imageio_ffmpeg.write_frames(
        str(out), (width, height), fps=fps, codec="libx264", pix_fmt_out="yuv420p",
        quality=7, macro_block_size=None,
    )
    writer.send(None)
    return writer


def _make_camera(mujoco, model):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis_link")
    if body < 0:
        raise RuntimeError("model has no pelvis_link body to track")
    cam.trackbodyid = body
    cam.distance = 3.0
    cam.azimuth = 100.0    # 3/4 side view
    cam.elevation = -12.0
    return cam


def _overlay(mujoco, renderer, text):
    context = getattr(renderer, "_mjr_context", None)
    if context is None:
        return
    mujoco.mjr_overlay(
        mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
        renderer._rect, text, "", context,
    )


def _render_frame(mujoco, renderer, data, camera, overlay_text, pixels):
    """Render the scene, draw the overlay, read pixels (mjr_render inside
    Renderer.render() would wipe an overlay drawn before it)."""
    renderer.update_scene(data, camera=camera)
    mujoco.mjr_render(renderer._rect, renderer._scene, renderer._mjr_context)
    if overlay_text:
        _overlay(mujoco, renderer, overlay_text)
    mujoco.mjr_readPixels(pixels, None, renderer._rect, renderer._mjr_context)
    return np.flipud(pixels)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_ENTRY))
    parser.add_argument("--clip", default=None,
                        help="NPZ name under data/motions/ or an explicit path "
                             "(default: the clip configured in the sim entry)")
    parser.add_argument("--seconds", type=float, default=None,
                        help="default: whole clip plus 2 s of the DONE hold")
    parser.add_argument("--out", required=True)
    parser.add_argument("--fps", type=int, default=25)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args(argv)
    if args.fps < 1 or args.fps > 50:
        parser.error("--fps must be in [1, 50]")

    resolved = load_run_config(args.config)
    plan = prepare_deployment(resolved)
    cfg = plan.resolved.data
    backend = plan.backend
    backend.start()

    from types import SimpleNamespace
    services = SimpleNamespace(dimension=plan.dimension,
                               joint_names=tuple(cfg["robot"]["joints"]))
    states = plan.catalog.instantiate(services)
    state = states["sonic_clip"]
    if args.clip is not None:
        candidate = Path(args.clip)
        if not candidate.is_file():
            candidate = ROOT / "data/motions" / args.clip
        state.clip = SonicClip.load(candidate)
    clip = state.clip
    seconds = args.seconds if args.seconds is not None else clip.duration_s + 2.0
    ticks = round(seconds * plan.control_hz)
    render_every = max(1, round(plan.control_hz / args.fps))

    out = Path(args.out).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    import mujoco
    renderer = mujoco.Renderer(backend.model, height=args.height, width=args.width)
    camera = _make_camera(mujoco, backend.model)
    pixels = np.empty((args.height, args.width, 3), dtype=np.uint8)
    writer = _load_encoder(out, args.width, args.height, args.fps)

    tick = 0
    frames = 0
    zmin, zmax = math.inf, -math.inf
    errors = []
    last_sub = ""
    robot = backend.read_state()
    kernel = RuntimeKernel(
        RuntimeConfig(plan.catalog, services, ControlFrame,
                      cfg["robot"]["position_min"], cfg["robot"]["position_max"],
                      cfg["runtime"]["start_state"], plan.deadline_s),
        robot, 0.0, preloaded_plugins=states)
    try:

        def make_input(request=None):
            nonlocal tick, robot
            tick += 1
            robot = backend.read_state()
            return RuntimeInput(tick / plan.control_hz, robot, None, requested_state=request)

        # fixedpos entry gate.
        for _ in range(round(5 * plan.control_hz)):
            execute_cycle(kernel, backend, make_input(2))
            if kernel.entry_gate_ready:
                break
        if not kernel.entry_gate_ready:
            raise SystemExit("fixedpos entry gate never became ready")
        execute_cycle(kernel, backend, make_input(6))

        for _ in range(ticks):
            cycle = execute_cycle(kernel, backend, make_input())
            if cycle.output.safety_halted:
                raise SystemExit(f"safety halt: {cycle.output.safety_reason}")
            robot = backend.read_state()
            z = float(robot.root_position_w[2])
            zmin, zmax = min(zmin, z), max(zmax, z)
            last_sub = cycle.output.skill_state
            if last_sub == "PLAYING":
                t = min(state.play_tick, state.clip.frames - 1)
                errors.append(robot.joint_pos - isaaclab_to_cadence(state.clip.q_ref[t]))
            if tick % render_every == 0:
                frame = _render_frame(mujoco, renderer, backend.data, camera,
                                      f"t={tick / plan.control_hz:5.1f}s  {last_sub}",
                                      pixels)
                writer.send(np.ascontiguousarray(frame))
                frames += 1
    finally:
        writer.close()
        renderer.close()
        kernel.reject()
        kernel.current.on_exit(ControlFrame(tick / plan.control_hz, robot), [])
        backend.close()
    rmse = float(np.sqrt(np.mean(np.square(errors)))) if errors else float("nan")
    print(f"rendered {frames} frames -> {out}")
    print(f"clip={clip.source.name} seconds={tick / plan.control_hz:.1f} "
          f"z=[{zmin:.3f},{zmax:.3f}] rmse={rmse:.4f} rad final_substate={last_sub}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
