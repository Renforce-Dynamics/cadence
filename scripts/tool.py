#!/usr/bin/env python3
"""Local repository tooling; external ecosystem dependencies come from wheels or an index."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / "scripts/tool.json").read_text())


def invoke(cmd, *, cwd=ROOT):
    subprocess.run([str(x) for x in cmd], cwd=cwd, check=True)


def main():
    # Run/deploy own their public help, including config and backend options.
    p = argparse.ArgumentParser(description=__doc__, add_help=not (len(sys.argv) > 1 and sys.argv[1] in {"run", "deploy"}))
    p.add_argument("action", choices=["setup", "build", "test", "doctor", "run", "deploy"])
    p.add_argument("--python", default=os.environ.get("CADENCE_PYTHON", sys.executable))
    p.add_argument(
        "--venv", default=os.environ.get("CADENCE_VENV", str(ROOT / ".venv"))
    )
    p.add_argument("--wheelhouse", default=os.environ.get("CADENCE_WHEELHOUSE"))
    p.add_argument("--out-dir", default=str(ROOT / "dist"))
    p.add_argument("--extra", action="append", default=[])
    args, extra = p.parse_known_args()
    if extra and extra[0] == "--":
        extra = extra[1:]
    uv = shutil.which("uv")
    python = Path(args.venv).expanduser().resolve() / "bin/python"
    packages = sorted((ROOT / "packages").glob("*/pyproject.toml")) + [
        ROOT / "pyproject.toml"
    ]
    if args.action in {"setup", "build"} and not uv:
        p.error("uv is required for setup/build")
    if args.action == "setup":
        invoke(
            [uv, "venv", "--allow-existing", "--python", args.python, python.parents[1]]
        )
        cmd = [uv, "pip", "install", "--python", python, "pytest>=8,<9"]
        if args.wheelhouse:
            cmd += ["--find-links", str(Path(args.wheelhouse).resolve())]
        for manifest in packages:
            value = str(manifest.parent)
            if manifest.parent == ROOT and args.extra:
                value += "[" + ",".join(args.extra) + "]"
            cmd += ["-e", value]
        invoke(cmd + extra)
    elif args.action == "build":
        for manifest in packages:
            for stale in (manifest.parent / "build").glob("lib*"):
                if stale.is_dir():
                    shutil.rmtree(stale)
            invoke(
                [
                    uv,
                    "build",
                    "--wheel",
                    "--python",
                    args.python,
                    "--out-dir",
                    Path(args.out_dir).resolve(),
                    manifest.parent,
                ]
                + extra
            )
    else:
        if not python.exists():
            p.error("environment missing; run setup or pass --venv")
        if args.action == "test":
            invoke([python, "-m", "pytest", "tests", "-q"] + extra)
        else:
            invoke([python] + CONFIG[args.action] + extra,
                   cwd=None if args.action in {"run", "deploy"} else ROOT)


if __name__ == "__main__":
    main()
