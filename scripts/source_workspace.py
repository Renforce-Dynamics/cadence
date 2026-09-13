#!/usr/bin/env python3
"""Install explicitly pinned submodule sources into a project environment."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = json.loads((ROOT / "source-workspace.json").read_text())


def run(*command, capture=False):
    result = subprocess.run([str(x) for x in command], cwd=ROOT, check=True,
                            text=True, stdout=subprocess.PIPE if capture else None)
    return result.stdout if capture else None


def check_sources():
    paths = [*MANIFEST["packages"]]
    for extra_paths in MANIFEST.get("optional_packages", {}).values():
        paths.extend(extra_paths)
    for relative in paths:
        source = (ROOT / relative).resolve()
        if not source.is_relative_to(ROOT) or not (source / "pyproject.toml").is_file():
            raise ValueError(f"missing source {relative}; run scripts/submodules.sh init")
    status = run("git", "submodule", "status", "--recursive", capture=True)
    if any(line[:1] in {"-", "+", "U"} for line in status.splitlines()):
        raise ValueError("submodules differ from pinned commits; run scripts/submodules.sh init")
    return status


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("action", choices=("init", "status", "check", "setup"))
    p.add_argument("--python", default=os.environ.get("CADENCE_PYTHON", sys.executable))
    p.add_argument("--venv", default=os.environ.get("CADENCE_VENV", str(ROOT / ".venv")))
    p.add_argument("--offline", action="store_true")
    p.add_argument("--extra", action="append", default=[])
    args = p.parse_args()
    if args.action == "init":
        run("git", "submodule", "sync", "--recursive")
        run("git", "submodule", "update", "--init", "--recursive")
        return 0
    if args.action == "status":
        run("git", "submodule", "status", "--recursive")
        return 0
    status = check_sources()
    if args.action == "check":
        print(status, end="")
        return 0
    uv = shutil.which("uv")
    if not uv:
        p.error("uv is required")
    env = Path(args.venv).expanduser().resolve()
    run(uv, "venv", "--allow-existing", "--python", args.python, env)
    command = [uv, "pip", "install", "--python", env / "bin/python", "pytest>=8,<9"]
    if args.offline:
        command.append("--offline")
    extras = list(dict.fromkeys([*MANIFEST.get("extras", []), *args.extra]))
    packages = list(MANIFEST["packages"])
    for extra in extras:
        packages.extend(MANIFEST.get("optional_packages", {}).get(extra, []))
    for relative in dict.fromkeys(packages):
        package = str((ROOT / relative).resolve())
        if relative == "." and extras:
            package += "[" + ",".join(extras) + "]"
        command += ["-e", package]
    run(*command)
    if MANIFEST.get("native_build"):
        run(ROOT / MANIFEST["native_build"])
    for extra in extras:
        build = MANIFEST.get("optional_native_build", {}).get(extra)
        if build:
            run(ROOT / build)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, subprocess.CalledProcessError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
