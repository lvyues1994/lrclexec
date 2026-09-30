"""Download Jazzy Nav2 packages into a local overlay without installing them."""

import argparse
import json
import re
import subprocess
import tempfile
from pathlib import Path

ROOTS = [
    "ros-jazzy-nav2-planner",
    "ros-jazzy-nav2-controller",
    "ros-jazzy-nav2-navfn-planner",
    "ros-jazzy-nav2-regulated-pure-pursuit-controller",
    "ros-jazzy-nav2-lifecycle-manager",
]

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--directory", type=Path, required=True)
args = parser.parse_args()
directory = args.directory.resolve()
debs = directory / "debs"
debs.mkdir(parents=True, exist_ok=True)
plan = subprocess.run(
    ["apt-get", "--simulate", "--no-install-recommends", "install", *ROOTS],
    text=True,
    capture_output=True,
    check=True,
).stdout
packages = dict(re.findall(r"^Inst (\S+)(?: \[[^\]]+\])? \((\S+)", plan, re.MULTILINE))
# Include already installed roots so the overlay's executables are self-contained.
for package in ROOTS:
    if package not in packages:
        policy = subprocess.run(
            ["apt-cache", "policy", package], text=True, capture_output=True, check=True
        ).stdout
        candidate = re.search(r"Candidate:\s+(\S+)", policy)
        if not candidate or candidate[1] == "(none)":
            raise RuntimeError(f"No APT candidate for {package}")
        packages[package] = candidate[1]
subprocess.run(
    ["apt", "download", *(f"{name}={version}" for name, version in packages.items())],
    cwd=debs,
    check=True,
)
prefix = directory / "root"
if prefix.is_symlink():
    raise RuntimeError("Refusing to replace a symlinked overlay root")
with tempfile.TemporaryDirectory(prefix="nav2-staging-", dir=directory) as temporary:
    staged = Path(temporary) / "root"
    staged.mkdir()
    extracted = set()
    for archive in sorted(debs.glob("*.deb")):
        metadata = subprocess.run(
            ["dpkg-deb", "--show", "--showformat=${Package}\t${Version}", str(archive)],
            text=True,
            capture_output=True,
            check=True,
        ).stdout
        name, version = metadata.split("\t")
        if packages.get(name) != version:
            continue
        subprocess.run(["dpkg-deb", "--extract", str(archive), str(staged)], check=True)
        extracted.add(name)
    missing = set(packages) - extracted
    if missing:
        raise RuntimeError(f"Downloaded packages missing: {sorted(missing)}")
    previous = Path(temporary) / "previous"
    if prefix.exists():
        prefix.rename(previous)
    try:
        staged.rename(prefix)
    except OSError:
        if previous.exists():
            previous.rename(prefix)
        raise
(directory / "versions.json").write_text(json.dumps(packages, indent=2) + "\n")
print(f"Nav2 overlay: {prefix / 'opt/ros/jazzy'}")
