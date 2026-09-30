"""Verify installed targets after moving their prefix, in fresh CMake projects."""

import os
import subprocess
import sys
import tempfile
from pathlib import Path


def run(*args):
    result = subprocess.run(
        args,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=60,
        check=False,
    )
    if result.returncode:
        raise RuntimeError(f"command failed: {args}\n{result.stdout}")


cmake, build, source, lexec = sys.argv[1:]
with tempfile.TemporaryDirectory(prefix="lrclexec-install-") as directory:
    root = Path(directory)
    original = root / "original"
    installed = root / "relocated"
    run(cmake, "--install", build, "--prefix", str(original))
    original.rename(installed)
    for config in (installed / "share/lrclexec/cmake").glob("*.cmake"):
        content = config.read_text()
        if source in content or build in content or lexec in content:
            raise RuntimeError(f"export contains developer path: {config}")
    for name, options, executables in (
        ("bundled", [], ["consumer"]),
        ("provider", [f"-DLEXEC_PROVIDER_SOURCE={lexec}"], ["consumer"]),
        ("siblings", ["-DCONSUMER_SIBLINGS=ON"], ["first/first", "second/second"]),
    ):
        consumer_build = root / name
        prefixes = (
            str(installed)
            + ";"
            + os.environ.get("CMAKE_PREFIX_PATH", "").replace(os.pathsep, ";")
        )
        run(
            cmake,
            "-S",
            str(Path(source) / "tests/consumer"),
            "-B",
            str(consumer_build),
            "-G",
            "Ninja",
            f"-DCMAKE_PREFIX_PATH={prefixes}",
            *options,
        )
        run(cmake, "--build", str(consumer_build), "-j2")
        for executable in executables:
            run(str(consumer_build / executable))
        print(f"installed consumer passed: {name}")
