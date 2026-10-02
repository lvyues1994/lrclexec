"""Save reproducible local measurements and the environment needed to interpret them."""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess


def read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def command(args):
    result = subprocess.run(args, capture_output=True, text=True, timeout=10)
    return result.stdout.strip() if result.returncode == 0 else None


def positive(value):
    value = int(value)
    if not 1 <= value <= 86400:
        raise argparse.ArgumentTypeError("must be 1..86400")
    return value


def save_report(output, runs, failure=None):
    groups = {}
    for run in runs:
        key = f"{run['executor']}/{run['mode']}/p{run['producers']}/w{run['window']}"
        groups.setdefault(key, []).append(run["throughput_per_second"])
    report = {"runs": runs, "failure": failure, "throughput": {
        key: {"median": statistics.median(values), "min": min(values), "max": max(values)}
        for key, values in groups.items()
    }}
    (output / "results.json").write_text(json.dumps(report, indent=2) + "\n")


def run(binary, output, name, command_line, environment, timeout):
    try:
        result = subprocess.run([str(binary), *command_line], env=environment,
                                capture_output=True, text=True, timeout=timeout)
        stdout, stderr, code = result.stdout, result.stderr, result.returncode
    except subprocess.TimeoutExpired as error:
        def text(value):
            return value.decode(errors="replace") if isinstance(value, bytes) else value or ""
        stdout, stderr, code = text(error.stdout), text(error.stderr), None
    (output / f"{name}.jsonl").write_text(stdout)
    (output / f"{name}.stderr.log").write_text(stderr)
    return stdout, code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage", choices=("baseline", "soak"), default="baseline")
    parser.add_argument("--seconds", type=positive, default=10)
    parser.add_argument("--warmup", type=int, choices=range(61), default=2)
    parser.add_argument("--repeats", type=int, choices=range(1, 11), default=3)
    parser.add_argument("--executors", nargs="+", choices=("single", "multi", "events"),
                        default=("single", "multi", "events"))
    parser.add_argument("--domain", type=int, choices=range(180, 201), default=180)
    args = parser.parse_args()
    binary = args.binary.resolve(strict=True)
    args.output.mkdir(parents=True, exist_ok=False)
    repository = Path(__file__).resolve().parents[1]
    source_files = command(["git", "-C", str(repository), "ls-files", "--cached", "--others",
                            "--exclude-standard"]) or ""
    cpu_info = read("/proc/cpuinfo") or ""
    model = next((line.split(":", 1)[1].strip() for line in cpu_info.splitlines()
                  if line.startswith("model name")), None)
    metadata = {
        "utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "binary": str(binary), "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
        "git_head": command(["git", "-C", str(repository), "rev-parse", "HEAD"]),
        "git_status": command(["git", "-C", str(repository), "status", "--short"]),
        "source_sha256": {name: hashlib.sha256((repository / name).read_bytes()).hexdigest()
                          for name in sorted(set(source_files.splitlines()))
                          if (repository / name).is_file()},
        "platform": platform.platform(), "cpu_model": model,
        "cpu_affinity": sorted(os.sched_getaffinity(0)),
        "cpu_governor": read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
        "rmw": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        "ros_packages": command(["dpkg-query", "-W", "-f=${Package} ${Version}\n",
                                  "ros-jazzy-rclcpp", "ros-jazzy-rclcpp-action"]),
        "cmake_cache": read(binary.parents[1] / "CMakeCache.txt"),
        "options": {key: str(value) if isinstance(value, Path) else value
                    for key, value in vars(args).items()},
        "workload": "shared scheduler with one mutually exclusive callback group",
    }
    (args.output / "environment.json").write_text(json.dumps(metadata, indent=2) + "\n")
    runs = []
    profiles = ((1, 1), (4, 256)) if args.stage == "baseline" else ((4, 32),)
    modes = ("post", "schedule") if args.stage == "baseline" else ("mixed",)
    repetitions = args.repeats if args.stage == "baseline" else 1
    for executor_index, executor in enumerate(args.executors):
        for producers, window in profiles:
            for mode in modes:
                for repetition in range(repetitions):
                    name = f"{executor}-{mode}-p{producers}-r{repetition}"
                    env = {**os.environ, "ROS_DOMAIN_ID": str(args.domain + executor_index),
                           "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST", "RCUTILS_LOGGING_USE_STDOUT": "0",
                           "ROS_LOG_DIR": str(args.output.resolve() / "ros-logs")}
                    stdout, code = run(binary, args.output, name,
                                       ["--executor", executor, "--mode", mode, "--seconds", str(args.seconds),
                                        "--warmup", str(args.warmup), "--producers", str(producers),
                                        "--window", str(window)], env, args.seconds + args.warmup + 25)
                    try:
                        records = [json.loads(line) for line in stdout.splitlines()]
                        summaries = [record for record in records if record["type"] == "summary"
                                     and record["phase"] == "measure"]
                    except (json.JSONDecodeError, KeyError, TypeError) as error:
                        save_report(args.output, runs, {"run": name, "return_code": code,
                                                       "parse_error": str(error)})
                        raise SystemExit(f"invalid output: {name}; inspect {args.output}") from error
                    if code != 0 or len(summaries) != 1 or summaries[0].get("passed") is not True:
                        save_report(args.output, runs, {"run": name, "return_code": code,
                                                       "summary": summaries[0] if summaries else None})
                        raise SystemExit(f"failed: {name}; inspect {args.output}")
                    summary = summaries[0]
                    samples = [record["rss_kib"] for record in records if record["type"] == "sample"
                               and record["phase"] == "measure" and record["rss_kib"] is not None]
                    summary.update({"file": f"{name}.jsonl", "domain": int(env["ROS_DOMAIN_ID"]),
                                    "rss_first_kib": samples[0] if samples else None,
                                    "rss_last_kib": samples[-1] if samples else None,
                                    "rss_peak_kib": max(samples) if samples else None})
                    runs.append(summary)
                    print(f"{name}: {summary['admitted']} completions; passed", flush=True)
    save_report(args.output, runs)


if __name__ == "__main__":
    main()
