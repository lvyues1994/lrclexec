"""Run MuJoCo/Nav2 navigation, optionally showing the native MuJoCo window."""

import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path


def records(path):
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass  # The writer may be in the middle of its final line.
    return rows


def last_record(path):
    if not path.exists():
        return {}
    with path.open("rb") as stream:
        stream.seek(max(0, path.stat().st_size - 2048))
        lines = stream.read().splitlines()
    for line in reversed(lines):
        try:
            return json.loads(line)
        except json.JSONDecodeError:
            continue
    return {}


class Processes:
    def __init__(self, directory, environment):
        self.directory = directory
        self.environment = environment
        self.children = {}
        self.outputs = []
        self.forced = []
        self.cleanup_errors = []

    def start(self, name, args):
        output = (self.directory / f"{name}.log").open("w")
        self.outputs.append(output)
        child = subprocess.Popen(
            args,
            stdout=output,
            stderr=subprocess.STDOUT,
            env=self.environment,
            start_new_session=True,
        )
        self.children[name] = child
        return child

    def stop(self, name, timeout=6):
        child = self.children.get(name)
        if child is None or child.poll() is not None:
            return
        try:
            os.killpg(child.pid, signal.SIGINT)
        except ProcessLookupError:
            child.wait(timeout=2)
            return
        try:
            child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.forced.append(name)
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=2)

    def close(self):
        # Keep ROS time and TF live until the Action has drained and Nav2 exits.
        previous = {
            number: signal.signal(number, signal.SIG_IGN)
            for number in (signal.SIGINT, signal.SIGTERM)
        }
        try:
            for name in ("navigator", "manager", "controller", "planner", "bridge"):
                try:
                    self.stop(name)
                except Exception as error:  # noqa: BLE001 - Reap every remaining child.
                    self.cleanup_errors.append(f"{name}: {error}")
        finally:
            for number, handler in previous.items():
                signal.signal(number, handler)
        for output in self.outputs:
            output.close()
        (self.directory / "exit.json").write_text(
            json.dumps(
                {
                    "codes": {
                        name: child.returncode for name, child in self.children.items()
                    },
                    "forced": self.forced,
                    "cleanup_errors": self.cleanup_errors,
                },
                indent=2,
            )
            + "\n"
        )


def wait_until(
    predicate, timeout, description, processes, *, allow_navigator_exit=False
):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        for name, child in processes.children.items():
            if (
                name != "navigator" or not allow_navigator_exit
            ) and child.poll() is not None:
                raise RuntimeError(f"{name} exited during {description}")
        time.sleep(0.1)
    raise RuntimeError(f"timeout during {description}")


def run_case(args, case, domain):
    directory = args.logs.resolve() / case
    directory.mkdir(parents=True, exist_ok=True)
    # Old telemetry must not satisfy this run's readiness or motion predicates.
    telemetry = directory / "physics.jsonl"
    telemetry.unlink(missing_ok=True)
    for name in ("result.json", "exit.json", "viewer.ppm", "viewer.ppm.tmp"):
        (directory / name).unlink(missing_ok=True)
    prefix = args.nav2_prefix.resolve()
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain)
    environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    environment["ROS_LOG_DIR"] = str(directory / "ros")
    environment["AMENT_PREFIX_PATH"] = (
        str(prefix) + os.pathsep + environment.get("AMENT_PREFIX_PATH", "")
    )
    environment["LD_LIBRARY_PATH"] = os.pathsep.join(
        [
            str(prefix / "lib"),
            str(prefix.parents[2] / "usr/lib/x86_64-linux-gnu"),
            environment.get("LD_LIBRARY_PATH", ""),
        ]
    )
    processes = Processes(directory, environment)
    source = Path(__file__).resolve().parent
    executables = args.build.resolve() / "simulation/mujoco"
    outcome = None
    failure = None
    user_closed = False
    target = 1.0 if case == "straight" else 4.0

    def window_closed():
        return args.view and bool(last_record(telemetry).get("viewer_closed", False))

    try:
        bridge_args = [
            str(executables / "mujoco_bridge"),
            "--ros-args",
            "-p",
            f"model:={source / 'scene.xml'}",
            "-p",
            f"telemetry:={telemetry}",
            "-p",
            f"map_includes_obstacle:={'false' if case == 'obstacle' else 'true'}",
            "-p",
            f"viewer:={'true' if args.view else 'false'}",
            "-p",
            f"viewer_target_x:={target}",
        ]
        if args.view:
            bridge_args.extend(["-p", f"viewer_capture:={directory / 'viewer.ppm'}"])
        processes.start("bridge", bridge_args)
        wait_until(
            lambda: len(records(telemetry)) >= 2,
            20 if args.view else 5,
            "bridge readiness",
            processes,
        )
        for name, package, executable in (
            ("planner", "nav2_planner", "planner_server"),
            ("controller", "nav2_controller", "controller_server"),
            ("manager", "nav2_lifecycle_manager", "lifecycle_manager"),
        ):
            if case == "startup-cancel":
                break
            command = [
                str(prefix / "lib" / package / executable),
                "--ros-args",
                "--params-file",
                str(source / "nav2.yaml"),
            ]
            if name == "manager":
                command.extend(["-r", "__node:=lifecycle_manager_navigation"])
            processes.start(name, command)
        navigator = processes.start(
            "navigator",
            [
                str(executables / "mujoco_navigator"),
                "--ros-args",
                "-p",
                f"target_x:={target}",
            ],
        )
        if case == "startup-cancel":
            wait_until(
                lambda: "WAITING_FOR_NAV2" in (directory / "navigator.log").read_text(),
                5,
                "lifecycle wait before cancellation",
                processes,
            )
            os.killpg(navigator.pid, signal.SIGINT)
        elif case == "cancel":
            wait_until(
                lambda: (
                    window_closed()
                    or any(
                        row["x"] > 0.2 and abs(row["v"]) > 0.05
                        for row in records(telemetry)
                    )
                ),
                30,
                "actual robot motion before cancellation",
                processes,
            )
            if not window_closed():
                os.killpg(navigator.pid, signal.SIGINT)

        def navigation_finished():
            nonlocal user_closed
            if any(row["collision"] for row in records(telemetry)):
                raise RuntimeError("robot collided during navigation")
            if window_closed():
                user_closed = True
                # SignalStop is installed before this flushed readiness marker.
                wait_until(
                    lambda: (
                        "WAITING_FOR_NAV2" in (directory / "navigator.log").read_text()
                        or navigator.poll() is not None
                    ),
                    5,
                    "navigator signal readiness before window cancellation",
                    processes,
                    allow_navigator_exit=True,
                )
                processes.stop("navigator")
            return navigator.poll() is not None

        wait_until(
            navigation_finished,
            70,
            "navigation completion",
            processes,
            allow_navigator_exit=True,
        )
        output = (directory / "navigator.log").read_text()
        canceled = case in ("cancel", "startup-cancel")
        terminal = "STOPPED" if canceled else "SUCCEEDED"
        if user_closed:
            terminal = "SUCCEEDED" if "SUCCEEDED" in output else "STOPPED"
            canceled = terminal == "STOPPED"
        if (
            navigator.returncode != 0
            or terminal not in output
            or "DRAINED" not in output
        ):
            raise RuntimeError(f"navigation did not drain with {terminal}: {output}")
        completed_at = records(telemetry)[-1]["time"]

        def stopped():
            rows = [row for row in records(telemetry) if row["time"] > completed_at]
            return len(rows) >= 3 and all(
                abs(row["v"]) < 0.03 and abs(row["w"]) < 0.08 for row in rows[-3:]
            )

        wait_until(
            stopped,
            5,
            "physical stop after Action terminal",
            processes,
            allow_navigator_exit=True,
        )
        rows = records(telemetry)
        if any(row["collision"] for row in rows):
            raise RuntimeError("robot collided with obstacle or wall")
        last = rows[-1]
        if not canceled and (
            math.hypot(last["x"] - target, last["y"]) > 0.15 or abs(last["yaw"]) > 0.18
        ):
            raise RuntimeError(f"Action success did not match physical goal: {last}")
        if (
            case == "obstacle"
            and not canceled
            and max(abs(row["y"]) for row in rows) < 0.55
        ):
            raise RuntimeError("robot did not physically go around scan-only obstacle")
        outcome = {
            "case": case,
            "terminal": terminal,
            "final": last,
            "max_lateral_offset": max(abs(row["y"]) for row in rows),
        }
        if args.view and not user_closed:
            print(
                f"{terminal}: robot stopped. Close the MuJoCo window to exit.",
                flush=True,
            )
            wait_until(
                window_closed,
                math.inf,
                "MuJoCo window closure",
                processes,
                allow_navigator_exit=True,
            )
        if args.view:
            outcome["closed_by_user"] = True
    except (Exception, KeyboardInterrupt) as error:  # noqa: BLE001 - Save failed runs.
        failure = error
    finally:
        processes.close()
    if failure is None and (processes.forced or processes.cleanup_errors):
        failure = RuntimeError(
            f"shutdown failure: forced={processes.forced}, errors={processes.cleanup_errors}"
        )
    if failure is None and any(
        child.returncode != 0 for child in processes.children.values()
    ):
        failure = RuntimeError("a simulation process failed during shutdown")
    if failure is not None:
        (directory / "result.json").write_text(
            json.dumps({"status": "failed", "error": str(failure)}, indent=2) + "\n"
        )
        raise failure
    outcome["status"] = "passed"
    (directory / "result.json").write_text(json.dumps(outcome, indent=2) + "\n")
    print(json.dumps(outcome))


def interrupted(number, _frame):
    raise KeyboardInterrupt(f"signal {number}")


signal.signal(signal.SIGINT, interrupted)
signal.signal(signal.SIGTERM, interrupted)
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--build", type=Path, required=True)
parser.add_argument("--nav2-prefix", type=Path, required=True)
parser.add_argument("--logs", type=Path, required=True)
parser.add_argument(
    "--view", action="store_true", help="show MuJoCo and keep the window open"
)
parser.add_argument(
    "--case",
    choices=["all", "straight", "obstacle", "cancel", "startup-cancel"],
    default=None,
)
parser.add_argument("--domain", type=int, default=220)
arguments = parser.parse_args()
selected = arguments.case or ("obstacle" if arguments.view else "all")
if arguments.view and selected == "all":
    parser.error("--view runs one case; use --case obstacle or --case straight")
cases = (
    ["straight", "obstacle", "cancel", "startup-cancel"]
    if selected == "all"
    else [selected]
)
if arguments.domain < 0 or arguments.domain + len(cases) - 1 > 232:
    parser.error("each DDS domain must be between 0 and 232")
for index, current in enumerate(cases):
    try:
        run_case(arguments, current, arguments.domain + index)
    except (Exception, KeyboardInterrupt) as error:
        raise SystemExit(
            f"{current} failed: {error}\nLogs: {arguments.logs.resolve() / current}"
        ) from error
