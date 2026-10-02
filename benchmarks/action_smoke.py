"""Require every Action stress scenario and its bounded accounting invariants."""
import json
import subprocess
import sys

try:
    result = subprocess.run(
        [sys.argv[1], "--executor", sys.argv[2], "--seconds", "2", "--lanes", "1"],
        text=True, capture_output=True, timeout=25, check=False,
    )
except subprocess.TimeoutExpired as error:
    for output in (error.stdout, error.stderr):
        if output:
            print(output.decode(errors="replace") if isinstance(output, bytes) else output,
                  file=sys.stderr)
    raise
print(result.stdout, end="")
print(result.stderr, end="", file=sys.stderr)
if result.returncode:
    raise SystemExit(result.returncode)
summaries = [row for line in result.stdout.splitlines()
             if (row := json.loads(line))["type"] == "summary"]
assert len(summaries) == 1
row = summaries[0]
cycles = row["cycles"]
assert cycles > 0 and row["passed"]
assert row["admitted"] == row["completed"] == 9 * cycles
assert row["values"] == row["aborted"] == 3 * cycles
assert row["stopped"] == 2 * cycles and row["rejected"] == cycles
assert row["feedback"] >= cycles and row["recovered_servers"] == 1
assert row["inflight"] == row["duplicates"] == row["business_live_after_join"] == 0
assert row["peak_inflight"] == 3 and row["business_peak_per_lane"] == 1
