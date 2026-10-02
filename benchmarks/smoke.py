"""Check the measurement contract, with no machine-specific performance threshold."""

import json
import os
import subprocess
import sys

binary, executor = sys.argv[1:]
for mode in ("post", "schedule", "mixed"):
    try:
        result = subprocess.run(
            [binary, "--executor", executor, "--mode", mode, "--seconds", "1",
             "--warmup", "0", "--producers", "4", "--window", "32"],
            env={**os.environ, "RCUTILS_LOGGING_USE_STDOUT": "0"},
            capture_output=True, text=True, timeout=15,
        )
    except subprocess.TimeoutExpired as error:
        for output in (error.stdout, error.stderr):
            if output:
                print(output.decode(errors="replace") if isinstance(output, bytes) else output,
                      file=sys.stderr)
        raise
    if result.returncode != 0:
        print(result.stdout, file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        raise SystemExit(f"{executor}/{mode}: child returned {result.returncode}")
    records = [json.loads(line) for line in result.stdout.splitlines()]
    summaries = [record for record in records if record["type"] == "summary"]
    assert len(summaries) == 1, records
    summary = summaries[0]
    assert summary["passed"] and summary["inflight"] == 0, summary
    assert summary["errors"] == summary["duplicates"] == 0, summary
    assert summary["admitted"] == summary["values"] + summary["stopped"], summary
    assert 0 < summary["peak_inflight"] <= 32, summary
    kinds = {"schedule", "timer_cancel", "timer_race", "service", "topic"} if mode == "mixed" else {mode}
    assert set(summary["latency"]) == kinds, summary
    assert sum(histogram["count"] for histogram in summary["latency"].values()) == summary["admitted"]
    assert all(histogram["p50_upper_ns"] <= histogram["p95_upper_ns"] <= histogram["p99_upper_ns"]
               for histogram in summary["latency"].values()), summary
    samples = [record for record in records if record["type"] == "sample"]
    assert samples and samples[-1]["inflight"] == 0, records
    print(f"{executor} / {mode}: accounting, latency and drain verified")
