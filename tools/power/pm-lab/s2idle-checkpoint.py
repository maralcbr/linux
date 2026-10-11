#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""LAB ONLY: fsync cycle intent and completed volatile traces before reset."""
import argparse
import json
import os
from pathlib import Path
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("log", type=Path)
parser.add_argument("--cycles", type=int, default=10)
parser.add_argument("--awake", type=float, default=120)
args = parser.parse_args()
trace = Path("/sys/kernel/debug/pm_lab_s2idle")
params = Path("/sys/module/thunderbolt_apple/parameters")
wake = Path("/sys/module/apple_mailbox/parameters/lab_s2idle_wake_ms")
if int(wake.read_text()) <= 0:
    parser.error("set apple_mailbox.lab_s2idle_wake_ms before running")
if args.cycles < 1 or args.awake < 0:
    parser.error("cycles must be positive and awake time nonnegative")
boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
with args.log.open("a") as log:
    directory = os.open(args.log.resolve().parent, os.O_RDONLY | os.O_DIRECTORY)
    os.fsync(directory)
    os.close(directory)
    for cycle in range(1, args.cycles + 1):
        for phase in ("before", "after"):
            record = {"boot": boot, "cycle": cycle, "phase": phase,
                      "time": time.time(), "trace": trace.read_text(),
                      "router_sleep": (params / "router_sleep").read_text().strip(),
                      "resume_recovery": (params / "resume_recovery").read_text().strip()}
            log.write(json.dumps(record) + "\n")
            log.flush()
            os.fsync(log.fileno())
            print(f"cycle={cycle} phase={phase}", flush=True)
            if phase == "before":
                Path("/sys/power/state").write_text("freeze\n")
        time.sleep(args.awake)
