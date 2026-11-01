#!/bin/zsh
# Open this file in Terminal. The operator types ARM at the Python prompt.
set -eu
cd -- "${0:A:h:h}"
if /usr/sbin/lsof -tiTCP:8765 -sTCP:LISTEN >/dev/null 2>&1; then
  print 'Port 8765 is already in use. Stop its current robot server first.'
  exit 1
fi
run_dir="artifacts/real_staged_$(date -u +%Y%m%dT%H%M%SZ)_$$.local"
mkdir -p artifacts
# Check actual writes before asking the operator to ARM. APFS free-space
# reporting alone did not prevent ENOSPC while writing the run manifest.
.venv/bin/python -c '
import os, shutil, tempfile
with tempfile.TemporaryFile(dir="artifacts") as probe:
    block = bytes(1024 * 1024)
    for _ in range(8):
        probe.write(block)
    probe.flush()
    os.fsync(probe.fileno())
free_mib = shutil.disk_usage(".").free / 1048576
print(f"Storage write check passed; free {free_mib:.0f} MiB")
'
startup_log="${run_dir}.startup.log"
print "Run: $run_dir"
print "Local stop (another terminal): touch $run_dir/STOP"
print "Startup log: $startup_log"
exec .venv/bin/python -u -m so101_mcp serve \
  --backend real --config config/physical_ai/real_staged.local.json \
  --transport streamable-http --port 8765 --enable-motion --run-dir "$run_dir" \
  > >(tee "$startup_log") 2>&1
