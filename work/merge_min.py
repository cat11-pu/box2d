#!/usr/bin/env python3
# Merge repeated e2e CSVs by taking the per-step minimum of every timing column.
# Hash/count columns are run-independent and copied from the first repeat.
import csv
import sys

out_dir, tag, workers, repeats = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
timing_cols = {
	"step_ms", "pairs_ms", "collide_ms", "solve_ms", "refit_ms", "transforms_ms",
}

rows_per_run = []
for r in range(1, repeats + 1):
	path = f"{out_dir}/raw_{tag}_w{workers}_r{r}.csv"
	with open(path) as f:
		lines = f.readlines()
	# First line is a metadata banner, second is the CSV header.
	reader = csv.DictReader(lines[1:])
	rows_per_run.append(list(reader))

banner = open(f"{out_dir}/raw_{tag}_w{workers}_r1.csv").readline().strip()
header = list(rows_per_run[0][0].keys())

print(banner)
print(",".join(header))

base = rows_per_run[0]
for i, row in enumerate(base):
	merged = dict(row)
	for col in timing_cols:
		merged[col] = f"{min(float(run[i][col]) for run in rows_per_run):.4f}"
	print(",".join(merged[col] for col in header))
