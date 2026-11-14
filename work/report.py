#!/usr/bin/env python3
# Before/after timing statistics for the 10k-body broad-phase benchmark.
import csv
import sys

DATA = sys.argv[1] if len(sys.argv) > 1 else "artifacts/final"
STAGES = [
	("step_ms", "total frame"),
	("pairs_ms", "broad phase"),
	("collide_ms", "narrow phase"),
	("solve_ms", "solver"),
	("refit_ms", "tree refit"),
	("transforms_ms", "transforms"),
]


def read(path):
	with open(path) as f:
		lines = f.readlines()
	return list(csv.DictReader(lines[1:]))


def pct(values, p):
	values = sorted(values)
	return values[min(len(values) - 1, int(len(values) * p))]


for workers in (1, 4, 8):
	base = read(f"{DATA}/base_w{workers}.csv")
	opt = read(f"{DATA}/opt_w{workers}.csv")
	assert len(base) == len(opt)
	print(f"\n=== workers = {workers} ({len(base)} steps, per-step min of 5 runs, ms) ===")
	print(f"{'stage':<13}{'base mean':>10}{'opt mean':>10}{'gain':>8}"
		  f"{'base p95':>10}{'opt p95':>10}{'base max':>10}{'opt max':>10}")
	for col, label in STAGES:
		bv = [float(r[col]) for r in base]
		ov = [float(r[col]) for r in opt]
		bm, om = sum(bv) / len(bv), sum(ov) / len(ov)
		gain = (bm - om) / bm * 100.0
		print(f"{label:<13}{bm:>10.4f}{om:>10.4f}{gain:>7.1f}%"
			  f"{pct(bv, 0.95):>10.4f}{pct(ov, 0.95):>10.4f}{max(bv):>10.4f}{max(ov):>10.4f}")

	b_over = sum(1 for r in base if float(r["step_ms"]) > 8.0)
	o_over = sum(1 for r in opt if float(r["step_ms"]) > 8.0)
	print(f"frames over 8 ms budget: base={b_over}  opt={o_over}")
