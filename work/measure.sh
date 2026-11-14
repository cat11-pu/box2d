#!/bin/bash
# Interleaved before/after measurement to cancel out machine-load drift.
# Each binary runs REPEATS times; the harness keeps the per-step minimum.
set -e
cd "$(dirname "$0")/.."

OUT=artifacts/final
mkdir -p "$OUT"
REPEATS=${REPEATS:-5}

for w in 1 4 8; do
	if [ "$w" = "1" ]; then
		affinity="2"
	elif [ "$w" = "4" ]; then
		affinity="2-5"
	else
		affinity="2-9"
	fi
	for round in $(seq 1 "$REPEATS"); do
		taskset -c $affinity ./work/e2e_base -w=$w -r=1 > "$OUT/raw_base_w${w}_r${round}.csv"
		taskset -c $affinity ./work/e2e_opt  -w=$w -r=1 > "$OUT/raw_opt_w${w}_r${round}.csv"
	done

	# Per-step minimum across rounds.
	python3 work/merge_min.py "$OUT" base $w "$REPEATS" > "$OUT/base_w${w}.csv"
	python3 work/merge_min.py "$OUT" opt  $w "$REPEATS" > "$OUT/opt_w${w}.csv"

	# Exact result consistency across every step and every repeat.
	for round in $(seq 1 "$REPEATS"); do
		cut -d, -f8,9,10,11 "$OUT/raw_base_w${w}_r${round}.csv" > "$OUT/.b"
		cut -d, -f8,9,10,11 "$OUT/raw_opt_w${w}_r${round}.csv"  > "$OUT/.o"
		if diff -q "$OUT/.b" "$OUT/.o" >/dev/null; then
			echo "w=$w round=$round consistency IDENTICAL"
		else
			echo "w=$w round=$round consistency MISMATCH"
		fi
	done
done
echo MEASURE_DONE
