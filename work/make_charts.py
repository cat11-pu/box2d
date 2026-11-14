#!/usr/bin/env python3
# Render before/after per-step timing curves as standalone SVG (stdlib only).
import csv
import statistics
import sys

RESULTS = "artifacts/results"
OUT = "artifacts/results"
ROUNDS = 8


def read(path):
	with open(path) as f:
		return list(csv.DictReader(f.readlines()[1:]))


def median_series(tag, workers, column):
	runs = [read(f"{RESULTS}/{tag}_w{workers}_r{r}.csv") for r in range(1, ROUNDS + 1)]
	n = len(runs[0])
	return [statistics.median(float(run[i][column]) for run in runs) for i in range(n)]


def polyline(points, x0, y0, w, h, xmax, ymax):
	return " ".join(
		f"{x0 + (x / xmax) * w:.1f},{y0 + h - (min(y, ymax) / ymax) * h:.1f}" for x, y in enumerate(points)
	)


def grid(x0, y0, w, h, ymax, ticks=5):
	out = []
	for i in range(ticks + 1):
		y = y0 + h - (i / ticks) * h
		value = (i / ticks) * ymax
		out.append(f'<line x1="{x0}" y1="{y:.1f}" x2="{x0 + w}" y2="{y:.1f}" stroke="#e6e6e6"/>')
		out.append(f'<text x="{x0 - 8}" y="{y + 4:.1f}" text-anchor="end" font-size="12" fill="#555">{value:.1f}</text>')
	return "\n".join(out)


def chart(scene, scene_title, workers, column, title, fname, budget=None):
	base = median_series(f"b_{scene}", workers, column)
	opt = median_series(f"o_{scene}", workers, column)
	x0, y0, w, h = 70, 40, 900, 360
	ymax = max(max(base), max(opt)) * 1.05
	if budget:
		ymax = max(ymax, budget * 1.1)
	xmax = len(base) - 1
	parts = [
		f'<svg xmlns="http://www.w3.org/2000/svg" width="{w + x0 + 30}" height="{h + y0 + 50}" font-family="sans-serif">',
		f'<text x="{x0}" y="22" font-size="18" font-weight="bold">{title}</text>',
		f'<text x="{x0}" y="36" font-size="12" fill="#666">{scene_title}, workers={workers}, per-step median of {ROUNDS} runs (ms)</text>',
		grid(x0, y0, w, h, ymax),
	]
	if budget:
		yb = y0 + h - (budget / ymax) * h
		parts.append(f'<line x1="{x0}" y1="{yb:.1f}" x2="{x0 + w}" y2="{yb:.1f}" stroke="#d62728" stroke-dasharray="6 4"/>')
		parts.append(f'<text x="{x0 + w - 4}" y="{yb - 5:.1f}" text-anchor="end" font-size="12" fill="#d62728">8 ms budget</text>')
	parts.append(f'<polyline fill="none" stroke="#1f77b4" stroke-width="1.6" points="{polyline(base, x0, y0, w, h, xmax, ymax)}"/>')
	parts.append(f'<polyline fill="none" stroke="#2ca02c" stroke-width="1.6" points="{polyline(opt, x0, y0, w, h, xmax, ymax)}"/>')
	parts.append(f'<rect x="{x0 + w - 150}" y="{y0 + 8}" width="14" height="14" fill="#1f77b4"/><text x="{x0 + w - 130}" y="{y0 + 20}" font-size="13">before (base)</text>')
	parts.append(f'<rect x="{x0 + w - 150}" y="{y0 + 28}" width="14" height="14" fill="#2ca02c"/><text x="{x0 + w - 130}" y="{y0 + 40}" font-size="13">after (bulk)</text>')
	parts.append(f'<line x1="{x0}" y1="{y0 + h}" x2="{x0 + w}" y2="{y0 + h}" stroke="#333"/>')
	parts.append(f'<line x1="{x0}" y1="{y0}" x2="{x0}" y2="{y0 + h}" stroke="#333"/>')
	parts.append(f'<text x="{x0 + w / 2}" y="{y0 + h + 35}" text-anchor="middle" font-size="13">step</text>')
	parts.append(f'<text x="20" y="{y0 + h / 2}" font-size="13" transform="rotate(-90 20,{y0 + h / 2})" text-anchor="middle">milliseconds</text>')
	parts.append("</svg>")
	with open(f"{OUT}/{fname}", "w") as f:
		f.write("\n".join(parts))


scenes = (("dyn", "10k dynamic rigid bodies"), ("stat", "10k dynamic + 3.6k static tiles"))
for scene, title in scenes:
	for workers in (1, 4, 8):
		chart(scene, title, workers, "step_ms", f"Total frame time ({title})", f"frame_{scene}_w{workers}.svg", budget=8.0)
		chart(scene, title, workers, "pairs_ms", f"Broad-phase pair time ({title})", f"pairs_{scene}_w{workers}.svg")
print("charts written to", OUT)
