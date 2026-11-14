#!/bin/bash
# In-process micro-benchmarks for the broad-phase tree primitives. Same deterministic
# input for both modes, so hit counts/hashes must match; only the hierarchy differs.
set -e
cd "$(dirname "$0")/.."
gcc -O2 -DNDEBUG -DNO_BULK -I src -I include work/tree_bench.c work/base_build/src/libbox2d.a -lm -lpthread -o /tmp/mb_base
gcc -O2 -DNDEBUG -I src -I include work/tree_bench.c work/opt_build/src/libbox2d.a -lm -lpthread -o /tmp/mb_opt
gcc -O2 -DNDEBUG -I src -I include /tmp/static_micro.c work/opt_build/src/libbox2d.a -lm -lpthread -o /tmp/sm
gcc -O2 -DNDEBUG -I src -I include /tmp/static_query.c work/opt_build/src/libbox2d.a -lm -lpthread -o /tmp/sq
echo "== 10k random proxies (tree_bench) =="
for i in 1 2 3 4 5; do
  echo "-- round $i --"
  taskset -c 2 /tmp/mb_base | grep -E "tree_insert|tree_query|hit_set|tree_rebuild"
  taskset -c 2 /tmp/mb_opt bulk | grep -E "tree_insert|tree_query|hit_set|tree_rebuild"
done
echo "== 3.6k static tile proxies =="
for i in 1 2 3 4 5; do
  taskset -c 2 /tmp/sm; taskset -c 2 /tmp/sm bulk
  taskset -c 2 /tmp/sq; taskset -c 2 /tmp/sq bulk
done
