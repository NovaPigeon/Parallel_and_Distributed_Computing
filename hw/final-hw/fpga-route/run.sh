#!/usr/bin/env bash
result_dir=$1

list=("boom_med_pb" "boom_soc_v2" "corescore_500" "corescore_1700" "mlcad_d181" "vtr_mcml")

# 遍历列表并输出每个元素
for item in "${list[@]}"; do
  echo "$item"
  make run-bench BENCHMARKS=$item CONFIG=config.cfg RESULT_DIR=$result_dir
done
