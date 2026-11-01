#!/usr/bin/env bash
# 批量跑车辆模型场景: plant_batch.sh <用例文件>
# 用例文件每行: 名称|vcu_can.lua|plant 参数|场景|飞控参数...
HERE="$(cd "$(dirname "$0")" && pwd)"
while IFS='|' read -r name lua pargs scene fargs; do
  [ -z "$name" ] && continue
  case "$name" in \#*) continue;; esac
  echo "=== $name"
  bash "$HERE/run_plant.sh" "$lua" "$pargs" "$scene" $fargs | grep -E "RESULT|PROFILE|\[text\] (VCU|Crash)"
  cp ~/plant_test/plant.log ~/plant_runs/"$name".log 2>/dev/null
done < "$1"
