#!/usr/bin/env bash
# 依次跑全部 SITL 测试, 只输出结果行
HERE="$(cd "$(dirname "$0")" && pwd)"
for t in vcu_work_test.py vcu_steer_test.py vcu_test.py vcu_drv_test.py; do
  echo "##### $t"
  bash "$HERE/run.sh" "${1:-$HERE/../scripts/vcu_can.lua}" "$t" | grep -E '^(PASS|FAIL)|PASS$|VCU: error'
done
