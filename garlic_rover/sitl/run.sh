#!/usr/bin/env bash
# 一次性: 起测试 SITL (-I1, 组播 CAN) -> 跑 vcu_test.py -> 停 SITL, 留下板载日志
HERE="$(cd "$(dirname "$0")" && pwd)"
LUA="${1:-$HERE/../scripts/vcu_can.lua}"
pkill -f "ardurover.*-I1" 2>/dev/null
rm -rf ~/vcu_test && mkdir -p ~/vcu_test/scripts && cd ~/vcu_test
cp "$LUA" scripts/vcu_can.lua
~/ardupilot/build/sitl/bin/ardurover --model rover --speedup 1 -w \
  --defaults ~/ardupilot/Tools/autotest/default_params/rover.parm,"$HERE/test.param" \
  -I1 --home 31.93778228,118.7901755,13.225,343.28 > sitl.log 2>&1 &
P=$!
sleep 4
timeout 300 python3 -u "$HERE/${2:-vcu_test.py}" 2>&1 | grep --line-buffered -v "EOF on TCP" | head -200
kill $P 2>/dev/null; wait $P 2>/dev/null
ls -la ~/vcu_test/logs 2>/dev/null
