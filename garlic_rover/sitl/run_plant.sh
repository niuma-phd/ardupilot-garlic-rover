#!/usr/bin/env bash
# 车辆模型 SITL 一次: run_plant.sh <vcu_can.lua> "<plant 参数>" <场景> [飞控参数 k=v ...]
# 例: run_plant.sh scripts/vcu_can.lua "STEER0=-25" ab ATC_STR_ACC_MAX=10
HERE="$(cd "$(dirname "$0")" && pwd)"
LUA="$(realpath "$1")"; PLANT_ARGS="$2"; SCENE="$3"; shift 3
case " $PLANT_ARGS " in *" OS=1 "*) export GARLIC_OVERSPEED=1;; *) unset GARLIC_OVERSPEED;; esac
pkill -f "ardurover.*-I1" 2>/dev/null; pkill -f garlic_plant.py 2>/dev/null; sleep 0.5
rm -f /tmp/plant_go /tmp/plant_vrun; rm -rf ~/plant_test && mkdir -p ~/plant_test/scripts && cd ~/plant_test
cp "$LUA" scripts/vcu_can.lua
python3 -u "$HERE/garlic_plant.py" $PLANT_ARGS LOG=$HOME/plant_test/plant.log > plant.out 2>&1 &
PP=$!
~/ardupilot/build/sitl/bin/ardurover --model JSON:127.0.0.1 --speedup 1 -w \
  --defaults ~/ardupilot/Tools/autotest/default_params/rover.parm,"$HERE/test.param" \
  -I1 --home 31.93778228,118.7901755,13.225,0 > sitl.log 2>&1 &
P=$!
sleep 3
if [ "$SCENE" = abfull ]; then
  timeout 420 python3 -u "$HERE/plant_ab.py" ~/plant_test/plant.log "$@" 2>&1 | grep --line-buffered -v "EOF on TCP"
else
  timeout 240 python3 -u "$HERE/plant_test.py" "$SCENE" ~/plant_test/plant.log "$@" 2>&1 | grep --line-buffered -v "EOF on TCP"
fi
kill -9 $P $PP 2>/dev/null; wait $P $PP 2>/dev/null; sleep 1
