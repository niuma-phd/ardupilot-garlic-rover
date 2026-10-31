# vcu_can.lua SITL 测试（模拟 VCU）

用 Python 扮演底盘 VCU，经 SITL 自带的**组播虚拟 CAN**（总线 0，`239.65.82.0:57732`）和真正的 `vcu_can.lua` 收发帧：VCU 侧发 NavSta / MotionSta，收 Nav_Cmd / HsAck；同时用 MAVLink 控制 SITL 解锁、切模式、上传航线，并读取提示文字和遥测。

## 环境

- Linux / WSL，已编译本仓库的 SITL：`~/ardupilot/build/sitl/bin/ardurover`。必须带 `get_desired_speed/turn_rate` 绑定，否则脚本不启动
- `pip install pymavlink`
- 测试实例用 `-I1`（MAVLink TCP 5770），不影响 `-I0` 的地面站调试实例

## 运行

```bash
bash garlic_rover/sitl/run_all.sh                # 三套测试依次跑，用仓库里的 scripts/vcu_can.lua
bash garlic_rover/sitl/run.sh "" vcu_work_test.py   # 只跑一套
```

每套测试会重新起一次 SITL（`-w` 清参数），先等 EKF 能解锁（约 40 s），板载日志留在 `~/vcu_test/logs/`。

| 文件 | 内容 |
|---|---|
| `vcu_test.py` | 协议 v1.1 回归：上电静默、HsAck、50 ms 周期、WorkFb 0～4 解锁检查和提示、行驶中 WorkFb→1 切 HOLD 且不断流、WorkFb=5 / byte5≠0 整帧无效、HS 2→3 `RC start OK` |
| `vcu_steer_test.py` | 转角按实际车速换算：MotionSta 报指令的 1.5 倍时，v_ref 跟 MotionSta、转角 = atan(w·max(\|v\|,1)·L/v_ref²)；MotionSta 停发时退回 AHRS |
| `vcu_work_test.py` | 作业请求（issue #10）：DO_AUX_FUNCTION 300 升杆 / 收杆、AUTO 中 Work_On 不掉、航线末尾自动收杆、失权清零、注入 WorkFb=4、`VCU_WORK_EN=0` |
| `test.param` | SITL 参数：开脚本、`CAN_P1_DRIVER=1` + `CAN_D1_PROTOCOL=10`（脚本 CAN）、`SIM_CAN_TYPE1=1`（组播），速度 / 转弯半径按实车 |

## 模拟 VCU 的 WorkFb 序列（`vcu_work_test.py` 的 `WorkVcu`）

- Nav_Cmd.Work_On 变化 → WorkFb=1，持续 `LIFT_S`（3 s）→ 升杆到 3、收杆到 0
- `inject_fault = True` 时动杆结束报 4（提升超时故障）
- WorkFb 为 1 或 4 时 MotionSta 车速报 0（行驶锁 0），其余时间报最近一帧 Nav_Cmd 的车速

## 限制

SITL 里车按 ArduPilot 自己的舵机输出在动，**不听 Nav_Cmd**。所以能验证脚本的协议逻辑、状态机和换算公式，验证不了实际车辆行为（如直线是否蛇形），这些要实车测。
