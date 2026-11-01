# vcu_can.lua SITL 测试（模拟 VCU）

用 Python 扮演底盘 VCU，经 SITL 自带的**组播虚拟 CAN**（总线 0，`239.65.82.0:57732`）和真正的 `vcu_can.lua` 收发帧：VCU 侧发 NavSta / MotionSta，收 Nav_Cmd / HsAck；同时用 MAVLink 控制 SITL 解锁、切模式、上传航线，并读取提示文字和遥测。

## 环境

- Linux / WSL，已编译本仓库的 SITL：`~/ardupilot/build/sitl/bin/ardurover`。必须带 `get_desired_speed/turn_rate` 绑定，否则脚本不启动
- `pip install pymavlink`
- 测试实例用 `-I1`（MAVLink TCP 5770），不影响 `-I0` 的地面站调试实例

## 运行

```bash
bash garlic_rover/sitl/run_all.sh                # 四套测试依次跑，用仓库里的 scripts/vcu_can.lua
bash garlic_rover/sitl/run.sh "" vcu_work_test.py   # 只跑一套
```

每套测试会重新起一次 SITL（`-w` 清参数），先等 EKF 能解锁（约 40 s），板载日志留在 `~/vcu_test/logs/`。

| 文件 | 内容 |
|---|---|
| `vcu_test.py` | 协议 v1.1 回归：上电静默、HsAck、50 ms 周期、WorkFb 0～4 解锁检查和提示、行驶中 WorkFb→1 切 HOLD 且不断流、WorkFb=5 / byte5≠0 整帧无效、HS 2→3 `RC start OK` |
| `vcu_steer_test.py` | 转角按实际车速换算：MotionSta 报指令的 1.5 倍时，v_ref 跟 MotionSta、转角 = atan(w·max(\|v\|,1)·L/v_ref²)；MotionSta 停发时退回 AHRS |
| `vcu_work_test.py` | 作业请求（issue #10）：DO_AUX_FUNCTION 300 升杆 / 收杆、AUTO 中 Work_On 不掉、航线末尾自动收杆、失权清零、注入 WorkFb=4、`VCU_WORK_EN=0` |
| `vcu_drv_test.py` | 协议外状态帧（MotionSta / HealthSta）由 `VCU_FB_CHK` 决定：默认 0 时 Sys_Fault、驱动器故障、Failsafe、MotionSta 停发都不禁止解锁、行驶中不切 HOLD，`VCU_FAULT`=0，`VCU_SYSFLT` / `VCU_DRVFLT` / `VCU_DRVCH` 约 1 Hz，变化时有提示；=1 时 Sys_Fault / Failsafe / 反馈超时禁止解锁、切 HOLD，`VCU_FAULT` 报 Sys_Fault 和 0x100+码，驱动器故障仍不拦 |
| `test.param` | SITL 参数：开脚本、`CAN_P1_DRIVER=1` + `CAN_D1_PROTOCOL=10`（脚本 CAN）、`SIM_CAN_TYPE1=1`（组播），速度 / 转弯半径按实车 |

## 模拟 VCU 的 WorkFb 序列（`vcu_work_test.py` 的 `WorkVcu`）

- Nav_Cmd.Work_On 变化 → WorkFb=1，持续 `LIFT_S`（3 s）→ 升杆到 3、收杆到 0
- `inject_fault = True` 时动杆结束报 4（提升超时故障）
- WorkFb 为 1 或 4 时 MotionSta 车速报 0（行驶锁 0），其余时间报最近一帧 Nav_Cmd 的车速

## 限制

SITL 里车按 ArduPilot 自己的舵机输出在动，**不听 Nav_Cmd**。所以能验证脚本的协议逻辑、状态机和换算公式，验证不了实际车辆行为（如直线是否蛇形），这些要实车测。

## 车辆模型（实车 VCU 特性，2026-11-01）

上面几套测试里车按 ArduPilot 舵机输出动，看不到实车行为。`garlic_plant.py` 用 SITL 的 JSON 外部物理接口提供车辆运动，同时在组播 CAN 上扮演 VCU，车只听 `vcu_can.lua` 的 Nav_Cmd，按实车 bin 测得的特性运动：前轮按 `STEER_RATE`（4.5°/s）追指令、限幅 `STEER_MAX`；有车速指令就跑 `V_RUN`（不随指令大小变，`SPEED_MODE=follow` 时跟随指令）；加速 2.5、减速 4.5 m/s²；有效转角 = 1.1 × 回报 + 1.5°；回报车速封顶 0.85 m/s；解锁前前轮停在 `STEER0`。

| 文件 | 内容 |
|---|---|
| `garlic_plant.py` | 车辆模型 + 模拟 VCU，参数 `key=value`（见文件头） |
| `run_plant.sh` | 起一次：`run_plant.sh <vcu_can.lua> "<模型参数>" <场景> [飞控参数 k=v ...]`，场景 `ab` / `goto` / `abfull` |
| `plant_test.py` | `ab`：AUTO 跑车头方向的直线；`goto`：开到这里。用模型真实位置评估横偏 |
| `plant_ab.py` | `abfull`：按地面站 AB 作业流程（对准任务 → HOLD → 作业任务）；`SKIP_ALIGN=1` 时手动对准后直接作业 |
| `plant_batch.sh` | 批量：用例文件每行五段，用竖线分隔：名称、lua、模型参数、场景、飞控参数 |

例：`bash run_plant.sh ../scripts/vcu_can.lua "STEER_MAX=13 STEER0=-13" ab TURN_RADIUS=5.2`

模型参数 `OS=1` 会设环境变量 `GARLIC_OVERSPEED`，只对 `~/ardupilot` 里加了这个测试开关的 SITL 有效；用本仓库编出的 SITL 时超速追车本来就开着（`Rover/system.cpp`）。模型没有包含实车"前轮朝指令反方向走好几秒"的现象。
