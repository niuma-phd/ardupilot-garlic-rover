# 大蒜播种车 Rover 固件（ArduPilot Rover 4.7.1 定制版）

本分支在 ArduPilot 官方 **Rover-4.7.1** 稳定版（commit `dbe7921`）上做了少量修改，用作阿克曼底盘大蒜播种车的导航控制器固件。

- 飞控：**X-MAV AP-H743v3**
- 导航控制器**不直接驱动电机和舵机**。它把导航算出的**目标线速度 v 和目标角速度 ω** 通过 CAN 发给底盘 VCU，由 VCU 自己做速度和转角闭环。
- 地面站：工程师调试用 Mission Planner；客户用定制版 QGC（不在本仓库）。

## 1. 与官方 Rover-4.7.1 的差异

只有下面几处改动，每处一个 commit，可以用 `git log Rover-4.7.1..garlic-rover-4.7` 查看。

| 改动 | 文件 | 说明 |
|---|---|---|
| Lua 绑定 | `libraries/AP_Scripting/generator/description/bindings.desc`、`libraries/AP_Scripting/docs/docs.lua` | 给 `AR_AttitudeControl` 增加 `get_desired_speed()`、`get_desired_turn_rate()` 两个 Lua 接口，让脚本读到导航算出的 v、ω。**只加接口，不改任何导航或控制逻辑** |
| 板号 | `Tools/AP_Bootloader/board_types.txt` | 增加 `AP_HW_X-MAV-AP-H743V3 1220`（从 ArduPilot master 移植） |
| 板级定义 | `libraries/AP_HAL_ChibiOS/hwdef/X-MAV-AP-H743v3/` | 从 master（上游 commit `3cbed584de`）原样移植。4.7.1 稳定版里没有这块板子 |
| Bootloader | `Tools/bootloaders/X-MAV-AP-H743v3_bl.{bin,hex}` | 从 master（上游 commit `82796131d9`）原样移植 |
| 车端脚本、参数、文档 | `garlic_rover/` | VCU 的 CAN 桥接脚本、基础参数、CAN 协议、台架测试手册、本说明 |
| CI | `.github/workflows/` | 去掉上游的 CI，改为只编译本车固件并发布 Release |

这块板子用到的传感器驱动（BMI088、ICM45686、SPL06、QMC5883P）4.7.1 里都有，**移植只增加了板级文件，没有改飞控代码**。

## 2. Release 里的文件

每个 Release（tag 形如 `v4.7.1-garlic.N`）包含：

| 文件 | 用途 |
|---|---|
| `garlic-rover-<版本>-X-MAV-AP-H743v3.apj` | **固件**。用 Mission Planner 的"加载自定义固件"刷入 |
| `garlic-rover-<版本>-X-MAV-AP-H743v3_with_bl.hex` | 固件加 Bootloader 的完整镜像，用于 STM32CubeProgrammer 整片烧写（一般用不到） |
| `X-MAV-AP-H743v3_bl.bin` | Bootloader。板子第一次装 ArduPilot 时用 DFU 刷（见台架测试手册 A0） |
| `vcu_can.lua` | 车端 Lua 脚本，放到 SD 卡的 `APM/scripts/` |
| `garlic_rover.param` | 基础参数，用 Mission Planner 加载 |
| `README.md`（本文件）、`CAN-protocol.md`、`bench-test-manual.md` | 配置说明、与 VCU 的 CAN 协议（即 `docs/CAN通信协议.md`）、台架测试步骤（即 `docs/台架测试手册.md`）。GitHub 不允许附件名含中文，所以 Release 里用英文文件名 |
| `garlic-rover-<版本>-X-MAV-AP-H743v3.zip` | 以上全部文件的打包 |

## 3. 配置步骤（概要）

详细步骤、每一步的预期现象、出错时怎么排查，见 [台架测试手册.md](docs/台架测试手册.md)。

1. **刷固件**
   - 板子上还没有 ArduPilot 时，先按住 BOOT 键插 USB，用 STM32CubeProgrammer 把 `X-MAV-AP-H743v3_bl.bin` 刷到 `0x08000000`；
   - 然后在 Mission Planner 里：初始设置 → 安装固件 → **加载自定义固件**，选 `.apj`；
   - **不要点车型图片**，那样会下载官方固件，本仓库的改动就没了。
2. **加速度计校准**：Rover 不校准就解不了锁。罗盘在参数里已关闭，不用校准。
3. **加载参数**：配置/调试 → 全部参数表 → 加载 `garlic_rover.param` → 写入参数 → 重启，**然后再加载、写入、重启一次**。
   - 原因：`SCR_ENABLE`、`CAN_P2_DRIVER` 是"总开关"类参数，为 0 时同组的其它参数被隐藏，第一遍会提示缺失。
4. **放脚本**：把 `vcu_can.lua` 放到 SD 卡的 `APM/scripts/`。用 Mission Planner 的 MAVFtp，或者用读卡器。
   - 这个文件夹在 `SCR_ENABLE=1`、重启过、并且插着 SD 卡时，才会由飞控自动创建。
5. **确认**：重启后，消息栏出现 `VCU: CAN bridge loaded`。
   - 如果出现 `VCU: fw lacks get_desired binding`，说明刷的是官方固件，要重刷本仓库的固件。
6. **接线**：
   - 飞控 **CAN2** 接 VCU，只接 CAN_H、CAN_L、GND，**不接 5V**，总线两端各接 120 Ω 终端电阻。针脚顺序以飞控底面丝印为准；
   - RTK GPS 接 **SERIAL3（GPS 口）**；数传接 **SERIAL1**；
   - 飞控电源输入 **6–35 V**，底盘电压更高时要加降压模块。

## 4. 车端脚本 `vcu_can.lua` 做什么

- **下发指令**：以 50 Hz 发 `0x101`，内容是目标线速度 v（m/s）和目标角速度 ω（rad/s，**逆时针为正**）。只在 AUTO / GUIDED / RTL / SMART_RTL / LOITER 模式且已解锁时发非零值，其它情况发 0。
- **接收反馈**：解析 VCU 的 `0x201`（运动反馈）和 `0x202`（状态反馈）。遇到以下任一情况，未解锁时**禁止解锁**，行驶中**切 HOLD 停车**并向地面站报警：
  - 反馈超时；
  - VCU 不在自动模式；
  - VCU 急停；
  - VCU 报故障；
  - RTK 没有固定解（掉解超过 3 秒）。
- **上报状态**：以 5 Hz 发 `NAMED_VALUE_FLOAT`（`VCU_OK`、`VCU_MODE`、`VCU_FAULT`、`VCU_BATV`、`VCU_SOC`）给地面站。调试量 `VCU_CMD_V/W`、`VCU_V/W` 默认不发（4G 按流量计费），调试时把 `VCU_DBG_HZ` 设为 5。
- **板载日志**：记录 `VCU` 消息，50 Hz，包含指令 v/ω、底盘实际 v/ω、转角、模式、故障码、定位状态。
- 协议细节见 [CAN通信协议.md](docs/CAN通信协议.md)。**VCU 协议有变化时只改这个脚本，不用改固件。**

脚本自己的参数（脚本运行一次并刷新参数表后才出现）：

| 参数 | 默认 | 含义 |
|---|---|---|
| `VCU_ENABLE` | 1 | 启用 CAN 桥接 |
| `VCU_FB_TMO` | 200 | VCU 反馈超时（ms），0 = 不检查（仅台架调试用） |
| `VCU_V_MAX` / `VCU_W_MAX` | 1.5 / 1.0 | 下发线速度 / 角速度限幅（m/s、rad/s） |
| `VCU_REV` | 0 | 是否允许倒车 |
| `VCU_DBG_HZ` | 0 | 调试量 `VCU_CMD_V/W`、`VCU_V/W` 上报地面站的频率（Hz），0 = 不发；板载日志不受影响 |
| `VCU_RTK_REQ` | 2 | RTK 要求：0 不检查，1 浮点解或固定解，2 只接受固定解 |
| `VCU_RTK_TMO` | 3000 | RTK 不满足要求多久（ms）后停车 |

## 5. 参数与调参

`garlic_rover.param` 里每一项都有注释，这里列出关键的几组。

**固定不要改的**（原因见参数文件注释）：
- `ATC_STR_RAT_P/I/D=0`、`ATC_SPEED_I/D=0`（`ATC_SPEED_P` 取允许的最小值 0.01）：ArduPilot 内部的转向和速度 PID 设成纯前馈。
  - 它们的输出不接执行机构，积分会一直累积到饱和；转向饱和后位置控制器会停止该方向的纠偏，压线变差。
- `WP_PIVOT_ANGLE=0`：阿克曼底盘不能原地转向。
- `EK3_SRC1_YAW=2`、`COMPASS_USE*=0`：航向只用双天线 GPS，不用磁罗盘。
- `CAN_P2_DRIVER=2`、`CAN_D2_PROTOCOL=10`：CAN2 交给 Lua 脚本使用。

**实车需要调的**：

| 类别 | 参数 | 作用 |
|---|---|---|
| 车辆几何 | `TURN_RADIUS` | 最小转弯半径，按底盘实测填写 |
| 速度 | `WP_SPEED`、`CRUISE_SPEED`、`SPEED_MAX`、`RTL_SPEED` | 作业速度、速度上限（与 `VCU_V_MAX` 一致） |
| 加减速 | `ATC_ACCEL_MAX`、`ATC_DECEL_MAX`、`WP_ACCEL`、`WP_JERK` | 下发的 v 变化有多快，要和 VCU 的执行能力匹配 |
| 转向限制 | `ATC_STR_RAT_MAX`、`ATC_STR_ACC_MAX`、`ATC_STR_DEC_MAX`、`ATC_TURN_MAX_G` | ω 的上限和变化率，横向加速度上限 |
| 压线精度 | `PSC_POS_P`、`PSC_VEL_P/I/D`、`WP_RADIUS` | 偏离航线后纠回来的力度；到点判定半径 |
| 定位 | `GPS1_MB_OFS_X/Y/Z`、`GPS1_POS_X/Y/Z`（`GPS1_TYPE=25`、`GPS1_MB_TYPE=1` 已设） | 双天线定向：主天线相对从天线、主天线相对飞控的位置，要量准 |
| CAN | `CAN_P2_BITRATE` | 和 VCU 一致（默认 500000） |
| 保护 | `FS_CRASH_CHECK`、`CRASH_*`、`FS_GCS_*`、`FENCE_*` | 陷车、地面站失联、越界时停车 |

**调参方法**：
1. 先空载低速（0.3 m/s）走直线和矩形。
2. 下载日志，对比 `THR.DesSpeed`、`STER.DesTurnRate`（期望值）和 `VCU.V`、`VCU.W`（底盘实际值）。实际值跟不上期望值时，先让底盘方调 VCU 的闭环。
3. 然后在 RTK 固定解下，逐步调上表"压线精度"一行的参数，看 `NTUN.XTrack`（横向误差）。

> 台架测试时如果临时改过 `VCU_RTK_REQ`、`VCU_FB_TMO`、`FS_CRASH_CHECK`、`LOG_DISARMED`、`FENCE_ENABLE`、`VCU_DBG_HZ`，**上车前务必改回** 2、200、1、0、1、0。

## 6. 自己编译

```bash
git clone --recurse-submodules -b garlic-rover-4.7 https://github.com/niuma-phd/ardupilot-garlic-rover.git
cd ardupilot-garlic-rover
Tools/environment_install/install-prereqs-ubuntu.sh -y   # 首次，需要 sudo
./waf configure --board X-MAV-AP-H743v3
./waf rover
# 产物：build/X-MAV-AP-H743v3/bin/ardurover.apj
```

## 7. CI 与发布

- 推送到 `garlic-rover-4.7` 分支或提交 Pull Request 时，GitHub Actions 会编译固件，产物在该次运行的 Artifacts 里。
- **发布新版本**：打一个 `v4.7.1-garlic.N` 格式的 tag 并推送。CI 编译后自动创建 Release，并上传第 2 节列出的全部文件：
  ```bash
  git tag v4.7.1-garlic.2
  git push origin v4.7.1-garlic.2
  ```
- 要支持新的飞控板，在 `.github/workflows/garlic-rover-firmware.yml` 的 `board` 列表里加板名即可（该板要已在 `libraries/AP_HAL_ChibiOS/hwdef/` 中）。
