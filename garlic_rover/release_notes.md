大蒜播种车导航控制器固件 **@TAG@**（commit `@SHA@`）。基于 ArduPilot 官方 **Rover-4.7.1** 稳定版，飞控 **X-MAV AP-H743v3**。

## 文件说明

| 文件 | 用途 |
|---|---|
| `garlic-rover-@TAG@-X-MAV-AP-H743v3.apj` | **固件**：Mission Planner → 初始设置 → 安装固件 → **加载自定义固件**（不要点车型图片） |
| `garlic-rover-@TAG@-X-MAV-AP-H743v3_with_bl.hex` | 固件加 Bootloader 的完整镜像，用 STM32CubeProgrammer 整片烧写时用 |
| `X-MAV-AP-H743v3_bl.bin` | Bootloader：板子第一次装 ArduPilot 时，按住 BOOT 键插 USB，用 STM32CubeProgrammer 刷到 `0x08000000` |
| `vcu_can.lua` | 车端脚本：放到 SD 卡的 `APM/scripts/` |
| `garlic_rover.param` | 基础参数：Mission Planner → 全部参数表 → 加载 → 写入 → 重启，**加载、写入两遍** |
| `README.md` | **配置与调参说明**（先看这个） |
| `CAN-protocol.md` | CAN通信协议：与底盘 VCU 的 CAN 协议（CAN2，500 kbit/s） |
| `bench-test-manual.md` | 台架测试手册：刷机、配置、CAN 和 RTK 的逐步测试方法，写到 Mission Planner 按钮级别 |
| `garlic-rover-@TAG@-X-MAV-AP-H743v3.zip` | 以上全部文件打包 |

## 快速上手

1. 刷 `.apj`。板子上还没有 ArduPilot 时，先用 DFU 刷 `X-MAV-AP-H743v3_bl.bin`。
2. 加速度计校准。
3. 加载 `garlic_rover.param` 两遍（每遍都写入并重启）。
4. 把 `vcu_can.lua` 放到 SD 卡的 `APM/scripts/`，重启。
5. 消息栏出现 `VCU: CAN bridge loaded` 即为正常。
6. 飞控 CAN2 接 VCU（只接 CAN_H、CAN_L、GND），RTK GPS 接 GPS 口（SERIAL3）。

## 与官方 Rover-4.7.1 的差异

- 增加 Lua 接口 `AR_AttitudeControl:get_desired_speed()` / `get_desired_turn_rate()`，只加接口，不改导航和控制逻辑。
- 从 ArduPilot master 移植 X-MAV AP-H743v3 的板级定义、板号（1220）和 Bootloader。
- 修复 OSD 空指针：`OSD_TYPE`=0 而 `OSD_TYPE2` 不为 0 时，官方 4.7.1 的 OSD 线程访问空指针，飞控启动即卡死（本板默认 `OSD_TYPE2`=5）。
- 车端 CAN 桥接脚本、基础参数和文档放在 `garlic_rover/` 目录。

> 台架测试时如果临时改过 `VCU_RTK_REQ`、`VCU_FB_TMO`、`FS_CRASH_CHECK`、`LOG_DISARMED`、`FENCE_ENABLE`、`VCU_DBG_HZ`，上车前务必改回 2、200、1、0、1、0。
