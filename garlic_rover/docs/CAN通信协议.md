# Nav（飞控）↔ 底盘 VCU CAN 通信协议 V1.0

适用：大蒜播种车。导航侧为 X-MAV AP-H743v3 飞控（ArduPilot Rover 4.7.2 + Lua 脚本 `vcu_can.lua`），底盘为 VCU。

本文档是甲方《Nav ↔ VCU CAN 通讯协议 v1.0》（2026-10-10 定版，合同附件）的**实现侧摘要**：帧定义、时限、握手规则以甲方原文为准，本文只记录导航侧的实现要点和双方确认的补充约定。甲方原件见项目资料 `01软件资料/03_通信/方案二_Nav直连VCU_甲方v1.0/`。

> 历史版本：V0.1 草案（标准帧 0x101/0x201/0x202、XOR 校验）已作废，由甲方 v1.0 取代。

## 1. 物理层

| 项目 | 约定 |
|---|---|
| 总线 | 整车 **CAN3** 总线；飞控侧用 **CAN2** 口接入（只接 CAN_H、CAN_L、GND，5V 不接） |
| 波特率 | **500 kbit/s**（飞控参数 `CAN_P2_BITRATE`） |
| 帧格式 | **CAN 2.0B 扩展帧（29 位 ID，IDE=1）**，数据帧，DLC 固定 8，无 CAN FD |
| 字节序 | 小端（Intel）；有符号数为 16 位二进制补码 |
| 校验 | 无逐帧校验；非法帧（ID/帧型/DLC/范围/保留位错）直接丢弃，不刷新任何时钟 |
| 节点地址 | Nav（飞控）源地址 0x41，VCU 0x7D，FCU 0x40（参考） |
| 终端电阻 | 总线两端各 120Ω，联调前核对 |

## 2. 坐标与符号

- 线速度 v：前进为正，m/s。
- 角速度 ω：俯视逆时针（左转）为正，rad/s。
- 前轮转角 δ：**左转为正**（2026-10-11 与甲方书面确认），deg。
- Ackermann 换算在 **Nav 侧**完成：δ = atan(ω·L / v)，L 为轴距（参数 `VCU_W_BASE`，默认 2.0 m）；|v| < 0.05 m/s 时保持上次转角，不回正；δ 限幅 min(45°, atan(L/`TURN_RADIUS`) 换算值)。

## 3. 报文列表

| ID | 名称 | 方向 | 周期 / 触发 | 超时 / 时限 |
|---|---|---|---|---|
| 0x180C7D41 | Nav_Cmd | Nav → VCU | 50 ms（授权期间不得断流） | VCU 看门狗 100 ms（固定） |
| 0x180D7D41 | Nav_HsAck | Nav → VCU | 事件应答 | 邀请后 2 s 内，每事务只答一次 |
| 0x180B417D | VCU_NavSta | VCU → 总线 | 100 ms 广播 | Nav 侧监控超时 **定值 500 ms** |
| 0x180A7D40 | FCU_SwapCmd | FCU → VCU | 事件 / 100 ms 重发 | **Nav 不消费** |
| 0x1802407D | VCU_MotionSta | VCU → 总线 | 50 ms | 仅遥测（见 §6） |
| 0x1806407D | VCU_HealthSta | VCU → 总线 | 500 ms | 仅遥测（见 §6） |

### 3.1 Nav_Cmd（0x180C7D41，Nav → VCU）

| 字节 | 信号 | 编码 |
|---|---|---|
| 0–1 | Target_Speed | int16 LE × 0.01 m/s，±5.00（原始 ±500） |
| 2–3 | Target_Steer | int16 LE × 0.01°，±45.00（原始 ±4500），**左正** |
| 4 | Work_On | 0 停 / 1 运行，电平保持 |
| 5–6 | Work_Speed / Work_Dir | **固定发 0**（本产品 workdrv 仅启停，VCU 不消费这两字节） |
| 7 | Seq | 当前授权会话序号，与 NavSta.Seq 一致才被消费 |

被消费的全部条件：帧校验通过 + OpSource=2 + Seq 匹配 + VCU 已因遥控器 Ready 0→1 新边沿进入 RUN。**Nav 永远无法让车自己动起来**。

### 3.2 Nav_HsAck（0x180D7D41，Nav → VCU）

| 字节 | 信号 | 编码 |
|---|---|---|
| 0 | AckType | 1 接受 / 2 拒绝 |
| 1 | Seq | 回显邀请事务序号 |
| 2 | SelfTest | 0 OK；0x05 定位/RTK；0x06 IMU；其余见甲方协议 3.2 编码空间 |
| 3 | Caps | **0x03**（bit0 速度/方向 + bit1 作业电机仅启停） |
| 4–7 | 保留 | 0 |

### 3.3 VCU_NavSta（0x180B417D，VCU → 总线，授权唯一真相）

| 字节 | 信号 | 编码 |
|---|---|---|
| 0 | OpSource | 0 遥控 / 1 切换中 / 2 无人 |
| 1 | HandoverState | 0 空闲 / 1 INVITING / 2 WAIT_START / 3 完成 / 4 超时失败 / 5 Nav 拒绝 / 6 NAV_LOST / 7 RC_LOST |
| 2 | Seq | 当前事务序号回显 |
| 3 | HsErr | 0 无 / 1 Nav 无响应 / 2 自检故障 / 3 停稳超时 / 4 Nav_Cmd 断链 / 5 遥控链断 / 6 未授权帧忽略溢出 |
| 4–7 | 保留 | 0（非零整帧丢弃） |

### 3.4 遥测帧（不参与授权判定）

- **VCU_MotionSta**（0x1802407D，50 ms）：byte0–1 实际速度 ×0.01 m/s；byte2–3 实际转角 ×0.01°；byte5 bit0–3 Veh_State（0 初始化/1 就绪/2 运行/5 故障）、bit5 Failsafe。
- **VCU_HealthSta**（0x1806407D，500 ms）：byte0 SOC %；byte1–2 电压 ×0.1 V（原始 ≤1000）；byte3–5 Drv1/2/3_Fault（供应商故障码，**码表由甲方提供**）；byte6 Sys_Fault（0 无/1 急停/2 链路断/3 FCU 超时/4 电池低温/5 电池高温/6 电量低/7 提升超时）。

## 4. 握手与核心规则（协议 04 章）

1. **上电静默**：Nav 只听 NavSta，不主动发任何帧。
2. **受邀应答**：见 `HandoverState=1`（INVITING）后 2 s 内完成自检并回 HsAck（SelfTest≠0 则视为拒绝），**每个 Seq 只应答一次**。
3. **授权才发**：见 `OpSource=2` 才开始 50 ms 周期发 Nav_Cmd，Seq 与 NavSta 一致。
4. **失权即静默**：`OpSource≠2`、失败态（HS=4/5/6）、或 NavSta 丢失 >500 ms → 立即停发 Nav_Cmd，不自动重试抢权，只能等新邀请。
5. **断流即停车**：授权期间必须持续发帧（包括 v=0 的全零帧）；VCU 100 ms 无有效帧 → 受控停车、HsErr=4、收权。
6. **遥控链断不丢权**：无人态下遥控链断（HS=7，HsErr=5）时 `OpSource` 保持 2，VCU 闭锁输出停车等恢复，**Nav 必须继续发帧不断流**。
7. **启动边沿制**：每次授权/恢复后，车辆只能由遥控器 Ready 0→1 **新边沿**启动；Nav 无启动/停止/急停权限，急停仅硬件回路。

## 5. Nav 侧实现（`vcu_can.lua`）

- 主循环 10 ms，保证 50 ms 发帧抖动 ≤10 ms；授权沿立即发第一帧。
- 自检顺序：IMU 健康（不过→0x06）→ RTK 达 `VCU_RTK_REQ`（不过→0x05）→ 通过（0）。
- 解锁前检查（任一不满足禁止解锁）：NavSta 新鲜、`OpSource=2`、MotionSta 新鲜且无 Failsafe、HealthSta.Sys_Fault=0、RTK 达标。
- 行驶中失效（失权/反馈异常/RTK 掉解超 `VCU_RTK_TMO`）→ 切 HOLD；**只要授权仍在就持续发 v=0 的 Nav_Cmd 不断流**（2026-10-11 双方确认）。
- 作业电机：`VCU_WORK_EN=1` 时，AUTO 模式行驶中（v>0.02 m/s）自动置 Work_On=1，其余为 0。
- 脚本崩溃 → 静默（VCU 看门狗收权停车，安全方向），约 1 s 后自动重启，重新等邀请。
- 下发限幅：|v| ≤ min(`VCU_V_MAX`, 5.0)；|δ| ≤ §2 限幅。倒车默认关闭（`VCU_REV=0`）。

## 6. 遥测与地面站上报

MotionSta / HealthSta 仅用于板载日志和地面站遥测，**授权判定只认 NavSta**。5 Hz 上报：

| 名称 | 含义 |
|---|---|
| `VCU_MODE` | 0 NavSta 无效；1 遥控/切换中；2 无人 |
| `VCU_FAULT` | Sys_Fault 1~7 直报；HsErr 报 0x10+码；驱动器故障报 0x100+码 |
| `VCU_BATV` / `VCU_SOC` | HealthSta 电压 / 电量（仅在 HealthSta 1500 ms 内新鲜时发送，否则不发） |
| `VCU_OK` | 综合就绪位（授权 + 反馈正常 + RTK 保持） |

板载日志 `VCU` 消息（50 Hz）：`CV,CD,AV,AD,Op,HS,Er,VS,FS,Ok,Fix` —— 指令 v/转角、实际 v/转角、OpSource、HandoverState、HsErr、Veh_State、Failsafe、就绪位、定位状态。

## 7. 联调检查表

- [ ] 波特率 500 k、终端电阻、扩展帧确认；总线上能看到 100 ms 周期的 0x180B417D
- [ ] 上电后 Nav 不发任何帧（静默）；VCU 模拟 INVITING 后 2 s 内收到 HsAck `01 <Seq> 00 03 00 00 00 00`
- [ ] OpSource=2 后 Nav_Cmd 以 50 ms 周期发出，Seq 一致；OpSource 切回后 Nav 立即停发
- [ ] Nav_Cmd 停发 100 ms：VCU 受控停车、HsErr=4、收权
- [ ] **Target_Steer 符号实测（联调首项）**：正转角 → 车轮向左；与 MotionSta 反馈符号一致
- [ ] 遥控器 Ready 0→1 边沿前 Nav_Cmd 不生效；HS=7 时 Nav 持续发帧、车辆保持静止
- [ ] NavSta 拔线 500 ms：Nav 静默；恢复后等新邀请，不自动抢权
- [ ] 实测从最后有效 Nav_Cmd 到车辆停稳的最大时间/距离（甲方协议安全验证边界）
