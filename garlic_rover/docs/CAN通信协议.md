# Nav（飞控）↔ 底盘 VCU CAN 通信协议 V1.1

适用：大蒜播种车。导航侧为 X-MAV AP-H743v3 飞控（ArduPilot Rover 4.7.2 + Lua 脚本 `vcu_can.lua`），底盘为 VCU。

本文档是甲方《Nav ↔ VCU CAN 通讯协议 v1.1》（2026-10-30；v1.0 为 2026-10-10 定版的合同附件）的**实现侧摘要**：帧定义、时限、握手规则以甲方原文为准，本文只记录导航侧的实现要点和双方确认的补充约定。甲方原件见项目资料 `01软件资料/03_通信/Nav_VCU_CAN通讯协议_v1.1.html`，变更说明见同目录 `Nav供应商_协议v1.1变更说明与联调事项.html`。

> v1.1 相对 v1.0：NavSta byte4 由保留改为 **WorkFb**（作业状态反馈，见 §3.3）；Target_Steer 左正书面化；Work_On 作业序列语义明确（升杆→作业，降杆→运输）；握手第 ⑤ 步 VCU 侧已对齐。线格式与 v1.0 字节级兼容（WorkFb 激活前 byte4 恒为 0）。

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
- 前轮转角 δ：**左转为正**（2026-10-11 双方确认，v1.1 协议 3.1 书面约定），deg。
- Ackermann 换算在 **Nav 侧**完成，按**实际车速**换算：δ = atan(a·L / v_ref²)，L 为等效轴距（参数 `VCU_W_BASE`，默认 1.2 m）。
  - 农机 VCU 自己定车速（运输约 5 km/h、作业约 2 km/h），只认走 / 停，实际车速常和 Target_Speed 不同（10-30 台架：指令 0.3～0.6 m/s，实际约 0.85 m/s）。
  - a 为导航要的横向加速度。Rover 4.7.2 航点导航（`AR_PosControl`）给出的期望转弯角速度是 ω = a / max(|v_des|, 1.0)，v_des 为**期望**车速、下限 1.0 m/s（`AR_ATTCONTROL_STEER_SPEED_MIN`），所以脚本先还原 a = ω·max(|v_des|, 1.0)。升级固件时要核对这个常数。
  - v_ref：VCU_MotionSta 200 ms 内新鲜时用其实际速度；否则用 AHRS 前向速度（EKF 地速投到航向上）。下限 `VCU_VREF_MIN`（默认 0.3 m/s），防止起步时转角乱跳。
  - 指令 |v| < 0.05 m/s 时保持上次转角，不回正；δ 限幅 min(45°, atan(L/`TURN_RADIUS`) 换算值)。

## 3. 报文列表

| ID | 名称 | 方向 | 周期 / 触发 | 超时 / 时限 |
|---|---|---|---|---|
| 0x180C7D41 | Nav_Cmd | Nav → VCU | 50 ms（授权期间不得断流） | VCU 看门狗 100 ms（固定） |
| 0x180D7D41 | Nav_HsAck | Nav → VCU | 事件应答 | 邀请后 2 s 内，每事务只答一次 |
| 0x180B417D | VCU_NavSta | VCU → 总线 | 100 ms 广播 | Nav 侧监控超时 **定值 500 ms** |
| 0x180A7D40 | FCU_SwapCmd | FCU → VCU | 事件 / 100 ms 重发 | **Nav 不消费** |
| 0x1802407D | VCU_MotionSta | VCU → 总线 | 50 ms | 不参与授权；参与解锁检查和行驶中保护（`VCU_FB_TMO`，默认 200 ms，见 §3.4） |
| 0x1806407D | VCU_HealthSta | VCU → 总线 | 500 ms | 不参与授权；Sys_Fault 参与解锁检查和行驶中保护（见 §3.4） |

### 3.1 Nav_Cmd（0x180C7D41，Nav → VCU）

| 字节 | 信号 | 编码 |
|---|---|---|
| 0–1 | Target_Speed | int16 LE × 0.01 m/s，±5.00（原始 ±500） |
| 2–3 | Target_Steer | int16 LE × 0.01°，±45.00（原始 ±4500），**正值=前轮左转**（v1.1 书面约定） |
| 4 | Work_On | 0 停 / 1 运行，电平保持。0→1：升杆→到位→作业（限速 2 km/h）；1→0：停作业→降杆→运输（限速 5 km/h），进度经 NavSta.WorkFb 反馈。由地面站 / 航线请求（见 §5），`VCU_WORK_EN=0`（本期默认）时恒发 0 |
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
| 4 | WorkFb（v1.1） | 0 运输就绪（行走 ≤5 km/h）/ 1 升/降杆中（行驶锁 0）/ 2 作业位就绪 / 3 作业中（行走 ≤2 km/h）/ 4 提升超时故障（行驶锁 0）；**>4 整帧丢弃** |
| 5–7 | 保留 | 0（非零整帧丢弃） |

WorkFb 自 VCU 公告激活起生效，激活前恒发 0（"反馈未激活"，不代表已确认运输位）。断链时 VCU 受控停车但提杆保持原位，WorkFb 如实上报。

### 3.4 状态帧（不参与授权判定，参与解锁检查和行驶中保护）

- **VCU_MotionSta**（0x1802407D，50 ms）：byte0–1 实际速度 ×0.01 m/s；byte2–3 实际转角 ×0.01°；byte5 bit0–3 Veh_State（0 初始化/1 就绪/2 运行/5 故障）、bit5 Failsafe。超过 `VCU_FB_TMO` 未收到或 Failsafe=1 → 禁止解锁 / 行驶中切 HOLD。
- **VCU_HealthSta**（0x1806407D，500 ms）：byte0 SOC %；byte1–2 电压 ×0.1 V（原始 ≤1000）；byte3–5 Drv1/2/3_Fault（供应商故障码，**码表由甲方提供**）；byte6 Sys_Fault（0 无/1 急停/2 链路断/3 FCU 超时/4 电池低温/5 电池高温/6 电量低/7 提升超时）。HealthSta 800 ms 内新鲜且 Sys_Fault≠0 → 禁止解锁 / 行驶中切 HOLD。**驱动器故障 Drv1/2/3 只上报（§6 `VCU_DRVFLT`），不参与解锁检查和行驶中保护**，由 VCU 自己决定能不能走。
- `VCU_FB_TMO=0`（台架调试）时以上两帧都不检查。

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
- 解锁前检查（按此顺序，任一不满足禁止解锁）：NavSta 新鲜、`OpSource=2`、WorkFb 不为 1（`lift moving`）/4（`lift fault`）、MotionSta 新鲜且无 Failsafe、HealthSta.Sys_Fault=0、RTK 达标。
- 行驶中失效（失权/WorkFb 变为 1 或 4/反馈异常/RTK 掉解超 `VCU_RTK_TMO`）→ 切 HOLD，提示 `VCU: <原因>, HOLD`（2 s 限频）；**只要授权仍在就持续发 v=0 的 Nav_Cmd 不断流**（2026-10-11 双方确认）。
- 握手提示：HS→2 `VCU: wait RC start`；HS→3 `VCU: RC start OK`（遥控器已按 Ready）；HS→4/5 `VCU: handover failed`；HS→7 `VCU: RC link lost`。
- WorkFb 提示：→1 `VCU: lift moving`；→4 `VCU: lift fault`；4→其他 `VCU: lift fault cleared`。
- 驱动器故障提示（普通级别 NOTICE，不带 PreArm 前缀，不影响解锁）：变为非 0 或换码 `VCU: drive fault 0x%02X Drv<路号>`（如 `VCU: drive fault 0x01 Drv1`）；变回 0 `VCU: drive fault cleared`（INFO）。
- 作业（Work_On）：`VCU_WORK_EN=1` 且已授权时，跟随 RC 辅助功能 **SCRIPTING1（300）** 的电平请求，不再跟 AUTO 模式和车速绑定（否则 VCU 动杆时行驶锁 0 → 切 HOLD → Work_On 掉 0 → 收杆，反复升降）。
  - 地面站下发 `MAV_CMD_DO_AUX_FUNCTION`（param1=300，param2=2 升杆作业 / 0 收杆）；航线末尾也放一条 `DO_AUX_FUNCTION 300 低`，作为收杆兜底（4G 断开时也能收杆）。
  - AB 线单条：AUTO 到对准点停 → 请求 2，等 WorkFb=3（=4 为故障）→ AUTO 起点到终点 → 航线末尾请求 0，等 WorkFb=0。
  - 失权（OpSource≠2 / NavSta 丢失）时脚本把请求清零并提示 `VCU: work request cleared`；恢复授权后保持 0，由地面站重新下发（协议：断链后提杆保持原位，由 Nav 重新决策）。
  - 升 / 降杆期间 WorkFb=1，车照样切 HOLD、持续发 v=0。Work_On 变化后 30 s 内的升降杆，以及静止时的升降杆，都是作业流程，**不报** `VCU: lift moving, HOLD`；行驶中出现非预期的 WorkFb=1/4 仍报警。
- 脚本崩溃 → 静默（VCU 看门狗收权停车，安全方向），约 1 s 后自动重启，重新等邀请。
- 下发限幅：|v| ≤ min(`VCU_V_MAX`, 5.0)；|δ| ≤ §2 限幅。倒车默认关闭（`VCU_REV=0`）。

## 6. 遥测与地面站上报

**授权判定只认 NavSta**；MotionSta / HealthSta 除板载日志和地面站遥测外，还参与解锁检查和行驶中保护（§3.4、§5）。5 Hz 上报：

| 名称 | 含义 |
|---|---|
| `VCU_MODE` | 0 NavSta 无效；1 遥控/切换中；2 无人 |
| `VCU_FAULT` | Sys_Fault 1~7 直报；HsErr 报 0x10+码（优先级 Sys_Fault > HsErr）。**garlic.16 起驱动器故障默认不进 VCU_FAULT**，改报 `VCU_DRVFLT`；`VCU_DRV_CHK=1` 时仍按 garlic.15 报 0x100+码（优先级最低） |
| `VCU_BATV` / `VCU_SOC` | HealthSta 电压 / 电量（仅在 HealthSta 1500 ms 内新鲜时发送，否则不发） |
| `VCU_OK` | 综合就绪位（授权 + WorkFb 非 1/4 + 反馈正常 + RTK 保持） |

`VCU_WORKFB`（NavSta.WorkFb 0～4）不在 5 Hz 里：变化时立即发一次，之后 1 Hz（NavSta 无效时不发）。

`VCU_DRVFLT` / `VCU_DRVCH`（garlic.16 起）：驱动器故障码 = Drv1→Drv2→Drv3 中第一个非 0 的码（0 无故障），路号 1～3（0 无故障）。变化时立即发一次，之后 1 Hz；HealthSta 1500 ms 内不新鲜时不发。码表由甲方提供。只上报不拦车。

`VCU_WREQ`（作业请求 0/1，即 SCRIPTING1 是否为高）：变化时立即发；`VCU_WORK_EN=1` 时随 `VCU_WORKFB` 1 Hz 补发。

板载日志 `VCU` 消息（50 Hz）：`CV,CD,AV,AD,Op,HS,Er,VS,FS,Ok,Fix,WF,VR,AS` —— 指令 v/转角、实际 v/转角、OpSource、HandoverState、HsErr、Veh_State、Failsafe、就绪位、定位状态、WorkFb、转角换算用车速 v_ref、AHRS 前向速度。AV 明显大于 AS 说明打滑（MotionSta 车速偏大），这时应考虑改用 AHRS 车速。

板载日志 `VCUW` 消息（50 Hz，Lua 日志单条最多 14 列所以单独记）：`WR,WO,SF,DF,DC` —— 作业请求、实际下发的 Work_On、HealthSta Sys_Fault、驱动器故障码及路号（同 `VCU_DRVFLT` / `VCU_DRVCH`）。

调试量（`VCU_DBG_HZ` > 0 时上报）：`VCU_CMD_V/W/D`、`VCU_VREF`、`VCU_V/W`。

### 6.1 故障码（`VCU_FAULT` / `VCU_DRVFLT`）

| 遥测 | 取值 | 来源 | 拦不拦车 |
|---|---|---|---|
| `VCU_FAULT` | 0 | 无故障 | — |
| `VCU_FAULT` | 0x01～0x07 | HealthSta.Sys_Fault 原值（1 急停 / 2 链路断 / 3 FCU 超时 / 4 电池低温 / 5 电池高温 / 6 电量低 / 7 提升超时） | 拦：禁止解锁（`PreArm: VCU: fault 0x0N`）、行驶中切 HOLD |
| `VCU_FAULT` | 0x11～0x16 | 0x10 + NavSta.HsErr（1 Nav 无响应 / 2 自检故障 / 3 停稳超时 / 4 Nav_Cmd 断链 / 5 遥控链断 / 6 未授权帧溢出） | 飞控不单独拦（VCU 会同时收权，按失权处理） |
| `VCU_DRVFLT` | 驱动器故障码（`VCU_DRVCH` 1～3 为哪一路） | HealthSta Drv1/2/3_Fault，取第一个非 0 | **飞控不拦**，只上报 + 提示 `VCU: drive fault 0x..`，由 VCU 决定能不能走 |
| `VCU_FAULT`（仅 `VCU_DRV_CHK=1`） | 0x100 + 驱动器故障码 | 同上 | 飞控不拦；地面站见 `VCU_FAULT`≠0 禁止开车 |

优先级 Sys_Fault > HsErr。garlic.15 及以前驱动器故障报为 `VCU_FAULT`=0x100+码，地面站见 `VCU_FAULT`≠0 即禁止开始作业 / 开到这里，garlic.16 起默认移出。

`VCU_DRV_CHK`（参数序号 12，garlic.16 新增）：0（默认）只上报；1 同时计入 `VCU_FAULT`。参数表由 11 扩到 12，**换上 garlic.16 脚本后要重启飞控**，只重启脚本会报 `could not add param table`（表大小上电后固定）；存量参数保留，回退旧脚本也不受影响。

## 7. 联调检查表

- [ ] 波特率 500 k、终端电阻、扩展帧确认；总线上能看到 100 ms 周期的 0x180B417D
- [ ] 上电后 Nav 不发任何帧（静默）；VCU 模拟 INVITING 后 2 s 内收到 HsAck `01 <Seq> 00 03 00 00 00 00`
- [ ] OpSource=2 后 Nav_Cmd 以 50 ms 周期发出，Seq 一致；OpSource 切回后 Nav 立即停发
- [ ] Nav_Cmd 停发 100 ms：VCU 受控停车、HsErr=4、收权
- [ ] **Target_Steer 符号实测（联调首项）**：正转角 → 车轮向左；与 MotionSta 反馈符号一致
- [ ] 遥控器 Ready 0→1 边沿前 Nav_Cmd 不生效；HS=7 时 Nav 持续发帧、车辆保持静止
- [ ] NavSta 拔线 500 ms：Nav 静默；恢复后等新邀请，不自动抢权
- [ ] WorkFb：byte4=0/2/3 NavSta 有效可解锁；=1/4 解锁失败并提示；=5 或 byte5≠0 整帧丢弃（500 ms 后 NavSta lost）
- [ ] 行驶中 WorkFb→1：切 HOLD，Nav_Cmd 仍 50 ms 周期发 v=0
- [ ] 实测从最后有效 Nav_Cmd 到车辆停稳的最大时间/距离（甲方协议安全验证边界）
