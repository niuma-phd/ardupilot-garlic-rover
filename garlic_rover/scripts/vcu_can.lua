--[[
 vcu_can.lua  大蒜播种车 Nav(飞控) <-> 底盘VCU  CAN 桥接脚本
 固件: ArduPilot Rover 4.7.2 + 补丁 (飞控 X-MAV AP-H743v3)

 协议: 《Nav <-> VCU CAN 通讯协议 v1.1》(2026-10-30, 甲方; v1.0 为 2026-10-10 定版)
   总线 CAN3, 500 kbit/s, 29 位扩展帧 (IDE=1), 经典 CAN, DLC=8, Intel 小端
   Nav 源地址 0x41, VCU 源地址 0x7D
   Nav_Cmd     0x180C7D41  Nav -> VCU  50ms 周期 (授权期间不得断流, VCU 看门狗 100ms)
   Nav_HsAck   0x180D7D41  Nav -> VCU  邀请 (HandoverState=1) 后 2s 内应答, 每事务只答一次
   VCU_NavSta  0x180B417D  VCU -> 总线 100ms 广播, 授权唯一真相, Nav 监控超时定值 500ms
     byte4 WorkFb (v1.1): 0 运输就绪 / 1 升降杆中 / 2 作业位就绪 / 3 作业中 / 4 提升超时故障,
     >4 整帧无效; byte5~7 保留. VCU 公告激活前恒发 0.
     WorkFb=1/4 (行驶锁 0) 时禁止解锁, 行驶中切 HOLD 并持续发 v=0 (静止时升/降杆不报警)
   Work_On: VCU_WORK_EN=1 且已授权时, 跟随 SCRIPTING1 (RC 辅助功能 300) 的电平请求;
     地面站 / 航线用 MAV_CMD_DO_AUX_FUNCTION(300, 2=升杆作业 / 0=收杆) 下发. 失权时请求清零
   FCU_SwapCmd 0x180A7D40  参考帧, Nav 不消费
   监听帧 (协议外, 不参与 VCU 授权判定, 授权只看 NavSta):
   VCU_MotionSta 0x1802407D  50ms
   VCU_HealthSta 0x1806407D  500ms
     VCU_FB_CHK=0 (默认): 只上报, 不拦车, 由 VCU 自己决定能不能走. Sys_Fault 报 VCU_SYSFLT, 驱动器故障报
       VCU_DRVFLT/VCU_DRVCH, 变化时提示; VCU_FAULT 只含 HsErr, VCU_OK 不受影响
     VCU_FB_CHK=1: 反馈超时 / Failsafe / Sys_Fault 非 0 时禁止解锁, 行驶中切 HOLD 并持续发 v=0 (VCU_FB_TMO=0
       时不检查); Sys_Fault 和驱动器故障 (0x100+码) 计入 VCU_FAULT, 地面站据此禁止开车 (garlic.15 做法).
       驱动器故障飞控始终不拦

 核心规则 (协议 04 章):
   1. 上电静默: 只听 NavSta, 不主动发任何帧
   2. 受邀应答: 见 INVITING 后 2s 内自检并回 HsAck (SelfTest/Caps=0x03), 每 Seq 只答一次
   3. 授权才发: 见 OpSource=2 才开始 50ms 周期发 Nav_Cmd, Seq 与 NavSta.Seq 一致
   4. 失权静默: OpSource!=2 / 失败态 / NavSta 丢 500ms -> 立即停发, 不抢权, 等新邀请
   5. 断流即停车: 授权期间必须持续发帧 (含 v=0), 断流 >100ms 触发 HsErr=4 收权
   6. Nav 无启动/急停权限: 车辆只能由遥控器 Ready 0->1 新边沿启动, 急停仅硬件回路

 我方自身保护触发 (HOLD / RTK 掉解 / 升降杆) 时持续发 v=0 的 Nav_Cmd, 不断流, 授权保留
 (2026-10-11 双方确认)

 运动学: Ackermann, a_lat = v^2*tan(delta)/L -> delta = atan(a_lat*L/v_ref^2), v_ref 为实际车速
   农机 VCU 自己定车速 (运输约 5km/h, 作业约 2km/h), 只认走/停, 实际车速常与指令不同.
   Rover 4.7.2 航点导航给的 w = a_lat / max(|v_des|, 1.0) (AR_PosControl, v_des 为期望车速),
   这里先还原 a_lat, 再按实际车速换算转角.
   v_ref: VCU_VREF_SRC=1 (默认) 用 AHRS 地速投到航向 (RTK); =0 时 MotionSta 新鲜就用 VCU 回报车速
     (实车 VCU 回报车速封顶约 0.85 m/s, 实际 1.4~1.7 m/s, 用它换算转角会大 3~4 倍); 下限 VCU_VREF_MIN
   Target_Steer 正值 = 前轮左转 (v1.1 书面约定), 与 w 逆时针为正天然一致
   delta 限幅 min(45deg, deg(atan(L/TURN_RADIUS))), 指令 |v|<0.05m/s 时保持上次转角

 前轮转向慢 (实车约 4.5 deg/s, 起步约 1 s 即到 1 m/s 以上, 2026-11-01 bin) 的适配, 只看转角不看车速:
   起步回正 VCU_STR_TOL: 车静止要起步时先把前轮回正, 实际转角 (MotionSta) 偏离正中超过此值就发 v=0 等前轮;
     只在起步时检查, 车开起来以后中途停顿不再回正 (车速指令归 0 后才算重新起步)
   停车回正 VCU_STR_PARK: 不在行驶状态时转角指令回 0, 下次起步不带着上次的转角
   零位补偿 VCU_STR_OFS: 叠加到下发的转角上 (bin 反算 VCU 报 0 时前轮实际偏左约 1.5 度, 需实车标定)
   小步增量 VCU_STR_STEP: VCU 转向按"锁定目标"执行 (转到目标或机械极限后才接受新指令, 甲方 11-02 确认),
     直接发期望转角时目标一旦远或到不了, 前轮要单向转好几秒才换指令 -> 起步绕圈 (issue #11).
     下发转角 = VCU 回报实际转角 + 朝期望方向最多 VCU_STR_STEP 度, 目标总能很快到位, 等效连续跟随;
     MotionSta 不新鲜时直接发期望转角. 日志 VCU.CD 为实际下发值, VCUW.DD 为期望值 (含零位补偿)

 所需参数(与 garlic_rover/params/garlic_rover.param 一致):
   SCR_ENABLE=1  CAN_P2_DRIVER=2  CAN_D2_PROTOCOL=10  CAN_P2_BITRATE=500000
   注: 甲方协议里的 "CAN3" 是整车总线名称; 飞控侧用 CAN2 口接入该总线(按实际接线调整)

 v/w 来源: AR_AttitudeControl:get_desired_speed()/get_desired_turn_rate()
   需要本仓库编译的固件 (Release 中的 *.apj; 官方固件没有这两个绑定), 官方固件下脚本不启动

 注: STATUSTEXT 只有50字节, 故脚本内提示文字用英文, 由QGC定制版翻译显示
--]]

local LOOP_MS = 10            -- 主循环 10ms, 保证 50ms 发帧抖动 <=10ms
local TX_PERIOD_MS = 50       -- Nav_Cmd 发送周期 (协议定值)
local NAVSTA_TMO_MS = 500     -- NavSta 监控超时 (协议定值)
local MOTION_FRESH_MS = 200   -- MotionSta 车速用于转角换算的新鲜度
local ATC_STEER_SPEED_MIN = 1.0  -- = AR_ATTCONTROL_STEER_SPEED_MIN (Rover 4.7.2), 升级固件时核对
local AUX_WORK = 300          -- 作业请求: RC 辅助功能 SCRIPTING1, 地面站/航线用 DO_AUX_FUNCTION 置高/低
local WORK_LIFT_EXPECT_MS = 30000  -- Work_On 变化后多久内的升/降杆算我方请求的 (VCU 动杆超时 10~15 s)

-- 脚本版本: 仓库里为 "dev", CI 打包 Release 时替换为 tag 名 (如 garlic.9)
local SCRIPT_VERSION = "dev"

-- MAV_SEVERITY
local MAV_EMERG, MAV_CRIT, MAV_WARN, MAV_NOTICE, MAV_INFO = 0, 2, 4, 5, 6

-- Rover 模式号
local MODE_HOLD = 4
local MODE_AUTO = 10
local DRIVE_MODES = { [5] = true, [10] = true, [11] = true, [12] = true, [15] = true }  -- LOITER AUTO RTL SMART_RTL GUIDED

-- CAN ID (29 位扩展帧, 发送时需置 EFF 位 bit31)
local CAN_EFF     = 0x80000000
local ID_NAV_CMD  = 0x180C7D41
local ID_NAV_ACK  = 0x180D7D41
local ID_NAV_STA  = 0x180B417D
local ID_MOTION   = 0x1802407D
local ID_HEALTH   = 0x1806407D

-- NavSta 枚举
local OP_NAV = 2              -- OpSource: 0 遥控, 1 切换中, 2 无人
local HS_INVITING, HS_WAIT_START, HS_DONE = 1, 2, 3
local HS_TIMEOUT, HS_REJECTED, HS_NAV_LOST, HS_RC_LOST = 4, 5, 6, 7
-- WorkFb (v1.1): 0 运输就绪, 1 升/降杆中(行驶锁 0), 2 作业位就绪, 3 作业中, 4 提升超时故障(行驶锁 0)
local WFB_LIFTING, WFB_LIFT_FAULT = 1, 4

-- SelfTest 故障码 (协议 3.2)
local ST_OK, ST_CAN, ST_GNSS, ST_IMU = 0x00, 0x02, 0x05, 0x06
local CAPS = 0x03             -- bit0 速度/方向 + bit1 作业电机(仅启停)

---------------------------------------------------------------------------
-- 参数表 VCU_*  (序号 1~8 与旧版保持一致, 存量参数自动继承; 9~12 为新增)
-- 表大小上电后固定: 加了参数的脚本换上后要重启飞控, 只重启脚本会报 could not add param table
---------------------------------------------------------------------------
local PARAM_TABLE_KEY = 71
assert(param:add_table(PARAM_TABLE_KEY, "VCU_", 17), "VCU: could not add param table")
local function bind_add_param(name, idx, default)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default), "VCU: could not add param " .. name)
  return Parameter("VCU_" .. name)
end
--[[ @Param: VCU_ENABLE  @DisplayName: 启用VCU CAN桥接  @Values: 0:关,1:开 --]]
local VCU_ENABLE  = bind_add_param("ENABLE", 1, 1)
--[[ @Param: VCU_DBG_HZ  @DisplayName: 调试量(VCU_CMD_V/W/D, VCU_VREF, VCU_V/W)上报地面站的频率(Hz), 0=不发. 板载日志不受影响 --]]
local VCU_DBG_HZ  = bind_add_param("DBG_HZ", 2, 0)
--[[ @Param: VCU_FB_TMO  @DisplayName: MotionSta反馈超时(ms), 0=不检查反馈(台架调试用) --]]
local VCU_FB_TMO  = bind_add_param("FB_TMO", 3, 200)
--[[ @Param: VCU_V_MAX  @DisplayName: 线速度限幅(m/s), 协议上限5.0 --]]
local VCU_V_MAX   = bind_add_param("V_MAX", 4, 1.38)
-- 参数序号 5 (原 VCU_W_MAX) 已废弃: 新协议发转角不发角速度, 保留空位, 其它参数序号不变
local VCU_W_MAX   = bind_add_param("W_MAX", 5, 1.0)
--[[ @Param: VCU_REV  @DisplayName: 允许倒车  @Values: 0:否,1:是 --]]
local VCU_REV     = bind_add_param("REV", 6, 0)
--[[ @Param: VCU_RTK_REQ  @DisplayName: RTK要求  @Values: 0:不检查,1:浮点解或固定解,2:仅固定解 --]]
local VCU_RTK_REQ = bind_add_param("RTK_REQ", 7, 2)
--[[ @Param: VCU_RTK_TMO  @DisplayName: RTK不满足要求持续多久(ms)后停车 --]]
local VCU_RTK_TMO = bind_add_param("RTK_TMO", 8, 3000)
--[[ @Param: VCU_W_BASE  @DisplayName: 等效轴距(m): 前辊转向轴到后辊轴线, Ackermann 运动学换算用 --]]
local VCU_W_BASE  = bind_add_param("W_BASE", 9, 1.2)
--[[ @Param: VCU_WORK_EN  @DisplayName: 允许作业(Work_On): 1 时按地面站/航线的 SCRIPTING1 请求升杆作业  @Values: 0:否,1:是 --]]
local VCU_WORK_EN = bind_add_param("WORK_EN", 10, 0)
--[[ @Param: VCU_VREF_MIN  @DisplayName: 转角换算用实际车速的下限(m/s), 起步时实际车速接近 0, 防止转角乱跳 --]]
local VCU_VREF_MIN = bind_add_param("VREF_MIN", 11, 0.3)
--[[ @Param: VCU_FB_CHK  @DisplayName: 协议外状态帧 (MotionSta/HealthSta) 是否参与拦车: 0 只上报; 1 反馈超时/Failsafe/Sys_Fault 禁止解锁、行驶中切 HOLD, Sys_Fault 和驱动器故障计入 VCU_FAULT (地面站据此禁止开车)  @Values: 0:只上报,1:参与拦车(garlic.15 做法) --]]
local VCU_FB_CHK = bind_add_param("FB_CHK", 12, 0)
--[[ @Param: VCU_VREF_SRC  @DisplayName: 转角换算用的实际车速来源: 0 MotionSta 回报车速 (新鲜时), 1 AHRS 前向速度 (RTK). 实车 VCU 回报车速封顶约 0.85 m/s, 用 1  @Values: 0:VCU回报,1:AHRS --]]
local VCU_VREF_SRC = bind_add_param("VREF_SRC", 13, 1)
-- 转向适配参数放在一个表里: ArduPilot 的 Lua 每个函数最多 100 个局部变量, 脚本最外层已接近上限
local STR = {}
--[[ @Param: VCU_STR_TOL  @DisplayName: 起步回正(度): 车静止要起步时先把前轮回正, 实际转角(MotionSta)偏离正中超过此值就不给车速, 只在起步时检查, 行驶中停顿不检查; 0=不等  @Units: deg --]]
STR.TOL = bind_add_param("STR_TOL", 14, 5)
--[[ @Param: VCU_STR_PARK  @DisplayName: 停车回正: 1 不在行驶状态(未解锁/非自动模式)时转角指令回 0; 0 保持上次转角  @Values: 0:保持,1:回正 --]]
STR.PARK = bind_add_param("STR_PARK", 15, 1)
--[[ @Param: VCU_STR_OFS  @DisplayName: 转角零位补偿(度, 左正): 叠加到下发的转角指令上, 补偿前轮零位偏差  @Units: deg --]]
STR.OFS = bind_add_param("STR_OFS", 16, 0)
--[[ @Param: VCU_STR_STEP  @DisplayName: 转角小步增量(度): 下发转角 = VCU 回报实际转角 + 朝期望方向最多此值 (VCU 锁定目标式转向的适配); 须大于 VCU 到位容差; 0=直接发期望转角  @Units: deg --]]
STR.STEP = bind_add_param("STR_STEP", 17, 2)

---------------------------------------------------------------------------
-- CAN
---------------------------------------------------------------------------
-- 固件检查: 必须是带 get_desired_* 绑定的补丁固件
if (AR_AttitudeControl == nil) or (AR_AttitudeControl.get_desired_speed == nil)
   or (AR_AttitudeControl.get_desired_turn_rate == nil) then
  gcs:send_text(MAV_CRIT, "VCU: fw lacks get_desired binding, flash patched fw")
  return
end

local can = CAN:get_device(25)
if not can then
  gcs:send_text(MAV_CRIT, "VCU: no scripting CAN, check CAN_Px_DRIVER/CAN_Dx_PROTOCOL=10")
  return
end

-- 解锁前检查: 未授权/VCU 未就绪时禁止解锁
local auth_id = arming:get_aux_auth_id()

---------------------------------------------------------------------------
-- 工具函数
---------------------------------------------------------------------------
local function clamp(x, lo, hi)
  if x < lo then return lo elseif x > hi then return hi end
  return x
end

local function to_i16(x)          -- 浮点 -> int16 (四舍五入并限幅)
  local n = math.floor(x + 0.5)
  return clamp(n, -32768, 32767)
end

local function put_i16(frame, idx, n)   -- 小端写
  n = n & 0xFFFF
  frame:data(idx, n & 0xFF)
  frame:data(idx + 1, (n >> 8) & 0xFF)
end

local function get_u16(frame, idx)
  return frame:data(idx) | (frame:data(idx + 1) << 8)
end

local function get_i16(frame, idx)
  local n = get_u16(frame, idx)
  if n >= 0x8000 then n = n - 0x10000 end
  return n
end

---------------------------------------------------------------------------
-- v / w 来源: 读取控制器期望值.  ArduPilot 转向率顺时针为正, 这里取反成逆时针为正
---------------------------------------------------------------------------
local function get_vw()
  return AR_AttitudeControl:get_desired_speed(), -AR_AttitudeControl:get_desired_turn_rate()
end

---------------------------------------------------------------------------
-- 接收状态
---------------------------------------------------------------------------
-- NavSta: 授权唯一真相
local navsta = { ms = 0, op = 0, hs = 0, seq = 0, err = 0, wfb = 0 }
-- MotionSta / HealthSta (协议外): 不参与 VCU 授权判定 (授权只看 NavSta). VCU_FB_CHK=1 时用于解锁检查和行驶中保护:
--   反馈超时 / Failsafe / HealthSta Sys_Fault 非 0 时禁止解锁, 行驶中切 HOLD 并持续发 v=0; VCU_FB_TMO=0 时不检查.
--   VCU_FB_CHK=0 时只上报. 驱动器故障 Drv1/2/3 始终只上报
local motion = { ms = 0, v = 0, steer_deg = 0, veh = -1, failsafe = false }
local health = { ms = 0, soc = 0, bat_v = 0, drv1 = 0, drv2 = 0, drv3 = 0, sys = 0 }

local function read_frames()
  for _ = 1, 25 do
    local f = can:read_frame()
    if not f then return end
    -- 只收扩展数据帧, DLC=8
    if f:isExtended() and f:dlc() == 8 then
      local id = f:id_signed()   -- 掩码后的 29 位 ID, 我们的 ID 均 <0x7FFFFFFF
      if id == ID_NAV_STA then
        -- 校验: 保留字节 5~7 为 0, 枚举不越界 (WorkFb >4 非法), 全对才刷新时钟
        if f:data(5) == 0 and f:data(6) == 0 and f:data(7) == 0
           and f:data(0) <= 2 and f:data(1) <= 7 and f:data(3) <= 6 and f:data(4) <= 4 then
          navsta.op  = f:data(0)
          navsta.hs  = f:data(1)
          navsta.seq = f:data(2)
          navsta.err = f:data(3)
          navsta.wfb = f:data(4)
          navsta.ms  = millis():toint()
        end
      elseif id == ID_MOTION then
        motion.v         = get_i16(f, 0) * 0.01
        motion.steer_deg = get_i16(f, 2) * 0.01
        motion.veh       = f:data(5) & 0x0F
        motion.failsafe  = (f:data(5) & 0x20) ~= 0
        motion.ms        = millis():toint()
      elseif id == ID_HEALTH then
        local raw_v = get_u16(f, 1)
        health.soc   = f:data(0)
        health.bat_v = (raw_v <= 1000) and (raw_v * 0.1) or 0   -- 超量程按无效处理
        health.drv1  = f:data(3)
        health.drv2  = f:data(4)
        health.drv3  = f:data(5)
        health.sys   = f:data(6)
        health.ms    = millis():toint()
      end
    end
  end
end

-- MotionSta / HealthSta 反馈检查: VCU_FB_CHK=0 (默认) 时不检查, 只上报; 台架也可用 VCU_FB_TMO=0 关闭
local function feedback_ok(now)
  local tmo = VCU_FB_TMO:get()
  if VCU_FB_CHK:get() <= 0 or tmo <= 0 then return true, nil end
  if motion.ms == 0 or now - motion.ms > tmo then return false, "feedback timeout" end
  if motion.failsafe then return false, "failsafe" end
  -- HealthSta 为诊断帧, 收到且在 800ms 内新鲜时才参与故障判定
  if health.ms > 0 and now - health.ms <= 800 and health.sys ~= 0 then
    return false, string.format("fault 0x%02X", health.sys)
  end
  return true, nil
end

-- 驱动器故障: 第一个非 0 的 Drv1/2/3_Fault 及其路号 (1~3); 无故障返回 0, 0. 只上报, 不拦车
local function drive_fault()
  if health.drv1 > 0 then return health.drv1, 1 end
  if health.drv2 > 0 then return health.drv2, 2 end
  if health.drv3 > 0 then return health.drv3, 3 end
  return 0, 0
end

-- AHRS 前向速度: EKF 地速投到航向上 (同 AR_AttitudeControl::get_forward_speed)
local function ahrs_fwd_speed()
  local gs = ahrs:groundspeed_vector()
  local yaw = ahrs:get_yaw_rad()
  return gs:x() * math.cos(yaw) + gs:y() * math.sin(yaw)
end

-- 转角换算用的实际车速: MotionSta 新鲜时用 VCU 回报车速, 否则用 AHRS 前向速度
local function actual_speed(now, ahrs_v)
  if VCU_VREF_SRC:get() <= 0 and motion.ms > 0 and now - motion.ms <= MOTION_FRESH_MS then return motion.v end
  return ahrs_v
end

-- 作业请求 (电平保持): SCRIPTING1 为 HIGH 时请求 Work_On=1; 从未设置过为 nil, 按 0
local function work_request()
  local pos = rc:get_aux_cached(AUX_WORK)
  return pos ~= nil and pos >= 2
end

-- WorkFb 检查: 升/降杆中或提升故障时 VCU 行驶锁 0, 禁止解锁 / 行驶中切 HOLD
local function workfb_block()
  if navsta.wfb == WFB_LIFTING then return "lift moving" end
  if navsta.wfb == WFB_LIFT_FAULT then return "lift fault" end
  return nil
end

-- RTK 检查. 返回: 当前是否满足, 行驶中是否仍可继续(允许短暂掉解 VCU_RTK_TMO), 当前定位状态
local last_rtk_ok_ms = 0
local function rtk_check(now)
  local fix = gps:status(gps:primary_sensor())
  local req = VCU_RTK_REQ:get()
  if req <= 0 then return true, true, fix end
  local need = (req >= 2) and gps.GPS_OK_FIX_3D_RTK_FIXED or gps.GPS_OK_FIX_3D_RTK_FLOAT
  if fix >= need then
    last_rtk_ok_ms = now
    return true, true, fix
  end
  return false, (now - last_rtk_ok_ms) <= VCU_RTK_TMO:get(), fix
end

-- 自检 (受邀时): IMU -> RTK, 返回协议 3.2 故障码, 0 = 通过
local function self_test()
  if not ins:get_gyro_health(0) then return ST_IMU end
  local req = VCU_RTK_REQ:get()
  if req > 0 then
    local fix = gps:status(gps:primary_sensor())
    local need = (req >= 2) and gps.GPS_OK_FIX_3D_RTK_FIXED or gps.GPS_OK_FIX_3D_RTK_FLOAT
    if fix < need then return ST_GNSS end
  end
  return ST_OK
end

---------------------------------------------------------------------------
-- 发送 (全部为扩展帧, EFF 位置位; Lua int 为 32 位, EFF 位须用 uint32_t)
---------------------------------------------------------------------------
local function new_ext_frame(id)
  local f = CANFrame()
  f:id(uint32_t(CAN_EFF | id))
  f:dlc(8)
  return f
end

local function send_nav_cmd(v, steer_deg, work_on, seq)
  local f = new_ext_frame(ID_NAV_CMD)
  put_i16(f, 0, to_i16(clamp(v, -5.0, 5.0) * 100))              -- Target_Speed x0.01 m/s
  put_i16(f, 2, to_i16(clamp(steer_deg, -45.0, 45.0) * 100))    -- Target_Steer x0.01 deg, 左正
  f:data(4, work_on and 1 or 0)                                 -- Work_On 电平保持
  f:data(5, 0)                                                  -- Work_Speed 固定 0 (workdrv 仅启停)
  f:data(6, 0)                                                  -- Work_Dir  固定 0
  f:data(7, seq & 0xFF)                                         -- 授权会话 Seq
  can:write_frame(f, 10000)
end

local function send_hs_ack(accept, seq, st)
  local f = new_ext_frame(ID_NAV_ACK)
  f:data(0, accept and 1 or 2)    -- 1 接受, 2 拒绝
  f:data(1, seq & 0xFF)           -- 回显邀请事务 Seq
  f:data(2, st)                   -- SelfTest 故障码
  f:data(3, CAPS)                 -- 0x03
  f:data(4, 0) f:data(5, 0) f:data(6, 0) f:data(7, 0)
  can:write_frame(f, 10000)
end

---------------------------------------------------------------------------
-- 主循环
---------------------------------------------------------------------------
local last_ack_seq = -1       -- 已应答的邀请 Seq (每事务只答一次)
local last_tx_ms = 0
local last_log_ms = 0
local last_gcs_ms = 0
local last_dbg_ms = 0
local last_warn_ms = 0
local last_wfb_ms = 0
local prev_hs = 0
local prev_wfb = 0
local last_drv_ms = 0
local prev_drv, prev_drv_ch = 0, 0
local prev_sys = 0
local prev_failsafe = false
local prev_work_req = false
local prev_work_on = false
local last_work_change_ms = -100000   -- Work_On 上次变化时刻 (升/降杆在此后 WORK_LIFT_EXPECT_MS 内属预期)
local was_authorized = false
local last_steer_deg = 0      -- |v|<0.05 时保持上次转角

local function update()
  local now = millis():toint()
  read_frames()

  if VCU_ENABLE:get() <= 0 then
    return update, LOOP_MS
  end

  -- NavSta 新鲜度: 监控超时定值 500ms, 超时即失权静默
  local navsta_valid = navsta.ms > 0 and (now - navsta.ms) < NAVSTA_TMO_MS
  local authorized = navsta_valid and navsta.op == OP_NAV

  -- 受邀应答: 2s 窗口内自检应答, 每笔事务只答一次
  if navsta_valid and navsta.hs == HS_INVITING and navsta.seq ~= last_ack_seq then
    local st = self_test()
    send_hs_ack(st == ST_OK, navsta.seq, st)
    last_ack_seq = navsta.seq
  end

  -- 握手状态边沿提示
  if navsta_valid and navsta.hs ~= prev_hs then
    if navsta.hs == HS_WAIT_START then
      gcs:send_text(MAV_INFO, "VCU: wait RC start")
    elseif navsta.hs == HS_DONE then
      gcs:send_text(MAV_INFO, "VCU: RC start OK")    -- 遥控器已按 Ready, VCU 进入 RUN
    elseif navsta.hs == HS_TIMEOUT or navsta.hs == HS_REJECTED then
      gcs:send_text(MAV_WARN, "VCU: handover failed")
    elseif navsta.hs == HS_RC_LOST then
      gcs:send_text(MAV_WARN, "VCU: RC link lost")
    end
  end
  if navsta_valid then prev_hs = navsta.hs end

  -- WorkFb 边沿提示 + 遥测: 变化时立即发 VCU_WORKFB, 之后 1 Hz (4G 按流量计费, 不进 5 Hz)
  if navsta_valid then
    local wfb_changed = navsta.wfb ~= prev_wfb
    if wfb_changed then
      if prev_wfb == WFB_LIFT_FAULT then gcs:send_text(MAV_INFO, "VCU: lift fault cleared") end
      if navsta.wfb == WFB_LIFTING then
        gcs:send_text(MAV_WARN, "VCU: lift moving")
      elseif navsta.wfb == WFB_LIFT_FAULT then
        gcs:send_text(MAV_CRIT, "VCU: lift fault")
      end
      prev_wfb = navsta.wfb
    end
    if wfb_changed or now - last_wfb_ms >= 1000 then
      last_wfb_ms = now
      gcs:send_named_float("VCU_WORKFB", navsta.wfb)
      -- 作业请求随 WorkFb 1 Hz 补发 (变化时另有立即发送), 地面站重连后也能核对
      if VCU_WORK_EN:get() > 0 then gcs:send_named_float("VCU_WREQ", work_request() and 1 or 0) end
    end
  end

  -- 协议外状态上报 (VCU_FB_CHK 两档都发): Sys_Fault / 驱动器故障变化时提示一次并立即发 VCU_SYSFLT (Sys_Fault 原值),
  -- VCU_DRVFLT (码) / VCU_DRVCH (路号 1~3), 之后 1 Hz; HealthSta 不新鲜 (1500 ms) 时不发
  if motion.ms > 0 and motion.failsafe ~= prev_failsafe then
    gcs:send_text(motion.failsafe and MAV_NOTICE or MAV_INFO,
                  motion.failsafe and "VCU: chassis failsafe" or "VCU: chassis failsafe cleared")
    prev_failsafe = motion.failsafe
  end
  if health.ms > 0 and now - health.ms <= 1500 then
    local sys_changed = health.sys ~= prev_sys
    if sys_changed then
      if health.sys > 0 then
        gcs:send_text(MAV_NOTICE, string.format("VCU: sys fault 0x%02X", health.sys))
      else
        gcs:send_text(MAV_INFO, "VCU: sys fault cleared")
      end
      prev_sys = health.sys
    end
    local drv, drv_ch = drive_fault()
    local drv_changed = drv ~= prev_drv or drv_ch ~= prev_drv_ch
    if drv_changed then
      if drv > 0 then
        gcs:send_text(MAV_NOTICE, string.format("VCU: drive fault 0x%02X Drv%d", drv, drv_ch))
      else
        gcs:send_text(MAV_INFO, "VCU: drive fault cleared")
      end
      prev_drv, prev_drv_ch = drv, drv_ch
    end
    if sys_changed or drv_changed or now - last_drv_ms >= 1000 then
      last_drv_ms = now
      gcs:send_named_float("VCU_SYSFLT", health.sys)
      gcs:send_named_float("VCU_DRVFLT", drv)
      gcs:send_named_float("VCU_DRVCH", drv_ch)
    end
  end

  local fb_ok, fb_why = feedback_ok(now)
  local rtk_now, rtk_keep, fix = rtk_check(now)
  local wfb_why = workfb_block()

  -- 解锁前检查: 已授权 + 提杆不在动作/故障 + VCU 反馈正常 + 当前满足 RTK 要求
  if auth_id then
    if not navsta_valid then arming:set_aux_auth_failed(auth_id, "VCU: NavSta lost")
    elseif not authorized then arming:set_aux_auth_failed(auth_id, "VCU: not authorized")
    elseif wfb_why then arming:set_aux_auth_failed(auth_id, "VCU: " .. wfb_why)
    elseif not fb_ok then arming:set_aux_auth_failed(auth_id, "VCU: " .. fb_why)
    elseif not rtk_now then arming:set_aux_auth_failed(auth_id, "VCU: no RTK fix")
    else arming:set_aux_auth_passed(auth_id) end
  end

  -- 行驶中保护: 失权/升降杆/反馈异常/RTK 掉解 -> 切 HOLD, 但只要授权还在就持续发 v=0 帧 (不断流)
  local ok = navsta_valid and authorized and wfb_why == nil and fb_ok and rtk_keep
  local why = fb_why
  if not navsta_valid then why = "NavSta lost"
  elseif not authorized then why = "not authorized"
  elseif wfb_why then why = wfb_why
  elseif fb_ok and not rtk_keep then why = "RTK lost" end

  local armed = arming:is_armed()
  local mode = vehicle:get_mode()
  local driving = armed and DRIVE_MODES[mode] == true

  local v, w, v_des = 0, 0, 0
  if driving then
    if ok then
      v, w = get_vw()
      v_des = v
      if VCU_REV:get() <= 0 and v < 0 then v = 0 end
      v = clamp(v, -VCU_V_MAX:get(), VCU_V_MAX:get())
    else
      vehicle:set_mode(MODE_HOLD)
      driving = false
      -- 我方请求的升/降杆 (Work_On 刚变过) 或静止时升/降杆, 是作业流程 (对准点放杆、终点收杆),
      -- 只切 HOLD 不报警; 行驶中出现非预期的升降杆才报
      local expected = now - last_work_change_ms < WORK_LIFT_EXPECT_MS
      local still = math.abs(AR_AttitudeControl:get_desired_speed()) < 0.05
                    and math.abs(actual_speed(now, ahrs_fwd_speed())) < 0.1
      if not (why == "lift moving" and (expected or still)) and now - last_warn_ms > 2000 then
        gcs:send_text(MAV_WARN, "VCU: " .. why .. ", HOLD")
        last_warn_ms = now
      end
    end
  end

  -- Ackermann 运动学: w -> 前轮转角 delta (左转为正), 按实际车速换算, 指令低速时保持上次转角
  local wheelbase = math.max(VCU_W_BASE:get(), 0.3)
  local turn_r = param:get("TURN_RADIUS") or 1.0
  local steer_max = math.min(45.0, math.deg(math.atan(wheelbase / math.max(turn_r, 0.3))))
  local ahrs_v = ahrs_fwd_speed()
  local v_ref = math.max(math.abs(actual_speed(now, ahrs_v)), VCU_VREF_MIN:get(), 0.05)
  local steer_deg = last_steer_deg
  if not driving and STR.PARK:get() > 0 then
    steer_deg = 0
    last_steer_deg = 0
  end
  if math.abs(v) >= 0.05 then
    -- 还原导航的横向加速度 a_lat = w * max(|v_des|, 1.0) (带 v_des 符号), 再 tan(delta) = a_lat*L/v_ref^2
    local s = math.max(math.abs(v_des), ATC_STEER_SPEED_MIN)
    if v_des < 0 then s = -s end
    steer_deg = clamp(math.deg(math.atan(w * s * wheelbase / (v_ref * v_ref))), -steer_max, steer_max)
    last_steer_deg = steer_deg
  end
  -- 起步回正: 车静止时先把前轮回到正中, 偏差在 VCU_STR_TOL 内才给车速 (VCU 转向约 4.5 deg/s, 起步约 1 s 即到
  -- 1 m/s 以上, 前轮不正就会带着转角冲出去). 静止时按 VREF_MIN 换算的转角没有意义, 不去对准它
  -- 只管起步: 车速指令归 0 (停车 / 不在行驶状态) 后算重新起步, 车开起来 (AHRS >= 0.3 m/s) 后不再检查.
  -- 行驶中车停顿一下 (作业拖地 / 打滑) 不算起步, 否则前轮不正就把车速压成 0 -> Crash (1103 外场作业段 3 次)
  if math.abs(v) <= 0.01 then
    STR.rolled = false
  elseif math.abs(ahrs_v) >= 0.3 then
    STR.rolled = true
  end
  local tol = STR.TOL:get()
  if tol > 0 and not STR.rolled and math.abs(v) > 0.01 and math.abs(ahrs_v) < 0.1 and motion.ms > 0 and now - motion.ms <= MOTION_FRESH_MS then
    steer_deg = 0
    last_steer_deg = 0
    if math.abs(STR.OFS:get() - motion.steer_deg) > tol then
      v = 0
    end
  end

  -- 作业 (Work_On): 由地面站/航线经 SCRIPTING1 请求, 电平保持, 不跟模式和车速走 (v1.1: 0->1 升杆进入作业,
  -- 1->0 降杆回运输, 动杆期间 WorkFb=1 行驶锁 0). Work_Speed/Work_Dir 固定 0.
  -- 失权 (OpSource!=2 / NavSta 丢失) 时清掉请求: 恢复授权后保持 0, 由地面站重新下发 (协议: 断链后由 Nav 重新决策)
  if was_authorized and not authorized and work_request() then
    rc:run_aux_function(AUX_WORK, 0)
    gcs:send_text(MAV_WARN, "VCU: work request cleared")
  end
  local work_req = work_request()
  local work_on = VCU_WORK_EN:get() > 0 and authorized and work_req
  if work_on ~= prev_work_on then
    prev_work_on = work_on
    last_work_change_ms = now
  end
  if work_req ~= prev_work_req then
    prev_work_req = work_req
    gcs:send_named_float("VCU_WREQ", work_req and 1 or 0)
  end

  -- 授权期间 50ms 周期发送, 断流 >100ms 会被 VCU 收权; 授权沿立即发第一帧
  if authorized and not was_authorized then last_tx_ms = 0 end
  -- 小步增量: 下发目标只比 VCU 回报的实际转角多走 VCU_STR_STEP 度 (VCU 到位才接受新指令, issue #11)
  local steer_want = steer_deg + STR.OFS:get()
  local steer_cmd = steer_want
  local step = STR.STEP:get()
  if step > 0 and motion.ms > 0 and now - motion.ms <= MOTION_FRESH_MS then
    steer_cmd = motion.steer_deg + clamp(steer_want - motion.steer_deg, -step, step)
  end
  if authorized and now - last_tx_ms >= TX_PERIOD_MS then
    last_tx_ms = now
    send_nav_cmd(v, steer_cmd, work_on, navsta.seq)
  end
  was_authorized = authorized

  -- 板载日志 (50Hz): 指令 v/转角 与 VCU 实际 v/转角、操作源、握手状态、故障、定位、作业状态、
  -- 转角换算用车速 VR 和 AHRS 前向速度 AS (与 AV 对比可看打滑)
  -- Lua 日志最多 14 列, 作业请求 WR 与实际下发的 Work_On WO 单独记在 VCUW
  if now - last_log_ms >= 20 then
    last_log_ms = now
    logger:write("VCU", "CV,CD,AV,AD,Op,HS,Er,VS,FS,Ok,Fix,WF,VR,AS", "ffffBBBBBBBBff",
                 v, steer_cmd, motion.v, motion.steer_deg,
                 navsta_valid and navsta.op or 0, navsta_valid and navsta.hs or 0,
                 navsta.err, clamp(motion.veh, 0, 255), motion.failsafe and 1 or 0,
                 ok and 1 or 0, fix, navsta.wfb, v_ref, ahrs_v)
    -- SF = HealthSta Sys_Fault, DF/DC = 驱动器故障码及路号 (HealthSta 最近一帧, 不看新鲜度)
    local drv, drv_ch = drive_fault()
    logger:write("VCUW", "WR,WO,SF,DF,DC,DD", "BBBBBf", work_req and 1 or 0, work_on and 1 or 0,
                 health.sys, drv, drv_ch, steer_want)
  end

  -- 5Hz 上报地面站 (客户地面站只用这 5 个值, 4G 按流量计费)
  -- VCU_MODE/VCU_FAULT/VCU_OK 始终发送; VCU_BATV/VCU_SOC 仅在收到 HealthSta 时发送
  if now - last_gcs_ms >= 200 then
    last_gcs_ms = now
    -- VCU_MODE: 0 未知(NavSta 无效), 1 遥控/切换中, 2 无人
    gcs:send_named_float("VCU_MODE", (not navsta_valid) and 0 or (navsta.op == OP_NAV and 2 or 1))
    -- VCU_FAULT (地面站见非 0 即禁止开车): HsErr 报 0x10+码 (协议内, 始终报). VCU_FB_CHK=1 时按 garlic.15:
    -- Sys_Fault 1~7 直接报 (优先于 HsErr), 驱动器故障报 0x100+码; VCU_FB_CHK=0 时两者只走 VCU_SYSFLT / VCU_DRVFLT
    local fault_code = 0
    local health_fresh = health.ms > 0 and now - health.ms <= 1500
    local fb_chk = VCU_FB_CHK:get() > 0
    if fb_chk and health_fresh and health.sys > 0 then
      fault_code = health.sys
    elseif navsta_valid and navsta.err > 0 then
      fault_code = 0x10 + navsta.err
    elseif fb_chk and health_fresh then
      local drv = drive_fault()
      if drv > 0 then fault_code = 0x100 + drv end
    end
    gcs:send_named_float("VCU_FAULT", fault_code)
    -- VCU_BATV/VCU_SOC: 仅在 HealthSta 新鲜 (1500 ms 内) 时发送; 没收到就完全不发,
    -- 避免地面站把 0 当成真实电量而显示 0% / 报低电量 (飞控单独供电, 自身电池监测不可信)
    if health_fresh then
      gcs:send_named_float("VCU_BATV", health.bat_v)
      gcs:send_named_float("VCU_SOC", health.soc)
    end
    gcs:send_named_float("VCU_OK", ok and 1 or 0)
  end

  -- 调试量: 只在 VCU_DBG_HZ > 0 时上报 (工程师用 Mission Planner 看), 板载日志里已有 50Hz 记录
  local dbg_hz = VCU_DBG_HZ:get()
  if dbg_hz > 0 and now - last_dbg_ms >= 1000 / math.min(dbg_hz, 10) then
    last_dbg_ms = now
    gcs:send_named_float("VCU_CMD_V", v)
    gcs:send_named_float("VCU_CMD_W", w)
    gcs:send_named_float("VCU_CMD_D", steer_deg)
    gcs:send_named_float("VCU_VREF", v_ref)
    gcs:send_named_float("VCU_V", motion.v)
    gcs:send_named_float("VCU_W", motion.v * math.tan(math.rad(motion.steer_deg)) / wheelbase)
  end

  return update, LOOP_MS
end

gcs:send_text(MAV_INFO, "VCU: CAN bridge loaded (Nav v1.1) " .. SCRIPT_VERSION)

-- 出错时静默 (VCU 看门狗 100ms 后受控停车收权, 安全方向), 1s 后重启脚本
local function protected_wrapper()
  local success, err = pcall(update)
  if not success then
    gcs:send_text(MAV_EMERG, "VCU: error " .. tostring(err))
    return protected_wrapper, 1000
  end
  return protected_wrapper, LOOP_MS
end

return protected_wrapper()
