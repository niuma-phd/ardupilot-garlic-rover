--[[
 vcu_can.lua  大蒜播种车 Nav(飞控) <-> 底盘VCU  CAN 桥接脚本
 固件: ArduPilot Rover 4.7.1 + 补丁 (飞控 X-MAV AP-H743v3)

 协议: 《Nav <-> VCU CAN 通讯协议 v1.0》(2026-10-10, 甲方定版)
   总线 CAN3, 500 kbit/s, 29 位扩展帧 (IDE=1), 经典 CAN, DLC=8, Intel 小端
   Nav 源地址 0x41, VCU 源地址 0x7D
   Nav_Cmd     0x180C7D41  Nav -> VCU  50ms 周期 (授权期间不得断流, VCU 看门狗 100ms)
   Nav_HsAck   0x180D7D41  Nav -> VCU  邀请 (HandoverState=1) 后 2s 内应答, 每事务只答一次
   VCU_NavSta  0x180B417D  VCU -> 总线 100ms 广播, 授权唯一真相, Nav 监控超时定值 500ms
   FCU_SwapCmd 0x180A7D40  参考帧, Nav 不消费
   监听帧 (仅日志/遥测, 不参与授权判定):
   VCU_MotionSta 0x1802407D  50ms
   VCU_HealthSta 0x1806407D  500ms

 核心规则 (协议 04 章):
   1. 上电静默: 只听 NavSta, 不主动发任何帧
   2. 受邀应答: 见 INVITING 后 2s 内自检并回 HsAck (SelfTest/Caps=0x03), 每 Seq 只答一次
   3. 授权才发: 见 OpSource=2 才开始 50ms 周期发 Nav_Cmd, Seq 与 NavSta.Seq 一致
   4. 失权静默: OpSource!=2 / 失败态 / NavSta 丢 500ms -> 立即停发, 不抢权, 等新邀请
   5. 断流即停车: 授权期间必须持续发帧 (含 v=0), 断流 >100ms 触发 HsErr=4 收权
   6. Nav 无启动/急停权限: 车辆只能由遥控器 Ready 0->1 新边沿启动, 急停仅硬件回路

 双方确认补充 (2026-10-11):
   - Target_Steer 左转为正 (与 w 逆时针为正 -> delta=atan(w*L/v) 天然一致)
   - 我方自身保护触发 (HOLD / RTK 掉解) 时持续发 v=0 的 Nav_Cmd, 不断流, 授权保留

 运动学: Ackermann, w = v*tan(delta)/L -> delta = atan(w*L/v), 左正
   delta 限幅 min(45deg, deg(atan(L/TURN_RADIUS))), |v|<0.05m/s 时保持上次转角

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

-- MAV_SEVERITY
local MAV_EMERG, MAV_CRIT, MAV_WARN, MAV_INFO = 0, 2, 4, 6

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
local HS_INVITING, HS_WAIT_START = 1, 2
local HS_TIMEOUT, HS_REJECTED, HS_NAV_LOST, HS_RC_LOST = 4, 5, 6, 7

-- SelfTest 故障码 (协议 3.2)
local ST_OK, ST_CAN, ST_GNSS, ST_IMU = 0x00, 0x02, 0x05, 0x06
local CAPS = 0x03             -- bit0 速度/方向 + bit1 作业电机(仅启停)

---------------------------------------------------------------------------
-- 参数表 VCU_*  (序号 1~8 与旧版保持一致, 存量参数自动继承; 9/10 为新增)
---------------------------------------------------------------------------
local PARAM_TABLE_KEY = 71
assert(param:add_table(PARAM_TABLE_KEY, "VCU_", 10), "VCU: could not add param table")
local function bind_add_param(name, idx, default)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default), "VCU: could not add param " .. name)
  return Parameter("VCU_" .. name)
end
--[[ @Param: VCU_ENABLE  @DisplayName: 启用VCU CAN桥接  @Values: 0:关,1:开 --]]
local VCU_ENABLE  = bind_add_param("ENABLE", 1, 1)
--[[ @Param: VCU_DBG_HZ  @DisplayName: 调试量(VCU_CMD_V/W/D, VCU_V/W)上报地面站的频率(Hz), 0=不发. 板载日志不受影响 --]]
local VCU_DBG_HZ  = bind_add_param("DBG_HZ", 2, 0)
--[[ @Param: VCU_FB_TMO  @DisplayName: MotionSta反馈超时(ms), 0=不检查反馈(台架调试用) --]]
local VCU_FB_TMO  = bind_add_param("FB_TMO", 3, 200)
--[[ @Param: VCU_V_MAX  @DisplayName: 线速度限幅(m/s), 协议上限5.0 --]]
local VCU_V_MAX   = bind_add_param("V_MAX", 4, 1.5)
-- 参数序号 5 (原 VCU_W_MAX) 已废弃: 新协议发转角不发角速度, 保留空位, 其它参数序号不变
local VCU_W_MAX   = bind_add_param("W_MAX", 5, 1.0)
--[[ @Param: VCU_REV  @DisplayName: 允许倒车  @Values: 0:否,1:是 --]]
local VCU_REV     = bind_add_param("REV", 6, 0)
--[[ @Param: VCU_RTK_REQ  @DisplayName: RTK要求  @Values: 0:不检查,1:浮点解或固定解,2:仅固定解 --]]
local VCU_RTK_REQ = bind_add_param("RTK_REQ", 7, 2)
--[[ @Param: VCU_RTK_TMO  @DisplayName: RTK不满足要求持续多久(ms)后停车 --]]
local VCU_RTK_TMO = bind_add_param("RTK_TMO", 8, 3000)
--[[ @Param: VCU_W_BASE  @DisplayName: 轴距(m), Ackermann 运动学换算用 --]]
local VCU_W_BASE  = bind_add_param("W_BASE", 9, 2.0)
--[[ @Param: VCU_WORK_EN  @DisplayName: 允许作业电机启停(AUTO模式行驶中自动开)  @Values: 0:否,1:是 --]]
local VCU_WORK_EN = bind_add_param("WORK_EN", 10, 0)

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
local navsta = { ms = 0, op = 0, hs = 0, seq = 0, err = 0 }
-- MotionSta / HealthSta: 仅日志与遥测, 不参与授权判定
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
        -- 校验: 保留字节 4~7 为 0, 枚举不越界, 全对才刷新时钟
        if f:data(4) == 0 and f:data(5) == 0 and f:data(6) == 0 and f:data(7) == 0
           and f:data(0) <= 2 and f:data(1) <= 7 and f:data(3) <= 6 then
          navsta.op  = f:data(0)
          navsta.hs  = f:data(1)
          navsta.seq = f:data(2)
          navsta.err = f:data(3)
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

-- MotionSta 反馈检查 (台架可用 VCU_FB_TMO=0 关闭)
local function feedback_ok(now)
  local tmo = VCU_FB_TMO:get()
  if tmo <= 0 then return true, nil end
  if motion.ms == 0 or now - motion.ms > tmo then return false, "feedback timeout" end
  if motion.failsafe then return false, "failsafe" end
  -- HealthSta 为诊断帧, 收到且在 800ms 内新鲜时才参与故障判定
  if health.ms > 0 and now - health.ms <= 800 and health.sys ~= 0 then
    return false, string.format("fault 0x%02X", health.sys)
  end
  return true, nil
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
local prev_hs = 0
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
    elseif navsta.hs == HS_TIMEOUT or navsta.hs == HS_REJECTED then
      gcs:send_text(MAV_WARN, "VCU: handover failed")
    elseif navsta.hs == HS_RC_LOST then
      gcs:send_text(MAV_WARN, "VCU: RC link lost")
    end
  end
  if navsta_valid then prev_hs = navsta.hs end

  local fb_ok, fb_why = feedback_ok(now)
  local rtk_now, rtk_keep, fix = rtk_check(now)

  -- 解锁前检查: 已授权 + VCU 反馈正常 + 当前满足 RTK 要求
  if auth_id then
    if not navsta_valid then arming:set_aux_auth_failed(auth_id, "VCU: NavSta lost")
    elseif not authorized then arming:set_aux_auth_failed(auth_id, "VCU: not authorized")
    elseif not fb_ok then arming:set_aux_auth_failed(auth_id, "VCU: " .. fb_why)
    elseif not rtk_now then arming:set_aux_auth_failed(auth_id, "VCU: no RTK fix")
    else arming:set_aux_auth_passed(auth_id) end
  end

  -- 行驶中保护: 失权/反馈异常/RTK 掉解 -> 切 HOLD, 但只要授权还在就持续发 v=0 帧 (不断流)
  local ok = navsta_valid and authorized and fb_ok and rtk_keep
  local why = fb_why
  if not navsta_valid then why = "NavSta lost"
  elseif not authorized then why = "not authorized"
  elseif fb_ok and not rtk_keep then why = "RTK lost" end

  local armed = arming:is_armed()
  local mode = vehicle:get_mode()
  local driving = armed and DRIVE_MODES[mode] == true

  local v, w = 0, 0
  if driving then
    if ok then
      v, w = get_vw()
      if VCU_REV:get() <= 0 and v < 0 then v = 0 end
      v = clamp(v, -VCU_V_MAX:get(), VCU_V_MAX:get())
    else
      vehicle:set_mode(MODE_HOLD)
      driving = false
      if now - last_warn_ms > 2000 then
        gcs:send_text(MAV_WARN, "VCU: " .. why .. ", HOLD")
        last_warn_ms = now
      end
    end
  end

  -- Ackermann 运动学: w -> 前轮转角 delta (左转为正), 低速保持上次转角
  local wheelbase = math.max(VCU_W_BASE:get(), 0.3)
  local turn_r = param:get("TURN_RADIUS") or 1.0
  local steer_max = math.min(45.0, math.deg(math.atan(wheelbase / math.max(turn_r, 0.3))))
  local steer_deg = last_steer_deg
  if math.abs(v) >= 0.05 then
    steer_deg = clamp(math.deg(math.atan(w * wheelbase / v)), -steer_max, steer_max)
    last_steer_deg = steer_deg
  end

  -- 作业电机: 仅启停 (Work_Speed/Work_Dir 固定 0), AUTO 行驶中自动开
  local work_on = VCU_WORK_EN:get() > 0 and mode == MODE_AUTO and driving and v > 0.02

  -- 授权期间 50ms 周期发送, 断流 >100ms 会被 VCU 收权; 授权沿立即发第一帧
  if authorized and not was_authorized then last_tx_ms = 0 end
  if authorized and now - last_tx_ms >= TX_PERIOD_MS then
    last_tx_ms = now
    send_nav_cmd(v, steer_deg, work_on, navsta.seq)
  end
  was_authorized = authorized

  -- 板载日志 (50Hz): 指令 v/转角 与 VCU 实际 v/转角、操作源、握手状态、故障、定位
  if now - last_log_ms >= 20 then
    last_log_ms = now
    logger:write("VCU", "CV,CD,AV,AD,Op,HS,Er,VS,FS,Ok,Fix", "ffffBBBBBBB",
                 v, steer_deg, motion.v, motion.steer_deg,
                 navsta_valid and navsta.op or 0, navsta_valid and navsta.hs or 0,
                 navsta.err, clamp(motion.veh, 0, 255), motion.failsafe and 1 or 0,
                 ok and 1 or 0, fix)
  end

  -- 5Hz 上报地面站 (客户地面站只用这 5 个值, 4G 按流量计费)
  -- VCU_MODE/VCU_FAULT/VCU_OK 始终发送; VCU_BATV/VCU_SOC 仅在收到 HealthSta 时发送
  if now - last_gcs_ms >= 200 then
    last_gcs_ms = now
    -- VCU_MODE: 0 未知(NavSta 无效), 1 遥控/切换中, 2 无人
    gcs:send_named_float("VCU_MODE", (not navsta_valid) and 0 or (navsta.op == OP_NAV and 2 or 1))
    -- VCU_FAULT: Sys_Fault 1~7 直接报; HsErr 报 0x10+码; 驱动器故障报 0x100+码
    local fault_code = 0
    local health_fresh = health.ms > 0 and now - health.ms <= 1500
    if health_fresh and health.sys > 0 then
      fault_code = health.sys
    elseif navsta_valid and navsta.err > 0 then
      fault_code = 0x10 + navsta.err
    elseif health_fresh then
      local drv = health.drv1 > 0 and health.drv1 or (health.drv2 > 0 and health.drv2 or health.drv3)
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
    gcs:send_named_float("VCU_V", motion.v)
    gcs:send_named_float("VCU_W", motion.v * math.tan(math.rad(motion.steer_deg)) / wheelbase)
  end

  return update, LOOP_MS
end

gcs:send_text(MAV_INFO, "VCU: CAN bridge loaded (Nav v1.0)")

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
