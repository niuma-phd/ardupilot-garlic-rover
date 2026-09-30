--[[
 vcu_can.lua  大蒜播种车 导航规划器 -> 底盘VCU  CAN 桥接脚本
 固件: ArduPilot Rover 4.7.1 + 补丁 (飞控 X-MAV AP-H743v3)

 功能:
   1. 以 50Hz 向 VCU 发送目标线速度 v (m/s) 和目标角速度 w (rad/s, 逆时针为正)  -> 0x101
   2. 接收 VCU 运动反馈 0x201 / 状态反馈 0x202, 超时/非自动/故障/急停 时禁止解锁; 行驶中则切 HOLD 并报警
   2b. RTK 检查: 未达到 VCU_RTK_REQ 要求时禁止解锁; 行驶中掉解超过 VCU_RTK_TMO 切 HOLD
   2c. 板载日志 VCU 消息: 指令 v/w 与 VCU 实际 v/w、转角、模式、故障、定位状态
   3. 把 VCU 反馈以 NAMED_VALUE_FLOAT 发到地面站 (QGC 可显示)
 注: STATUSTEXT 只有50字节, 故脚本内提示文字用英文, 由QGC定制版翻译显示

 v/w 来源: AR_AttitudeControl:get_desired_speed()/get_desired_turn_rate()
   需要本仓库编译的固件 (Release 中的 *.apj; 官方固件没有这两个绑定), 官方固件下脚本不启动

 协议定义见 garlic_rover/docs/CAN通信协议.md (标准帧, 小端 Intel, 最后一字节 XOR 校验)

 所需参数(示例, CAN2 接 VCU):
   SCR_ENABLE=1  CAN_P2_DRIVER=2  CAN_D2_PROTOCOL=10  CAN_P2_BITRATE=500000
--]]

local LOOP_MS = 20

-- MAV_SEVERITY
local MAV_EMERG, MAV_CRIT, MAV_WARN, MAV_INFO = 0, 2, 4, 6

-- Rover 模式号
local MODE_HOLD = 4
local DRIVE_MODES = { [5] = true, [10] = true, [11] = true, [12] = true, [15] = true }  -- LOITER AUTO RTL SMART_RTL GUIDED

-- CAN ID
local ID_CMD      = 0x101
local ID_FB_MOTION = 0x201
local ID_FB_STATUS = 0x202

-- VCU 模式 (0x202 byte0)
local VCU_MODE_AUTO = 2

---------------------------------------------------------------------------
-- 参数表 VCU_*
---------------------------------------------------------------------------
local PARAM_TABLE_KEY = 71
assert(param:add_table(PARAM_TABLE_KEY, "VCU_", 8), "VCU: could not add param table")
local function bind_add_param(name, idx, default)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default), "VCU: could not add param " .. name)
  return Parameter("VCU_" .. name)
end
--[[ @Param: VCU_ENABLE  @DisplayName: 启用VCU CAN桥接  @Values: 0:关,1:开 --]]
local VCU_ENABLE  = bind_add_param("ENABLE", 1, 1)
-- 参数序号 2 (原 VCU_SRC) 已废弃, 保留空位, 其它参数序号不变
--[[ @Param: VCU_FB_TMO  @DisplayName: VCU反馈超时(ms), 0=不检查反馈(台架调试用) --]]
local VCU_FB_TMO  = bind_add_param("FB_TMO", 3, 200)
--[[ @Param: VCU_V_MAX  @DisplayName: 线速度限幅(m/s) --]]
local VCU_V_MAX   = bind_add_param("V_MAX", 4, 1.5)
--[[ @Param: VCU_W_MAX  @DisplayName: 角速度限幅(rad/s) --]]
local VCU_W_MAX   = bind_add_param("W_MAX", 5, 1.0)
--[[ @Param: VCU_REV  @DisplayName: 允许倒车  @Values: 0:否,1:是 --]]
local VCU_REV     = bind_add_param("REV", 6, 0)
--[[ @Param: VCU_RTK_REQ  @DisplayName: RTK要求  @Values: 0:不检查,1:浮点解或固定解,2:仅固定解 --]]
local VCU_RTK_REQ = bind_add_param("RTK_REQ", 7, 2)
--[[ @Param: VCU_RTK_TMO  @DisplayName: RTK不满足要求持续多久(ms)后停车 --]]
local VCU_RTK_TMO = bind_add_param("RTK_TMO", 8, 3000)

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

-- 解锁前检查: VCU 未就绪时禁止解锁
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

local function xor_ok(frame)
  local x = 0
  for i = 0, 6 do x = x ~ frame:data(i) end
  return x == frame:data(7)
end

---------------------------------------------------------------------------
-- v / w 来源: 读取控制器期望值.  ArduPilot 转向率顺时针为正, 这里取反成逆时针为正
---------------------------------------------------------------------------
local function get_vw()
  return AR_AttitudeControl:get_desired_speed(), -AR_AttitudeControl:get_desired_turn_rate()
end

---------------------------------------------------------------------------
-- 反馈状态
---------------------------------------------------------------------------
local fb = {
  motion_ms = 0, status_ms = 0,
  v = 0, w = 0, steer_deg = 0,
  mode = -1, estop = false, fault = false, fault_code = 0,
  bat_v = 0, soc = 0,
}

local function read_frames()
  for _ = 1, 25 do
    local f = can:read_frame()
    if not f then return end
    if (not f:isExtended()) and f:dlc() == 8 and xor_ok(f) then
      local id = f:id_signed()
      if id == ID_FB_MOTION then
        fb.v = get_i16(f, 0) * 0.001
        fb.w = get_i16(f, 2) * 0.001
        fb.steer_deg = get_i16(f, 4) * 0.01
        fb.motion_ms = millis():toint()
      elseif id == ID_FB_STATUS then
        fb.mode = f:data(0)
        fb.estop = (f:data(1) & 0x01) ~= 0
        fb.fault = (f:data(1) & 0x02) ~= 0
        fb.fault_code = get_u16(f, 2)
        fb.bat_v = get_u16(f, 4) * 0.01
        fb.soc = f:data(6)
        fb.status_ms = millis():toint()
      end
    end
  end
end

-- 返回 ok, 原因
local function vcu_ready(now)
  local tmo = VCU_FB_TMO:get()
  if tmo <= 0 then return true, nil end
  -- 0x201 为 50Hz, 0x202 为 10Hz, 状态帧超时放宽 300ms
  if now - fb.motion_ms > tmo or now - fb.status_ms > tmo + 300 then return false, "feedback timeout" end
  if fb.estop then return false, "estop" end
  if fb.fault then return false, string.format("fault 0x%04X", fb.fault_code) end
  if fb.mode ~= VCU_MODE_AUTO then return false, "not in auto" end
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

---------------------------------------------------------------------------
-- 发送
---------------------------------------------------------------------------
local counter = 0
local function send_cmd(v, w, auto_en, estop_req)
  local f = CANFrame()
  f:id(ID_CMD)
  f:dlc(8)
  put_i16(f, 0, to_i16(v * 1000))
  put_i16(f, 2, to_i16(w * 1000))
  local flags = 0
  if auto_en then flags = flags | 0x01 end
  if estop_req then flags = flags | 0x02 end
  if v < 0 then flags = flags | 0x04 end
  f:data(4, flags)
  f:data(5, 0)
  f:data(6, counter & 0x0F)
  local x = 0
  for i = 0, 6 do x = x ~ f:data(i) end
  f:data(7, x)
  counter = (counter + 1) & 0x0F
  can:write_frame(f, 10000)
end

---------------------------------------------------------------------------
-- 主循环
---------------------------------------------------------------------------
local last_warn_ms = 0
local last_gcs_ms = 0

local function update()
  local now = millis():toint()
  read_frames()

  if VCU_ENABLE:get() <= 0 then
    return update, LOOP_MS
  end

  local armed = arming:is_armed()
  local mode = vehicle:get_mode()
  local driving = armed and DRIVE_MODES[mode] == true
  local estop_req = SRV_Channels:get_emergency_stop()
  local vcu_ok, why = vcu_ready(now)
  local rtk_now, rtk_keep, fix = rtk_check(now)
  -- 解锁前: VCU 就绪且当前满足 RTK 要求
  if auth_id then
    if not vcu_ok then arming:set_aux_auth_failed(auth_id, "VCU: " .. why)
    elseif not rtk_now then arming:set_aux_auth_failed(auth_id, "VCU: no RTK fix")
    else arming:set_aux_auth_passed(auth_id) end
  end
  -- 行驶中: VCU 就绪, RTK 允许短暂掉解
  local ok = vcu_ok and rtk_keep
  if vcu_ok and not rtk_keep then why = "RTK lost" end

  local v, w = 0, 0
  if driving and not estop_req then
    if ok then
      v, w = get_vw()
      if VCU_REV:get() <= 0 and v < 0 then v = 0 end
      v = clamp(v, -VCU_V_MAX:get(), VCU_V_MAX:get())
      w = clamp(w, -VCU_W_MAX:get(), VCU_W_MAX:get())
    else
      vehicle:set_mode(MODE_HOLD)
      driving = false
      if now - last_warn_ms > 2000 then
        gcs:send_text(MAV_WARN, "VCU: " .. why .. ", HOLD")
        last_warn_ms = now
      end
    end
  end

  send_cmd(v, w, driving and not estop_req, estop_req)

  -- 板载日志 (50Hz), 用于对比指令与底盘实际响应
  logger:write("VCU", "CV,CW,V,W,Str,Md,Flt,Ok,Fix", "fffffBHBB",
               v, w, fb.v, fb.w, fb.steer_deg, clamp(fb.mode, 0, 255), fb.fault_code,
               ok and 1 or 0, fix)

  -- 5Hz 上报地面站
  if now - last_gcs_ms >= 200 then
    last_gcs_ms = now
    gcs:send_named_float("VCU_CMD_V", v)
    gcs:send_named_float("VCU_CMD_W", w)
    gcs:send_named_float("VCU_V", fb.v)
    gcs:send_named_float("VCU_W", fb.w)
    gcs:send_named_float("VCU_MODE", fb.mode)
    gcs:send_named_float("VCU_FAULT", fb.fault_code)
    gcs:send_named_float("VCU_BATV", fb.bat_v)
    gcs:send_named_float("VCU_SOC", fb.soc)
    gcs:send_named_float("VCU_OK", ok and 1 or 0)
  end

  return update, LOOP_MS
end

gcs:send_text(MAV_INFO, "VCU: CAN bridge loaded")

-- 出错时停车并报警, 1s 后重启脚本
local function protected_wrapper()
  local success, err = pcall(update)
  if not success then
    gcs:send_text(MAV_EMERG, "VCU: error " .. tostring(err))
    pcall(send_cmd, 0, 0, false, false)
    return protected_wrapper, 1000
  end
  return protected_wrapper, LOOP_MS
end

return protected_wrapper()
