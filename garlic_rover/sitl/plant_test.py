#!/usr/bin/env python3
"""车辆模型 (garlic_plant.py) 下的跑直 / 起步场景.

用法: plant_test.py <场景> <plant 日志> [参数 k=v ...]
  场景 ab   : AUTO 跑 AB 直线 (起点在车下, 沿车头方向 LEN 米)
  场景 goto : GUIDED 开到车头前方 LEN 米
  额外 k=v 作为飞控参数写入 (如 ATC_STR_ACC_MAX=10)
评估用车辆模型的真实位置 (plant 日志), 不用飞控估计.
"""
import math, sys, time
from pymavlink import mavutil

scene, plog = sys.argv[1], sys.argv[2]
extra = {}
LEN = 40.0
for a in sys.argv[3:]:
    k, v = a.split("=", 1)
    if k == "LEN":
        LEN = float(v)
    else:
        extra[k] = float(v)

# 实车参数 (05_测试记录 bin, 2026-11-01)
REAL = dict(WP_SPEED=0.55, CRUISE_SPEED=0.55, CRUISE_THROTTLE=35, WP_ACCEL=0.5, WP_JERK=1.0, WP_RADIUS=0.5,
            ATC_TURN_MAX_G=0.15, PSC_POS_P=0.2, PSC_VEL_P=1.0, PSC_VEL_I=0, PSC_VEL_D=0, TURN_RADIUS=2.1,
            GUID_OPTIONS=0, WP_PIVOT_ANGLE=0, ATC_STR_RAT_FF=0.2, ATC_STR_RAT_P=0, ATC_STR_RAT_I=0,
            ATC_STR_RAT_MAX=45, ATC_STR_ACC_MAX=60, ATC_STR_ANG_P=2.0, ATC_ACCEL_MAX=0.5, ATC_DECEL_MAX=0.8,
            ATC_SPEED_P=0.01, ATC_SPEED_I=0, VCU_RTK_REQ=0, VCU_W_BASE=1.2, VCU_VREF_MIN=0.3,
            ARMING_CHECK=0, FS_GCS_ENABLE=0, FS_CRASH_CHECK=0)
REAL.update(extra)

m = mavutil.mavlink_connection("tcp:127.0.0.1:5770", source_system=255)
m.wait_heartbeat()
M = mavutil.mavlink
m.mav.request_data_stream_send(1, 1, M.MAV_DATA_STREAM_ALL, 5, 1)


def pump(dur):
    t0 = time.time()
    while time.time() - t0 < dur:
        x = m.recv_match(blocking=True, timeout=0.1)
        if x is not None and x.get_type() == "STATUSTEXT" and "PreArm" not in x.text:
            print("    [text]", x.text)


def setp(k, v):
    for _ in range(3):
        m.mav.param_set_send(1, 1, k.encode(), v, M.MAV_PARAM_TYPE_REAL32)
        t1 = time.time()
        while time.time() - t1 < 1.0:
            x = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if x is not None and x.param_id == k:
                return
    print("    参数未确认", k)


for k, v in REAL.items():
    setp(k, v)
print("参数已设:", " ".join("%s=%g" % (k, v) for k, v in extra.items()) or "(实车原参数)")

# 等定位
t0 = time.time()
gp = None
while time.time() - t0 < 120:
    x = m.recv_match(type=["GLOBAL_POSITION_INT", "EKF_STATUS_REPORT"], blocking=True, timeout=1)
    if x is not None and x.get_type() == "EKF_STATUS_REPORT" and (x.flags & 0x10) and gp is not None and time.time() - t0 > 20:
        break
    if x is not None and x.get_type() == "GLOBAL_POSITION_INT" and x.lat != 0:
        gp = x
gp = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2) or gp
lat0, lon0 = gp.lat / 1e7, gp.lon / 1e7
hdg = gp.hdg / 100.0
k_e = 111319.5 * math.cos(math.radians(lat0))
tlat = lat0 + LEN * math.cos(math.radians(hdg)) / 111319.5
tlon = lon0 + LEN * math.sin(math.radians(hdg)) / k_e
print("起点 %.7f %.7f 航向 %.1f，目标 %.0f m" % (lat0, lon0, hdg, LEN))


def mode(n):
    m.mav.command_long_send(1, 1, M.MAV_CMD_DO_SET_MODE, 0, 1, n, 0, 0, 0, 0, 0)


def arm(a):
    m.mav.command_long_send(1, 1, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1 if a else 0, 0 if a else 21196, 0, 0, 0, 0, 0)


if scene == "ab":
    items = [(lat0, lon0, 16), (0, 0, 178), (lat0, lon0, 16), (tlat, tlon, 16)]
    m.mav.mission_count_send(1, 1, len(items), 0)
    for _ in range(len(items)):
        req = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT"], blocking=True, timeout=3)
        s = req.seq
        la, lo, c = items[s]
        p = (2, 0.55, -1, 0) if c == 178 else (0, 0, 0, 0)
        m.mav.mission_item_int_send(1, 1, s, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, c, 0, 1, *p,
                                    int(la * 1e7), int(lo * 1e7), 0, 0)
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=3)
    print("航线上传", ack and ack.type)
    mode(15); arm(True); open("/tmp/plant_go", "w").close(); pump(1.0)
    mode(10); m.mav.command_long_send(1, 1, M.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
else:
    mode(15); arm(True); open("/tmp/plant_go", "w").close(); pump(1.0)
    m.mav.command_int_send(1, 1, M.MAV_FRAME_GLOBAL, M.MAV_CMD_DO_REPOSITION, 0, 0, -1, 1, 0, float("nan"),
                           int(round(tlat * 1e7)), int(round(tlon * 1e7)), 0)
t_start = time.time()
pump(LEN / 1.7 * 2 + 12)
mode(4); arm(False); pump(1)

# 评估: plant 日志里起步后的真实轨迹
rows = [list(map(float, l.split())) for l in open(plog) if l.strip()]
# 起点 = 第一次车速 > 0 前的位置
i0 = next((i for i, r in enumerate(rows) if r[4] > 0.01), None)
if i0 is None:
    print("RESULT 车没有动"); sys.exit()
n0, e0 = rows[i0][1], rows[i0][2]
ux, uy = math.cos(math.radians(hdg)), math.sin(math.radians(hdg))
xt, along, yaws = [], [], []
for r in rows[i0:]:
    dn, de = r[1] - n0, r[2] - e0
    a = dn * ux + de * uy
    if a > LEN + 2: break
    xt.append(dn * uy - de * ux)       # 右为正
    along.append(a); yaws.append(r[3])
end = rows[i0 + len(xt) - 1]
dfin = math.hypot(end[1] - n0 - LEN * ux, end[2] - e0 - LEN * uy)
first5 = [abs(x) for x, a in zip(xt, along) if a < 5]
tail = [x for x, a in zip(xt, along) if a > 10]
rms_tail = math.sqrt(sum(x * x for x in tail) / len(tail)) if tail else float("nan")
dyaw = [((y - hdg + 180) % 360) - 180 for y in yaws]
print("RESULT 前5m最大偏离 %.2f m | 全程最大偏离 %.2f m | 10m后RMS %.2f m | 航向偏差 %.0f~%.0f° | 沿线最远 %.1f m | 终点距目标 %.1f m" % (
    max(first5) if first5 else float("nan"), max(abs(x) for x in xt), rms_tail, min(dyaw), max(dyaw), max(along), dfin))
prof = []
for d in range(0, int(LEN) + 1, 4):
    j = min(range(len(along)), key=lambda i: abs(along[i] - d))
    prof.append("%d:%+.2f" % (d, xt[j]))
print("PROFILE 沿线距离:偏差(右正) " + " ".join(prof))
print("START 起步时前轮 %.1f°" % rows[i0][5])
# 模型 STALL_S>0 时 (日志第 9 列): 停顿结束后多久重新给车速, 车有没有重新开起来
stall = [i for i, r in enumerate(rows) if len(r) > 8 and r[8]]
if stall:
    j = stall[-1] + 1
    t_end = rows[j][0] if j < len(rows) else rows[-1][0]
    after = [r for r in rows[j:] if r[0] - t_end <= 6.0]
    resume = next((r[0] - t_end for r in after if r[6] >= 0.05), None)
    moved = next((r[0] - t_end for r in after if r[4] > 0.3), None)
    # 停顿开始到结束后 3 s 内, 车速指令被压成 0 的时长 (日志 20 Hz)
    win = [r for r in rows if rows[stall[0]][0] <= r[0] <= t_end + 3.0]
    zero_s = sum(1 for r in win if r[6] < 0.01) * 0.05
    print("STALL 停顿 %.1f s，停顿中前轮 %.1f° | 停顿起到停顿后 3 s 车速指令为 0 共 %.1f s | 停顿后重新给车速 %s | 重新开起来 %s" % (
        rows[stall[-1]][0] - rows[stall[0]][0], rows[stall[-1]][5], zero_s,
        "%.1f s" % resume if resume is not None else "无", "%.1f s" % moved if moved is not None else "无"))
