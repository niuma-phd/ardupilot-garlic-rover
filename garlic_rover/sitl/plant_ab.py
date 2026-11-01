#!/usr/bin/env python3
"""车辆模型下按地面站 AB 作业流程连续测试: 对准任务 (运输车速) -> 作业任务 (作业车速).

地面站 HHUABRun: 对准任务 = home, DO_CHANGE_SPEED 运输, 引导点 (A 前 leadD+leadLc), 对准点 (A 前 leadD);
到对准点停稳 1.5 s 判对准 (|横偏| <= 0.20 m, |航向差| <= 8 deg). 作业任务 = home, DO_CHANGE_SPEED 作业, A, B.

用法: plant_ab.py <plant 日志> [k=v ...]
  车辆模型以 home 为原点, 初始车头 YAW0 (由 run_plant.sh 的 plant 参数给).
  A_N/A_E: A 点相对 home (m), LINE_HDG: AB 方向 (deg), LEN: AB 长度, V_TRANS / V_WORK: 两段的实际车速
  其余 k=v 写入飞控参数
"""
import math, sys, time
from pymavlink import mavutil

plog = sys.argv[1]
O = dict(SKIP_ALIGN=0.0, A_N=12.0, A_E=6.0, LINE_HDG=0.0, LEN=30.0, LEAD_D=1.0, LEAD_LC=5.0, V_TRANS=1.38, V_WORK=0.55,
         SPD_TRANS=1.39, SPD_WORK=0.55)
extra = {}
for a in sys.argv[2:]:
    k, v = a.split("=", 1)
    if k in O:
        O[k] = float(v)
    else:
        extra[k] = float(v)

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
st = {"reached": -1, "mode": -1}


def pump(dur, until=None):
    t0 = time.time()
    while time.time() - t0 < dur:
        x = m.recv_match(blocking=True, timeout=0.1)
        if x is None:
            continue
        ty = x.get_type()
        if ty == "STATUSTEXT" and "PreArm" not in x.text:
            print("    [text]", x.text)
        elif ty == "MISSION_ITEM_REACHED":
            st["reached"] = x.seq
        elif ty == "HEARTBEAT" and x.get_srcSystem() == 1 and x.type != 6:
            st["mode"] = x.custom_mode
        if until and until():
            return True
    return False


def setp(k, v):
    for _ in range(3):
        m.mav.param_set_send(1, 1, k.encode(), v, M.MAV_PARAM_TYPE_REAL32)
        t1 = time.time()
        while time.time() - t1 < 1.0:
            x = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if x is not None and x.param_id == k:
                return


for k, v in REAL.items():
    setp(k, v)
print("飞控参数:", " ".join("%s=%g" % (k, v) for k, v in extra.items()) or "(实车原参数)")

# 等定位
t0 = time.time()
while time.time() - t0 < 120:
    x = m.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=1)
    if x is not None and (x.flags & 0x10) and time.time() - t0 > 20:
        break
home = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
lat0, lon0 = home.lat / 1e7, home.lon / 1e7
k_e = 111319.5 * math.cos(math.radians(lat0))
to_ll = lambda n, e: (lat0 + n / 111319.5, lon0 + e / k_e)
h = math.radians(O["LINE_HDG"])
ux, uy = math.cos(h), math.sin(h)
A = (O["A_N"], O["A_E"])
along_pt = lambda d: (A[0] + d * ux, A[1] + d * uy)
LEAD = along_pt(-(O["LEAD_D"] + O["LEAD_LC"]))
ALIGN = along_pt(-O["LEAD_D"])
B = along_pt(O["LEN"])


def mode(n):
    m.mav.command_long_send(1, 1, M.MAV_CMD_DO_SET_MODE, 0, 1, n, 0, 0, 0, 0, 0)


def upload(points, speed):
    items = [(lat0, lon0, 16, 0)] + [(0, 0, 178, speed)] + [(*to_ll(*p), 16, 0) for p in points]
    m.mav.mission_count_send(1, 1, len(items), 0)
    for _ in range(len(items)):
        req = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT"], blocking=True, timeout=3)
        la, lo, c, sp = items[req.seq]
        p = (1, sp, -1, 0) if c == 178 else (0, 0, 0, 0)
        m.mav.mission_item_int_send(1, 1, req.seq, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, c, 0, 1, *p,
                                    int(round(la * 1e7)), int(round(lo * 1e7)), 0, 0)
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=3)
    return ack is not None and ack.type == 0


def last_t():
    with open(plog) as f:
        lines = [l for l in f if l.strip()]
    return float(lines[-1].split()[0])


def run_mission(points, speed, vrun, timeout, first):
    open("/tmp/plant_vrun", "w").write("%g" % vrun)
    if not upload(points, speed):
        print("    航线上传失败")
    st["reached"] = -1
    if first:
        # 地面站: 未解锁时 GUIDED 解锁后 MISSION_START
        mode(10)
        m.mav.command_long_send(1, 1, M.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
    else:
        # 地面站: 对准后放杆期间车在 HOLD, _beginWork 再 setCurrentMissionSequence(1) + 切 AUTO
        mode(4)
        pump(1.0)
        m.mav.mission_set_current_send(1, 1, 1)
        pump(0.3)
        mode(10)
    last_seq = len(points) + 1
    ok = pump(timeout, until=lambda: st["reached"] >= last_seq)
    return ok


# ---- 对准任务 (运输车速) ----
mode(15)
m.mav.command_long_send(1, 1, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
open("/tmp/plant_go", "w").close()
pump(1.0)
t_a0 = last_t()
if O["SKIP_ALIGN"]:
    # 手动对准后直接作业: 车停在 A 前, 带初始横偏 / 航向差
    ok1 = True
    t_a1 = t_a0
    ok2 = run_mission([A, B], O["SPD_WORK"], O["V_WORK"], O["LEN"] / max(O["V_WORK"], 0.2) * 2 + 30, True)
else:
    ok1 = run_mission([LEAD, ALIGN], O["SPD_TRANS"], O["V_TRANS"], 90, True)
    pump(1.5)                                   # 停稳 (kSettleMs)
    t_a1 = last_t()
    # ---- 作业任务 (作业车速) ----
    ok2 = run_mission([A, B], O["SPD_WORK"], O["V_WORK"], O["LEN"] / max(O["V_WORK"], 0.2) * 2 + 30, False)
pump(2)
t_b1 = last_t()
mode(4)
m.mav.command_long_send(1, 1, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
pump(1)

# ---- 评估 (车辆模型真实位置) ----
rows = [list(map(float, l.split())) for l in open(plog) if l.strip()]
def seg(ta, tb):
    return [r for r in rows if ta <= r[0] <= tb]
def xt_of(r):                     # 相对 AB 线, 右为正
    dn, de = r[1] - A[0], r[2] - A[1]
    return dn * uy - de * ux, dn * ux + de * uy
def herr(r):
    return ((r[3] - O["LINE_HDG"] + 180) % 360) - 180

al = seg(t_a0, t_a1) if not O["SKIP_ALIGN"] else []
moved = [r for r in al if r[4] > 0.01]
t_move = moved[0][0] if moved else float("nan")
# 引导段: 起步到距引导点最近处; 对准段: 引导点之后
dl = [math.hypot(r[1] - LEAD[0], r[2] - LEAD[1]) for r in al]
i_lead = min(range(len(al)), key=lambda i: dl[i]) if al else 0
guide, lead_in = al[:i_lead + 1], al[i_lead:]
yaws = [r[3] for r in guide]
turned = 0.0
for a, b in zip(yaws, yaws[1:]):
    turned += ((b - a + 180) % 360) - 180
end_r = al[-1]
xt_end, along_end = xt_of(end_r)
li_xt = [abs(xt_of(r)[0]) for r in lead_in if r[4] > 0.01]
if al: print("RESULT 对准任务: %s，起步等待 %.1f s，引导段最近经过引导点 %.2f m、累计转向 %.0f°；对准段最大横偏 %.2f m；"
      "停稳后 横偏 %+.2f m（限 0.20）航向差 %+.1f°（限 8）距对准点 %.2f m → %s" % (
          "到达对准点" if ok1 else "未到对准点", t_move - t_a0, dl[i_lead], turned,
          max(li_xt) if li_xt else float("nan"), xt_end, herr(end_r),
          math.hypot(end_r[1] - ALIGN[0], end_r[2] - ALIGN[1]),
          "对准合格" if abs(xt_end) <= 0.20 and abs(herr(end_r)) <= 8 else "对准不合格"))
wk = seg(t_a1, t_b1)
pts = [(xt_of(r), r) for r in wk]
on_ab = [x for (x, a), r in pts if 0 <= a <= O["LEN"]]
tail = [x for (x, a), r in pts if 5 <= a <= O["LEN"]]
end_w = wk[-1]
hd = [herr(r) for (x, a), r in pts if 0 <= a <= O["LEN"]]
print("RESULT 作业任务: %s，A→B 最大横偏 %.2f m，A 后 5 m 起 RMS %.2f m，航向差 %.0f~%.0f°，终点距 B %.2f m" % (
    "到达 B" if ok2 else "未到 B", max(abs(x) for x in on_ab) if on_ab else float("nan"),
    math.sqrt(sum(x * x for x in tail) / len(tail)) if tail else float("nan"),
    min(hd) if hd else float("nan"), max(hd) if hd else float("nan"),
    math.hypot(end_w[1] - B[0], end_w[2] - B[1])))
def settle(th):
    xs = [(a, x) for (x, a), r in pts if a >= -O["LEAD_D"]]
    for i in range(len(xs)):
        if all(abs(x) <= th for a, x in xs[i:] if a <= O["LEN"]):
            return xs[i][0]
    return float("nan")
x0 = xt_of(wk[0])[0] if wk else float("nan")
print("RESULT 收敛: 起步横偏 %+.2f m 航向差 %+.1f°；横偏进入并保持 ≤0.20 m 于 A 后 %.1f m，≤0.10 m 于 A 后 %.1f m；最大超调(反向) %.2f m" % (
    x0, herr(wk[0]) if wk else float("nan"), settle(0.20), settle(0.10),
    max([0.0] + [-x if x0 > 0 else x for (x, a), r in pts if 0 <= a <= O["LEN"]])))
prof = []
for d in range(0, int(O["LEN"]) + 1, 3):
    c = [(abs(a - d), x) for (x, a), r in pts]
    if c:
        prof.append("%d:%+.2f" % (d, min(c)[1]))
print("PROFILE A→B 沿线距离:横偏(右正) " + " ".join(prof))
