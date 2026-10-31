#!/usr/bin/env python3
"""issue #10: Work_On 由 SCRIPTING1 (DO_AUX_FUNCTION 300) 请求, 不跟 AUTO/车速走.
模拟 VCU 按 v1.1 作业序列回报 WorkFb: Work_On 变化 -> 1 持续 LIFT_S 秒 -> 3 (升杆) / 0 (收杆); 可注入 4."""
import sys, threading, time
from pymavlink import mavutil
from vcu_test import Vcu, Mav, ID_STA, ID_MOTION, check, results

LIFT_S = 3.0
AUX_WORK = 300
HOME = (31.93778228, 118.7901755)


class WorkVcu(Vcu):
    """WorkFb 跟随 Nav_Cmd.Work_On; 动杆或故障时行驶锁 0 (MotionSta 车速报 0)."""
    inject_fault = False

    def __init__(self):
        self.work_seen = 0
        self.lift_until = None
        self.lift_target = 0
        super().__init__()
        threading.Thread(target=self._work, daemon=True).start()

    def _work(self):
        while True:
            with self.lock:
                cmd = self.cmds[-1] if self.cmds else None
            if cmd is not None and cmd[3] != self.work_seen:
                self.work_seen = cmd[3]
                self.sta[4] = 1
                self.lift_until = time.time() + LIFT_S
                self.lift_target = 3 if self.work_seen else 0
            if self.lift_until and time.time() >= self.lift_until:
                self.lift_until = None
                self.sta[4] = 4 if self.inject_fault else self.lift_target
            time.sleep(0.02)

    def _tx(self):
        n = 0
        while True:
            if n % 2 == 0 and self.send_sta:
                self.send(ID_STA, self.sta)
            with self.lock:
                v = self.cmds[-1][1] if self.cmds else 0.0
            if self.sta[4] in (1, 4):
                v = 0.0
            raw = int(round(v * 100)) & 0xFFFF
            self.send(ID_MOTION, [raw & 0xFF, raw >> 8, 0, 0, 0, 2, 0, 0])
            n += 1
            time.sleep(0.05)


class WorkMav(Mav):
    def __init__(self, url):
        self.mission_q = []
        self.seq = -1
        super().__init__(url)

    def _rx(self):
        while True:
            msg = self.m.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            t = time.time()
            ty = msg.get_type()
            with self.lock:
                if ty == "STATUSTEXT":
                    self.texts.append((t, msg.text))
                    print(f"    [text] {msg.text}")
                elif ty == "NAMED_VALUE_FLOAT":
                    self.nvf.append((t, msg.name, msg.value))
                elif ty == "HEARTBEAT" and msg.get_srcSystem() == 1 and msg.type != 6:
                    self.armed = bool(msg.base_mode & 128)
                    self.mode = msg.custom_mode
                elif ty in ("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"):
                    self.mission_q.append(msg)
                elif ty == "MISSION_CURRENT":
                    self.seq = msg.seq

    def aux(self, pos):
        self.cmd(mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION, AUX_WORK, pos)

    def upload(self, items):
        """items: (cmd, p1, p2, lat, lon)"""
        self.mission_q.clear()
        self.m.mav.mission_count_send(1, 1, len(items), 0)
        deadline = time.time() + 10
        while time.time() < deadline:
            with self.lock:
                q, self.mission_q = self.mission_q, []
            for msg in q:
                if msg.get_type() == "MISSION_ACK":
                    return msg.type == 0
                c, p1, p2, lat, lon = items[msg.seq]
                self.m.mav.mission_item_int_send(
                    1, 1, msg.seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, c, 0, 1,
                    p1, p2, 0, 0, int(lat * 1e7), int(lon * 1e7), 0, 0)
            time.sleep(0.05)
        return False


def work_bytes(vcu, t0, t1=None):
    with vcu.lock:
        return [c for c in vcu.cmds if c[0] >= t0 and (t1 is None or c[0] <= t1)]


def main():
    vcu = WorkVcu()
    mav = WorkMav("tcp:127.0.0.1:5770")
    time.sleep(3)
    for p, v in (("VCU_RTK_REQ", 0), ("ARMING_CHECK", 0), ("VCU_WORK_EN", 1)):
        mav.set_param(p, v)
    vcu.sta = [1, 1, 7, 0, 0, 0, 0, 0]
    time.sleep(0.6)
    vcu.sta = [2, 3, 7, 0, 0, 0, 0, 0]
    time.sleep(1)
    mav.set_mode(15)
    time.sleep(0.5)
    t_wait = time.time()
    while not mav.armed and time.time() - t_wait < 120:
        mav.arm(True)
        time.sleep(3)
    check("解锁 (GUIDED 静止)", mav.armed)

    # 1. 静止放杆: Work_On=1 -> WorkFb 1 -> 3, 只切 HOLD 不报警, Nav_Cmd 不断流
    t0 = time.time()
    mav.aux(2)
    time.sleep(LIFT_S + 1.5)
    cmds = work_bytes(vcu, t0 + 0.3)
    gaps = [b[0] - a[0] for a, b in zip(cmds, cmds[1:])]
    txt = mav.texts_since(t0)
    check("1. 请求后 Nav_Cmd Work_On=1", cmds and all(c[3] == 1 for c in cmds), f"n={len(cmds)}")
    check("1. 静止放杆 WorkFb 1->3", vcu.sta[4] == 3)
    check("1. 静止放杆不报 HOLD 警告", not any(", HOLD" in x for x in txt), str(txt))
    check("1. 放杆期间 Nav_Cmd 不断流", cmds and max(gaps) < 0.08, f"max_gap={max(gaps):.3f}")
    wr = mav.nvf_since(t0, "VCU_WREQ")
    check("1. VCU_WREQ=1 立即发, 之后约 1 Hz", wr and wr[0][1] == 1 and wr[0][0] - t0 < 0.5 and len(wr) >= 3,
          str([(round(t - t0, 2), v) for t, v in wr][:6]))

    # 2/3. AUTO 作业: 起点 -> 终点 -> 航线末尾 DO_AUX_FUNCTION 低 (收杆兜底)
    d = 0.00011   # 约 12 m
    ok = mav.upload([
        (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, HOME[0], HOME[1]),
        (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, HOME[0] + d, HOME[1]),
        (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, HOME[0] + d, HOME[1] + d * 1.2),
        (mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION, AUX_WORK, 0, 0, 0),
    ])
    check("上传 AB 线航线 (末尾 DO_AUX_FUNCTION 300 低)", ok)
    t0 = time.time()
    mav.cmd(mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0)
    mav.set_mode(10)
    t_end = None
    while time.time() - t0 < 90:
        if mav.seq >= 3 or work_bytes(vcu, t0) and work_bytes(vcu, t0)[-1][3] == 0:
            t_end = time.time()
            break
        time.sleep(0.2)
    with mav.lock:
        t_wp2 = next((tt for tt, x in mav.texts if tt >= t0 and x.startswith("Reached waypoint #2")), t_end)
    drive = work_bytes(vcu, t0 + 1, t_wp2 - 0.3)
    vs = [c[1] for c in drive]
    check("2. AUTO 行驶中 Work_On 保持 1 (含航点处减速)",
          drive and max(vs) > 0.3 and all(c[3] == 1 for c in drive),
          f"n={len(drive)} v_min={min(vs) if vs else None:.2f} v_max={max(vs) if vs else None:.2f}")
    time.sleep(LIFT_S + 2)
    after = work_bytes(vcu, (t_end or time.time()) + 0.5)
    txt = mav.texts_since(t0)
    check("3. 航线末尾自动收杆: Work_On=0, WorkFb 1->0, 不报 HOLD 警告",
          after and all(c[3] == 0 for c in after) and vcu.sta[4] == 0 and not any(", HOLD" in x for x in txt),
          f"WorkFb={vcu.sta[4]} {[x for x in txt if 'VCU' in x]}")

    # 4. 失权时请求清零, 恢复授权后保持 0
    mav.set_mode(4)
    mav.aux(2)
    time.sleep(LIFT_S + 1)
    t0 = time.time()
    vcu.sta = [1, 0, 7, 0, vcu.sta[4], 0, 0, 0]       # OpSource=1 失权
    time.sleep(1)
    txt = mav.texts_since(t0)
    wr = mav.nvf_since(t0, "VCU_WREQ")
    check("4. 失权: 提示 work request cleared, VCU_WREQ=0",
          "VCU: work request cleared" in txt and wr and wr[0][1] == 0, f"{txt} {wr[:2]}")
    vcu.sta = [2, 3, 8, 0, vcu.sta[4], 0, 0, 0]       # 新会话授权
    time.sleep(1.5)
    re = work_bytes(vcu, time.time() - 1)
    check("4. 恢复授权后 Work_On 保持 0", re and all(c[3] == 0 for c in re), f"n={len(re)}")
    time.sleep(LIFT_S + 1)

    # 5. 注入提升超时故障 4: 解锁失败 lift fault
    mav.arm(False)
    time.sleep(1)
    vcu.inject_fault = True
    t0 = time.time()
    mav.aux(2)
    time.sleep(LIFT_S + 1)
    mav.arm(True)
    time.sleep(1.5)
    txt = mav.texts_since(t0)
    check("5. 注入 WorkFb=4: 提示 lift fault, 解锁失败",
          vcu.sta[4] == 4 and "VCU: lift fault" in txt and not mav.armed and any("Arm: VCU: lift fault" in x for x in txt),
          str([x for x in txt if 'VCU' in x]))
    vcu.inject_fault = False
    mav.aux(0)
    time.sleep(LIFT_S + 1)

    # 6. VCU_WORK_EN=0: 请求高也恒发 0
    mav.set_param("VCU_WORK_EN", 0)
    time.sleep(0.5)
    t0 = time.time()
    mav.aux(2)
    time.sleep(1.5)
    c6 = work_bytes(vcu, t0 + 0.2)
    check("6. VCU_WORK_EN=0: Work_On 恒 0", c6 and all(c[3] == 0 for c in c6), f"n={len(c6)}")

    errs = [x for x in mav.texts_since(0) if x.startswith("VCU: error")]
    check("全程脚本无报错", not errs, str(errs[:2]))
    print()
    print(f"{sum(ok for _, ok in results)}/{len(results)} PASS")
    sys.exit(0 if all(ok for _, ok in results) else 1)


if __name__ == "__main__":
    main()
