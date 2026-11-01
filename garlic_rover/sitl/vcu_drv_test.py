#!/usr/bin/env python3
"""协议外状态帧 (MotionSta / HealthSta) 只上报不拦车 (VCU_FB_CHK=0, 默认):
Sys_Fault / 驱动器故障 / Failsafe / MotionSta 停发都不禁止解锁、不切 HOLD; VCU_FAULT 只含 HsErr,
Sys_Fault 报 VCU_SYSFLT, 驱动器故障报 VCU_DRVFLT / VCU_DRVCH, 变化时提示.
VCU_FB_CHK=1 (garlic.15 做法): 反馈超时 / Failsafe / Sys_Fault 禁止解锁、行驶中切 HOLD,
Sys_Fault 和驱动器故障 (0x100+码) 进 VCU_FAULT; 驱动器故障飞控仍不拦."""
import sys, threading, time
from pymavlink import mavutil
from vcu_test import Vcu, Mav, ID_STA, ID_MOTION, check, results

ID_HEALTH = 0x1806407D
HOME = (31.93778228, 118.7901755)


class HealthVcu(Vcu):
    """MotionSta 可控 (Failsafe 位 / 停发), 另发 HealthSta (500 ms): SOC 80 %, 50.0 V, Drv1/2/3, Sys_Fault"""

    def __init__(self):
        self.drv = [0, 0, 0]
        self.sys = 0
        self.failsafe = False
        self.send_motion = True
        super().__init__()
        threading.Thread(target=self._health, daemon=True).start()

    def _tx(self):
        n = 0
        while True:
            if n % 2 == 0 and self.send_sta:
                self.send(ID_STA, self.sta)
            if self.send_motion:
                # 静止, Veh_State=2 运行, bit5 Failsafe
                self.send(ID_MOTION, [0, 0, 0, 0, 0, 2 | (0x20 if self.failsafe else 0), 0, 0])
            n += 1
            time.sleep(0.05)

    def _health(self):
        while True:
            self.send(ID_HEALTH, [80, 500 & 0xFF, 500 >> 8, *self.drv, self.sys, 0])
            time.sleep(0.5)


class SevMav(Mav):
    """另记 STATUSTEXT 的严重级别"""

    def __init__(self, url):
        self.sev = []
        super().__init__(url)
        self.m.message_hooks.append(self._hook)

    def _hook(self, _conn, msg):
        if msg.get_type() == "STATUSTEXT":
            self.sev.append((time.time(), msg.severity, msg.text))


def main():
    vcu = HealthVcu()
    mav = SevMav("tcp:127.0.0.1:5770")
    time.sleep(3)
    for p, v in (("VCU_RTK_REQ", 0), ("VCU_DBG_HZ", 0), ("ARMING_CHECK", 0)):
        mav.set_param(p, v)
    time.sleep(1)

    # 握手授权, HS=3
    vcu.sta = [1, 1, 7, 0, 0, 0, 0, 0]
    time.sleep(0.6)
    vcu.sta = [2, 3, 7, 0, 0, 0, 0, 0]
    time.sleep(1.0)

    def last(name, t0):
        v = mav.nvf_since(t0, name)
        return v[-1][1] if v else None

    def driving_ok(name, t0):
        with vcu.lock:
            vs = [c[1] for c in vcu.cmds if c[0] > t0 + 0.5]
        check(name, mav.mode == 15 and mav.armed and vs and max(vs) > 0.2,
              f"mode={mav.mode} v_max={max(vs) if vs else None} {[x for x in mav.texts_since(t0) if 'VCU' in x]}")

    def try_arm(name, expect_ok, expect_text=None):
        if mav.armed:
            mav.arm(False)
            time.sleep(1)
        mav.set_mode(15)
        time.sleep(0.5)
        t1 = time.time()
        mav.arm(True)
        time.sleep(1.5)
        armed, txt = mav.armed, mav.texts_since(t1)
        if armed:
            mav.arm(False)
            time.sleep(1)
        if expect_ok:
            check(name, armed, str(txt))
        else:
            check(name, not armed and any(expect_text in x for x in txt), str(txt))

    # ---- VCU_FB_CHK=0 (默认): 只上报 ----
    t0 = time.time()
    vcu.drv = [0x01, 0, 0]
    time.sleep(3.2)
    fault = [v for _, v in mav.nvf_since(t0 + 1, "VCU_FAULT")]
    drv = mav.nvf_since(t0, "VCU_DRVFLT")
    check("1. 默认 VCU_FB_CHK=0", True)
    check("1. Drv1=0x01: VCU_FAULT 恒 0", fault and all(v == 0 for v in fault), str(set(fault)))
    check("1. VCU_DRVFLT=1, VCU_DRVCH=1, VCU_SYSFLT=0",
          last("VCU_DRVFLT", t0) == 1 and last("VCU_DRVCH", t0) == 1 and last("VCU_SYSFLT", t0) == 0)
    first1 = [t for t, v in drv if v == 1]
    check("1. VCU_DRVFLT 变化立即发, 之后约 1 Hz", first1 and first1[0] - t0 < 0.8 and 3 <= len(drv) <= 5,
          str([(round(t - t0, 2), v) for t, v in drv]))
    sev = [(s, x) for t, s, x in mav.sev if t >= t0 and "drive fault" in x]
    check("1. 提示 VCU: drive fault 0x01 Drv1, NOTICE 级, 只发一次",
          sev == [(5, "VCU: drive fault 0x01 Drv1")], str(sev))

    # 2. 驱动器故障 + Sys_Fault=3 时都能解锁 (先等 EKF, 最多 120 s)
    t0 = time.time()
    vcu.sys = 3
    time.sleep(1.5)
    sev = [(s, x) for t, s, x in mav.sev if t >= t0 and "sys fault" in x]
    check("2. Sys_Fault=3: 提示 VCU: sys fault 0x03 (NOTICE), VCU_SYSFLT=3, VCU_FAULT=0",
          sev == [(5, "VCU: sys fault 0x03")] and last("VCU_SYSFLT", t0) == 3
          and all(v == 0 for _, v in mav.nvf_since(t0 + 0.5, "VCU_FAULT")), str(sev))
    mav.set_mode(15)
    time.sleep(0.5)
    t_arm = time.time()
    while not mav.armed and time.time() - t_arm < 120:
        mav.arm(True)
        time.sleep(3)
    print(f"    (等待可解锁 {time.time() - t_arm:.0f} s)")
    bad = [x for x in mav.texts_since(t_arm) if "PreArm" in x and "VCU" in x]
    check("2. Sys_Fault=3 + Drv1 时能解锁, 无 VCU PreArm", mav.armed and not bad, str(bad))
    check("2. VCU_OK=1", last("VCU_OK", time.time() - 1) == 1)

    # 3. GUIDED 行驶中: 换码 / Failsafe / MotionSta 停发都不切 HOLD
    t0 = time.time()
    mav.goto(HOME[0] + 0.0004, HOME[1])
    time.sleep(3)
    driving_ok("3. Sys_Fault + 驱动器故障下 GUIDED 行驶, 不切 HOLD", t0)
    t0 = time.time()
    vcu.drv = [0, 0x22, 0]
    time.sleep(1.5)
    check("3. 换码 Drv2=0x22: 提示 + VCU_DRVFLT=0x22 VCU_DRVCH=2",
          "VCU: drive fault 0x22 Drv2" in mav.texts_since(t0) and last("VCU_DRVFLT", t0) == 0x22
          and last("VCU_DRVCH", t0) == 2)
    t0 = time.time()
    vcu.failsafe = True
    time.sleep(1.5)
    check("3. Failsafe=1: 提示 VCU: chassis failsafe", "VCU: chassis failsafe" in mav.texts_since(t0))
    driving_ok("3. Failsafe=1 行驶不切 HOLD", t0)
    t0 = time.time()
    vcu.failsafe = False
    time.sleep(1.0)
    check("3. Failsafe 清除提示", "VCU: chassis failsafe cleared" in mav.texts_since(t0))
    t0 = time.time()
    vcu.send_motion = False
    time.sleep(1.5)
    driving_ok("3. MotionSta 停发行驶不切 HOLD", t0)
    vcu.send_motion = True
    time.sleep(1.0)

    # ---- VCU_FB_CHK=1: garlic.15 做法 ----
    t0 = time.time()
    mav.set_param("VCU_FB_CHK", 1)
    time.sleep(1.5)
    txt = mav.texts_since(t0)
    check("4. FB_CHK=1 行驶中 Sys_Fault=3 切 HOLD", mav.mode == 4 and any("fault 0x03, HOLD" in x for x in txt),
          f"mode={mav.mode} {txt}")
    fault = [v for _, v in mav.nvf_since(t0 + 0.8, "VCU_FAULT")]
    check("4. FB_CHK=1: VCU_FAULT=3 (Sys_Fault 优先)", fault and all(v == 3 for v in fault), str(set(fault)))
    try_arm("4. FB_CHK=1: Sys_Fault=3 禁止解锁", False, "VCU: fault 0x03")
    vcu.sys = 0
    time.sleep(1.5)
    t0 = time.time()
    time.sleep(1.2)
    fault = [v for _, v in mav.nvf_since(t0, "VCU_FAULT")]
    check("4. FB_CHK=1: 驱动器故障 VCU_FAULT=0x122", fault and all(v == 0x122 for v in fault), str(set(fault)))
    try_arm("4. FB_CHK=1: 驱动器故障飞控仍能解锁", True)
    vcu.failsafe = True
    time.sleep(0.5)
    try_arm("4. FB_CHK=1: Failsafe 禁止解锁", False, "VCU: failsafe")
    vcu.failsafe = False
    vcu.send_motion = False
    time.sleep(0.5)
    try_arm("4. FB_CHK=1: MotionSta 停发禁止解锁", False, "VCU: feedback timeout")
    vcu.send_motion = True

    # 5. 回到 0, 清除
    mav.set_param("VCU_FB_CHK", 0)
    time.sleep(1.5)
    t0 = time.time()
    vcu.drv = [0, 0, 0]
    vcu.sys = 3
    time.sleep(1.2)
    vcu.sys = 0
    time.sleep(1.5)
    txt = mav.texts_since(t0)
    fault = [v for _, v in mav.nvf_since(t0 + 0.5, "VCU_FAULT")]
    check("5. 清除: drive fault cleared / sys fault cleared, VCU_DRVFLT=0 VCU_SYSFLT=0 VCU_FAULT=0",
          "VCU: drive fault cleared" in txt and "VCU: sys fault cleared" in txt and last("VCU_DRVFLT", t0) == 0
          and last("VCU_SYSFLT", t0) == 0 and fault and all(v == 0 for v in fault), f"{txt} fault={set(fault)}")
    try_arm("5. 全部清除后可解锁", True)

    print()
    print(f"{sum(ok for _, ok in results)}/{len(results)} PASS")
    sys.exit(0 if all(ok for _, ok in results) else 1)


if __name__ == "__main__":
    main()
