#!/usr/bin/env python3
"""驱动器故障只上报不拦车 (garlic.16): HealthSta Drv1/2/3_Fault 不进 VCU_FAULT (VCU_DRV_CHK=0),
改报 VCU_DRVFLT / VCU_DRVCH + "VCU: drive fault 0x.."; 能解锁、GUIDED 行驶不切 HOLD.
回归: Sys_Fault 仍禁止解锁 / 行驶中切 HOLD; VCU_DRV_CHK=1 时 VCU_FAULT=0x100+码 但飞控仍不拦."""
import sys, threading, time
from pymavlink import mavutil
from vcu_test import Vcu, Mav, check, results

ID_HEALTH = 0x1806407D
HOME = (31.93778228, 118.7901755)


class HealthVcu(Vcu):
    """另发 HealthSta (500 ms): SOC 80 %, 50.0 V, Drv1/2/3, Sys_Fault"""

    def __init__(self):
        self.drv = [0, 0, 0]
        self.sys = 0
        super().__init__()
        threading.Thread(target=self._health, daemon=True).start()

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
    for p, v in (("VCU_RTK_REQ", 0), ("VCU_DBG_HZ", 0), ("ARMING_CHECK", 0), ("VCU_DRV_CHK", 0)):
        mav.set_param(p, v)
    time.sleep(1)

    # 握手授权, HS=3
    vcu.sta = [1, 1, 7, 0, 0, 0, 0, 0]
    time.sleep(0.6)
    vcu.sta = [2, 3, 7, 0, 0, 0, 0, 0]
    time.sleep(1.0)

    # 1. 注入 Drv1=0x01 (= garlic.15 的 VCU_FAULT 0x101)
    t0 = time.time()
    vcu.drv = [0x01, 0, 0]
    time.sleep(3.2)
    fault = [v for _, v in mav.nvf_since(t0 + 1, "VCU_FAULT")]
    drv = mav.nvf_since(t0, "VCU_DRVFLT")
    ch = [v for _, v in mav.nvf_since(t0, "VCU_DRVCH")]
    check("1. Drv1=0x01: VCU_FAULT 恒 0", fault and all(v == 0 for v in fault), str(set(fault)))
    check("1. VCU_DRVFLT=1, VCU_DRVCH=1", drv and drv[-1][1] == 1 and ch and ch[-1] == 1,
          f"drv={[v for _, v in drv]} ch={ch}")
    first1 = [t for t, v in drv if v == 1]
    check("1. VCU_DRVFLT 变化立即发, 之后约 1 Hz", first1 and first1[0] - t0 < 0.8 and 3 <= len(drv) <= 5,
          str([(round(t - t0, 2), v) for t, v in drv]))
    sev = [(s, x) for t, s, x in mav.sev if t >= t0 and "drive fault" in x]
    check("1. 提示 VCU: drive fault 0x01 Drv1, NOTICE 级, 只发一次",
          sev == [(5, "VCU: drive fault 0x01 Drv1")], str(sev))

    # 2. 驱动器故障下能解锁 (先等 EKF, 最多 120 s), 没有 PreArm 提到 drive / fault
    mav.set_mode(15)
    time.sleep(0.5)
    t_arm = time.time()
    while not mav.armed and time.time() - t_arm < 120:
        mav.arm(True)
        time.sleep(3)
    print(f"    (等待可解锁 {time.time() - t_arm:.0f} s)")
    bad = [x for x in mav.texts_since(t_arm) if "PreArm" in x and "VCU" in x]
    check("2. Drv1 故障时能解锁, 无 VCU PreArm", mav.armed and not bad, str(bad))

    # 3. GUIDED 行驶中仍有驱动器故障: 不切 HOLD, Nav_Cmd v>0
    t0 = time.time()
    mav.goto(HOME[0] + 0.0003, HOME[1])
    time.sleep(4)
    with vcu.lock:
        vs = [c[1] for c in vcu.cmds if c[0] > t0 + 2]
    check("3. 驱动器故障下 GUIDED 行驶, 不切 HOLD", mav.mode == 15 and vs and max(vs) > 0.2,
          f"mode={mav.mode} v_max={max(vs) if vs else None} {mav.texts_since(t0)}")

    # 4. 换路换码: Drv1=0, Drv2=0x22
    t0 = time.time()
    vcu.drv = [0, 0x22, 0]
    time.sleep(1.5)
    drv = mav.nvf_since(t0, "VCU_DRVFLT")
    check("4. 换码 Drv2=0x22: 提示 + VCU_DRVFLT=0x22 VCU_DRVCH=2",
          "VCU: drive fault 0x22 Drv2" in mav.texts_since(t0) and drv and drv[-1][1] == 0x22
          and mav.nvf_since(t0, "VCU_DRVCH")[-1][1] == 2, str([v for _, v in drv]))
    check("4. 行驶不受影响", mav.mode == 15 and mav.armed, f"mode={mav.mode}")

    # 5. 行驶中 Sys_Fault=3 (回归): 切 HOLD, VCU_FAULT=3
    t0 = time.time()
    vcu.sys = 3
    time.sleep(1.5)
    txt = mav.texts_since(t0)
    fault = [v for _, v in mav.nvf_since(t0 + 0.8, "VCU_FAULT")]
    check("5. 行驶中 Sys_Fault=3 切 HOLD", mav.mode == 4 and any("fault 0x03, HOLD" in x for x in txt),
          f"mode={mav.mode} {txt}")
    check("5. VCU_FAULT=3 (Sys_Fault 优先)", fault and all(v == 3 for v in fault), str(set(fault)))
    mav.arm(False)
    time.sleep(1.5)

    # 6. Sys_Fault=3 时禁止解锁 (回归)
    mav.set_mode(15)
    time.sleep(0.5)
    t0 = time.time()
    mav.arm(True)
    time.sleep(1.5)
    txt = mav.texts_since(t0)
    check("6. Sys_Fault=3 禁止解锁 (VCU: fault 0x03)", not mav.armed and any("VCU: fault 0x03" in x for x in txt), str(txt))
    if mav.armed:
        mav.arm(False)
        time.sleep(1)
    vcu.sys = 0
    time.sleep(1.5)

    # 7. VCU_DRV_CHK=1: VCU_FAULT=0x100+码, 但飞控仍能解锁
    mav.set_param("VCU_DRV_CHK", 1)
    time.sleep(1.5)
    t0 = time.time()
    time.sleep(1.5)
    fault = [v for _, v in mav.nvf_since(t0, "VCU_FAULT")]
    check("7. VCU_DRV_CHK=1: VCU_FAULT=0x122", fault and all(v == 0x122 for v in fault), str(set(fault)))
    mav.arm(True)
    time.sleep(1.5)
    check("7. VCU_DRV_CHK=1 飞控仍能解锁", mav.armed, str(mav.texts_since(t0)))
    if mav.armed:
        mav.arm(False)
        time.sleep(1)
    mav.set_param("VCU_DRV_CHK", 0)
    time.sleep(1.5)

    # 8. 清除: 提示 cleared, VCU_DRVFLT=0
    t0 = time.time()
    vcu.drv = [0, 0, 0]
    time.sleep(1.5)
    drv = mav.nvf_since(t0, "VCU_DRVFLT")
    fault = [v for _, v in mav.nvf_since(t0 + 0.5, "VCU_FAULT")]
    check("8. 清除: VCU: drive fault cleared, VCU_DRVFLT=0, VCU_FAULT=0",
          "VCU: drive fault cleared" in mav.texts_since(t0) and drv and drv[-1][1] == 0
          and fault and all(v == 0 for v in fault), f"drv={[v for _, v in drv]} fault={set(fault)}")

    print()
    print(f"{sum(ok for _, ok in results)}/{len(results)} PASS")
    sys.exit(0 if all(ok for _, ok in results) else 1)


if __name__ == "__main__":
    main()
