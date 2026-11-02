#!/usr/bin/env python3
"""三-B 转角换算: 实际车速 (MotionSta) 与指令不一致时, 转角按 atan(w*max(|v|,1)*L/v_ref^2) 换算;
MotionSta 停发时 v_ref 退回 AHRS 前向速度."""
import math, struct, sys, time
from vcu_test import Vcu, Mav, ID_STA, ID_MOTION, check, results

L, VREF_MIN, STEER_MAX = 1.2, 0.3, math.degrees(math.atan(1.2 / 2.1))


class ScaledVcu(Vcu):
    """MotionSta 车速 = 最近一帧 Nav_Cmd 车速 x scale; scale=None 时停发 MotionSta."""
    scale = 1.5

    def _tx(self):
        n = 0
        while True:
            if n % 2 == 0 and self.send_sta:
                self.send(ID_STA, self.sta)
            if self.scale is not None:
                with self.lock:
                    v_cmd = self.cmds[-1][1] if self.cmds else 0.0
                raw = int(round(v_cmd * self.scale * 100)) & 0xFFFF
                self.send(ID_MOTION, [raw & 0xFF, raw >> 8, 0, 0, 0, 2, 0, 0])
            n += 1
            time.sleep(0.05)


def dbg_samples(mav, t0):
    """同一循环里连发的 VCU_CMD_V/W/D、VCU_VREF 组成一组."""
    with mav.lock:
        rows = [(t, n, v) for (t, n, v) in mav.nvf if t >= t0]
    out, cur = [], {}
    for t, n, v in rows:
        if n == "VCU_CMD_V":
            cur = {"t": t, "V": v}
        elif n in ("VCU_CMD_W", "VCU_CMD_D", "VCU_VREF") and cur:
            cur[n[4:]] = v
            if len(cur) == 5:
                out.append(cur)
                cur = {}
    return out


def expected(s):
    vd = s["V"]
    sp = math.copysign(max(abs(vd), 1.0), vd)
    d = math.degrees(math.atan(s["CMD_W"] * sp * L / (s["VREF"] ** 2)))
    return max(-STEER_MAX, min(STEER_MAX, d))


def main():
    vcu = ScaledVcu()
    mav = Mav("tcp:127.0.0.1:5770")
    time.sleep(3)
    # 本测试验证按 MotionSta 车速换算 (VCU_VREF_SRC=0); 默认 1 时 v_ref 恒为 AHRS 前向速度, 同 8c.
    # 起步回正 (VCU_STR_TOL) 会在静止时把转角压成 0, 小步增量 (VCU_STR_STEP) 会改下发值, 都关掉以便比对换算公式
    for p, v in (("VCU_RTK_REQ", 0), ("VCU_DBG_HZ", 10), ("ARMING_CHECK", 0), ("VCU_VREF_SRC", 0), ("VCU_STR_TOL", 0),
                 ("VCU_STR_STEP", 0)):
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
    check("解锁", mav.armed)

    # 目标点在左前方, 有转弯
    mav.goto(31.93778228 + 0.0004, 118.7901755 - 0.0004)
    time.sleep(3)
    t0 = time.time()
    time.sleep(6)
    ss = [s for s in dbg_samples(mav, t0) if s["V"] >= 0.05]
    vref_ok = [abs(s["VREF"] - max(1.5 * s["V"], VREF_MIN)) < 0.05 for s in ss]
    d_err = [abs(s["CMD_D"] - expected(s)) for s in ss]
    turning = [s for s in ss if abs(s["CMD_D"]) > 1.0]
    check("8a. MotionSta=1.5x 指令: v_ref 跟 MotionSta", ss and sum(vref_ok) >= 0.9 * len(ss),
          f"n={len(ss)} ok={sum(vref_ok)} 例 V={ss[-1]['V']:.2f} VREF={ss[-1]['VREF']:.2f}" if ss else "")
    check("8b. 转角 = atan(w*max(|v|,1)*L/v_ref^2)", ss and max(d_err) < 0.5 and turning,
          f"max_err={max(d_err) if d_err else None:.3f} deg, 转弯样本 {len(turning)}, "
          f"例 W={ss[-1]['CMD_W']:.3f} D={ss[-1]['CMD_D']:.2f}" if ss else "")
    # 对比旧公式: 同一组数据下旧公式 atan(w*L/v) 的转角
    if turning:
        s = turning[-1]
        old = math.degrees(math.atan(s["CMD_W"] * L / s["V"]))
        print(f"    对比: V={s['V']:.2f} 实际={s['VREF']:.2f} W={s['CMD_W']:.3f} 新转角={s['CMD_D']:.2f} 旧公式={old:.2f}")

    # MotionSta 停发 (台架 VCU_FB_TMO=0) -> v_ref 退回 AHRS 前向速度
    mav.set_param("VCU_FB_TMO", 0)
    time.sleep(0.5)
    vcu.scale = None
    time.sleep(1)
    t0 = time.time()
    time.sleep(5)
    ss = [s for s in dbg_samples(mav, t0) if s["V"] >= 0.05]
    # SITL 车按 ArduPilot 自己的输出在走, 地速约等于指令; 不再是 1.5 倍
    ok = [abs(s["VREF"] - max(s["V"], VREF_MIN)) < 0.15 for s in ss]
    check("8c. MotionSta 停发: v_ref 退回 AHRS 前向速度 (约等于 SITL 地速)", ss and sum(ok) >= 0.9 * len(ss),
          f"n={len(ss)} 例 V={ss[-1]['V']:.2f} VREF={ss[-1]['VREF']:.2f}" if ss else "")
    d_err = [abs(s["CMD_D"] - expected(s)) for s in ss]
    check("8d. 退回 AHRS 时转角公式不变", ss and max(d_err) < 0.5, f"max_err={max(d_err) if d_err else None}")
    mav.arm(False)
    time.sleep(1)
    print()
    print(f"{sum(ok for _, ok in results)}/{len(results)} PASS")
    sys.exit(0 if all(ok for _, ok in results) else 1)


if __name__ == "__main__":
    main()
