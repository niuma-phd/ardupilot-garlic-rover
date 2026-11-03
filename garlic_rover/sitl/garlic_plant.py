#!/usr/bin/env python3
"""大蒜车车辆模型: SITL JSON 外部物理 + 模拟 VCU (组播虚拟 CAN 总线 0).

SITL 里车不再按 ArduPilot 舵机输出动, 而是像实车一样只听 vcu_can.lua 发的 Nav_Cmd
(目标车速 / 前轮转角), 按实车日志 (2026-11-01 bin) 测得的 VCU 特性运动:
  - 前轮转角按 STEER_RATE (4.5 deg/s) 限速追指令, 限幅 STEER_MAX (25 deg); 左右不同时用 STEER_MAX_L / STEER_MAX_R
  - LATCH=1: 锁定目标式转向 (实车 VCU, 甲方 11-02 确认): 转到锁定目标 (LATCH_TOL 内) 或机械极限后才接受最新指令,
    到位后新目标离当前位置不超过 MIN_ACT 不动 (1103 bin 拟合: LATCH_TOL 2, MIN_ACT 3, STEER_RATE 3.5)
  - TGT_OFS: VCU 按自己的中位执行指令, 回报原始角度: 指令 T -> 执行器(回报)转到 T+TGT_OFS (1102/1103 bin: -1.8).
    配 STEER_K=1 STEER_OFS=1.8 即"前轮正时回报 -1.8"
  - 车速: SPEED_MODE=run 时有指令 (>=0.05) 就跑 V_RUN (1.7 m/s), 不随指令大小变 (实车如此);
          SPEED_MODE=follow 时跟随指令车速. 加速 ACCEL (2.5), 减速 DECEL (4.5) m/s^2
  - 有效转角 = STEER_K (1.1) * 回报转角 + STEER_OFS (1.5 deg, 左正), 自行车模型轴距 WBASE (1.2 m)
  - MotionSta 回报车速封顶 AV_CAP (0.85 m/s), 回报转角为执行器转角
  - STALL_S>0: 车开起来 STALL_AT 秒后停顿 STALL_S 秒 (车速 0, 模拟作业拖地 / 打滑, 1103 外场),
    停顿期间前轮停在 STALL_STEER (不设则照常追指令); 日志第 9 列为停顿标志
NavSta 始终为 无人授权 (Op=2, HS=3, Seq=7), WorkFb=0.

用法: garlic_plant.py [key=value ...]  如 SPEED_MODE=follow STEER0=-25
"""
import json, math, os, socket, struct, sys, threading, time

CFG = dict(STEER_RATE=4.5, STEER_MAX=25.0, STEER_MAX_L=None, STEER_MAX_R=None, LATCH=0.0, LATCH_TOL=0.5, SPEED_MODE="run", V_RUN=1.7, ACCEL=2.5, DECEL=4.5,
           STEER_K=1.1, STEER_OFS=1.5, WBASE=1.2, AV_CAP=0.85, STEER0=0.0, YAW0=None, LOG="", PORT=9012,
           STALL_AT=0.0, STALL_S=0.0, STALL_STEER=None, TGT_OFS=0.0, MIN_ACT=0.0)
for a in sys.argv[1:]:
    k, v = a.split("=", 1)
    CFG[k] = v if k in ("SPEED_MODE", "LOG") else float(v)

MCAST, PORT, MAGIC = "239.65.82.0", 57732, 0x2934
EFF = 0x80000000
ID_CMD, ID_STA, ID_MOTION = 0x180C7D41, 0x180B417D, 0x1802407D


def crc16_ccitt(buf, crc=0xFFFF):
    for b in buf:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Can:
    def __init__(self):
        self.cmd_v = 0.0
        self.cmd_steer = 0.0
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind((MCAST, PORT))
        self.rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                           struct.pack("4s4s", socket.inet_aton(MCAST), socket.inet_aton("127.0.0.1")))
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton("127.0.0.1"))
        self.tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        threading.Thread(target=self._rx, daemon=True).start()

    def send(self, can_id, data):
        body = struct.pack("<HI", 0, EFF | can_id) + bytes(data)
        self.tx.sendto(struct.pack("<HH", MAGIC, crc16_ccitt(body)) + body, (MCAST, PORT))

    def _rx(self):
        while True:
            pkt = self.rx.recv(128)
            if len(pkt) < 18:
                continue
            magic, crc, flags, mid = struct.unpack("<HHHI", pkt[:10])
            if magic != MAGIC or crc != crc16_ccitt(pkt[4:]):
                continue
            if (mid & 0x1FFFFFFF) == ID_CMD:
                v, st = struct.unpack("<hh", pkt[10:14])
                self.cmd_v, self.cmd_steer = v * 0.01, st * 0.01


def i16(x):
    return struct.pack("<h", max(-32768, min(32767, int(round(x)))))


def main():
    can = Can()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", int(CFG["PORT"])))
    logf = open(CFG["LOG"], "w") if CFG["LOG"] else None
    st = None           # 车辆状态
    last_tx = 0.0
    n_sta = 0
    last_fc = -1
    while True:
        data, addr = sock.recvfrom(256)
        if len(data) < 40:
            continue
        magic, frame_rate, fc = struct.unpack("<HHI", data[:8])
        if magic != 18458:
            continue
        if st is None or fc < last_fc:      # SITL 重启 -> 复位
            yaw0 = math.radians(CFG["YAW0"]) if CFG["YAW0"] is not None else 0.0
            st = dict(t=0.0, n=0.0, e=0.0, yaw=yaw0, v=0.0, steer=CFG["STEER0"], r=0.0, a=0.0, moved=False)
        if fc == last_fc:
            dt = 0.0
        else:
            dt = 1.0 / max(frame_rate, 1)
        last_fc = fc

        # VCU 执行: 转角限速追指令. 测试脚本解锁 (建 /tmp/plant_go) 之前前轮停在 STEER0 (模拟手动开到起点后前轮留下的转角)
        if not st["moved"] and (can.cmd_v >= 0.05 or (int(st["t"] * 50) % 5 == 0 and os.path.exists("/tmp/plant_go"))):
            st["moved"] = True
        lim_l = CFG["STEER_MAX_L"] if CFG["STEER_MAX_L"] is not None else CFG["STEER_MAX"]
        lim_r = CFG["STEER_MAX_R"] if CFG["STEER_MAX_R"] is not None else CFG["STEER_MAX"]
        want = can.cmd_steer + CFG["TGT_OFS"] if st["moved"] else st["steer"]
        if CFG["LATCH"]:
            lt = st.get("latch")
            at_lim = lt is not None and ((lt > st["steer"] and st["steer"] >= lim_l - 0.05) or
                                         (lt < st["steer"] and st["steer"] <= -lim_r + 0.05))
            if lt is None or not st["moved"]:
                st["latch"] = want
            elif abs(lt - st["steer"]) <= CFG["LATCH_TOL"] or at_lim:
                # 到位 (或到极限) 后停在原地, 新目标离当前位置超过 MIN_ACT 才接受
                st["latch"] = want if abs(want - st["steer"]) > CFG["MIN_ACT"] else st["steer"]
            want = st["latch"]
        tgt = max(-lim_r, min(lim_l, want))
        dmax = CFG["STEER_RATE"] * dt
        st["steer"] += max(-dmax, min(dmax, tgt - st["steer"]))
        # 车速. 测试脚本可写 /tmp/plant_vrun 切换实际车速 (运输 / 作业), 每 0.1 s 读一次
        cv = can.cmd_v
        if int(st["t"] * 50) % 5 == 0 and os.path.exists("/tmp/plant_vrun"):
            try:
                CFG["V_RUN"] = float(open("/tmp/plant_vrun").read())
            except ValueError:
                pass
        if CFG["SPEED_MODE"] == "run":
            v_goal = CFG["V_RUN"] if cv >= 0.05 else 0.0
        else:
            v_goal = max(cv, 0.0)
        dv = v_goal - st["v"]
        lim = (CFG["ACCEL"] if dv > 0 else CFG["DECEL"]) * dt
        dv = max(-lim, min(lim, dv))
        st["a"] = dv / dt if dt > 0 else 0.0
        st["v"] = max(0.0, st["v"] + dv)
        if st.get("t_go") is None and st["v"] > 0.01:
            st["t_go"] = st["t"]
        stall = (CFG["STALL_S"] > 0 and st.get("t_go") is not None
                 and CFG["STALL_AT"] <= st["t"] - st["t_go"] < CFG["STALL_AT"] + CFG["STALL_S"])
        if stall:
            st["v"], st["a"] = 0.0, 0.0
            if CFG["STALL_STEER"] is not None:
                st["steer"] = CFG["STALL_STEER"]
        # 自行车模型 (转角左正, 航向顺时针正)
        eff = math.radians(CFG["STEER_K"] * st["steer"] + CFG["STEER_OFS"])
        st["r"] = -st["v"] * math.tan(eff) / CFG["WBASE"] if st["v"] > 0.001 else 0.0
        st["yaw"] = (st["yaw"] + st["r"] * dt) % (2 * math.pi)
        st["n"] += st["v"] * math.cos(st["yaw"]) * dt
        st["e"] += st["v"] * math.sin(st["yaw"]) * dt
        st["t"] += dt

        vn, ve = st["v"] * math.cos(st["yaw"]), st["v"] * math.sin(st["yaw"])
        out = {"timestamp": st["t"],
               "imu": {"gyro": [0.0, 0.0, st["r"]], "accel_body": [st["a"], st["v"] * st["r"], -9.80665]},
               "position": [st["n"], st["e"], 0.0],
               "attitude": [0.0, 0.0, st["yaw"]],
               "velocity": [vn, ve, 0.0]}
        sock.sendto(("\n" + json.dumps(out) + "\n").encode(), addr)

        # VCU 状态帧 (按物理时间 50 ms)
        if st["t"] - last_tx >= 0.05:
            last_tx = st["t"]
            n_sta += 1
            if n_sta % 2 == 0:
                can.send(ID_STA, [2, 3, 7, 0, 0, 0, 0, 0])
            can.send(ID_MOTION, i16(min(st["v"], CFG["AV_CAP"]) * 100) + i16(st["steer"] * 100) + bytes([0, 2, 0, 0]))
            if logf:
                logf.write("%.2f %.3f %.3f %.2f %.3f %.2f %.3f %.2f %d\n" % (
                    st["t"], st["n"], st["e"], math.degrees(st["yaw"]), st["v"], st["steer"], cv, can.cmd_steer, stall))
                logf.flush()


if __name__ == "__main__":
    main()
