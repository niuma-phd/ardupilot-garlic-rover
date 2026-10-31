#!/usr/bin/env python3
"""vcu_can.lua (协议 v1.1) SITL 回归: Python 扮演 VCU, 走 SITL 组播 CAN 总线 0."""
import socket, struct, threading, time, sys
from pymavlink import mavutil

MCAST, PORT, MAGIC = "239.65.82.0", 57732, 0x2934
EFF = 0x80000000
ID_CMD, ID_ACK, ID_STA, ID_MOTION = 0x180C7D41, 0x180D7D41, 0x180B417D, 0x1802407D


def crc16_ccitt(buf, crc=0xFFFF):
    for b in buf:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Vcu:
    def __init__(self):
        self.sta = [0, 0, 0, 0, 0, 0, 0, 0]   # NavSta byte0..7
        self.send_sta = True
        self.cmds = []                          # (t, v, steer, work, seq)
        self.acks = []                          # (t, bytes)
        self.lock = threading.Lock()
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx.bind((MCAST, PORT))
        self.rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                           struct.pack("4s4s", socket.inet_aton(MCAST), socket.inet_aton("127.0.0.1")))
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton("127.0.0.1"))
        self.tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._tx, daemon=True).start()

    def send(self, can_id, data):
        body = struct.pack("<HI", 0, EFF | can_id) + bytes(data)
        self.tx.sendto(struct.pack("<HH", MAGIC, crc16_ccitt(body)) + body, (MCAST, PORT))

    def _tx(self):
        n = 0
        while True:
            if n % 2 == 0 and self.send_sta:
                self.send(ID_STA, self.sta)
            # MotionSta: 静止, Veh_State=2 运行, 无 Failsafe
            self.send(ID_MOTION, [0, 0, 0, 0, 0, 2, 0, 0])
            n += 1
            time.sleep(0.05)

    def _rx(self):
        while True:
            pkt = self.rx.recv(128)
            if len(pkt) < 10:
                continue
            magic, crc, flags, mid = struct.unpack("<HHHI", pkt[:10])
            data = pkt[10:]
            if magic != MAGIC or crc != crc16_ccitt(pkt[4:]):
                continue
            mid &= 0x1FFFFFFF
            t = time.time()
            with self.lock:
                if mid == ID_CMD:
                    v, st = struct.unpack("<hh", data[:4])
                    self.cmds.append((t, v * 0.01, st * 0.01, data[4], data[7]))
                elif mid == ID_ACK:
                    self.acks.append((t, bytes(data)))


class Mav:
    def __init__(self, url):
        self.m = mavutil.mavlink_connection(url, source_system=255)
        self.m.wait_heartbeat()
        self.texts, self.nvf = [], []
        self.armed, self.mode = False, -1
        self.lock = threading.Lock()
        threading.Thread(target=self._rx, daemon=True).start()
        self.m.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

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

    def cmd(self, c, *p):
        p = list(p) + [0] * (7 - len(p))
        self.m.mav.command_long_send(1, 1, c, 0, *p)

    def set_param(self, name, val):
        self.m.mav.param_set_send(1, 1, name.encode(), val, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def set_mode(self, mode):
        self.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, mode)

    def arm(self, on=True):
        self.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1 if on else 0, 0 if on else 21196)

    def goto(self, lat, lon):
        self.m.mav.set_position_target_global_int_send(
            0, 1, 1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000,
            int(lat * 1e7), int(lon * 1e7), 0, 0, 0, 0, 0, 0, 0, 0, 0)

    def texts_since(self, t0):
        with self.lock:
            return [x for (t, x) in self.texts if t >= t0]

    def nvf_since(self, t0, name):
        with self.lock:
            return [(t, v) for (t, n, v) in self.nvf if t >= t0 and n == name]


results = []


def check(name, cond, detail=""):
    results.append((name, bool(cond)))
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}")


def main():
    vcu = Vcu()
    t_boot = time.time()
    mav = Mav("tcp:127.0.0.1:5770")
    time.sleep(3)
    for p, v in (("VCU_RTK_REQ", 0), ("VCU_DBG_HZ", 0), ("ARMING_CHECK", 0)):
        mav.set_param(p, v)
    time.sleep(1)
    loaded = [x for x in mav.texts_since(0) if "CAN bridge loaded" in x]
    check("启动消息 Nav v1.1", any("(Nav v1.1)" in x for x in loaded), str(loaded))
    with vcu.lock:
        check("上电静默 (无邀请不发帧)", not vcu.cmds and not vcu.acks)

    # 握手: INVITING -> HsAck, 授权 -> Nav_Cmd 50 ms
    vcu.sta = [1, 1, 7, 0, 0, 0, 0, 0]
    time.sleep(0.6)
    with vcu.lock:
        acks = list(vcu.acks)
    check("HsAck 01 07 00 03", len(acks) == 1 and acks[0][1][:4] == bytes([1, 7, 0, 3]), str([a[1].hex() for a in acks]))
    vcu.sta = [2, 2, 7, 0, 0, 0, 0, 0]
    time.sleep(1.0)
    t0 = time.time()
    vcu.sta = [2, 3, 7, 0, 0, 0, 0, 0]
    time.sleep(0.5)
    check("6. HS 2->3: VCU: RC start OK", "VCU: RC start OK" in mav.texts_since(t0))
    with vcu.lock:
        cmds = [c for c in vcu.cmds if c[0] > t0]
    gaps = [b[0] - a[0] for a, b in zip(cmds, cmds[1:])]
    check("Nav_Cmd 50 ms 周期, Seq=7", cmds and max(gaps) < 0.08 and all(c[4] == 7 for c in cmds),
          f"n={len(cmds)} max_gap={max(gaps) if gaps else None:.3f}")

    def try_arm(wfb, expect_ok, expect_text=None):
        vcu.sta[4] = wfb
        time.sleep(0.5)
        mav.set_mode(15)
        time.sleep(0.5)
        t1 = time.time()
        mav.arm(True)
        time.sleep(1.5)
        armed = mav.armed
        txt = mav.texts_since(t1)
        if armed:
            mav.arm(False)
            time.sleep(1.0)
        name = f"WorkFb={wfb}: {'可解锁' if expect_ok else '解锁失败 ' + expect_text}"
        if expect_ok:
            check(name, armed, str(txt))
        else:
            check(name, (not armed) and any(expect_text in x for x in txt), str(txt))

    # 1. WorkFb=0 授权、解锁、GUIDED 行驶 (先等 EKF 定位好, 最多 120 s)
    vcu.sta = [2, 3, 7, 0, 0, 0, 0, 0]
    time.sleep(0.5)
    mav.set_mode(15)
    time.sleep(0.5)
    t_wait = time.time()
    while not mav.armed and time.time() - t_wait < 120:
        mav.arm(True)
        time.sleep(3)
    print(f"    (等待可解锁 {time.time() - t_wait:.0f} s)")
    check("1. WorkFb=0 解锁", mav.armed)
    t0 = time.time()
    mav.goto(31.93778228 + 0.0003, 118.7901755)
    time.sleep(4)
    with vcu.lock:
        vs = [c[1] for c in vcu.cmds if c[0] > t0 + 2]
    check("1. WorkFb=0 GUIDED 行驶 Nav_Cmd v>0", vs and max(vs) > 0.2, f"v_max={max(vs) if vs else None}")

    # 5. 行驶中 WorkFb->1: HOLD, 不断流, v=0
    t0 = time.time()
    vcu.sta[4] = 1
    time.sleep(1.5)
    txt = mav.texts_since(t0)
    with vcu.lock:
        cmds = [c for c in vcu.cmds if c[0] > t0 + 0.3]
    gaps = [b[0] - a[0] for a, b in zip(cmds, cmds[1:])]
    check("5. 行驶中 WorkFb->1 切 HOLD", mav.mode == 4 and any("lift moving, HOLD" in x for x in txt), f"mode={mav.mode} {txt}")
    check("5. HOLD 后 Nav_Cmd 不断流且 v=0", cmds and max(gaps) < 0.08 and all(c[1] == 0 for c in cmds),
          f"n={len(cmds)} max_gap={max(gaps):.3f} v={set(c[1] for c in cmds)}")
    check("3. ->1 提示 VCU: lift moving", "VCU: lift moving" in txt)
    wf = mav.nvf_since(t0, "VCU_WORKFB")
    check("4. VCU_WORKFB 变化立即发且约 1 Hz", wf and wf[0][1] == 1 and wf[0][0] - t0 < 0.3 and len(wf) <= 3,
          str([(round(t - t0, 2), v) for t, v in wf]))
    mav.arm(False)
    time.sleep(1)

    # 2 / 4. 解锁检查
    try_arm(1, False, "VCU: lift moving")
    t0 = time.time()
    try_arm(4, False, "VCU: lift fault")
    check("3. ->4 提示 VCU: lift fault", "VCU: lift fault" in mav.texts_since(t0))
    t0 = time.time()
    try_arm(2, True)
    check("3. 4->2 提示 VCU: lift fault cleared", "VCU: lift fault cleared" in mav.texts_since(t0))
    try_arm(3, True)
    try_arm(0, True)

    # 3. WorkFb=5 整帧无效
    def invalid_frame(name, sta):
        vcu.sta = [2, 3, 7, 0, 0, 0, 0, 0]
        time.sleep(1)
        t0 = time.time()
        vcu.sta = sta
        time.sleep(1.2)
        with vcu.lock:
            cmds = [c[0] for c in vcu.cmds if c[0] > t0]
        last = (max(cmds) - t0) if cmds else 0
        t1 = time.time()
        mav.arm(True)
        time.sleep(1.5)
        txt = mav.texts_since(t1)
        armed = mav.armed
        if armed:
            mav.arm(False)
        check(name, 0.35 < last < 0.65 and not armed and any("NavSta lost" in x for x in txt),
              f"Nav_Cmd 停发于 {last:.2f}s, armed={armed}, {txt}")
        vcu.sta = [2, 3, 7, 0, 0, 0, 0, 0]
        time.sleep(1)

    invalid_frame("3. WorkFb=5 整帧无效, ~500 ms 后 NavSta lost", [2, 3, 7, 0, 5, 0, 0, 0])
    invalid_frame("7. byte5 非 0 整帧无效", [2, 3, 7, 0, 0, 1, 0, 0])

    print()
    print(f"{sum(ok for _, ok in results)}/{len(results)} PASS")
    sys.exit(0 if all(ok for _, ok in results) else 1)


if __name__ == "__main__":
    main()
