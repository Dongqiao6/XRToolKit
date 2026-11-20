#!/usr/bin/env python3
"""从 JAKA 控制器**只读**取实时关节角。

控制器在 10000 端口持续推送状态 JSON（每条约几 KB，含 `joint_actual_position`，
单位度）。连上就收，不用登录，也**不往控制器发任何字节**——所以不会改变机器人
的任何状态，和别的程序（老师的驱动、遥操作）同时连也互不影响。

为什么不走 ROS 话题：jaka_driver 发的 `/jaka_driver/joint_position` 要驱动在跑
才有；驱动一停 rerun 里的臂就不动了。直接读控制器，驱动在不在都行。

用法：

    live = JakaLive({"left": "192.168.71.37", "right": "192.168.71.36"})
    live.start()
    q = live.q()          # {"left_joint_1": 弧度, ...}，只含新鲜的那几条臂
    print(live.status())

命令行自查（打印 5 秒）：

    python3 jaka_live.py 192.168.71.37 192.168.71.36
"""
import json
import math
import socket
import sys
import threading
import time

from .jaka_model import ARM_JOINTS

# 超过这么久没新数据，就当这条臂「失联」。控制器 10000 端口默认**每 2 秒**才推
# 一条（11-14 实测：千兆网、延迟 0.7 ms，3 条消息间隔 2.0 s），门槛要比它长。
STALE_S = 3.0


class _ArmReader(threading.Thread):
    def __init__(self, arm, ip, port=10000):
        super().__init__(daemon=True, name=f"jaka-{arm}")
        self.arm, self.ip, self.port = arm, ip, port
        self.deg, self.t, self.err, self.n = None, 0.0, "", 0
        self._stop = threading.Event()

    def run(self):
        dec = json.JSONDecoder()
        while not self._stop.is_set():
            try:
                with socket.create_connection((self.ip, self.port), timeout=3) as s:
                    s.settimeout(3)
                    buf = ""
                    while not self._stop.is_set():
                        chunk = s.recv(65536)
                        if not chunk:
                            raise ConnectionError("控制器关闭了连接")
                        buf += chunk.decode("utf-8", "ignore")
                        # 流里是一条接一条的 JSON，每条都以 {"len" 开头。按这个找开头，
                        # 不能找任意的 {——刚连上时多半落在半条消息中间，那里的 {
                        # 是某个嵌套字段，从它开始永远解不出来，会一直干等。
                        while True:
                            i = buf.find('{"len"')
                            if i < 0:
                                buf = ""
                                break
                            try:
                                obj, end = dec.raw_decode(buf, i)
                            except ValueError:
                                buf = buf[i:]          # 半条，等下一块
                                if len(buf) > 1_000_000:
                                    buf = ""           # 防止坏流把内存撑爆
                                break
                            buf = buf[end:]
                            j = obj.get("joint_actual_position")
                            if isinstance(j, list) and len(j) >= 6:
                                self.deg, self.t, self.err = j[:6], time.time(), ""
                                self.n += 1
            except Exception as e:                      # 断了就过一秒重连
                self.err = f"{type(e).__name__}: {e}"
                self._stop.wait(1.0)

    def stop(self):
        self._stop.set()


class JakaLive:
    def __init__(self, ips):
        self.readers = {arm: _ArmReader(arm, ip) for arm, ip in ips.items() if ip}

    def start(self):
        for r in self.readers.values():
            r.start()
        return self

    def stop(self):
        for r in self.readers.values():
            r.stop()

    def q(self):
        """新鲜的关节角，弧度。失联的臂不给——调用方自己决定用什么兜底。"""
        out, now = {}, time.time()
        for arm, r in self.readers.items():
            if r.deg is not None and now - r.t <= STALE_S:
                for name, v in zip(ARM_JOINTS[arm], r.deg):
                    out[name] = math.radians(float(v))
        return out

    def status(self):
        now = time.time()
        parts = []
        for arm, r in self.readers.items():
            if r.deg is not None and now - r.t <= STALE_S:
                parts.append(f"{arm} ✓ " + " ".join(f"{v:+.1f}" for v in r.deg))
            else:
                parts.append(f"{arm} ✗ {r.err or '还没收到'}")
        return "   ".join(parts)


if __name__ == "__main__":
    ips = sys.argv[1:] or ["192.168.71.37", "192.168.71.36"]
    live = JakaLive({"left": ips[0], "right": ips[1] if len(ips) > 1 else ""}).start()
    for _ in range(5):
        time.sleep(1.0)
        print(live.status())
    rate = {a: r.n / 5 for a, r in live.readers.items()}
    print("每秒收到几条:", rate)
