#!/usr/bin/env python3
"""MCU 仿真器：模拟电控下位机的 ITL 串口协议（42B 反馈帧），同时校验视觉端发来的 28B 指令帧。

纯标准库，无第三方依赖。帧规格对照仓库根目录《摆臂部署与协议迁移计划.md》第三节。

用法：
    python3 scripts/mcu_emulator.py [--rate 200] [--mode-cycle 3] [--inject-garbage 200]

启动后打印 pty 从端路径（如 /dev/pts/3），把它填进 yaml 的 com_port，再启动视觉程序：
    终端2: ros2 run sp_vision gimbal_test -- <config_yaml>

校验内容：
  - 每帧 28B 定长、head 'V','G'、tail 'V'、mode ∈ {0,1,2}、6 个 float 有限；
  - mode=0 的 IDLE 帧必须回显最近反馈角（yaw/pitch），速度/加速度为 0；
  - 收到 mode=2（开火）时 bullet_count 递增；
  - --inject-garbage 每 N 帧注入随机垃圾字节，验证视觉端逐字节重同步能力。
退出：Ctrl+C。全部 PASS 时退出码 0。
"""
import argparse
import math
import os
import pty
import random
import select
import socket
import struct
import sys
import threading
import time

TX_FMT = "<2sB6fB"       # 28B：head('V','G') mode 6×float32 tail('V')
RX_FMT = "<2sB4f4ffHB"   # 42B：head('G','V') mode q[4] yaw,yaw_vel,pitch,pitch_vel bullet_speed bullet_count tail('G')


class Stats:
    def __init__(self):
        self.lock = threading.Lock()
        self.rx_frames = 0
        self.tx_frames = 0
        self.fail = 0
        self.echo_checked = 0
        self.echo_fail = 0
        self.resync_events = 0

    def line(self):
        with self.lock:
            return (
                f"tx={self.tx_frames} rx={self.rx_frames} fail={self.fail} "
                f"echo_checked={self.echo_checked} echo_fail={self.echo_fail} "
                f"resync_injections={self.resync_events}"
            )


def main():
    ap = argparse.ArgumentParser(description="ITL 协议 MCU 仿真器")
    ap.add_argument("--rate", type=float, default=200.0, help="反馈帧率（Hz）")
    ap.add_argument("--mode-cycle", type=float, default=3.0, help="反馈 mode 0/1/2/3 循环周期（秒）")
    ap.add_argument("--inject-garbage", type=int, default=0, help="每 N 帧注入随机垃圾字节（测试重同步），0 关闭")
    ap.add_argument("--bullet-speed", type=float, default=18.5, help="弹速反馈（m/s）")
    ap.add_argument(
        "--tcp", default=None, metavar="HOST:PORT",
        help="用 TCP 替代 pty（配合 socat 桥接：socat pty,link=/tmp/vgimbal,raw,echo=0 tcp-listen:PORT）",
    )
    args = ap.parse_args()

    if args.tcp:
        host, port = args.tcp.rsplit(":", 1)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while True:  # 等待 socat/视觉端就绪
            try:
                sock.connect((host, int(port)))
                break
            except OSError:
                time.sleep(0.1)
        master_fd = sock.fileno()  # 归一化为 fd，与 pty 分支共用 os.write/os.read/select
        print(f"[EMU] connected to TCP {args.tcp}（socat 桥接，pty 端填进 yaml 的 com_port）", flush=True)
    else:
        master_fd, slave_fd = pty.openpty()
        slave_name = os.ttyname(slave_fd)
        print(f"[EMU] pty slave: {slave_name}  （填进 yaml 的 com_port）", flush=True)

    state = {
        "fb_yaw": 0.0,
        "fb_pitch": 0.0,
        "bullet_count": 0,
        "last_cmd": None,  # (mode, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc)
        "last_fb": (0.0, 0.0),
    }
    lock = threading.Lock()
    stats = Stats()
    quit_event = threading.Event()
    period = 1.0 / args.rate

    def sender():
        t0 = time.monotonic()
        frame_no = 0
        next_t = t0
        while not quit_event.is_set():
            t = time.monotonic() - t0
            with lock:
                mode = int(t // args.mode_cycle) % 4
                a = 0.4 * math.sin(t * 0.6)  # 云台缓慢摆动
                q = (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))  # wxyz
                cmd = state["last_cmd"]
                if cmd is not None and cmd[0] > 0:  # mode>0 时反馈角一阶滞后跟随指令
                    state["fb_yaw"] += 0.08 * (cmd[1] - state["fb_yaw"])
                    state["fb_pitch"] += 0.08 * (cmd[4] - state["fb_pitch"])
                fb_yaw, fb_pitch = state["fb_yaw"], state["fb_pitch"]
                bullet_count = state["bullet_count"]
            frame = struct.pack(
                RX_FMT, b"GV", mode,
                q[0], q[1], q[2], q[3],
                fb_yaw, 0.0, fb_pitch, 0.0,
                args.bullet_speed, bullet_count, ord("G"),
            )
            with lock:
                state["last_fb"] = (fb_yaw, fb_pitch)
            data = frame
            if args.inject_garbage and frame_no % args.inject_garbage == 0:
                junk = bytes(random.randrange(256) for _ in range(random.randint(1, 5)))
                data = junk + frame
                with lock:
                    stats.resync_events += 1
            try:
                os.write(master_fd, data)
            except OSError as e:
                # 视觉端尚未打开串口（或刚关闭）：不中断发送线程，稍后重试
                if not state.get("write_err_logged"):
                    state["write_err_logged"] = True
                    print(f"[EMU] write error: {e!r}", flush=True)
                time.sleep(0.05)
                next_t = time.monotonic() - t0
                continue
            with lock:
                stats.tx_frames += 1
            frame_no += 1
            next_t += period
            delay = next_t - (time.monotonic() - t0)
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic() - t0 + period

    def receiver():
        buf = b""
        while not quit_event.is_set():
            try:
                r, _, _ = select.select([master_fd], [], [], 0.1)
                if not r:
                    continue
                chunk = os.read(master_fd, 4096)
            except OSError:
                continue
            if not chunk:
                break  # 视觉端已关闭
            buf += chunk
            while True:
                idx = buf.find(b"VG")
                if idx < 0:
                    buf = buf[-1:]  # 保留可能的半个帧头
                    break
                buf = buf[idx:]
                if len(buf) < 28:
                    break
                head, mode, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc, tail = struct.unpack(
                    TX_FMT, buf[:28]
                )
                ok = True
                msgs = []
                if head != b"VG" or tail != ord("V"):
                    ok = False
                    msgs.append("head/tail mismatch")
                if mode not in (0, 1, 2):
                    ok = False
                    msgs.append(f"bad mode={mode}")
                for name, v in (
                    ("yaw", yaw), ("yaw_vel", yaw_vel), ("yaw_acc", yaw_acc),
                    ("pitch", pitch), ("pitch_vel", pitch_vel), ("pitch_acc", pitch_acc),
                ):
                    if not math.isfinite(v):
                        ok = False
                        msgs.append(f"{name} not finite")
                if mode == 0:
                    # IDLE 回显断言：mode=0 帧应回显最近反馈角，速度/加速度为 0
                    with lock:
                        fb_yaw, fb_pitch = state["last_fb"]
                    with lock:
                        stats.echo_checked += 1
                    if (
                        abs(yaw - fb_yaw) > 1e-5 or abs(pitch - fb_pitch) > 1e-5
                        or abs(yaw_vel) > 1e-5 or abs(yaw_acc) > 1e-5
                        or abs(pitch_vel) > 1e-5 or abs(pitch_acc) > 1e-5
                    ):
                        ok = False
                        with lock:
                            stats.echo_fail += 1
                        msgs.append(
                            f"IDLE echo mismatch: yaw={yaw:.4f} fb={fb_yaw:.4f} "
                            f"pitch={pitch:.4f} fb={fb_pitch:.4f}"
                        )
                if mode == 2:
                    with lock:
                        state["bullet_count"] = (state["bullet_count"] + 1) % 65536
                with lock:
                    state["last_cmd"] = (mode, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc)
                    stats.rx_frames += 1
                    if not ok:
                        stats.fail += 1
                if not ok:
                    print(f"[EMU][FAIL] frame #{stats.rx_frames}: {', '.join(msgs)}", flush=True)
                buf = buf[28:]

    def reporter():
        while not quit_event.wait(5.0):
            print(f"[EMU] {stats.line()}", flush=True)

    threads = [
        threading.Thread(target=sender, daemon=True),
        threading.Thread(target=receiver, daemon=True),
        threading.Thread(target=reporter, daemon=True),
    ]
    for th in threads:
        th.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    quit_event.set()
    print(f"[EMU] final: {stats.line()}")
    if stats.fail == 0 and stats.echo_fail == 0:
        print("[EMU] ALL PASS")
        sys.exit(0)
    print("[EMU] FAILURES DETECTED")
    sys.exit(1)


if __name__ == "__main__":
    main()
