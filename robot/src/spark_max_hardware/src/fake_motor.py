#!/usr/bin/env python3
"""Fake motors for the `test_loopback` protocol ONLY. This is NOT the SPARK MAX protocol.

Reads invented command frames (std id 0x100+dev, float32 duty) and answers with invented
feedback frames (std id 0x200+dev, float32 position_rad, float32 velocity_rad_s) so the
ros2_control plumbing can be tested on a virtual CAN bus.
"""
import argparse
import socket
import struct
import time

FRAME = "=IB3x8s"  # struct can_frame: can_id, dlc, pad, data[8] (16 bytes)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--ids", type=int, nargs="+", default=[1])
    ap.add_argument("--max-vel", type=float, default=20.0, help="rad/s at 100%% duty")
    ap.add_argument("--rate", type=float, default=100.0, help="feedback Hz")
    ap.add_argument("--mute-after", type=float, default=0.0,
                    help="stop sending feedback after N seconds (tests the staleness path)")
    a = ap.parse_args()

    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((a.iface,))
    s.setblocking(False)

    duty = {i: 0.0 for i in a.ids}
    pos = {i: 0.0 for i in a.ids}
    start = last = time.monotonic()
    period = 1.0 / a.rate
    while True:
        try:
            while True:
                cid, dlc, data = struct.unpack(FRAME, s.recv(16))
                dev = (cid & socket.CAN_SFF_MASK) - 0x100
                if dev in duty and dlc >= 4:
                    duty[dev] = struct.unpack("<f", data[:4])[0]
        except BlockingIOError:
            pass

        now = time.monotonic()
        dt, last = now - last, now
        muted = a.mute_after > 0 and now - start > a.mute_after
        for i in a.ids:
            vel = duty[i] * a.max_vel
            pos[i] += vel * dt
            if not muted:
                payload = struct.pack("<ff", pos[i], vel)
                s.send(struct.pack(FRAME, 0x200 + i, 8, payload))
        time.sleep(max(0.0, period - (time.monotonic() - now)))


if __name__ == "__main__":
    main()
