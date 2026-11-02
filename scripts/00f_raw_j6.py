"""CAN の生データで J5・J6 の角度を表示する（SDK を通さない）
使い方: python scripts/00f_raw_j6.py   … Ctrl+C で終了

SDK は範囲外の値を捨てるため、原点がずれていると表示が止まって見える。
ID 0x2A7 の 8 バイト = J5, J6（ビッグエンディアン int32、単位 0.001°）
"""
import struct
import can
import _bootstrap  # noqa: F401
from piper_pp.config import CFG

bus = can.interface.Bus(channel=CFG["can_channel"], interface="socketcan")
print("Ctrl+C で終了")
try:
    while True:
        m = bus.recv(1.0)
        if m is None:
            print("\r受信なし（電源・CAN を確認）        ", end="", flush=True); continue
        if m.arbitration_id == 0x2A7:
            j5, j6 = (v / 1000 for v in struct.unpack(">ii", bytes(m.data)))
            print(f"\rJ5={j5:+9.2f}  J6={j6:+9.2f} deg   ", end="", flush=True)
except KeyboardInterrupt:
    print()
finally:
    bus.shutdown()
