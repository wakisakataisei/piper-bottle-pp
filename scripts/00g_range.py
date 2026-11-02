"""J6 の可動範囲を CAN の生データで測る（J6 は脱力状態で手で回す）
使い方: python scripts/00g_range.py   … 両端まで回したら Ctrl+C
正常な機体なら中心がほぼ 0°。大きくずれていたら 00h_calib_j6.py で原点を書き直す
"""
import struct
import can
import _bootstrap  # noqa: F401
from piper_pp.config import CFG

bus = can.interface.Bus(channel=CFG["can_channel"], interface="socketcan")
lo = hi = None
print("J6を両端までゆっくり回してください。終わったら Ctrl+C")
try:
    while True:
        m = bus.recv(1.0)
        if m is None or m.arbitration_id != 0x2A7:
            continue
        j6 = struct.unpack(">ii", bytes(m.data))[1] / 1000
        lo = j6 if lo is None else min(lo, j6)
        hi = j6 if hi is None else max(hi, j6)
        print(f"\rJ6={j6:+8.2f}  最小={lo:+8.2f}  最大={hi:+8.2f} deg   ", end="", flush=True)
except KeyboardInterrupt:
    bus.shutdown()
    if lo is None:
        print("\nデータを受信できませんでした")
    else:
        print(f"\n範囲: {lo:+.2f} 〜 {hi:+.2f} deg  幅 {hi-lo:.2f}  中心 {(lo+hi)/2:+.2f} deg")
