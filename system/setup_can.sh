#!/usr/bin/env bash
# USB-CAN（gs_usb）を探して udev ルールを作り、can_piper として 1 Mbps で up する
# 使い方: USB-CAN を挿した状態で  bash system/setup_can.sh
# Jetson 本体の CAN（mttcan）は can0 になるので、USB-CAN は can1 などになる。名前を固定するために udev を使う
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)

PORT=""; IF=""
for d in /sys/class/net/*; do
  n=$(basename "$d")
  drv=$(basename "$(readlink -f "$d/device/driver" 2>/dev/null)" 2>/dev/null || true)
  if [ "$drv" = "gs_usb" ]; then
    IF=$n; PORT=$(basename "$(readlink -f "$d/device")"); break
  fi
done
[ -n "$PORT" ] || { echo "gs_usb の CAN が見つかりません（lsusb で 1d50:606f を確認、build_gs_usb.sh を実行済みか）"; exit 1; }
echo "USB-CAN: インターフェース ${IF}、USB ポート ${PORT}"

sed "s/@USB_PORT@/${PORT}/" "${HERE}/90-can-piper.rules.template" | sudo tee /etc/udev/rules.d/90-can-piper.rules >/dev/null
sudo udevadm control --reload-rules

if [ "$IF" != "can_piper" ]; then
  sudo ip link set "$IF" down
  sudo ip link set "$IF" name can_piper
fi
sudo ip link set can_piper type can bitrate 1000000
sudo ip link set can_piper up
ip -details link show can_piper | head -4
echo "完了。以後は同じ USB ポートに挿せば自動で can_piper になる（別のポートに挿したら再実行）"
