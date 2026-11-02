#!/usr/bin/env bash
# JetPack 6 (L4T R36.x) のカーネルに含まれない gs_usb（USB-CAN）モジュールをビルドして入れる
# 使い方: bash system/build_gs_usb.sh
# - ソースは kernel.org から実行中のカーネルと同じ版の gs_usb.c を取得する（GPL のためリポジトリには含めない）
# - /lib/modules/$(uname -r)/updates/ に入れる（drivers/net/can/usb/ は JetPack に存在しない）
# - 起動時に読み込むよう /etc/modules-load.d/gs_usb.conf を作る
set -euo pipefail

KVER_FULL=$(uname -r)                       # 例 5.15.148-tegra
KVER=${KVER_FULL%%-*}                        # 例 5.15.148
KDIR=/lib/modules/${KVER_FULL}/build
WORK=${HOME}/gs_usb_build

if modinfo gs_usb >/dev/null 2>&1; then
  echo "gs_usb はすでに入っています: $(modinfo -n gs_usb)"; exit 0
fi
[ -d "${KDIR}" ] || { echo "カーネルヘッダ ${KDIR} がありません（sudo apt install nvidia-l4t-kernel-headers）"; exit 1; }

mkdir -p "${WORK}" && cd "${WORK}"
URL="https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/net/can/usb/gs_usb.c?h=v${KVER}"
echo "取得: ${URL}"
wget -q -O gs_usb.c "${URL}"
echo "obj-m += gs_usb.o" > Makefile
make -C "${KDIR}" M="${WORK}" modules

sudo mkdir -p "/lib/modules/${KVER_FULL}/updates"
sudo cp gs_usb.ko "/lib/modules/${KVER_FULL}/updates/"
sudo depmod -a
sudo modprobe can_dev || true
sudo modprobe gs_usb
echo gs_usb | sudo tee /etc/modules-load.d/gs_usb.conf >/dev/null
echo "完了: $(modinfo -n gs_usb)"
lsmod | grep gs_usb
