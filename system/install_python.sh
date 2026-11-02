#!/usr/bin/env bash
# Python 環境（venv）、pyAgxArm、pyorbbecsdk（v1 系）を入れる
# 使い方: bash system/install_python.sh   （リポジトリの一番上で実行）
# 所要: pyorbbecsdk のビルドに Jetson Orin で数分
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
VENV=${VENV:-$HOME/venv_pp}

sudo apt install -y python3-venv python3-dev python3-pip can-utils cmake build-essential libusb-1.0-0-dev pybind11-dev

# 1) venv（ROS2 Humble の Python と共存させるため system-site-packages 付き）
[ -d "$VENV" ] || python3 -m venv --system-site-packages "$VENV"
source "$VENV/bin/activate"
pip install -U pip

# 2) pyAgxArm（LGPL-3.0。このリポジトリには含めず、依存として入れる）
pip install "python-can>3.3.4" numpy scipy
pip install "git+https://github.com/agilexrobotics/pyAgxArm.git"

# 3) このリポジトリ（piper_pp）を編集可能モードで入れる
pip install -e "$REPO"

# 4) pyorbbecsdk v1 系（Dabai DC1 は OpenNI 系のため v1 = main ブランチを使う。v2 系では認識しない）
if ! python -c "import pyorbbecsdk" 2>/dev/null; then
  cd "$HOME"
  [ -d pyorbbecsdk ] || git clone -b main https://github.com/orbbec/pyorbbecsdk.git
  cd pyorbbecsdk
  pip install pybind11
  mkdir -p build && cd build
  cmake -Dpybind11_DIR="$(python -m pybind11 --cmakedir)" ..
  make -j"$(nproc)" && make install
  sudo bash ../scripts/install_udev_rules.sh
  sudo udevadm control --reload-rules && sudo udevadm trigger
  grep -q "pyorbbecsdk/install/lib" ~/.bashrc || echo 'export PYTHONPATH=$PYTHONPATH:$HOME/pyorbbecsdk/install/lib' >> ~/.bashrc
  export PYTHONPATH=${PYTHONPATH:-}:$HOME/pyorbbecsdk/install/lib
fi

grep -q "$VENV/bin/activate" ~/.bashrc || echo "source $VENV/bin/activate" >> ~/.bashrc

# 5) 確認
python - <<'PY'
import pyAgxArm, numpy, scipy, cv2
print("pyAgxArm OK / numpy", numpy.__version__, "/ scipy", scipy.__version__, "/ OpenCV", cv2.__version__)
assert hasattr(cv2, "aruco"), "cv2.aruco がありません（opencv-contrib-python が必要）"
try:
    import pyorbbecsdk; print("pyorbbecsdk OK")
except Exception as e:
    print("pyorbbecsdk NG:", e)
PY
echo "完了。新しいターミナルを開くか source ~/.bashrc"
