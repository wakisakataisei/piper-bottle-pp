"""設定と機体固有データの読み書き

- config/robot.json : 機体ごとの設定値（Git で管理）
- data/             : 機体ごとに測って作るファイル（Git では管理しない）
    poses.json        保存姿勢（look, down など）
    T_flange_cam.npy  ハンドアイ校正の結果（フランジ → カラーカメラ）
    stations.json     台A・Bのキャップ位置と前進補正
環境変数 PIPER_PP_DATA で data/ の場所を変えられる。
"""
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONFIG_FILE = ROOT / "config" / "robot.json"
DATA = Path(os.environ.get("PIPER_PP_DATA", ROOT / "data"))
DATA.mkdir(parents=True, exist_ok=True)

CFG = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))


def data_path(name):
    return DATA / name


def load_json(name, default=None):
    p = data_path(name)
    if not p.exists():
        return {} if default is None else default
    return json.loads(p.read_text(encoding="utf-8"))


def save_json(name, obj):
    data_path(name).write_text(json.dumps(obj, indent=1, ensure_ascii=False), encoding="utf-8")


def load_pose(name):
    poses = load_json("poses.json")
    if name not in poses:
        raise SystemExit(f"保存姿勢 '{name}' がありません。scripts/02_jog.py で 's {name}' を実行して保存してください")
    return poses[name]["joints"]
