# data/ — 機体ごとに測って作るファイル

このフォルダの中身は Git で管理しない（.gitignore）。**別の機体のデータを使い回さない**こと。
場所は環境変数 `PIPER_PP_DATA` で変えられる。

| ファイル | 作るスクリプト | 内容 |
|---|---|---|
| poses.json | 02_jog.py（`s look`）、ros2/save_pose.py | 保存姿勢。関節角[rad]・フランジ・TCP |
| T_flange_cam.npy | 05_handeye.py | フランジ → カラーカメラ の 4x4 変換 |
| handeye_samples.json | 05_handeye.py | 校正に使ったサンプル（再計算用） |
| stations.json | 09_pick_place.py `--setup A/B`, `--set-advance`（`adv_pick`・`zoff`・`surface` は手で編集） | 台A・Bのキャップ上端[m]、台ごとの補正（README の「台ごとの補正」） |
| pick_trials.jsonl | 08_pick_test.py | 把持試験の記録 |
| pp_trials.csv | 09_pick_place.py | 往復試験の記録（1サイクル1行） |
| *.jpg / *.png | 04_snap.py, 06_locate.py ほか | 確認用画像 |

`examples/` はファイル形式の見本（数値は使わない）。
