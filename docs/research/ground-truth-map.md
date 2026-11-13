# 正解の地図（ground truth）の作り方と「誤って空き」率の評価

作成: 2026-11-14。実験 4-1（カメラの床の false-free rate）で使う。**手順は机上で書いたもので、実機ではまだ試していない。**

## 1. 何を測るか

カメラ（RealSense の `/cliff/floor`、RICOH THETA の `/theta/floor`）が床と見たセルは、探索の判断用の地図でだけ「未知 → 空き」になる（`seen.py`、SLAM の地図は変えない）。そのセルのうち、実際は壁や障害物だったものの割合を測る。

- **false_free**: 正解の地図で、占有セルから `--tol`（既定 0.10 m）以内にあるカメラ床セル
- **true_free**: それ以外で、正解の地図で空きのセル
- **unknown**: 正解の地図で未知、または地図の外（確かめられない。割合の計算には入れない）
- **false-free rate** = false_free / (false_free + true_free)
- **non_traversable**: true_free のうち、正解の占有から `--robot-radius`（既定 0.30 m）以内のセル。床ではあるが、ロボットの中心は入れない。参考として別に数える

`--tol` について。SLAM の壁は 1〜2 セル（5〜10 cm）の厚みで描かれ、出発点の置き方や SLAM のずれによって、正解の地図と数 cm ずれる。0.10 m は、壁の際のカメラ床も「誤り」に数える保守的な値である。この値で結果が大きく変わらないことを示すため、評価では tol = 0 / 0.05 / 0.10 / 0.20 m の率も毎回出す（`tol sweep`）。論文では 0.10 m の値を主に使い、ほかの値は感度として載せる。

## 2. 実行ごとの記録（自動で作られる）

自動探索の 1 回が終わると（完了・停止ボタン・手動操縦・エラーのどれでも）、`~/cugo_logs/explore_YYYYmmdd_HHMMSS.jsonl` の隣に、同じ名前の **`.seen.npz`** が作られる（`node.dump_explore_run` → `falsefree.save_run`）。書き込みは別スレッドで行い、失敗してもログに警告が出るだけである。

| キー | 中身 |
|---|---|
| `cells_xy` (N,2) | カメラ床セルの中心（map 座標 [m]）。min_hits 未満のセルも含む |
| `hits` (N,) | 見た回数（1 フレーム 1 回） |
| `src` (N,) | 見たカメラ。1 = RealSense、2 = THETA、3 = 両方 |
| `obs_dist` (N,) | 見たときのセンサー（点群の座標系の原点）からの最短距離 [m] |
| `cell`, `min_hits` | SeenFloor の設定（既定 0.05 m、3 回） |
| `grid`, `grid_res`, `grid_origin` | 最後の slam_toolbox の `/map`（-1 未知、0..100）、解像度、原点 [x, y, yaw] |
| `traj` (M,3) | 記録の sample の t, x, y |
| `meta` | 元の JSONL の名前など（JSON 文字列） |

評価では、実行のときの `min_hits` 以上のセルだけを使う（`--min-hits` で変えて試せる）。

## 3. 正解の地図の作り方

### 3.1 準備

1. 床に**出発点の印**をつける。養生テープで、ロボットの後ろの端と左の端が当たる L 字を貼り、前の向きを矢印で書く。以後のすべての実行と正解の地図づくりは、ここから同じ向きで始める。
2. ガラスや鏡など、LiDAR が見えない所の位置を記録しておく（写真・メジャー）。正解の地図で未知や空きになっていたら、あとで手で直す（3.4）。
3. 人や物が動かない時間帯を選ぶ。椅子や台車は、実験の間ずっと同じ位置に置いたままにする。

### 3.2 地図を作る

1. ロボットを出発点の印に置き、**bringup を起動し直す**（`mode:=slam`）。これで map 座標の原点が出発点になる。
2. Web UI の手動操縦で、**ゆっくり**（0.1〜0.15 m/s、旋回もゆっくり）走らせる。
   - 同じ場所を**2〜3 回**通る。行きと帰りで向きを変え、壁を両側から見る。
   - 一周してスタートの近くに戻り、**ループを閉じる**（地図の壁が二重にならないことを Web UI で確かめる）。
   - 椅子の脚や柱など小さな物の周りも回る。
3. 地図に二重の壁・ずれがないことを確かめたら、Web UI の「地図保存」で `gt_<場所>`（例: `gt_corridor3f`）として保存する。`~/maps/gt_corridor3f.yaml` と `.pgm` ができる（slam_toolbox の save_map、map_server 形式）。
   - Web UI が使えないときは `ros2 run nav2_map_server map_saver_cli -f ~/maps/gt_corridor3f` でもよい。
4. うまくいかなかったら（壁が二重・大きくずれた）、bringup を起動し直して最初から作る。

### 3.3 実行ごとの手順

1. ロボットを出発点の印に、同じ向きで置く。
2. **bringup を起動し直す**のが確実。Web UI の「地図リセット」でもよいが、その場合はリセットの直後に Web UI の姿勢が (0, 0, 0) の近くか確かめる（odom が積算されたままだと、map の原点が出発点にならないことがある。ずれていたら起動し直す）。
3. 自動探索を始める。終わると `explore_*.jsonl` と `explore_*.seen.npz` ができる。
4. 条件（`explore_variant`、カメラのあり・なし）を変えて、同じ手順で繰り返す。

### 3.4 正解の地図を手で直す（必要なとき）

ガラス・鏡・黒い面は LiDAR で見えないため、正解の地図でも未知や空きになりうる。そのままだと、ガラスの向こうのカメラ床が true_free や unknown に数えられてしまう。3.1 で測った位置に、画像編集ソフト（GIMP など）で黒（0）の線を描き足す。描き足した所は論文に書く。PGM の上の行が地図の +y 側である。

## 4. 評価の実行

PC にリポジトリと numpy / scipy があればよい（ROS は要らない）。matplotlib は図を描くときだけ使う。

```bash
# 正解の地図と比べる（既定では位置を少し合わせる）
python3 jetson/scripts/eval_false_free.py --gt ~/maps/gt_corridor3f.yaml ~/cugo_logs/explore_*.seen.npz

# 位置合わせなし（出発点で正確に合わせたとき）／手で与える（x, y [m]、yaw [度]）
python3 jetson/scripts/eval_false_free.py --gt gt.yaml --no-align run.seen.npz
python3 jetson/scripts/eval_false_free.py --gt gt.yaml --tf 0.05 -0.10 2.0 run.seen.npz

# 表・セルごとの CSV・図
python3 jetson/scripts/eval_false_free.py --gt gt.yaml --json out.json --csv cells.csv --plot fig.png run.seen.npz

# 正解の地図がまだ無いとき: 実行自身の最後の /map（LiDAR）と比べる
python3 jetson/scripts/eval_false_free.py --self-check ~/cugo_logs/explore_*.seen.npz
```

### 4.1 位置合わせ

出発点が同じなら、実行の map 座標と正解の地図の座標は一致するはずである。置き方の数 cm・数度のずれを吸収するため、既定では次の総当たりで補正する。

- 実行の最後の `/map` の占有セル（最大 4000 点）を、出発点（map の原点）を中心に回してずらし、正解の占有までの距離 d から 点数 = 平均 max(0, 1 − d / 0.2 m) を計算する。
- 探す範囲は ±0.5 m（刻みは正解の解像度）と ±10°（刻み 1°）。`--align-xy`・`--align-yaw`・`--align-step-*` で変えられる。
- 出力の `transform` が選ばれた変換である。`score` は選んだ変換の点数、`identity` は補正なしの点数、`inliers` は 0.1 m 以内に正解の占有がある点の割合。
- 目安: inliers が 0.8 より小さい、または選ばれた変換が範囲の端（±0.5 m、±10°）にあるときは、合わせに失敗している可能性が高い。図（`--plot`）で確かめ、必要なら `--tf` で与える。

### 4.2 出力の読み方

```
== explore_20261120_101500.seen.npz
  transform: x=+0.050 m y=-0.000 m yaw=+1.00 deg  (align score=0.912 identity=0.850 inliers=95.0% points=4000)
  min_hits=3 tol=0.10 m robot_radius=0.30 m
  total          cells= 12000 false=  150 true= 11000 unknown=  850 nontrav= 2100 rate=1.35%
  realsense      ...（そのカメラが見たセル。両方が見たセルは両方に入る）
  theta_only     ...
  dist 0-1 m     ...（見たときの距離ごと）
  tol sweep: 0.00:0.40%  0.05:0.90%  0.10:1.35%  0.20:3.10%
```

（数字は説明用の例であり、結果ではない。）

### 4.3 自己チェック（--self-check）

正解の地図の代わりに、その実行の最後の `/map` を使う。「カメラが床と見たが、あとで LiDAR が壁と見た」セルが false_free になる。正解の地図を作る前の確認用である。LiDAR が最後まで見なかったセル（ガラスの向こう、LiDAR の高さより低い障害物）は unknown か true_free に数えられるので、本番の評価には使わない。

## 5. ROS なしでは確かめていないこと

- 探索の終わりに `.seen.npz` が実際に書かれること（node と explorer の変更は、単体テストと構文チェックだけ）
- `src`（どのカメラか）が `/cliff/floor` と `/theta/floor` で正しく分かれること
- `obs_dist` の基準点。点群の `frame_id` の原点からの距離で、どちらの床も base_footprint で出しているはずなので「ロボットからの距離」になる（実機の frame_id は未確認）
- 地図リセットの後に map の原点が出発点になるか（3.3 の 2）
