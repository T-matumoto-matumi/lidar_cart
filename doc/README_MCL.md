# MCL (Monte Carlo Localization / Particle Filter) アルゴリズム解説

このドキュメントでは、`lidar_cart` パッケージに実装されている MCL (Monte Carlo Localization) ノード (`mcl_node.py`) について解説します。

## 概要

MCL は、パーティクルフィルタ (Particle Filter) を用いてロボットの自己位置推定を行うアルゴリズムです。
多数の「パーティクル（粒子）」を用いて、ロボットの位置の確率分布を近似的に表現します。
各パーティクルは「ロボットがここにいるかもしれない」という仮説（位置 $x, y$ と向き $\theta$）を持っています。

アルゴリズムの主なサイクルは以下の通りです。

1.  **予測 (Motion Update)**: ロボットの移動情報（オドメトリ）に基づいて、全パーティクルを移動させます。この際、ノイズを加えることで不確実性を表現します。
2.  **更新 (Sensor Update / Measurement Update)**: センサ情報（LiDAR）と地図を照らし合わせ、各パーティクルの「もっともらしさ（重み）」を計算します。
3.  **リサンプリング (Resampling)**: 重みの大きいパーティクルを増やし、重みの小さいパーティクルを減らすことで、確率の高い場所にパーティクルを集中させます。

## 実装ファイル

*   **ファイルパス**: `src/lidar_cart/scripts/mcl_node.py`
*   **クラス名**: `MclNode`, `ParticleFilter`

## コード解説

`ParticleFilter` クラスの主要なメソッドについて解説します。

### 1. `motion_update(self, delta_linear, delta_angular, dt)`

オドメトリの入力に基づいてパーティクルの状態を更新します（予測ステップ）。
速度・角速度モーションモデルを使用し、入力にガウスノイズを加えてパーティクルを拡散させます。

```python
def motion_update(self, delta_linear, delta_angular, dt):
    # ノイズパラメータ (alpha1 ~ alpha4)
    # 入力にノイズを加える (制御ノイズ)
    noise_v = np.random.normal(0, ..., self.num_particles)
    noise_w = np.random.normal(0, ..., self.num_particles)
    
    v_hat = v + noise_v
    w_hat = w + noise_w
    
    # 状態遷移方程式に従ってパーティクルを移動
    # 直進成分と回転成分を計算し、全パーティクルの座標 (x, y, theta) を更新
    # ... (Vectorized calculation) ...
```

### 2. `sensor_update(self, scan_ranges, ...)`

LiDAR の観測値を用いてパーティクルの重みを更新します（尤度計算ステップ）。

1.  **スキャン点の座標変換**: ロボット座標系のスキャン点を、各パーティクルの仮説位置に基づいて地図座標系の点に変換します。
2.  **地図参照**: 変換された点が地図上の「障害物あり」のセルに乗っているかをチェックします。
3.  **重み計算（尤度）**:
    *   パーティクルが正しい位置にいれば、スキャン点は地図の障害物と重なるはずです。
    *   地図を参照し、障害物セル (`value > 50`) にヒットした点の数をカウントしたり、確率モデルを用いて重みを計算します。
    *   この実装では、障害物にヒットした場合に高い重みを付与する簡易的な尤度場モデルのような計算を行っています。

```python
def sensor_update(self, scan_ranges, ...):
    # ...
    # 各パーティクルごとに、LiDARの点群を地図座標系へ変換
    x_map = p_x + xs_robot * cos_theta - ys_robot * sin_theta
    y_map = p_y + xs_robot * sin_theta + ys_robot * cos_theta
    
    # 地図上の値を参照 (occupancy grid)
    # map_vals = ...
    
    # 重みの計算
    # 障害物(>50)にヒットしたら重み 1.0, それ以外は 0.1
    occupied_mask = (map_vals > 50) & in_bounds_mask
    weights[occupied_mask] = 1.0
    
    # 全スキャン点の尤度の積 (log-sum) をとり、パーティクル全体の重みを更新
    log_weights = np.sum(np.log(weights), axis=1)
    self.weights = np.exp(log_weights - max_log_weight)
    
    # 正規化
    self.weights /= np.sum(self.weights)
```

### 3. `resample(self)`

重みに基づいてパーティクルを選別します（リサンプリングステップ）。
ここでは **系統的リサンプリング (Systematic Resampling / Low Variance Sampling)** を実装しています。
これにより、パーティクルの多様性を維持しつつ、確率の高い場所にパーティクルを集約させます。
リサンプリングは、有効パーティクル数 (Effective Sample Size) が閾値を下回った場合のみ実行することで、パーティクル枯渇問題 (Particle Deprivation) を防いでいます。

```python
def resample(self):
    # 有効パーティクル数を計算 (1 / sum(w^2))
    neff = 1.0 / np.sum(self.weights**2)
    
    # 閾値チェック
    if neff > self.num_particles / 2.0:
        return
        
    # Low Variance Sampling アルゴリズム
    r = np.random.uniform(0, 1.0 / self.num_particles)
    # ... (累積重みに基づいて新しいパーティクルを選択) ...
```

### 4. `get_estimated_pose(self)`

現在の自己位置推定結果を計算します。
全パーティクルの位置の加重平均（または単なる平均）を計算します。
角度 ($\theta$) の平均については、角度の周期性を考慮して `atan2(sum(sin), sum(cos))` で計算します。

```python
def get_estimated_pose(self):
    x = np.mean(self.particles[:, 0])
    y = np.mean(self.particles[:, 1])
    # 角度の平均ベクトル計算
    theta = np.arctan2(np.sum(np.sin(...)), np.sum(np.cos(...)))
    return x, y, theta
```
