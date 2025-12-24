# DWA (Dynamic Window Approach) アルゴリズム解説

このドキュメントでは、`lidar_cart` パッケージに実装されている DWA アルゴリズムを用いたローカルプランナー (`local_planner.py`) について解説します。

## 概要

DWA (Dynamic Window Approach) は、ロボットの運動学的制約（速度・加速度）を考慮しながら、現在の状態から「次の瞬間に到達可能な速度空間（ダイナミックウィンドウ）」内で最適な速度指令 $(v, \omega)$ を探索するアルゴリズムです。
主に障害物回避をしながらゴールに向かうローカルプランニングに使用されます。

評価関数 $G(v, \omega)$ は以下のような項の重み付き和で表されます。

$$ G(v, \omega) = \alpha \cdot \text{heading}(v, \omega) + \beta \cdot \text{dist}(v, \omega) + \gamma \cdot \text{velocity}(v, \omega) $$

*   **heading**: ゴール方向への向きやすさ（ゴール地点との角度差）
*   **dist**: 障害物までの距離（近いとペナルティ）
*   **velocity**: 速度の大きさ（速いほうが良い）

## 実装ファイル

*   **ファイルパス**: `src/lidar_cart/scripts/local_planner.py`
*   **クラス名**: `LocalPlanner`

## コード解説

主要なメソッドについて解説します。

### 1. `calc_dynamic_window(self, u)`

現在の速度 $u = [v, \omega]$ から、次のタイムステップで取りうる速度の範囲（ダイナミックウィンドウ）を計算します。
以下の3つの制約の積集合（共通部分）をとります。

1.  **VS (Specification limits)**: ロボットの最大/最小速度・旋回速度（ハードウェア限界）。
2.  **VD (Dynamic limits)**: 現在の速度から、加速度制約内で次の `dt` 間に変化可能な速度範囲。
3.  **(VA (Admissible limits))**: 安全に停止できる速度（この実装では簡易的に障害物コストで代用しているため明示的には計算されていませんが、一般的には含まれます）。

```python
def calc_dynamic_window(self, u):
    # ...
    # 1. ハードウェア限界
    vs = [min_speed, max_speed, -max_yawrate, max_yawrate]
          
    # 2. ダイナミクス限界 (現在速度 +- 加速度 * dt)
    vd = [u[0] - max_accel * dt,
          u[0] + max_accel * dt,
          u[1] - max_dyawrate * dt,
          u[1] + max_dyawrate * dt]
          
    # 積集合 (Intersection)
    v_min = max(vs[0], vd[0])
    v_max = min(vs[1], vd[1])
    w_min = max(vs[2], vd[2])
    w_max = min(vs[3], vd[3])
    
    return [v_min, v_max, w_min, w_max]
```

### 2. `predict_trajectory(self, x_init, v, w)`

ある速度 $(v, \omega)$ で走行し続けた場合の、将来のロボットの軌跡を予測します。
`predict_time` 秒先までの位置をシミュレーションします。

```python
def predict_trajectory(self, x_init, v, w):
    # ...
    while cur_time <= predict_time:
        x[2] += w * dt # 向きの更新
        x[0] += v * math.cos(x[2]) * dt # X座標の更新
        x[1] += v * math.sin(x[2]) * dt # Y座標の更新
        traj.append([x[0], x[1], x[2], v, w])
        cur_time += dt
    return np.array(traj)
```

### 3. `dwa_search(self, x, u)`

最適な制御入力を探索するメインループです。

1.  ダイナミックウィンドウを計算 (`calc_dynamic_window`)。
2.  ウィンドウ内の全ての $(v, \omega)$ の組み合わせについてループします。
3.  各組み合わせについて軌跡を予測 (`predict_trajectory`)。
4.  評価関数（コスト関数）を計算します。
    *   `to_goal_cost`: ゴール方向との誤差。
    *   `speed_cost`: 最高速度との差（速いほどコスト低）。
    *   `obstacle_cost`: 障害物に衝突するかどうか。
5.  最小コストとなる $(v, \omega)$ を返します。

```python
def dwa_search(self, x, u):
    dw = self.calc_dynamic_window(u)
    
    # 探索ループ
    for v in np.arange(dw[0], dw[1], v_reso):
        for w in np.arange(dw[2], dw[3], y_reso):
            traj = self.predict_trajectory(x, v, w)
            
            # コスト計算
            to_goal_cost = ... * self.calc_to_goal_cost(traj, goal)
            speed_cost = ... * (max_speed - traj[-1, 3])
            ob_cost = ... * self.calc_obstacle_cost(traj)
            
            final_cost = to_goal_cost + speed_cost + ob_cost
            
            if final_cost < min_cost:
                min_cost = final_cost
                best_u = [v, w]
                
    return best_u
```

### 4. `calc_obstacle_cost(self, traj)`

予測軌跡が障害物と衝突するかを判定します。
この実装では、LiDARの最近傍点とロボット半径を比較する簡易的な判定を行っています。

*   軌跡上のいずれかの点で、障害物との距離が `robot_radius` 以下になれば、コストを無限大 (`inf`) にしてその経路を却下します。
