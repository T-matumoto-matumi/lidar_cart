# A* (A-Star) アルゴリズム解説

このドキュメントでは、`lidar_cart` パッケージに実装されている A* アルゴリズムを用いたグローバルプランナー (`global_planner.py`) について解説します。

## 概要

A* アルゴリズムは、グラフ探索アルゴリズムの一つであり、スタート地点からゴール地点までの最短経路を効率的に探索するために広く利用されています。
ダイクストラ法を拡張したもので、**ヒューリスティック関数 (Heuristic Function)** を用いることで、ゴールに近いと思われるノードを優先的に探索し、計算コストを削減しています。

コスト関数 $f(n)$ は以下のように定義されます。

$$ f(n) = g(n) + h(n) $$

*   $g(n)$: スタート地点から現在のノード $n$ までの実際の移動コスト
*   $h(n)$: 現在のノード $n$ からゴール地点までの推定コスト（ヒューリスティック）

## 実装ファイル

*   **ファイルパス**: `src/lidar_cart/scripts/global_planner.py`
*   **クラス名**: `GlobalPlanner`

## コード解説

主要なメソッドについて解説します。

### 1. `heuristic(self, x1, y1, x2, y2)`

ゴールまでの推定コスト（ヒューリスティック）を計算する関数です。
ここでは、シンプルに **ユークリッド距離** を使用しています。

```python
def heuristic(self, x1, y1, x2, y2):
    # Euclidean distance
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
```

### 2. `plan_path(self, start, goal)`

A* アルゴリズムのメインループです。

1.  **座標変換**: ワールド座標 (m) をグリッド座標 (index) に変換します。
2.  **初期化**: 
    *   `open_set`: 探索候補のノードを入れる優先度付きキュー (Priority Queue)。
    *   `g_score`: 各ノードまでの最小コストを記録する辞書。
    *   `f_score`: 各ノードの $f(n)$ スコアを記録する辞書。
3.  **探索ループ (`while open_set`)**:
    *   最も $f(n)$ が小さいノードを取り出します (`heapq.heappop`)。
    *   ゴールに到達したら経路を再構築して返します。
    *   現在のノードの周囲 (8近傍) を探索します (`directions`)。
    *   **障害物判定**: グリッドの値を確認し、障害物がある場合はスキップします。
    *   **コスト更新**: 隣接ノードへの新しい移動コストが、既知のコストより小さければ更新し、`open_set` に追加します。

```python
# A* Algorithm Loop 部分の抜粋
while open_set:
    # ... (タイムアウト処理) ...

    current = heapq.heappop(open_set)[1] # f値が最小のノードを取得
    
    if current == (goal_gx, goal_gy):
        return self.reconstruct_path(...) # ゴール到達
        
    for dx, dy in directions:
        neighbor = (current[0] + dx, current[1] + dy)
        
        # ... (範囲外チェック) ...
        # ... (障害物チェック) ...
        
        # コスト計算 (斜め移動は sqrt(2), 直線移動は 1)
        dist = math.sqrt(dx*dx + dy*dy)
        tentative_g_score = g_score[current] + dist
        
        # より良い経路が見つかった場合更新
        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g_score
            f_score[neighbor] = tentative_g_score + self.heuristic(...)
            heapq.heappush(open_set, (f_score[neighbor], neighbor))
```

### 3. `reconstruct_path(self, came_from, current, grid_to_world, header)`

ゴールからスタートまで `came_from` 辞書を遡って経路を復元します。
最後にリストを逆順 (`reverse`) にし、グリッド座標をワールド座標の `Path` メッセージに変換して返します。

```python
def reconstruct_path(self, came_from, current, grid_to_world, header):
    # ...
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
        
    total_path.reverse() # スタート -> ゴールの順に直す
    # ...
```
