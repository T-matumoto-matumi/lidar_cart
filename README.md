# lidar_cart

ROS 2 Humble環境で動作する、2D LiDAR搭載ロボットのGazeboシミュレーションパッケージです。
対向二輪（Diff Drive）および4輪独立ステア（4WS / Omnidirectional）の2種類のモデルを提供します。

## 特徴
- **対向二輪モデル (Diff Drive)**: 2輪 + キャスター構成
- **4WSモデル (Omnidirectional)**: 4輪独立ステア（シミュレーション上はPlanar Moveによる全方向移動）
- 2D LiDAR (Gazebo Ray Sensor) 搭載
- 基本的な障害物環境 (`obstacle.world`)
- 円軌道を描く自動走行ノード (`drive_node`)
- 手動操縦モード (`teleop_twist_keyboard` 対応)

## 動作環境
- **OS**: Linux (WSL2対応)
- **ROS Version**: ROS 2 Humble
- **Simulator**: Gazebo Classic (Gazebo 11)

## インストールとビルド

```bash
# ワークスペースへの移動
cd /home/mimimi/lidar_cart

# 依存関係のインストール（必要に応じて）
sudo apt install ros-humble-teleop-twist-keyboard

# ビルド
colcon build --symlink-install --packages-select lidar_cart

# 環境変数の読み込み
source install/setup.bash
```

## 実行方法

### 1. 対向二輪モデル (自動走行)

```bash
ros2 launch lidar_cart sim.launch.py
```

### 2. 4WSモデル (自動走行)

```bash
ros2 launch lidar_cart sim.launch.py robot_model:=four_ws
```

### 3. 手動操縦 (Teleop)

自動走行を行わず、キーボードで操作したい場合は `drive_mode:=manual` を指定します。

ターミナル1:
```bash
ros2 launch lidar_cart sim.launch.py drive_mode:=manual
```

ターミナル2:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Headlessモード (GUIなし)
WSL2などでGUIが重い、またはクラッシュする場合は、GUIなしモードで実行してください。

```bash
ros2 launch lidar_cart sim.launch.py gui:=false
```

-------------

## トラブルシューティング

### GazeboやRviz2がクラッシュする場合 (D3D12エラー)
WSL2環境で `D3D12: Removing Device` エラーや `Segmentation fault` で落ちる場合、GPUドライバとの相性問題が原因です。以下の環境変数を設定してソフトウェアレンダリングを強制することで改善します。

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch lidar_cart sim.launch.py
# または
rviz2
```

### プロセスがゾンビ化した場合 (Address already in use)
Gazeboが正しく終了せず、バックグラウンドにプロセスが残ってしまい再起動できない（`Address already in use` エラーが出る）場合は、以下のコマンドでクリーンアップしてください。

```bash
pkill -f gzserver
pkill -f gzclient
pkill -f robot_state_publisher
pkill -f drive_node
```

**それでも直らない場合 (手動強制終了):**
`pkill` で消えない場合は、PIDを特定して `kill -9` してください。

```bash
# プロセスの確認
ps aux | grep -E 'gzserver|gzclient|ros2 launch'

# 確認したPIDを指定して強制終了 (例: PIDが12151の場合)
kill -9 12151
```

### Audioエラーについて
起動時に `[Err] [OpenAL.cc:84] Unable to open audio device` というエラーが表示されることがありますが、これはシミュレーション動作には影響しないため無視して問題ありません。
