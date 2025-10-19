# DYNAMIXEL Controller

DYNAMIXEL XL330-M288-T サーボモーターの制御を行うROSパッケージです。キーボードによる手動制御と、YOLOを用いた人物追従機能を提供します。首振りロボットの頭部制御に最適化されています。

## 概要

このパッケージは、DYNAMIXEL SDK を使用してXL330-M288-T サーボモーターを直接制御し、2つの動作モードを提供します：
1. **キーボード制御モード**: 手動での矢印キー操作
2. **人物追従モード**: YOLOによる人検出結果に基づく自動追跡

安全な角度制限、滑らかな動作制御、エラーハンドリングを実装しています。

## 特徴

- **2つの制御モード**: キーボード制御と人物追従の切り替え可能
- **直接SDK制御**: DYNAMIXEL SDK を使用した高速・高精度制御
- **YOLO連携**: リアルタイム人物検出による自動追跡
- **柔軟な未検出時動作**: 3つの動作パターンから選択可能
- **安全機能**: 角度制限（-180°〜+180°）、トルク制御、エラーハンドリング
- **ROS統合**: ROSパラメータシステム、トピック配信対応
- **設定可能**: 移動ステップ、速度、角度制限などをパラメータで調整可能

## 必要な機材

- **サーボモーター**: DYNAMIXEL XL330-M288-T (ID: 1)
- **インターフェース**: U2D2 (USB to RS485 converter)
- **電源**: 12V電源 (XL330用)
- **接続**: 3Pinケーブル (XL330 ↔ U2D2)

## システム要件

- **ROS**: ROS1 Noetic
- **OS**: Ubuntu 20.04
- **Python**: Python 3.8+
- **依存パッケージ**: `dynamixel_sdk`, `rospy`, `std_msgs`, `vision_msgs`

## インストール

### 1. 依存関係のインストール

```bash
# DYNAMIXEL SDK のインストール
sudo apt update
sudo apt install ros-noetic-dynamixel-sdk

# Vision messages (人物追従機能用)
sudo apt install ros-noetic-vision-msgs

# 必要なPythonライブラリ
sudo apt install python3-termios
```

### 2. ワークスペースのビルド

```bash
cd ~/yolo_ws
catkin_make
source devel/setup.bash
```

### 3. udev ルールの設定 (オプション)

U2D2を固定デバイス名で使用する場合:

```bash
# udevルールファイルを作成
sudo nano /etc/udev/rules.d/99-dynamixel-workbench.rules
```

以下の内容を記述:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ENV{ID_MM_DEVICE_IGNORE}="1", ATTRS{serial}=="*", SYMLINK+="ttyDXL", GROUP="dialout", MODE="0666"
```

```bash
# ルールを適用
sudo udevadm control --reload-rules
sudo udevadm trigger

# ユーザーをdialoutグループに追加
sudo usermod -a -G dialout $USER
# ログアウト・ログインが必要
```

## 使用方法

### 1. モード切り替えによる起動

#### キーボード制御モード（デフォルト）

```bash
# ワークスペースの環境設定
cd ~/yolo_ws
source devel/setup.bash

# キーボード制御モードで起動
roslaunch dynamixel_controller controller.launch mode:=keyboard device_name:=/dev/ttyUSB0
```

#### 人物追従モード

```bash
# 人物追従モードで起動
roslaunch dynamixel_controller controller.launch mode:=tracking device_name:=/dev/ttyUSB0
```

### 2. 人物追従時の未検出動作設定

人物が検出されなくなった時の動作を3つのパターンから選択できます：

#### パターン1: 中央復帰（デフォルト）
```bash
# 人物が見つからない場合、中央位置（0度）に戻る
roslaunch dynamixel_controller controller.launch mode:=tracking no_person_behavior:=return_to_center
```

#### パターン2: 最終位置で停止
```bash
# 人物が見つからない場合、最後の位置で停止
roslaunch dynamixel_controller controller.launch mode:=tracking no_person_behavior:=stay
```

#### パターン3: 探索動作
```bash
# 人物が見つからない場合、左右に探索動作を行う
roslaunch dynamixel_controller controller.launch mode:=tracking no_person_behavior:=search
```

### 3. 人物追従の詳細設定

```bash
# 探索動作の詳細設定例
roslaunch dynamixel_controller controller.launch \
  mode:=tracking \
  no_person_behavior:=search \
  search_amplitude_deg:=90 \
  search_speed_deg_per_sec:=45 \
  no_detection_timeout_sec:=2.0 \
  device_name:=/dev/ttyUSB0
```

### 4. キーボード制御モードでの操作

キーボードモードで起動後、以下のキーで操作:

| キー | 動作 | 説明 |
|------|------|------|
| `←` (左矢印) | 左回転 | 10度ずつ左に回転 |
| `→` (右矢印) | 右回転 | 10度ずつ右に回転 |
| `r` | リセット | 中央位置(0度)に戻る |
| `h` | ヘルプ | 操作方法を表示 |
| `q` | 終了 | 安全にプログラム終了 |

### 5. 人物追従の前提条件

人物追従モードを使用するには、以下のノードが起動している必要があります：

```bash
# カメラノードの起動（例：USB カメラ）
rosrun usb_cam usb_cam_node

# YOLOによる人物検出ノードの起動
rosrun person_detector person_detector_node.py
```

## パラメータ設定

### 基本パラメータ

#### モード選択
- `mode`: 動作モード (`keyboard` または `tracking`)

#### デバイス設定
- `device_name`: U2D2のデバイスパス (例: `/dev/ttyUSB0`)
- `baudrate`: 通信速度 (デフォルト: 57600)

#### モーター設定
- `motor_id`: モーターID (デフォルト: 1)
- `movement_step`: キーボードモードでの1回の移動角度 (デフォルト: 10度)
- `profile_velocity`: 回転速度 (デフォルト: 50)
- `min_angle_deg`: 最小角度 (デフォルト: -180度)
- `max_angle_deg`: 最大角度 (デフォルト: 180度)

### 人物追従モード専用パラメータ

#### カメラ設定
- `detections_topic`: 人物検出結果のトピック名 (デフォルト: `/detected_persons`)
- `image_width`: カメラ画像の幅 (デフォルト: 640)
- `image_height`: カメラ画像の高さ (デフォルト: 480)
- `horizontal_fov_deg`: カメラの水平視野角 (デフォルト: 70.0度)

#### 未検出時動作設定
- `no_person_behavior`: 未検出時の動作 (`return_to_center`, `stay`, `search`)
- `no_detection_timeout_sec`: 未検出判定までの時間 (デフォルト: 1.0秒)

#### 探索動作設定（no_person_behavior:=search時）
- `search_amplitude_deg`: 探索範囲 (デフォルト: ±60度)
- `search_speed_deg_per_sec`: 探索速度 (デフォルト: 30度/秒)

#### 制御設定
- `control_rate_hz`: 制御ループの周波数 (デフォルト: 10Hz)

### 設定ファイルでの詳細設定

`config/dynamixel_params.yaml` で詳細な制御パラメータを設定:

```yaml
# 制御パラメータ
control_table:
  Profile_Velocity: 50      # 移動速度
  Profile_Acceleration: 10  # 加速度
  Position_P_Gain: 800      # P制御ゲイン
  Current_Limit: 1000       # 電流制限 (mA)
```

## トピック

### 配信トピック

#### キーボード制御モード
- `/dynamixel_status` (`std_msgs/String`): モーター状態情報
  - フォーマット: `"angle:45.0,direction:right"`

#### 人物追従モード
- `/person_tracker/status` (`std_msgs/String`): 追跡状態情報
  - フォーマット: `"target_angle:18.1,position:2250"`

### 購読トピック

#### 人物追従モード
- `/detected_persons` (`vision_msgs/Detection2DArray`): YOLO人物検出結果
  - 人物のバウンディングボックス情報を受信
  - 複数人検出時は画像中央に最も近い人物を追跡対象とする

## トラブルシューティング

### よくある問題と解決方法

1. **ポートが開けない**
   ```
   [ERROR] Failed to open the port
   ```
   - U2D2の接続を確認
   - デバイス名が正しいか確認 (`ls /dev/tty*`)
   - 権限を確認 (`sudo chmod 666 /dev/ttyUSB0`)

2. **モーターが応答しない**
   ```
   [ERROR] Failed to enable torque
   ```
   - モーターの電源を確認
   - モーターIDが正しいか確認 (デフォルト: 1)
   - ボーレートが正しいか確認 (デフォルト: 57600)

3. **キーが反応しない（キーボードモード）**
   - ターミナルにフォーカスがあることを確認
   - `Ctrl+C` で一度終了し、再起動

4. **人物追従が動作しない**
   ```
   [ERROR] No detections received
   ```
   - カメラノードが起動していることを確認: `rostopic list | grep camera`
   - person_detectorノードが起動していることを確認: `rostopic hz /detected_persons`
   - カメラの視野角設定を確認: `horizontal_fov_deg`パラメータ

5. **追跡が不安定**
   - カメラの解像度設定を確認: `image_width`, `image_height`パラメータ
   - 検出タイムアウトを調整: `no_detection_timeout_sec`パラメータ
   - 制御周波数を調整: `control_rate_hz`パラメータ

### デバッグ方法

#### 基本的なデバッグ
```bash
# ログの詳細表示
roslaunch dynamixel_controller controller.launch --screen

# モーター接続テスト
python3 -c "
from dynamixel_sdk import *
port = PortHandler('/dev/ttyUSB0')
packet = PacketHandler(2.0)
print('Port opened:', port.openPort())
print('Baudrate set:', port.setBaudRate(57600))
port.closePort()
"
```

#### 人物追従モードのデバッグ
```bash
# 検出結果の確認
rostopic echo -n 5 /detected_persons

# 追跡状態の確認
rostopic echo /person_tracker/status

# カメラ画像の確認
rosrun image_view image_view image:=/usb_cam/image_raw
```

## ファイル構成

```
dynamixel_controller/
├── CMakeLists.txt           # ビルド設定
├── package.xml              # パッケージ依存関係
├── README.md               # このファイル
├── config/
│   └── dynamixel_params.yaml  # 詳細設定パラメータ
├── launch/
│   └── controller.launch    # 起動設定ファイル（モード切り替え対応）
└── src/
    ├── keyboard_teleop.py   # キーボード制御プログラム
    └── person_tracker.py    # 人物追従プログラム
```

## 技術仕様

### モーター仕様
- **モデル**: DYNAMIXEL XL330-M288-T
- **制御方式**: 位置制御 (Position Control Mode)
- **分解能**: 4096 positions/revolution (0.088°/step)
- **動作範囲**: 0° ~ 360° (制限設定で -180° ~ +180°)
- **通信**: TTL Half Duplex UART

### 制御仕様
- **制御周期**: 10Hz (人物追従モード時)
- **角度精度**: 約0.1度
- **移動ステップ**: 10度 (キーボードモード、設定変更可能)
- **追跡精度**: リアルタイム座標→角度変換
- **検出頻度**: 最大7Hz (YOLO検出性能に依存)
- **最大速度**: 設定可能 (デフォルト: 50)

### 人物追従仕様
- **検出対象**: 人物（YOLO v5使用）
- **追跡方式**: 画像中央に最も近い人物を優先
- **視野角対応**: カメラFOVに基づく正確な角度変換
- **未検出時動作**: 3パターン（中央復帰/停止/探索）
- **探索範囲**: ±60度（設定変更可能）
- **探索速度**: 30度/秒（設定変更可能）

## 将来の拡張

このパッケージは以下の機能拡張を想定して設計されています:

1. ✅ **YOLO人検出連携**: 人物検出結果に基づく自動追跡 *(実装済み)*
2. **特定人物追跡**: 顔認識による個人識別と追跡
3. **軌道計画**: より滑らかな動作のための軌道生成
4. **複数モーター制御**: パン・チルト機構への対応
5. **音声認識連携**: 音声コマンドでの制御
6. **Web インターフェース**: ブラウザからの遠隔操作
7. **学習機能**: 追跡パターンの学習と最適化

## ライセンス

MIT License

## 作者

Kohei

## 更新履歴

- **v2.0.0** (2024-12): 人物追従機能追加
  - YOLO連携による自動人物追跡
  - 3つの未検出時動作パターン（中央復帰/停止/探索）
  - モード切り替え機能（キーボード/追従）
  - 詳細なパラメータ設定
  - 包括的なデバッグ機能

- **v1.0.0** (2024-09): 初回リリース
  - キーボード制御機能
  - 安全制限機能
  - ROS統合
