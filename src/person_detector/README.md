# 人物検出ROSノード (Person Detector)

YOLOv5を使用してWEBカメラの映像から人物を検出し、中心座標をパブリッシュするROSノードです。

## 機能

- **人物検出**: YOLOv5sモデルを使用してリアルタイムで人物を検出
- **CPU推論**: GPUを使用せず、CPUのみで推論を実行
- **処理時間表示**: 1フレームあたりの処理時間(ms)とFPSをリアルタイムで表示
- **中心座標パブリッシュ**: 検出された人物のバウンディングボックス中心座標をROSトピックで配信

## 要件

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+
- PyTorch (CPU版)
- OpenCV
- Ultralytics YOLOv5

## インストール済みライブラリ

以下のライブラリが既にインストールされています：
- `torch` (CPU版)
- `torchvision` (CPU版)
- `opencv-python`
- `ultralytics`
- `cv_bridge`

## 🚀 実行手順マニュアル

### 前提条件
- USBカメラが接続されていること
- ROS Noeticがインストールされていること
- 必要なPythonライブラリがインストールされていること

### 手順1: ワークスペースの準備
```bash
# ワークスペースディレクトリに移動
cd ~/yolo_ws

# ROS環境をセットアップ
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 手順2: カメラノードの起動
```bash
# USBカメラノードを起動（バックグラウンド実行）
roslaunch usb_cam usb_cam-test.launch &
```

**期待される出力:**
```
[INFO] Starting 'head_camera' (/dev/video0) at 640x480 via mmap (yuyv) at 30 FPS
[INFO] Using transport "raw"
```

### 手順3: 人物検出ノードの起動
```bash
# 人物検出ノードを起動
roslaunch person_detector person_detector.launch
```

**期待される出力:**
```
[INFO] YOLOv5sモデルをロード中...
[INFO] YOLOv5sモデルのロード完了
[INFO] 人物検出ノードが開始されました
[INFO] 入力トピック: /usb_cam/image_raw
[INFO] 出力トピック: /detected_person/center_point
[INFO] 信頼度閾値: 0.5
[INFO] Processing Time: 104.72 ms (FPS: 9.5)
[INFO] 検出された人物数: 1
```

### 手順4: 動作確認

#### 4.1 トピックの確認
```bash
# 別のターミナルでトピック一覧を確認
rostopic list
```

**期待される出力:**
```
/usb_cam/camera_info
/usb_cam/image_raw
/detected_person/center_point
/rosout
/rosout_agg
```

#### 4.2 中心座標の確認
```bash
# 検出された人物の中心座標を確認
rostopic echo /detected_person/center_point
```

**期待される出力:**
```
x: 320.5
y: 240.2
z: 0.0
---
```

#### 4.3 処理時間とFPSの確認
ノードのログ出力で以下が確認できます：
- 処理時間: 約100-110ms
- FPS: 約9-10 FPS
- 検出された人物数

### 手順5: 視覚的確認
- 人物検出ノードを起動すると、OpenCVウィンドウが開きます
- 検出された人物は緑の矩形で囲まれます
- 中心点は赤い円で表示されます
- 画像上部に処理時間とFPSが表示されます

### 手順6: 終了
```bash
# Ctrl+Cでノードを終了
# または、別のターミナルで以下を実行
pkill -f "roslaunch person_detector"
pkill -f "roslaunch usb_cam"
```

## トラブルシューティング

### 問題1: カメラが認識されない
```bash
# カメラデバイスの確認
ls /dev/video*

# カメラの権限設定
sudo usermod -a -G video $USER
# ログアウト・ログイン後に再試行
```

### 問題2: YOLOv5モデルのダウンロードエラー
```bash
# 手動でモデルをダウンロード
cd ~/yolo_ws/src/yolov5
python3 -c "from ultralytics import YOLO; model = YOLO('yolov5s.pt')"
```

### 問題3: メモリ不足エラー
- 画像サイズを小さくする
- バッチサイズを1に設定する
- 他のアプリケーションを終了する

### 問題4: ノードが起動しない
```bash
# 依存関係の確認
rospack find person_detector

# パッケージのビルド
catkin_make

# 環境の再読み込み
source devel/setup.bash
```

## パフォーマンス指標

### 正常動作時の指標
- **処理時間**: 100-110ms
- **FPS**: 9-10 FPS
- **検出精度**: 人物クラスのみを検出
- **メモリ使用量**: 約2-3GB

### 最適化のヒント
- 画像サイズを小さくすると処理速度が向上
- 信頼度閾値を上げると誤検出が減少
- CPU性能が高いほどFPSが向上

## トピック詳細

### 入力トピック
- **`/usb_cam/image_raw`** (sensor_msgs/Image)
  - WEBカメラからの画像データ
  - フォーマット: BGR8
  - 解像度: 640x480

### 出力トピック
- **`/detected_person/center_point`** (geometry_msgs/Point)
  - 検出された人物の中心座標
  - `x`: 画像内のX座標（ピクセル）
  - `y`: 画像内のY座標（ピクセル）
  - `z`: 常に0（2D画像のため）

## パラメータ設定

launchファイルで以下のパラメータを変更可能：

```xml
<param name="input_topic" value="/usb_cam/image_raw" />
<param name="output_topic" value="/detected_person/center_point" />
<param name="confidence_threshold" value="0.5" />
<param name="resize_factor" value="0.5" />  <!-- 画像サイズ縮小係数 -->
```

## ラグ改善方法

### 現在のラグ状況
- **推定ラグ**: 0.5-1.0秒程度
- **原因**: 処理時間（100-110ms）+ バッファリング + 表示遅延

### 改善方法

#### 1. 画像サイズの縮小
```xml
<!-- launchファイルで設定 -->
<param name="resize_factor" value="0.3" />  <!-- 30%サイズに縮小 -->
```

**効果**:
- 処理時間: 100ms → 40-50ms
- FPS: 9-10 → 15-20
- ラグ: 0.5-1.0秒 → 0.2-0.3秒

#### 2. 信頼度閾値の調整
```xml
<param name="confidence_threshold" value="0.7" />  <!-- より厳密な検出 -->
```

#### 3. カメラフレームレートの調整
```bash
# カメラのフレームレートを下げる
roslaunch usb_cam usb_cam-test.launch video_device:=/dev/video0 image_width:=320 image_height:=240
```

#### 4. キューの最適化
- 入力キュー: `queue_size=1`（最新フレームのみ処理）
- 出力キュー: `queue_size=1`（遅延を最小化）

### 推奨設定（低ラグ版）
```xml
<param name="resize_factor" value="0.3" />
<param name="confidence_threshold" value="0.6" />
```

**期待される改善**:
- 処理時間: 40-50ms
- FPS: 15-20
- ラグ: 0.2-0.3秒

## ライセンス

このプロジェクトはMITライセンスの下で公開されています。 

## 📊 パフォーマンス監視方法

### 方法1: htop（推奨）
```bash
# 新しいターミナルで実行
htop
```
**操作**:
- `F6`: ソート項目選択（CPU%、MEM%等）
- `F5`: ツリービュー表示
- `q`: 終了

### 方法2: カスタム監視スクリプト
```bash
# 新しいターミナルで実行
./monitor_performance.sh
```

### 方法3: コマンドライン監視
```bash
# CPU使用率（1秒間隔）
watch -n 1 'top -bn1 | grep "Cpu(s)"'

# メモリ使用率（1秒間隔）
watch -n 1 'free -h'

# 特定プロセスの監視
watch -n 1 'ps aux | grep person_detector_node'
```

### 方法4: システムモニター（GUI）
```bash
gnome-system-monitor
```

## 🎯 監視のポイント

### 正常動作時の指標
- **CPU使用率**: 50-80%（YOLOv5推論時）
- **メモリ使用量**: 2-3GB
- **プロセス数**: 1つのPythonプロセス

### 監視すべき項目
1. **全体のCPU使用率**
2. **人物検出ノードのCPU使用率**
3. **メモリ使用量**
4. **プロセス数**

### 実行手順
1. **ターミナル1**: カメラノード起動
2. **ターミナル2**: 人物検出ノード起動
3. **ターミナル3**: `htop`で監視
4. **ターミナル4**: `./monitor_performance.sh`で詳細監視

これで実行中のパフォーマンスを詳細に監視できます！ 