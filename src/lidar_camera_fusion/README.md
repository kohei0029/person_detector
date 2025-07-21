# lidar_camera_fusion

LiDARとカメラのセンサーフュージョンに関するROSパッケージです。

## 概要

このパッケージは、LiDARデータとカメラ画像を統合して、より高精度な環境認識を実現するためのノードを提供します。

## 依存関係

- rospy
- tf2_ros
- sensor_msgs
- vision_msgs
- geometry_msgs

## 使用方法

### ビルド

```bash
cd ~/yolo_ws
catkin_make
```

### 実行

```bash
roslaunch lidar_camera_fusion lidar_camera_fusion.launch
```

## パッケージ構造

```
lidar_camera_fusion/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── lidar_camera_fusion.launch
└── src/
    └── (Pythonノードファイル)
```

## トピック

### 入力トピック
- `/lidar_points` (sensor_msgs/PointCloud2): LiDAR点群データ
- `/camera/image_raw` (sensor_msgs/Image): カメラ画像

### 出力トピック
- `/fused_data` (sensor_msgs/PointCloud2): 融合されたデータ

## パラメータ

- `camera_frame_id`: カメラのフレームID
- `lidar_frame_id`: LiDARのフレームID
- `fusion_rate`: 融合処理の実行頻度 (Hz)

## ライセンス

TODO

## メンテナー

kohei (kohei@todo.todo) 