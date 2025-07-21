#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
人物検出ROSノード（改修版）
YOLOv5を使用してWEBカメラの映像から人物を検出し、vision_msgs/Detection2DArrayをパブリッシュする
CPU負荷削減・10FPS制御・LiDAR統合対応版
"""

import rospy
import cv2
import time
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D, Pose, Point, Quaternion
from ultralytics import YOLO

class PersonDetectorNode:
    def __init__(self):
        """人物検出ノードの初期化"""
        rospy.init_node('person_detector_node', anonymous=True)
        
        # パラメータの取得（デフォルト値付き）
        self.input_topic = rospy.get_param('~input_topic', '/usb_cam/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/detected_persons')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.resize_factor = float(rospy.get_param('~resize_factor', 0.5))  # 画像サイズ縮小係数
        
        # YOLOv5モデルのロード（CPU版）
        rospy.loginfo("YOLOv5sモデルをロード中...")
        self.model = YOLO('yolov5s.pt')
        rospy.loginfo("YOLOv5sモデルのロード完了")
        
        # 人物クラスのインデックス（COCOデータセットでは0）
        self.person_class_id = 0
        
        # OpenCVブリッジの初期化
        self.bridge = CvBridge()
        
        # パブリッシャーとサブスクライバーの設定
        self.image_sub = rospy.Subscriber(self.input_topic, Image, self.image_callback, queue_size=1)
        self.detection_pub = rospy.Publisher(self.output_topic, Detection2DArray, queue_size=1)
        
        # 処理時間とFPSの計算用変数
        self.frame_count = 0
        self.start_time = time.time()
        
        # フレームレート制御用変数（10FPS = 0.1秒間隔）
        self.last_processed_time = 0
        
        rospy.loginfo("人物検出ノード（改修版）が開始されました")
        rospy.loginfo(f"入力トピック: {self.input_topic}")
        rospy.loginfo(f"出力トピック: {self.output_topic}")
        rospy.loginfo(f"信頼度閾値: {self.confidence_threshold}")
        rospy.loginfo(f"画像縮小係数: {self.resize_factor}")
        rospy.loginfo("フレームレート制御: 10FPS")
        rospy.loginfo("視覚的デバッグ表示: 無効")
    
    def image_callback(self, msg):
        """画像コールバック関数：人物検出とDetection2DArrayのパブリッシュ"""
        # フレームレート制御（10FPS）
        current_time = time.time()
        if current_time - self.last_processed_time < 0.1:  # 0.1秒未満の場合は処理をスキップ
            return
        
        # 処理開始時間を記録
        start_time = time.time()
        
        try:
            # ROSのImageメッセージをOpenCV画像に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 画像サイズを縮小して処理速度を向上
            original_height, original_width = cv_image.shape[:2]
            resize_factor = float(self.resize_factor)
            new_width = int(original_width * resize_factor)
            new_height = int(original_height * resize_factor)
            resized_image = cv2.resize(cv_image, (new_width, new_height))
            
            # YOLOv5モデルで推論を実行（縮小画像で処理）
            results = self.model(resized_image, device='cpu')
            
            # Detection2DArrayメッセージを作成
            detection_array = Detection2DArray()
            detection_array.header.stamp = rospy.Time.now()
            detection_array.header.frame_id = msg.header.frame_id
            detection_array.detections = []  # Initialize detections list
            
            # 人物クラスの検出結果のみを抽出
            person_detections = []
            for result in results:
                for box in result.boxes:
                    # 人物クラス（class_id=0）かつ信頼度が閾値以上の場合
                    if box.cls.item() == self.person_class_id and box.conf.item() > self.confidence_threshold:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        
                        # 座標を元の画像サイズに戻す
                        x1 = x1 / resize_factor
                        y1 = y1 / resize_factor
                        x2 = x2 / resize_factor
                        y2 = y2 / resize_factor
                        
                        person_detections.append({
                            'bbox': [x1, y1, x2, y2],
                            'confidence': box.conf.item()
                        })
            
            # 検出された人物をDetection2Dメッセージに変換
            for person in person_detections:
                x1, y1, x2, y2 = person['bbox']
                
                # バウンディングボックスの中心座標とサイズを計算
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
                width = x2 - x1
                height = y2 - y1
                
                # Detection2Dメッセージを作成
                detection = Detection2D()
                detection.header.stamp = rospy.Time.now()
                detection.header.frame_id = msg.header.frame_id
                detection.results = []  # Initialize results list
                
                # バウンディングボックス情報を設定
                detection.bbox.center.x = center_x
                detection.bbox.center.y = center_y
                detection.bbox.size_x = width
                detection.bbox.size_y = height
                
                # 検出結果情報を設定（ポーズ情報付き）
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = 0  # 人物クラスID
                hypothesis.score = person['confidence']
                
                # ポーズ情報を設定（人物の中心座標）
                hypothesis.pose.pose.position.x = center_x
                hypothesis.pose.pose.position.y = center_y
                hypothesis.pose.pose.position.z = 0.0
                hypothesis.pose.pose.orientation.x = 0.0
                hypothesis.pose.pose.orientation.y = 0.0
                hypothesis.pose.pose.orientation.z = 0.0
                hypothesis.pose.pose.orientation.w = 1.0
                
                detection.results.append(hypothesis)
                
                # 検出結果を配列に追加
                detection_array.detections.append(detection)
            
            # Detection2DArrayをパブリッシュ
            self.detection_pub.publish(detection_array)
            
            # 処理時間とFPSを計算
            end_time = time.time()
            processing_time_ms = (end_time - start_time) * 1000
            
            # FPSの計算（移動平均）
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # 30フレームごとにFPSを更新
                current_time = time.time()
                elapsed_time = current_time - self.start_time
                fps = self.frame_count / elapsed_time
                self.start_time = current_time
                self.frame_count = 0
            else:
                fps = 1.0 / (processing_time_ms / 1000.0) if processing_time_ms > 0 else 0.0
            
            # 処理時間とFPSをログ出力
            rospy.loginfo(f"Processing Time: {processing_time_ms:.2f} ms (FPS: {fps:.1f})")
            
            # 検出された人物数を表示
            if person_detections:
                rospy.loginfo(f"検出された人物数: {len(person_detections)}")
            
            # フレームレート制御用の時刻を更新
            self.last_processed_time = current_time
            
        except Exception as e:
            rospy.logerr(f"画像処理中にエラーが発生しました: {str(e)}")
    
    def run(self):
        """ノードの実行"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("人物検出ノードを終了します")

if __name__ == '__main__':
    try:
        # 人物検出ノードのインスタンスを作成
        detector = PersonDetectorNode()
        
        # ノードを実行
        detector.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("人物検出ノードが中断されました")
    except Exception as e:
        rospy.logerr(f"予期しないエラーが発生しました: {str(e)}")
