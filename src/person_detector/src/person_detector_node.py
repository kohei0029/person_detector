#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
人物検出ROSノード
YOLOv5を使用してWEBカメラの映像から人物を検出し、中心座標をパブリッシュする
CPU版・処理時間表示付き
"""

import rospy
import cv2
import time
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ultralytics import YOLO

class PersonDetectorNode:
    def __init__(self):
        """人物検出ノードの初期化"""
        rospy.init_node('person_detector_node', anonymous=True)
        
        # パラメータの取得（デフォルト値付き）
        self.input_topic = rospy.get_param('~input_topic', '/usb_cam/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/detected_person/center_point')
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
        self.center_pub = rospy.Publisher(self.output_topic, Point, queue_size=1)
        
        # 処理時間とFPSの計算用変数
        self.frame_count = 0
        self.start_time = time.time()
        
        rospy.loginfo("人物検出ノードが開始されました")
        rospy.loginfo(f"入力トピック: {self.input_topic}")
        rospy.loginfo(f"出力トピック: {self.output_topic}")
        rospy.loginfo(f"信頼度閾値: {self.confidence_threshold}")
        rospy.loginfo(f"画像縮小係数: {self.resize_factor}")
    
    def image_callback(self, msg):
        """画像コールバック関数：人物検出と中心座標のパブリッシュ"""
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
            
            # 検出された人物の中心座標を計算してパブリッシュ
            for person in person_detections:
                x1, y1, x2, y2 = person['bbox']
                
                # バウンディングボックスの中心座標を計算
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
                
                # geometry_msgs/Pointメッセージを作成
                point_msg = Point()
                point_msg.x = center_x
                point_msg.y = center_y
                point_msg.z = 0.0  # 2D画像なのでz座標は0
                
                # 中心座標をパブリッシュ
                self.center_pub.publish(point_msg)
                
                # デバッグ用：バウンディングボックスを画像に描画
                cv2.rectangle(cv_image, 
                            (int(x1), int(y1)), 
                            (int(x2), int(y2)), 
                            (0, 255, 0), 2)
                
                # 中心点を描画
                cv2.circle(cv_image, 
                          (int(center_x), int(center_y)), 
                          5, (0, 0, 255), -1)
                
                # 信頼度を表示
                cv2.putText(cv_image, 
                           f"Person: {person['confidence']:.2f}", 
                           (int(x1), int(y1) - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, (0, 255, 0), 2)
            
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
            
            # デバッグ用：処理時間を画像に表示
            cv2.putText(cv_image, 
                       f"Time: {processing_time_ms:.1f}ms FPS: {fps:.1f}", 
                       (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (255, 255, 255), 2)
            
            # デバッグ用：画像を表示（オプション）
            cv2.imshow('Person Detection', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"画像処理中にエラーが発生しました: {str(e)}")
    
    def run(self):
        """ノードの実行"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("人物検出ノードを終了します")
        finally:
            cv2.destroyAllWindows()

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
