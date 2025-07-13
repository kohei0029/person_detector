#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
人物検出ROSノード
YOLOv5を使用してWEBカメラの映像から人物を検出し、中心座標と面積をパブリッシュする
CPU版・処理時間表示付き・低ラグ版
"""

import rospy
import cv2
import time
import numpy as np
import threading
from collections import deque
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from person_detector.msg import DetectedPerson, DetectedPersonArray
from ultralytics import YOLO

class PersonDetectorNode:
    def __init__(self):
        """人物検出ノードの初期化"""
        rospy.init_node('person_detector_node', anonymous=True)
        
        # パラメータの取得（デフォルト値付き）
        self.input_topic = rospy.get_param('~input_topic', '/usb_cam/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/person_detector/detections')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.resize_factor = float(rospy.get_param('~resize_factor', 0.5))
        self.enable_display = rospy.get_param('~enable_display', True)
        
        # YOLOv5モデルのロード
        rospy.loginfo("YOLOv5sモデルをロード中...")
        self.model = YOLO('yolov5s.pt')
        rospy.loginfo("YOLOv5sモデルのロード完了")
        
        self.person_class_id = 0
        self.bridge = CvBridge()
        
        # パブリッシャーとサブスクライバーの設定
        self.image_sub = rospy.Subscriber(self.input_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.detections_pub = rospy.Publisher(self.output_topic, DetectedPersonArray, queue_size=1)
        
        # 処理時間とFPSの計算用変数
        self.frame_count = 0
        self.start_time = time.time()
        
        # 表示用の画像キュー（スレッドセーフ）
        self.display_queue = deque(maxlen=2)
        self.display_lock = threading.Lock()
        
        if self.enable_display:
            self.display_thread = threading.Thread(target=self.display_worker, daemon=True)
            self.display_thread.start()
        
        rospy.loginfo("人物検出ノード（拡張版）が開始されました")
        rospy.loginfo(f"出力トピック: {self.output_topic}")

    def display_worker(self):
        """表示専用スレッド"""
        while not rospy.is_shutdown():
            try:
                # キューから画像を取得
                with self.display_lock:
                    if self.display_queue:
                        display_image = self.display_queue.popleft()
                    else:
                        time.sleep(0.01)  # 10ms待機
                        continue
                
                # 画像を表示
                cv2.imshow('Person Detection', display_image)
                cv2.waitKey(1)
                
            except Exception as e:
                rospy.logerr(f"表示処理中にエラーが発生しました: {str(e)}")
                time.sleep(0.01)
    
    def image_callback(self, msg):
        """画像コールバック関数"""
        start_time = time.time()
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 画像リサイズ
            original_height, original_width = cv_image.shape[:2]
            resized_image = cv2.resize(cv_image, (int(original_width * self.resize_factor), int(original_height * self.resize_factor)))
            
            # 推論実行
            results = self.model(resized_image, device='cpu', verbose=False)
            
            detected_person_array = DetectedPersonArray()
            detected_person_array.header.stamp = rospy.Time.now()
            detected_person_array.header.frame_id = msg.header.frame_id

            person_detections_for_display = []

            for res in results:
                for box in res.boxes:
                    if box.cls.item() == self.person_class_id and box.conf.item() > self.confidence_threshold:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        
                        # 座標を元の画像サイズに復元
                        x1_orig = x1 / self.resize_factor
                        y1_orig = y1 / self.resize_factor
                        x2_orig = x2 / self.resize_factor
                        y2_orig = y2 / self.resize_factor
                        
                        # メッセージ作成
                        person_msg = DetectedPerson()
                        person_msg.center.x = (x1_orig + x2_orig) / 2
                        person_msg.center.y = (y1_orig + y2_orig) / 2
                        person_msg.center.z = 0.0
                        person_msg.area = (x2_orig - x1_orig) * (y2_orig - y1_orig)
                        
                        detected_person_array.detections.append(person_msg)
                        
                        # 表示用に保存
                        person_detections_for_display.append({
                            'bbox': [x1_orig, y1_orig, x2_orig, y2_orig],
                            'confidence': box.conf.item(),
                            'center': [person_msg.center.x, person_msg.center.y]
                        })

            # 検出結果をパブリッシュ
            self.detections_pub.publish(detected_person_array)
            
            # FPS計算
            processing_time_ms = (time.time() - start_time) * 1000
            fps = 1.0 / (processing_time_ms / 1000.0) if processing_time_ms > 0 else 0.0
            
            # 表示用画像の準備
            if self.enable_display:
                display_image = cv_image.copy()
                for p in person_detections_for_display:
                    x1, y1, x2, y2 = p['bbox']
                    cv2.rectangle(display_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.circle(display_image, (int(p['center'][0]), int(p['center'][1])), 5, (0, 0, 255), -1)
                
                cv2.putText(display_image, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                with self.display_lock:
                    self.display_queue.append(display_image)
                    
        except Exception as e:
            rospy.logerr(f"画像処理中にエラー: {str(e)}")
    
    def run(self):
        """ノードの実行"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("人物検出ノードを終了します")
        finally:
            if self.enable_display:
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
