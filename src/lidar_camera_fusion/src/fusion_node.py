#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
import tf2_ros
import struct

class LidarCameraFusionNode:
    def __init__(self):
        # ROSノードの初期化
        rospy.init_node('lidar_camera_fusion_node', anonymous=True)
        
        # TFリスナーの初期化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        
        # サブスクライバーの設定
        self.camera_sub = rospy.Subscriber('/detected_persons', Detection2DArray, self.camera_callback)
        self.lidar_sub = rospy.Subscriber('/mock_lidar/person_points', PointCloud2, self.lidar_callback)
        
        # パブリッシャーの設定
        self.fusion_pub = rospy.Publisher('/fused_persons/person_pose', PoseStamped, queue_size=10)
        
        # カメラ検出結果の保持
        self.latest_detections = []
        self.detection_timestamp = None
        
        # カメラ内部パラメータ（仮の値）
        self.camera_params = {
            'fx': 525.0,  # 焦点距離 X
            'fy': 525.0,  # 焦点距離 Y
            'cx': 319.5,  # 光学中心 X
            'cy': 239.5,  # 光学中心 Y
            'width': 640,  # 画像幅
            'height': 480  # 画像高さ
        }
        
        rospy.loginfo("LiDAR-Camera Fusion Node started")
        rospy.loginfo("Subscribing to /detected_persons and /mock_lidar/person_points")
        rospy.loginfo("Publishing fused results to /fused_persons/person_pose")
    
    def camera_callback(self, msg):
        """カメラ検出結果のコールバック"""
        self.latest_detections = msg.detections
        self.detection_timestamp = msg.header.stamp
        rospy.logdebug("Received %d camera detections", len(self.latest_detections))
    
    def extract_points_from_pointcloud(self, pointcloud_msg):
        """PointCloud2メッセージから点群データを抽出"""
        points = []
        
        # フィールド情報の取得
        x_offset = None
        y_offset = None
        z_offset = None
        
        for field in pointcloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if x_offset is None or y_offset is None or z_offset is None:
            rospy.logwarn("Required fields (x, y, z) not found in pointcloud")
            return []
        
        # バイトデータから点を抽出
        for i in range(pointcloud_msg.width):
            point_data = pointcloud_msg.data[i * pointcloud_msg.point_step:(i + 1) * pointcloud_msg.point_step]
            
            x = struct.unpack('f', point_data[x_offset:x_offset + 4])[0]
            y = struct.unpack('f', point_data[y_offset:y_offset + 4])[0]
            z = struct.unpack('f', point_data[z_offset:z_offset + 4])[0]
            
            points.append([x, y, z])
        
        return np.array(points)
    
    def transform_points_to_camera_frame(self, points, target_frame='camera_link', source_frame='lidar_link'):
        """LiDAR座標系の点群をカメラ座標系に変換"""
        try:
            # 最新の変換を取得
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            
            transformed_points = []
            for point in points:
                # geometry_msgs/PointStampedを作成
                point_stamped = PointStamped()
                point_stamped.header.frame_id = source_frame
                point_stamped.header.stamp = rospy.Time.now()
                point_stamped.point.x = point[0]
                point_stamped.point.y = point[1]
                point_stamped.point.z = point[2]
                
                # 座標変換
                transformed_point = do_transform_point(point_stamped, transform)
                transformed_points.append([
                    transformed_point.point.x,
                    transformed_point.point.y,
                    transformed_point.point.z
                ])
            
            return np.array(transformed_points)
            
        except (tf2_ros.TransformException, tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn("TF transform failed: %s", str(e))
            return np.array([])
    
    def project_3d_to_2d(self, points_3d):
        """3D点を2D画像座標に投影"""
        projected_points = []
        
        for point in points_3d:
            x, y, z = point
            
            # Z座標が正でない場合は投影しない
            if z <= 0:
                continue
            
            # カメラ投影（ピンホールカメラモデル）
            u = (self.camera_params['fx'] * x / z) + self.camera_params['cx']
            v = (self.camera_params['fy'] * y / z) + self.camera_params['cy']
            
            # 画像範囲内かチェック
            if 0 <= u < self.camera_params['width'] and 0 <= v < self.camera_params['height']:
                projected_points.append([u, v, x, y, z])  # 2D座標と元の3D座標を保持
        
        return np.array(projected_points)
    
    def is_point_in_bounding_box(self, point_2d, bbox):
        """点がバウンディングボックス内にあるか判定"""
        u, v = point_2d[0], point_2d[1]
        
        # バウンディングボックスの境界
        x_min = bbox.center.x - bbox.size_x / 2
        x_max = bbox.center.x + bbox.size_x / 2
        y_min = bbox.center.y - bbox.size_y / 2
        y_max = bbox.center.y + bbox.size_y / 2
        
        return x_min <= u <= x_max and y_min <= v <= y_max
    
    def find_fusion_candidates(self, projected_points):
        """投影された点とカメラ検出結果のマッチング"""
        fusion_candidates = []
        
        if not self.latest_detections:
            return fusion_candidates
        
        for detection in self.latest_detections:
            bbox = detection.bbox
            matched_points = []
            
            for point in projected_points:
                if self.is_point_in_bounding_box(point[:2], bbox):
                    matched_points.append(point)
            
            if matched_points:
                # マッチした点の平均3D座標を計算
                matched_points = np.array(matched_points)
                avg_3d_pos = np.mean(matched_points[:, 2:5], axis=0)  # x, y, zの平均
                
                fusion_candidates.append({
                    'detection': detection,
                    'matched_points': matched_points,
                    'avg_3d_position': avg_3d_pos,
                    'num_matched_points': len(matched_points)
                })
                
                rospy.loginfo("Fusion success: %d points matched with detection ID %s", 
                            len(matched_points), detection.id)
        
        return fusion_candidates
    
    def publish_fusion_result(self, fusion_candidates):
        """フュージョン結果をパブリッシュ"""
        for candidate in fusion_candidates:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "camera_link"
            
            # 3D位置を設定
            pose_msg.pose.position.x = candidate['avg_3d_position'][0]
            pose_msg.pose.position.y = candidate['avg_3d_position'][1]
            pose_msg.pose.position.z = candidate['avg_3d_position'][2]
            
            # 向きは単位クォータニオンで設定（デフォルト）
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            
            self.fusion_pub.publish(pose_msg)
            
            rospy.loginfo("Published fused person pose: x=%.2f, y=%.2f, z=%.2f", 
                         pose_msg.pose.position.x, 
                         pose_msg.pose.position.y, 
                         pose_msg.pose.position.z)
    
    def lidar_callback(self, msg):
        """LiDAR点群のコールバック"""
        try:
            # 点群データを抽出
            points_3d = self.extract_points_from_pointcloud(msg)
            
            if len(points_3d) == 0:
                rospy.logwarn("No points extracted from pointcloud")
                return
            
            rospy.logdebug("Extracted %d points from LiDAR data", len(points_3d))
            
            # LiDAR座標系からカメラ座標系に変換
            transformed_points = self.transform_points_to_camera_frame(points_3d)
            
            if len(transformed_points) == 0:
                rospy.logwarn("No points transformed to camera frame")
                return
            
            # 3D点を2D画像座標に投影
            projected_points = self.project_3d_to_2d(transformed_points)
            
            if len(projected_points) == 0:
                rospy.logdebug("No points projected to image plane")
                return
            
            rospy.logdebug("Projected %d points to image plane", len(projected_points))
            
            # カメラ検出結果とのマッチング
            fusion_candidates = self.find_fusion_candidates(projected_points)
            
            # フュージョン結果をパブリッシュ
            if fusion_candidates:
                self.publish_fusion_result(fusion_candidates)
            else:
                rospy.logdebug("No fusion candidates found")
                
        except Exception as e:
            rospy.logerr("Error in LiDAR callback: %s", str(e))

if __name__ == '__main__':
    try:
        # 必要なインポートを追加
        from geometry_msgs.msg import PointStamped
        
        fusion_node = LidarCameraFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("LiDAR-Camera Fusion Node stopped")
    except Exception as e:
        rospy.logerr("Unexpected error: %s", str(e)) 