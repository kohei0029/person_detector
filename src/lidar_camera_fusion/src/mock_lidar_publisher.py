#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

class MockLidarPublisher:
    def __init__(self):
        # ROSノードの初期化
        rospy.init_node('mock_lidar_publisher', anonymous=True)
        
        # パブリッシャーの設定
        self.publisher = rospy.Publisher('/mock_lidar/person_points', PointCloud2, queue_size=10)
        
        # パブリッシュ周期の設定（10Hz）
        self.rate = rospy.Rate(10)
        
        # 人物の位置（LiDAR座標系）
        self.person_center = np.array([2.0, 0.0, 0.8])  # x=2.0m, y=0.0m, z=0.8m
        
        # 人物のサイズ（半径）
        self.person_radius = 0.3  # 30cm半径
        
        # 点群の点の数
        self.num_points = 50
        
        rospy.loginfo("Mock LiDAR Publisher started")
        rospy.loginfo("Publishing person point cloud at: x=%.1f, y=%.1f, z=%.1f", 
                     self.person_center[0], self.person_center[1], self.person_center[2])
    
    def generate_person_pointcloud(self):
        """人物を模擬する点群データを生成"""
        points = []
        
        # 人物の中心周りにランダムな点を生成
        for _ in range(self.num_points):
            # 球面座標でランダムな位置を生成
            r = self.person_radius * np.sqrt(np.random.random())  # 半径方向（一様分布）
            theta = 2 * np.pi * np.random.random()  # 水平角度
            phi = np.pi * np.random.random()  # 垂直角度
            
            # 球面座標から直交座標に変換
            x_offset = r * np.sin(phi) * np.cos(theta)
            y_offset = r * np.sin(phi) * np.sin(theta)
            z_offset = r * np.cos(phi)
            
            # 人物の中心位置にオフセットを加算
            point = self.person_center + np.array([x_offset, y_offset, z_offset])
            
            # 点の強度（LiDARの反射強度を模擬）
            intensity = np.random.uniform(0.5, 1.0)
            
            points.append([point[0], point[1], point[2], intensity])
        
        return np.array(points)
    
    def create_pointcloud2_msg(self, points):
        """PointCloud2メッセージを作成"""
        # ヘッダーの設定
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "lidar_link"
        
        # 点群データの作成
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        
        # フィールドの設定（x, y, z, intensity）
        cloud_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)
        ]
        
        cloud_msg.point_step = 16  # 4 bytes * 4 fields
        cloud_msg.row_step = cloud_msg.point_step * len(points)
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.is_dense = True
        
        # バイトデータの作成
        cloud_msg.data = []
        for point in points:
            cloud_msg.data.extend(struct.pack('ffff', point[0], point[1], point[2], point[3]))
        
        return cloud_msg
    
    def run(self):
        """メインループ"""
        while not rospy.is_shutdown():
            try:
                # 人物の点群データを生成
                points = self.generate_person_pointcloud()
                
                # PointCloud2メッセージを作成
                cloud_msg = self.create_pointcloud2_msg(points)
                
                # パブリッシュ
                self.publisher.publish(cloud_msg)
                
                rospy.logdebug("Published point cloud with %d points", len(points))
                
                # 指定された周期でスリープ
                self.rate.sleep()
                
            except Exception as e:
                rospy.logerr("Error in mock lidar publisher: %s", str(e))

if __name__ == '__main__':
    try:
        publisher = MockLidarPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mock LiDAR Publisher stopped")
    except Exception as e:
        rospy.logerr("Unexpected error: %s", str(e)) 