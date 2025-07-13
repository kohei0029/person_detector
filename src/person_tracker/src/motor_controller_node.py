#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人物追跡・DYNAMIXEL制御ノード
control_mode: tracking/keyboard
"""
import rospy
from geometry_msgs.msg import Point
from person_detector.msg import DetectedPersonArray
from dynamixel_workbench_msgs.srv import DynamixelCommand
import threading

# キーボード制御用
try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False

class MotorControllerNode:
    def __init__(self):
        # ROSパラメータから制御モードを取得
        self.control_mode = rospy.get_param('~control_mode', 'tracking')
        rospy.loginfo(f"control_mode: {self.control_mode}")

        # DYNAMIXELサービスクライアントの準備
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
        self.dxl_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        self.motor_id = 1  # モーターID
        self.goal_velocity = 0
        self.lock = threading.Lock()

        # 画像サイズ（必要に応じてパラメータ化）
        self.img_width = int(rospy.get_param('~image_width', 640))
        self.img_center_x = self.img_width // 2
        self.Kp = float(rospy.get_param('~Kp', 0.1))  # 比例ゲイン
        self.max_velocity = int(rospy.get_param('~max_velocity', 200))

        if self.control_mode == 'tracking':
            # 人物検出トピックを購読
            rospy.Subscriber('/detected_person_array', DetectedPersonArray, self.person_callback)
        elif self.control_mode == 'keyboard':
            if not PYNPUT_AVAILABLE:
                rospy.logerr('pynputがインストールされていません。\n$ pip install pynput でインストールしてください。')
                exit(1)
            # キーボード監視スレッドを開始
            self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
            self.keyboard_thread.daemon = True
            self.keyboard_thread.start()
        else:
            rospy.logerr('control_modeはtrackingまたはkeyboardを指定してください')
            exit(1)

    def person_callback(self, msg):
        # 検出人物がいない場合は停止
        if not msg.detections:
            self.set_velocity(0)
            return
        # 面積が最大の人物を選択
        target = max(msg.detections, key=lambda p: p.area)
        center_x = target.center.x
        # 画像中心との誤差を計算
        error = center_x - self.img_center_x
        # P制御で目標速度を計算
        velocity = int(self.Kp * error)
        # 最大速度制限
        velocity = max(-self.max_velocity, min(self.max_velocity, velocity))
        rospy.loginfo(f"追跡: center_x={center_x}, error={error}, velocity={velocity}")
        self.set_velocity(velocity)

    def set_velocity(self, velocity):
        with self.lock:
            if velocity == self.goal_velocity:
                return  # 変更なし
            self.goal_velocity = velocity
        # DYNAMIXELにGoal_Velocityを書き込む
        try:
            resp = self.dxl_command('', self.motor_id, 'Goal_Velocity', 0, velocity)
            if not resp.comm_result:
                rospy.logwarn(f"DYNAMIXEL書き込み失敗: {resp.error_msg}")
        except rospy.ServiceException as e:
            rospy.logerr(f"DYNAMIXELサービスコール失敗: {e}")

    def keyboard_listener(self):
        # キーボード入力を非同期で監視
        def on_press(key):
            try:
                if key == keyboard.Key.left:
                    self.set_velocity(50)
                elif key == keyboard.Key.right:
                    self.set_velocity(-50)
            except Exception as e:
                rospy.logerr(f"キー押下エラー: {e}")
        def on_release(key):
            try:
                if key in [keyboard.Key.left, keyboard.Key.right]:
                    self.set_velocity(0)
            except Exception as e:
                rospy.logerr(f"キー離しエラー: {e}")
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

if __name__ == '__main__':
    rospy.init_node('motor_controller_node')
    node = MotorControllerNode()
    rospy.loginfo('motor_controller_node 起動')
    rospy.spin() 