#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Person Tracker Node for DYNAMIXEL Controller

This node subscribes to person detection results from YOLO and automatically
controls the DYNAMIXEL motor to track the detected person closest to the image center.
"""

import rospy
import math
import time
from typing import Optional
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String

# Import DYNAMIXEL SDK
try:
    from dynamixel_sdk import *
except ImportError:
    rospy.logerr("Failed to import dynamixel_sdk. Please install it using:")
    rospy.logerr("sudo apt install ros-noetic-dynamixel-sdk")
    import sys
    sys.exit(1)

# Control table address for XL330
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_VELOCITY = 112

# Protocol version
PROTOCOL_VERSION = 2.0

class PersonTrackerNode:
    def __init__(self):
        """Initialize person tracker node"""
        rospy.init_node('person_tracker', anonymous=True)
        
        # Detection input parameters
        self.detections_topic = rospy.get_param('~detections_topic', '/detected_persons')
        
        # Camera parameters (obtained from Step 1 confirmation)
        self.image_width = int(rospy.get_param('~image_width', 640))
        self.image_height = int(rospy.get_param('~image_height', 480))
        self.horizontal_fov_deg = float(rospy.get_param('~horizontal_fov_deg', 70.0))
        
        # DYNAMIXEL parameters (consistent with keyboard_teleop.py)
        self.device_name = rospy.get_param('~device_name', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 57600)
        self.motor_id = rospy.get_param('~motor_id', 1)
        self.center_position = rospy.get_param('~center_position', 2048)
        self.position_per_degree = rospy.get_param('~position_per_degree', 11.378)
        self.min_angle_deg = rospy.get_param('~min_angle_deg', -180.0)
        self.max_angle_deg = rospy.get_param('~max_angle_deg', 180.0)
        self.profile_velocity = rospy.get_param('~profile_velocity', 50)
        
        # Tracking behavior parameters
        self.no_person_behavior = rospy.get_param('~no_person_behavior', 'return_to_center')
        self.no_detection_timeout_sec = rospy.get_param('~no_detection_timeout_sec', 1.0)
        self.search_speed_deg_per_sec = rospy.get_param('~search_speed_deg_per_sec', 30.0)
        self.search_amplitude_deg = rospy.get_param('~search_amplitude_deg', 60.0)
        self.control_rate_hz = rospy.get_param('~control_rate_hz', 10.0)
        
        # Internal state variables
        self.last_target_angle_deg = 0.0
        self.last_detection_time = None
        self.search_direction = 1  # 1: right, -1: left
        self.is_searching = False
        
        # Initialize DYNAMIXEL communication
        self.portHandler = PortHandler(self.device_name)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        # Open port
        if not self.portHandler.openPort():
            rospy.logerr(f"Failed to open port {self.device_name}")
            rospy.signal_shutdown("Port open failed")
            return
            
        # Set baudrate
        if not self.portHandler.setBaudRate(self.baudrate):
            rospy.logerr(f"Failed to set baudrate to {self.baudrate}")
            rospy.signal_shutdown("Baudrate set failed")
            return
            
        rospy.loginfo(f"Successfully opened port {self.device_name} at {self.baudrate} baud")
        
        # ROS publishers and subscribers (initialize before motor init)
        self.detection_sub = rospy.Subscriber(
            self.detections_topic, Detection2DArray, self.detection_callback, queue_size=1)
        self.status_pub = rospy.Publisher('/person_tracker/status', String, queue_size=10)
        
        # Initialize motor
        self.initialize_motor()
        
        # Control loop timer
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate_hz), self.control_loop)
        
        rospy.loginfo("Person Tracker Node Started")
        rospy.loginfo(f"Detection topic: {self.detections_topic}")
        rospy.loginfo(f"Image size: {self.image_width}x{self.image_height}")
        rospy.loginfo(f"Horizontal FOV: {self.horizontal_fov_deg}°")
        rospy.loginfo(f"No person behavior: {self.no_person_behavior}")
        
    def initialize_motor(self):
        """Initialize DYNAMIXEL motor settings"""
        rospy.loginfo("Initializing DYNAMIXEL motor...")
        
        # Set operating mode to position control
        self.write_control_table(ADDR_OPERATING_MODE, 3, 1)
        time.sleep(0.1)
        
        # Set profile velocity
        self.write_control_table(ADDR_PROFILE_VELOCITY, self.profile_velocity, 4)
        time.sleep(0.1)
        
        # Enable torque
        self.write_control_table(ADDR_TORQUE_ENABLE, 1, 1)
        time.sleep(0.1)
        
        # Move to center position (0 degrees)
        self.move_to_angle(0.0)
        time.sleep(1.0)
        
        rospy.loginfo("Motor initialized to center position (0 degrees)")
        
    def write_control_table(self, addr, value, size):
        """Write value to DYNAMIXEL control table"""
        if size == 1:
            result, error = self.packetHandler.write1ByteTxRx(
                self.portHandler, self.motor_id, addr, value)
        elif size == 4:
            result, error = self.packetHandler.write4ByteTxRx(
                self.portHandler, self.motor_id, addr, value)
        else:
            rospy.logerr(f"Unsupported write size: {size}")
            return False
            
        if result != COMM_SUCCESS:
            rospy.logerr(f"Write failed: {self.packetHandler.getTxRxResult(result)}")
            return False
        elif error != 0:
            rospy.logerr(f"Write error: {self.packetHandler.getRxPacketError(error)}")
            return False
            
        return True
        
    def angle_to_position(self, angle_deg):
        """Convert angle in degrees to DYNAMIXEL position units"""
        # Clamp angle to valid range
        angle_deg = max(self.min_angle_deg, min(self.max_angle_deg, angle_deg))
        
        # Convert to position units
        position = self.center_position + int(angle_deg * self.position_per_degree)
        
        # Ensure position is within valid range
        position = max(0, min(4095, position))
        
        return position
        
    def move_to_angle(self, angle_deg):
        """Move motor to specified angle"""
        target_position = self.angle_to_position(angle_deg)
        
        success = self.write_control_table(ADDR_GOAL_POSITION, target_position, 4)
        
        if success:
            self.last_target_angle_deg = angle_deg
            rospy.logdebug(f"Moving to {angle_deg:.1f}° (position: {target_position})")
            
            # Publish status
            status_msg = String()
            status_msg.data = f"target_angle:{angle_deg:.1f},position:{target_position}"
            self.status_pub.publish(status_msg)
        else:
            rospy.logerr(f"Failed to move to angle {angle_deg:.1f}°")
            
    def detection_callback(self, msg):
        """Process person detection results"""
        if not msg.detections:
            # No detections
            return
            
        # Find the person closest to the image center
        image_center_x = self.image_width / 2.0
        best_detection = None
        min_distance_to_center = float('inf')
        
        for detection in msg.detections:
            person_center_x = detection.bbox.center.x
            distance_to_center = abs(person_center_x - image_center_x)
            
            if distance_to_center < min_distance_to_center:
                min_distance_to_center = distance_to_center
                best_detection = detection
                
        if best_detection is None:
            return
            
        # Convert person center X to target angle
        person_x = best_detection.bbox.center.x
        
        # Normalize to [-1, 1] range (left to right)
        normalized_x = (person_x - image_center_x) / (self.image_width / 2.0)
        
        # Convert to angle using horizontal FOV
        target_angle_deg = - normalized_x * (self.horizontal_fov_deg / 2.0)
        
        # Move motor to track the person
        self.move_to_angle(target_angle_deg)
        
        # Update detection time
        self.last_detection_time = time.time()
        self.is_searching = False
        
        rospy.loginfo(f"Tracking person at x={person_x:.1f} -> angle={target_angle_deg:.1f}°")
        
    def control_loop(self, event):
        """Main control loop for handling no-person behaviors"""
        current_time = time.time()
        
        # Check if we have recent detections
        has_recent_detection = (
            self.last_detection_time is not None and 
            (current_time - self.last_detection_time) <= self.no_detection_timeout_sec
        )
        
        if has_recent_detection:
            return
            
        # No recent detection - execute behavior based on parameter
        if self.no_person_behavior == 'stay':
            # Stay at current position
            pass
            
        elif self.no_person_behavior == 'return_to_center':
            # Return to center position if not already there
            if abs(self.last_target_angle_deg) > 1.0:
                self.move_to_angle(0.0)
                rospy.loginfo("No person detected - returning to center")
                
        elif self.no_person_behavior == 'search':
            # Search by sweeping left and right
            if not self.is_searching:
                self.is_searching = True
                rospy.loginfo("No person detected - starting search mode")
                
            # Calculate next search position
            step_size = self.search_speed_deg_per_sec / self.control_rate_hz
            next_angle = self.last_target_angle_deg + (self.search_direction * step_size)
            
            # Check boundaries and reverse direction if needed
            if next_angle > self.search_amplitude_deg:
                next_angle = self.search_amplitude_deg
                self.search_direction = -1
            elif next_angle < -self.search_amplitude_deg:
                next_angle = -self.search_amplitude_deg
                self.search_direction = 1
                
            self.move_to_angle(next_angle)
            
        else:
            # Default behavior: return to center
            if abs(self.last_target_angle_deg) > 1.0:
                self.move_to_angle(0.0)
                
    def cleanup(self):
        """Cleanup function"""
        rospy.loginfo("Shutting down person tracker...")
        
        # Disable torque
        self.write_control_table(ADDR_TORQUE_ENABLE, 0, 1)
        
        # Close port
        self.portHandler.closePort()
        
        rospy.loginfo("Person tracker shutdown complete")
        
    def run(self):
        """Run the node"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

if __name__ == '__main__':
    try:
        tracker = PersonTrackerNode()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Person tracker failed: {str(e)}")
