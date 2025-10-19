#!/usr/bin/env python3

"""
DYNAMIXEL XL330-M288-T Keyboard Teleop Node

This node controls a DYNAMIXEL XL330-M288-T motor using keyboard input.
Left/Right arrow keys rotate the motor within -180 to +180 degree range.

Author: Kohei
License: MIT
"""

import rospy
import sys
import termios
import tty
import math
import os
import time
from std_msgs.msg import String

# Import DYNAMIXEL SDK
try:
    from dynamixel_sdk import *  # Uses Dynamixel SDK library
except ImportError:
    rospy.logerr("Failed to import dynamixel_sdk. Please install it using:")
    rospy.logerr("sudo apt install ros-noetic-dynamixel-sdk")
    sys.exit(1)

# Control table address for XL330
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_VELOCITY = 112

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0' # ttyDXL'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095
DXL_MOVING_STATUS_THRESHOLD = 20

class DynamixelKeyboardTeleop:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('dynamixel_keyboard_teleop', anonymous=True)
        
        # Parameters
        self.motor_id = rospy.get_param('~motor_id', DXL_ID)
        self.device_name = rospy.get_param('~device_name', DEVICENAME)
        self.baudrate = rospy.get_param('~baudrate', BAUDRATE)
        self.center_position = rospy.get_param('~center_position', 2048)
        self.position_per_degree = rospy.get_param('~position_per_degree', 11.378)
        self.movement_step = rospy.get_param('~movement_step', 10)
        self.min_angle_deg = rospy.get_param('~min_angle_deg', -180)
        self.max_angle_deg = rospy.get_param('~max_angle_deg', 180)
        self.profile_velocity = rospy.get_param('~profile_velocity', 50)
        
        # Current angle state (in degrees, 0 = center/front)
        self.current_angle = 0.0
        
        # Initialize PortHandler instance
        self.portHandler = PortHandler(self.device_name)
        
        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        # Open port
        if not self.portHandler.openPort():
            rospy.logerr("Failed to open the port")
            sys.exit(1)
            
        # Set port baudrate
        if not self.portHandler.setBaudRate(self.baudrate):
            rospy.logerr("Failed to change the baudrate")
            sys.exit(1)
            
        rospy.loginfo("Succeeded to open the port and set baudrate")
        
        # Initialize motor
        self.initialize_motor()
        
        # Key mappings
        self.key_bindings = {
            '\x1b[C': 'right',  # Right arrow key
            '\x1b[D': 'left',   # Left arrow key
            'q': 'quit',
            'Q': 'quit',
            '\x03': 'quit',     # Ctrl+C
            'h': 'help',
            'H': 'help',
            'r': 'reset',       # Reset to center
            'R': 'reset'
        }
        
        rospy.loginfo("DYNAMIXEL Keyboard Teleop Node Started")
        rospy.loginfo("Use arrow keys to control motor:")
        rospy.loginfo("  ← Left Arrow:  Rotate left")
        rospy.loginfo("  → Right Arrow: Rotate right") 
        rospy.loginfo("  r: Reset to center position")
        rospy.loginfo("  h: Show help")
        rospy.loginfo("  q: Quit")
        rospy.loginfo("Current angle: {:.1f} degrees".format(self.current_angle))
        
        # Publisher for status messages
        self.status_pub = rospy.Publisher('/dynamixel_status', String, queue_size=10)
        
    def initialize_motor(self):
        """Initialize motor to center position (0 degrees)"""
        rospy.loginfo("Initializing motor...")
        
        # Set operating mode to position control
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.motor_id, ADDR_OPERATING_MODE, 3)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to set operating mode: {}".format(
                self.packetHandler.getTxRxResult(dxl_comm_result)))
            return False
        elif dxl_error != 0:
            rospy.logerr("Error setting operating mode: {}".format(
                self.packetHandler.getRxPacketError(dxl_error)))
            return False
            
        # Set Profile Velocity
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.motor_id, ADDR_PROFILE_VELOCITY, self.profile_velocity)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to set profile velocity: {}".format(
                self.packetHandler.getTxRxResult(dxl_comm_result)))
        elif dxl_error != 0:
            rospy.logerr("Error setting profile velocity: {}".format(
                self.packetHandler.getRxPacketError(dxl_error)))
        
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to enable torque: {}".format(
                self.packetHandler.getTxRxResult(dxl_comm_result)))
            return False
        elif dxl_error != 0:
            rospy.logerr("Error enabling torque: {}".format(
                self.packetHandler.getRxPacketError(dxl_error)))
            return False
            
        rospy.loginfo("Torque enabled")
        
        # Move to center position
        center_pos = self.angle_to_position(0.0)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.motor_id, ADDR_GOAL_POSITION, center_pos)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to set goal position: {}".format(
                self.packetHandler.getTxRxResult(dxl_comm_result)))
            return False
        elif dxl_error != 0:
            rospy.logerr("Error setting goal position: {}".format(
                self.packetHandler.getRxPacketError(dxl_error)))
            return False
            
        # Wait for movement to complete
        time.sleep(1.0)
        self.current_angle = 0.0
        rospy.loginfo("Motor initialized to center position (0 degrees)")
        return True
        
    def angle_to_position(self, angle_deg):
        """Convert angle in degrees to DYNAMIXEL position units"""
        # Clamp angle to valid range
        angle_deg = max(self.min_angle_deg, min(self.max_angle_deg, angle_deg))
        
        # Convert to position units
        # Center (0 deg) = 2048, positive angles = clockwise
        position = self.center_position + int(angle_deg * self.position_per_degree)
        
        # Ensure position is within valid range (0-4095 for XL330)
        position = max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, position))
        
        return position
        
    def position_to_angle(self, position):
        """Convert DYNAMIXEL position units to angle in degrees"""
        angle_deg = (position - self.center_position) / self.position_per_degree
        return angle_deg
        
    def get_present_position(self):
        """Get current motor position"""
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, self.motor_id, ADDR_PRESENT_POSITION)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to read present position: {}".format(
                self.packetHandler.getTxRxResult(dxl_comm_result)))
            return None
        elif dxl_error != 0:
            rospy.logerr("Error reading present position: {}".format(
                self.packetHandler.getRxPacketError(dxl_error)))
            return None
            
        return dxl_present_position
        
    def move_motor(self, direction):
        """Move motor in specified direction"""
        if direction == 'left':
            new_angle = self.current_angle - self.movement_step
            direction_str = "left"
        elif direction == 'right':
            new_angle = self.current_angle + self.movement_step
            direction_str = "right"
        else:
            return
            
        # Check angle limits
        if new_angle < self.min_angle_deg:
            rospy.logwarn("Cannot move further left. Limit: {} degrees".format(self.min_angle_deg))
            return
        elif new_angle > self.max_angle_deg:
            rospy.logwarn("Cannot move further right. Limit: {} degrees".format(self.max_angle_deg))
            return
            
        # Convert to position and send command
        target_position = self.angle_to_position(new_angle)
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.motor_id, ADDR_GOAL_POSITION, target_position)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to set goal position: {}".format(
                self.packetHandler.getTxRxResult(dxl_comm_result)))
            return
        elif dxl_error != 0:
            rospy.logerr("Error setting goal position: {}".format(
                self.packetHandler.getRxPacketError(dxl_error)))
            return
            
        self.current_angle = new_angle
        rospy.loginfo("Moving {}: {:.1f}° (position: {})".format(
            direction_str, self.current_angle, target_position))
        
        # Publish status
        status_msg = String()
        status_msg.data = "angle:{:.1f},direction:{}".format(self.current_angle, direction_str)
        self.status_pub.publish(status_msg)
            
    def reset_position(self):
        """Reset motor to center position"""
        rospy.loginfo("Resetting to center position...")
        target_position = self.angle_to_position(0.0)
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.motor_id, ADDR_GOAL_POSITION, target_position)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to reset position: {}".format(
                self.packetHandler.getTxRxResult(dxl_comm_result)))
            return
        elif dxl_error != 0:
            rospy.logerr("Error resetting position: {}".format(
                self.packetHandler.getRxPacketError(dxl_error)))
            return
            
        self.current_angle = 0.0
        rospy.loginfo("Reset to center: 0.0° (position: {})".format(target_position))
        
        # Publish status
        status_msg = String()
        status_msg.data = "angle:0.0,direction:reset"
        self.status_pub.publish(status_msg)
            
    def show_help(self):
        """Display help information"""
        rospy.loginfo("=== DYNAMIXEL Keyboard Teleop Help ===")
        rospy.loginfo("Controls:")
        rospy.loginfo("  ← Left Arrow:  Rotate left by {} degrees".format(self.movement_step))
        rospy.loginfo("  → Right Arrow: Rotate right by {} degrees".format(self.movement_step))
        rospy.loginfo("  r: Reset to center position (0°)")
        rospy.loginfo("  h: Show this help")
        rospy.loginfo("  q: Quit")
        rospy.loginfo("Current status:")
        rospy.loginfo("  Angle: {:.1f}°".format(self.current_angle))
        rospy.loginfo("  Range: {}° to {}°".format(self.min_angle_deg, self.max_angle_deg))
        rospy.loginfo("  Step size: {}°".format(self.movement_step))
        
    def get_key(self):
        """Get keyboard input (non-blocking)"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
            
            # Handle arrow keys (escape sequences)
            if key == '\x1b':  # ESC sequence
                key += sys.stdin.read(2)
                
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
        
    def cleanup(self):
        """Cleanup function to disable torque and close port"""
        rospy.loginfo("Disabling motor torque...")
        
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to disable torque: {}".format(
                self.packetHandler.getTxRxResult(dxl_comm_result)))
        elif dxl_error != 0:
            rospy.logerr("Error disabling torque: {}".format(
                self.packetHandler.getRxPacketError(dxl_error)))
        else:
            rospy.loginfo("Torque disabled")
            
        # Close port
        self.portHandler.closePort()
        rospy.loginfo("Port closed")
        
    def run(self):
        """Main control loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        try:
            while not rospy.is_shutdown():
                # Non-blocking key input
                try:
                    key = self.get_key()
                    
                    if key in self.key_bindings:
                        action = self.key_bindings[key]
                        
                        if action == 'quit':
                            rospy.loginfo("Quitting...")
                            break
                        elif action == 'left':
                            self.move_motor('left')
                        elif action == 'right':
                            self.move_motor('right')
                        elif action == 'reset':
                            self.reset_position()
                        elif action == 'help':
                            self.show_help()
                    else:
                        # Ignore unknown keys silently to avoid spam
                        pass
                        
                except Exception as e:
                    # Handle keyboard interrupt and other exceptions gracefully
                    if not rospy.is_shutdown():
                        rospy.logdebug("Key input exception: {}".format(e))
                        
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("Keyboard interrupt received")
        except Exception as e:
            rospy.logerr("Unexpected error: {}".format(e))
        finally:
            self.cleanup()
            rospy.loginfo("DYNAMIXEL Keyboard Teleop Node Stopped")

if __name__ == '__main__':
    try:
        teleop = DynamixelKeyboardTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Failed to start teleop node: {}".format(e))
        sys.exit(1)