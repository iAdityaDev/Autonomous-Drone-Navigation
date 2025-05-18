import rclpy
import time
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint, WaypointReached
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from collections import deque


def set_mode(self, mode):
    mode_client = self.create_client(SetMode, '/mavros/set_mode')
    while not mode_client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('Waiting for /mavros/set_mode service...')
    if mode == 'GUIDED':
        req = SetMode.Request()
        req.custom_mode = mode
        future = mode_client.call_async(req)
        # rclpy.spin_until_future_complete(self, future)

        future.add_done_callback(self.arm_drone)
        
        # if future.result() and future.result().mode_sent:
        #     self.get_logger().info(f'Mode set to {mode}')
        # else:
        #     self.get_logger().error(f'Failed to set mode to {mode}')
    if mode == 'AUTO':
        req = SetMode.Request()
        req.custom_mode = mode
        future = mode_client.call_async(req)
        future.add_done_callback(self.auto_switch)

def auto_switch(self, future):
    if future.result():
        print("switch to auto successful")      
    else:
        print("AUTO FAIL")  

def arm_drone(self, future):
    
    if future.result() and future.result().mode_sent:
        self.get_logger().info('mode call successful')
    else:
        self.get_logger().error('Failed to set mode')
        
#arming after modeset

    arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
    while not arm_client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('Waiting for arming service...')
    req = CommandBool.Request()
    req.value = True
    future = arm_client.call_async(req)
    future.add_done_callback(self.takeoff_drone)
    # rclpy.spin_until_future_complete(self, future)
    # if future.result() and future.result().success:
    #     self.get_logger().info('Drone armed successfully')
    # else:
    #     self.get_logger().error('Failed to arm drone')

def takeoff_drone(self, future):
    
    if future.result() and future.result().mode_sent:
        self.get_logger().info('arming call successful')
    else:
        self.get_logger().error(f'Failed to arm')
        
#take off after arming
    
    takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
    while not takeoff_client.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('Waiting for takeoff service...')
    req = CommandTOL.Request()
    req.altitude = 12.2
    req.latitude = 0.0
    req.longitude = 0.0
    req.min_pitch = 0.0
    req.yaw = 0.0
    future = takeoff_client.call_async(req)
    # rclpy.spin_until_future_complete(self, future)
    
    future.add_done_callback(self.takeoff_succ)
    
    # if future.result() and future.result().success:
    #     self.get_logger().info('Takeoff command successful')
    # else:
    #     self.get_logger().error('Takeoff command failed')
        
def takeoff_succ(self, future): 
    
    if future.result() and future.result().success:
        self.get_logger().info('Takeoff command successful')
    else:
        self.get_logger().error('Takeoff command failed')
