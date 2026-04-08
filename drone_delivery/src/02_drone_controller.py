#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, UInt16
from geometry_msgs.msg import Twist
from mavros_msgs.msg import WaypointReached
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.srv import SetMode
import time 

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.srv import GetState, ChangeState
from rclpy.timer import Timer



class MyNode(LifecycleNode):
    def __init__(self):
        super().__init__('drone_controller_node')
        self.get_logger().info('drone_controller_node is created')
        
        
    def on_configure(self,state:State):
        self.get_logger().info('configuring the Node.....')
        
        return TransitionCallbackReturn.SUCCESS
    
    
        
    def on_activate(self, state:State):
        self.get_logger().info('Activating the node.........')
        
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10) 
        # self.waypoint_subs = self.create_subscription(WaypointReached, '/mavros/mission/reached', self.mission_callback, 10)
        # self.yaw_subscribe = self.create_subscription(Twist, '/align_vel', self.alignment_callback, 10)    
        
        
        self.setmode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.setmode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')
        
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming servie...')
        self.system()

        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff ssrv...')

        return TransitionCallbackReturn.SUCCESS

    
    
    def on_deactivate(self, state:State):
        self.get_logger().info('Deactivating the Node......')
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down...')
        
        return TransitionCallbackReturn.SUCCESS
     
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        
        return TransitionCallbackReturn.SUCCESS

                                                                    
    def mission_callback(self, msg):
        print('in mission callback')
        self.system()
        # if msg.wp_seq == self.final_point:
            # self.system()
        # else:
            # print("kya me yaha hu")
            # return
        
    def system(self):
        req = SetMode.Request()
        req.custom_mode = "GUIDED"
        future = self.setmode_client.call_async(req)
        future.add_done_callback(self.arm_rover)
        
    def arm_rover(self, future):

        if future.result() and future.result().mode_sent:
            self.get_logger().info('mode call successful')
        else:
            self.get_logger().error('Failed to set mode')
            
        req = CommandBool.Request()
        req.value = True
        future = self.arm_client.call_async(req)
        future.add_done_callback(self.takeoff_drone)
        

    
    def takeoff_drone(self, future):
        if future.result() and future.result().success:
            self.get_logger().info('arming call successful')
        else:
            self.get_logger().error(f'Failed to arm')

        req = CommandTOL.Request()
        req.altitude = 15.0
        req.latitude = 0.0
        req.longitude = 0.0
        future = self.takeoff_client.call_async(req)
        future.add_done_callback(self.auto_mode)

    def auto_mode(self,future):
        if future.result() and future.result().success:
            self.get_logger().info('takeoff call successful')
            time.sleep(15)
            req = SetMode.Request()
            req.custom_mode = "AUTO"
            future = self.setmode_client.call_async(req)
            # future.add_done_callback(self.arm_rover)
        else:
            self.get_logger().error(f'Failed to takeoff')


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try : 
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()