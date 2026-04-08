#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from mavros_msgs.srv import WaypointPush , WaypointPull
from geometry_msgs.msg import Twist
from mavros_msgs.msg import Waypoint, WaypointReached , WaypointList
from std_msgs.msg import Int32  
from lifecycle_msgs.msg import State , Transition
from lifecycle_msgs.srv import GetState, ChangeState
from collections import deque 
from std_msgs.msg import Bool

class lifecycle_srvclient(Node):
    def __init__(self):
        super().__init__('lifecycle_srvclient')
        self.get_logger().info('lifecycle_srvclient nodeis created')
    
        self.wp_pusher = self.create_client(WaypointPush,'/mavros/mission/push')
        while not self.wp_pusher.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/mission/push service...')

        
        self.create_subscription(WaypointReached,
                                '/mavros/mission/reached',
                                self.waypoint_reached_callback,
                                10)    

        self.wp_puller_topic = self.create_subscription(WaypointList,
                                                        '/mavros/mission/waypoints',
                                                        self.wp_to_deque,
                                                        10) 
        
        self.wp_puller_srv = self.create_client(WaypointPull,'/mavros/mission/pull')
        while not self.wp_puller_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/mission/pull service...')

        self.lifecycle_cli_control = self.create_client(ChangeState,'/drone_controller_node/change_state')
        while not self.lifecycle_cli_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for lifecycle/client/control service...')

        self.lifecycle_cli_align = self.create_client(ChangeState,'/image_display_node/change_state')
        while not self.lifecycle_cli_align.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for lifecycle/client/align service...')

        self.config_control_node()
        self.config_align_node()

        self.wp_dq = deque()
        self.wp_initialx_lat = 0.0   
        self.wp_initialy_long = 0.0   
        self.wp_initialz_alt = 0.0 
        self.iscontroller_active = False 
        self.isaligning_active = False 
        
    def config_control_node(self):

        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        self.get_logger().info('Configuing_control_nOde')
        config = self.lifecycle_cli_control.call_async(request)
        rclpy.spin_until_future_complete(self,config)

    def config_align_node(self):

        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        self.get_logger().info('Configuing_align_nOde')
        config = self.lifecycle_cli_align.call_async(request)
        rclpy.spin_until_future_complete(self,config)

    def activate_control_node(self):

        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        self.get_logger().info('activating_control_nOde')
        active = self.lifecycle_cli_control.call_async(request)
        active.add_done_callback(self.control_activation_callback)

    def control_activation_callback(self, future):
        print('in the control_activation_callback/*/*/*/')      

    def activate_align_node(self):

        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        self.get_logger().info('activating_align_nOde')
        active = self.lifecycle_cli_align.call_async(request)
        active.add_done_callback(self.align_activation_callback)

    def align_activation_callback(self, future):
        print('in the align_activation_callback/*/*/*/*/*')

    def deactivate_control_node(self):

        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        self.get_logger().info('deactivating_control_nOde')
        active = self.lifecycle_cli_control.call_async(request)
        active.add_done_callback(self.control_deactivation_callback)

    def control_deactivation_callback(self, future):
        print('in the control_deactivation_callback/*/*/*/') 

    def wp_to_deque(self,msg):
        Waypoint = msg.waypoints
        
        if (Waypoint[0].x_lat != self.wp_initialx_lat) or (Waypoint[0].y_long != self.wp_initialy_long) or (Waypoint[0].z_alt != self.wp_initialz_alt) :
            print(self.wp_initialy_long)
            self.wp_dq.append(Waypoint)
            self.wp_initialx_lat = Waypoint[0].x_lat 
            self.wp_initialy_long = Waypoint[0].y_long 
            self.wp_initialz_alt = Waypoint[0].z_alt
        print(len(self.wp_dq))
        print(self.wp_dq)


        if len(self.wp_dq) == 1:
            self.activate_control_node()
                  
    def waypoint_reached_callback(self, msg):
        print('here i am working very good*/*/*/*/*/*/*/*/*/*/5*/*/*/*/*/*/*/*/*/*/')
        self.activate_align_node()
            

def main(args=None):
    rclpy.init(args=args)
    node = lifecycle_srvclient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('keyboard interrupt')
    finally:
        rclpy.shutdown()
        Node.destroy_node()
    
if __name__ == '__main__':
    main()