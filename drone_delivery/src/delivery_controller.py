#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint


class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10) 
        # self.points = [[-353636662, 1491658175, 587540], [-353624050, 1491651144, 587540], [-353641682, 1491645576, 587540]]
        self.points = {1 : [-353636662, 1491658175, 587540], 
                       2 : [-353624050, 1491651144, 587540], 
                       3 : [-353641682, 1491645576, 587540]}

        self.waypoints = []
        
        self.waypoint_client = self.create_client(WaypointPush, '/mavros/mission/push')
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = WaypointPush.Request()
    
    def ControlBegin(self):
        if len(self.points) != 0:
            if 1 in self.points:
                point_arr = self.points.get(1)
                wp1 = Waypoint()
                wp1.frame =  3 
                wp1.command= 22
                wp1.is_current = True
                wp1.autocontinue = True
                wp1.param1 = 10.0
                wp1.param2 = 0.0
                wp1.param3 = 0.0
                wp1.param4 = 0.0
                wp1.x_lat = point_arr[0]
                wp1.y_long = point_arr[1]
                wp1.z_alt = 20.0
                self.points.pop(1)
            
        else:
            pass
                
            
            

            
            
        self.req.start_index = 0
        self.req.waypoints = 0

        
    
    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

#

