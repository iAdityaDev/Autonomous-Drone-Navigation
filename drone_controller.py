#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, UInt16
from geometry_msgs.msg import Twist
from mavros_msgs.srv import SetMode, CommandBool



class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        print("shi me started")
        self.publisher_ = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        self.subscription = self.create_subscription(Int32, '/depth_from_detection', self.listener_callback, 10)
        self.point_sub = self.create_subscription(UInt16, '/mavros/mission/reached', self.mission_callback, 10)
        self.yaw_subscribe = self.create_subscription(Twist, '/align_vel', self.alignment_callback, 10)
        
        self.final_points = 1
        self.Kp = 0.1
        self.Kd = 0.1
        self.depth_val = 0        
        self.velocity_command = Twist()
        print("started")
        
        
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')
        print("idhr ni ara?")

    def alignment_callback(self, msg):

        self.velocity_command.linear.x = msg.linear.x
        self.velocity_command.linear.y = msg.linear.y
        

    def mission_callback(self, msg):
        if msg.data == self.final_point:
            self.system()
        else:
            print("kya me yaha hu")
            return

    def listener_callback(self, msg):
    # implement logic on when to activate this script

        self.previous_depth_val = self.depth_val
        self.depth_val = msg.data  # msg is Int32, so access .data


    def system(self):
        req = SetMode.Request()
        req.custom_mode = "GUIDED"
        future = self.mode_client.call_async(req)
        future.add_done_callback(self.arm_rover)
        
    def arm_rover(self, future):

        if future.result() and future.result().mode_sent:
            self.get_logger().info('mode call successful')
        else:
            self.get_logger().error('Failed to set mode')
            
        arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        req = CommandBool.Request()
        req.value = True
        future = arm_client.call_async(req)
        future.add_done_callback(self.start_follow_behaviour)
        

    
    def start_follow_behaviour(self, future):
        if future.result() and future.result().success:
            self.get_logger().info('arming call successful')
        else:
            self.get_logger().error(f'Failed to arm')

        self.p_error = self.depth_val * self.Kp 
        self.d_error = self.Kd*(self.depth_val - self.previous_depth_val)
        self.velocity_command.linear.x = 1.0 + self.p_error + self.d_error
        self.publisher_.publish(self.velocity_command)
    

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()