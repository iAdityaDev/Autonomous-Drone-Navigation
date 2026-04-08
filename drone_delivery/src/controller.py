#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint, WaypointReached
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from collections import deque
from sensor_msgs.msg import NavSatFix



class WaypointPusher(Node):
    def __init__(self):
        super().__init__('waypoint_pusher_node')
        self.takeoff_done = 0.0
        self.current_lat = 0.0
        self.current_lon = 0.0
        
        # Waypoint push client
        self.cli = self.create_client(WaypointPush, '/mavros/mission/push')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/mission/push service...')
        
        # Send waypoints first
        self.queue = deque([(-35.3634, 149.1654), (-35.3634, 149.1654), (-35.3634, 149.1654), (-35.3634, 149.1654)])
        self.send_waypoints()

        # Subscribe to mission/reached to detect waypoint completions
        self.create_subscription(WaypointReached, '/mavros/mission/reached', self.waypoint_reached_callback, 10)
    
    def setting(self):
        self.set_mode('GUIDED')
        # self.arm_drone()
        # self.takeoff_drone()
        time.sleep(15)
        print("auto switch ki valoo", self.takeoff_done)
            

    def gps_callback(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def send_waypoints(self):
        # self.setting()
        request = WaypointPush.Request()
        request.start_index = 0
        print("orig queue", self.queue)
        wp = self.queue.popleft()
        print("popped queue", self.queue)
        print("attempting to push waypoints", wp)

        # Waypoint 1 - Takeoff
        wp1 = Waypoint()
        wp1.frame = 3
        wp1.command = 22  # MAV_CMD_NAV_TAKEOFF
        wp1.is_current = True
        wp1.autocontinue = True
        wp1.param1 = 0.0
        wp1.param2 = 0.0
        wp1.param3 = 0.0
        wp1.param4 = 0.0
        wp1.x_lat = self.current_lat
        wp1.y_long = self.current_lon
        wp1.z_alt = 10.0

        print("idhr tk aagya1")
        # Waypoint 2 - Move to location
        wp2 = Waypoint()
        wp2.frame = 3
        wp2.command = 16  # NAV_WAYPOINT
        wp2.is_current = False
        wp2.autocontinue = True
        wp2.param1 = 0.0
        wp2.param2 = 0.0
        wp2.param3 = 0.0
        wp2.param4 = 0.0
        wp2.x_lat = wp[0]
        wp2.y_long = wp[1]
        wp2.z_alt = 12.2
        print("idhr tk aagya2")

        # Waypoint 3 - Change altitude
        wp3 = Waypoint()
        wp3.frame = 3
        wp3.command = 16
        wp3.is_current = False
        wp3.autocontinue = True
        wp3.param1 = 0.0
        wp3.param2 = 0.0
        wp3.param3 = 0.0
        wp3.param4 = 0.0
        wp3.x_lat = wp[0]
        wp3.y_long = wp[1]
        wp3.z_alt = 6.1
        print("idhr tk aagya3")
        
        # Waypoint 4 - RTL
        wp4 = Waypoint()
        wp4.frame = 3
        wp4.command = 20  # MAV_CMD_NAV_RETURN_TO_LAUNCH
        wp4.is_current = False
        wp4.autocontinue = True
        wp4.param1 = 0.0
        wp4.param2 = 0.0
        wp4.param3 = 0.0
        wp4.param4 = 0.0
        wp4.x_lat = 0.0
        wp4.y_long = 0.0
        wp4.z_alt = 0.0
        print("idhr tk aagya4")
        
        request.waypoints = [wp1, wp2, wp3, wp4]
        print("idhr tk aagya5")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/mission/push service...')
        
        future = self.cli.call_async(request)
        print("idhr tk aagya6")


        future.add_done_callback(self.handle_cli_response)

        print("attempting to send")

        print("setting bithare")
        self.setting()

    def handle_cli_response(self, future):
        result = future.result()
        if result and result.success:
            self.get_logger().info("Service call succeeded")
            self.get_logger().info(f'Waypoint push result: success={future.result().success}, wp_received={future.result().wp_transfered}')
            self.setting()
        else:
            self.get_logger().error("Service call failed")

    def waypoint_reached_callback(self, msg):
        print('here i am working very good*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/')
        # This will be called when any waypoint is reached
        reached_wp = msg.wp_seq
        self.get_logger().info(f'Waypoint {reached_wp} reached')

        # Check if we reached waypoint #4 (RTL)
        if reached_wp == 3:  # Waypoint #4 is at index 3 (since index starts from 0)
            self.get_logger().info('RTL completed, drone has returned to launch position')
            print("kya idhar aya?")
            self.send_waypoints()
 


    # def set_mode(self, mode):
    #     mode_client = self.create_client(SetMode, '/mavros/set_mode')
    #     while not mode_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('Waiting for /mavros/set_mode service...')
    #     req = SetMode.Request()
    #     req.custom_mode = mode
    #     future = mode_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     if future.result() and future.result().mode_sent:
    #         self.get_logger().info(f'Mode set to {mode}')
    #     else:
    #         self.get_logger().error(f'Failed to set mode to {mode}')

    # def arm_drone(self):
    #     arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
    #     while not arm_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('Waiting for arming service...')
    #     req = CommandBool.Request()
    #     req.value = True
    #     future = arm_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     if future.result() and future.result().success:
    #         self.get_logger().info('Drone armed successfully')
    #     else:
    #         self.get_logger().error('Failed to arm drone')

    # def takeoff_drone(self):
    #     takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
    #     while not takeoff_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('Waiting for takeoff service...')
    #     req = CommandTOL.Request()
    #     req.altitude = 12.2
    #     req.latitude = 0.0
    #     req.longitude = 0.0
    #     req.min_pitch = 0.0
    #     req.yaw = 0.0
    #     future = takeoff_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     if future.result() and future.result().success:
    #         self.get_logger().info('Takeoff command successful')
    #     else:
    #         self.get_logger().error('Takeoff command failed')


    def set_mode(self, mode):
        mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')
        if mode == 'GUIDED':
            req = SetMode.Request()
            req.custom_mode = mode
            future = mode_client.call_async(req)

            future.add_done_callback(self.arm_drone)
            

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


    def takeoff_drone(self, future):
        
        if future.result():
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

        
        future.add_done_callback(self.takeoff_succ)

            
    def takeoff_succ(self, future): 
        
        if future.result() and future.result().success:
            self.get_logger().info('Takeoff command successful')
            self.takeoff_done = 1
            time.sleep(5)
            self.set_mode('AUTO')
        else:
            self.get_logger().error('Takeoff command failed')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPusher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    
#to do

    #The setting chain runs twice righ now for some reason, first time without takeoff second time with takeoff. figure that out

    #the sequence for arming and takeoff is farzi, should not require to manually takeoff, 
    #to make the following sequence work
    #mode guided --> arm throttle --> mode auto --> and it should automatically take off

    #ek to eliminate time, line 266
    #setup some form of interrupt routine thatswitches to auto once it reaches the set altitude