#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint

class WaypointScript(Node):
    def __init__(self):
        super().__init__('waypoint_script')
        
        self.current_lat = -35.363261
        self.current_lon = 149.165230
        
        wp = [-35.363500, 149.165500]
        
        self.waypoint_client = self.create_client(WaypointPush, '/mavros/mission/push')
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for waypoint service...')
        
        self.create_and_push_waypoints(wp)
    
    def create_and_push_waypoints(self, wp):
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
        print("Waypoint 1 created - Takeoff")

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
        wp2.z_alt = 5.0
        print("Waypoint 2 created - Move to target")

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
        print("Waypoint 3 created - Change altitude")

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
        print("Waypoint 4 created - RTL")

        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = [wp1,wp2,wp3,wp4]
        future = self.waypoint_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                print(f"Successfully pushed {response.wp_transfered} waypoints!")
            else:
                print("Failed to push waypoints")
        else:
            print("Service call failed")

def main(args=None):
    rclpy.init(args=args)
    
    waypoint_script = WaypointScript()
    
    rclpy.spin_once(waypoint_script, timeout_sec=1.0)
    waypoint_script.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()