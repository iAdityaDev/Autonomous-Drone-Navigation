#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import ParamSet 
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType
from mavros_msgs.msg import Waypoint


class Geofencing(Node):
    def __init__(self):
        super().__init__('drone_geofencing_Node')
        self.get_logger().info('geo NOde is creste')
        self.fence_pts = [
            # (-35.363262, 149.165500),   
            # (-35.363381, 149.165763),       
            # (-35.363619, 149.165763),   
            # (-35.363738, 149.165500),   
            # (-35.363619, 149.165237),   
            # (-35.363381, 149.165237),   
            # (-35.363262, 149.165500)    

            (-35.36398719, 149.16526129),   
            (-35.36360440, 149.16604363),       
            (-35.36290067, 149.16613846),   
            (-35.36253720, 149.16493413),   
            (-35.36213507, 149.16550785),   
            (-35.36185666, 149.16423240),   
            (-35.36295867, 149.16390998),
            (-35.36404906, 149.16429404),
            (-35.36398719, 149.16526129)
            
            # (-35.36445605, 149.165121112),   
            # (-35.36358454, 149.16642655),       
            # (-35.36224908, 149.16646713),   
            # (-35.36204559, 149.16518199),   
            # (-35.36138366, 149.16708941),   
            # (-35.36061693, 149.16466794),   
            # (-35.36149398, 149.16244262),
            # (-35.36291160, 149.16267936),
            # (-35.36529446, 149.16343015),
            # (-35.36445605, 149.165121112)  

        ]
        self.fence_pts_extended = [
            (-35.36405249, 149.16528585),   
            (-35.36363522, 149.16611490),       
            (-35.36289046, 149.16620882),   
            (-35.36253393, 149.16502677),   
            (-35.36213514, 149.16559351),   
            (-35.36179708, 149.16420420),   
            (-35.36296177, 149.16382205),
            (-35.36413172, 149.16425601),
            (-35.36405249, 149.16528585)            
        ]
        
        self.fence_pts_cli = self.create_client(WaypointPush, '/mavros/geofence/push')
        while not self.fence_pts_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for fenceing servie...')

        self.fence_param_cli = self.create_client(SetParameters, '/mavros/geofence/set_parameters')
        while not self.fence_param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for fenceing_parmas servie...')

        self.waypoint_client = self.create_client(WaypointPush, '/mavros/mission/push')
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for waypoint service...')

        self.initial_lat = -35.3632621
        self.initial_long = 149.1652375
        self.wp_lat = -35.36222082
        self.wp_long = 149.16516177
        self.intersecting_pts = []
        self.final_wp = []

        self.create_gf()
        self.is_wp_inside_gf(self.fence_pts,self.wp_lat,self.wp_long)
        #self.gf_params()
        
    def create_gf(self):
        print('in the gf pusher')
        wps = []
        for i,(lat,lon) in enumerate(self.fence_pts_extended):
            wp = Waypoint()
            wp.frame = Waypoint.FRAME_GLOBAL
            wp.command = 5001
            wp.is_current = False
            wp.autocontinue = True
            wp.param1 = float(len(self.fence_pts_extended))
            wp.param2 = 0.0 # or 1 
            wp.param3 = 0.0
            wp.param4 = 0.0
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = 0.0
            wps.append(wp)

        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = wps
        future = self.fence_pts_cli.call_async(req)
        print('waiting for the rotation to stop')
        rclpy.spin_until_future_complete(self, future)

    def is_wp_inside_gf(self,fence_pts,wp_lat,wp_long):
        x , y = wp_lat,wp_long
        n = len(self.fence_pts)
        inside = False

        for i in range(n):
            x1,y1 = self.fence_pts[i]
            x2,y2 = self.fence_pts[(i+1) % n]

            if (x==x1) and (y==y1):
                True 
            
            if min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2):
                cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
                if abs(cross) < 1e-10:
                    return True

            if (y1 > y) != (y2 > y):  
                intersect_x = (y - y1) * (x2 - x1) / (y2 - y1) + x1
                if x <= intersect_x: 
                    inside = not inside

        print(inside)
            
        if inside :
            self.value = self.is_stpath_crossing_gf(self.initial_lat,self.initial_long,self.wp_lat,self.wp_long)
            if not self.value:
                self.create_and_push_waypoints(self.intersecting_pts)
            if self.value:
                pass       
        else:
            print('outside')

    def is_stpath_crossing_gf(self,initial_lat,initial_long,wp_lat,wp_long):
        print('inside is_path_crossing_gf')
        x1 = self.initial_lat
        y1 = self.initial_long
        x2 = self.wp_lat
        y2 = self.wp_long
        n = len(self.fence_pts)
        is_stpath_inside_gf = True

        for i in range(n):
            p_lat1,p_long1 = self.fence_pts[i]       
            p_lat2,p_long2 = self.fence_pts[(i+1) % n]

            c1 = (x2-x1)*(p_long1-y1) - (y2-y1)*(p_lat1-x1) # - 
            c2 = (x2-x1)*(p_long2-y1) - (y2-y1)*(p_lat2-x1) # - 
            c3 = (p_lat2-p_lat1)*(p_long1-y1) - (p_long2-p_lat1)*(p_lat1-x1) # +       
            c4 = (p_lat2-p_lat1)*(p_long1-y2) - (p_long2-p_lat1)*(p_lat1-x2) # +   

            if (c1*c2<0) and (c3*c4<0):
                print('intersecting and both points do not on the same side')
                self.intersecting_pts.append((p_lat2,p_long2))
                is_stpath_inside_gf = False
            else: 
                print('line is inside the gf')

        self.intersecting_pts.append((x2,y2))
        print(self.intersecting_pts)
        return is_stpath_inside_gf

    def create_and_push_waypoints(self, intersecting_pts):

        # Waypoint 1 - Takeoff
        wp1 = Waypoint()
        wp1.frame = 3
        wp1.command = 22
        wp1.is_current = True
        wp1.autocontinue = True
        wp1.param1 = 0.0
        wp1.param2 = 0.0
        wp1.param3 = 0.0
        wp1.param4 = 0.0
        wp1.x_lat = self.initial_lat
        wp1.y_long = self.initial_long
        wp1.z_alt = 10.0
        self.final_wp.append(wp1)

        for i in range(len(self.intersecting_pts)):
            print(i)
            print(self.intersecting_pts[i])
            x,y = self.intersecting_pts[i]
            print(x)
            print(y)

            wp2 = Waypoint()
            wp2.frame = 3
            wp2.command = 16 
            wp2.is_current = False
            wp2.autocontinue = True
            wp2.param1 = 0.0
            wp2.param2 = 0.0
            wp2.param3 = 0.0
            wp2.param4 = 0.0
            wp2.x_lat = x
            wp2.y_long = y
            wp2.z_alt = 5.0
            self.final_wp.append(wp2)

        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = self.final_wp
        future = self.waypoint_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
    

def main(args=None):
    rclpy.init(args=args)
    node = Geofencing()

    rclpy.spin(node)
    Node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
