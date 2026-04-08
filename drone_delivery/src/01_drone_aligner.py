#!/usr/bin/env python3 
import cv2
import rclpy
# from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import Int32 , Float32 
from geometry_msgs.msg import Twist
import math
import numpy as np
# from rclpy.executors import ExternalShutdownException
# from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode
# from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
# from rclpy.timer import Timer


class MyNode(LifecycleNode):
    def __init__(self):
        super().__init__('image_display_node')
        self.get_logger().info('AlignerNode created')

        
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring node...')
        # loading model params and other inits
        self.model = YOLO("yolov8n.pt")  
        self.br = CvBridge()
        self.frame = None  # ← Store latest frame here
        self.kp = 0.01

        self.new_twist = Twist()

        return TransitionCallbackReturn.SUCCESS
    
    

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating node...')
        
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        print(self.velocity_publisher)
        # self.new_twist = Twist()

        self.subscription = self.create_subscription(
            Image, 
            '/world/iris_runway/model/iris_with_gimbal_9002/model/gimbal/link/pitch_link/sensor/camera/image',  # Change topic name if needed
            self.listener_callback, 
            10)
        print(f'subscription {self.subscription}')
        
        self.timer = self.create_timer(1/100.0, self.display_image)
        
        return TransitionCallbackReturn.SUCCESS



    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating node...')
        
        if self.subscription:
            self.destroy_subscription(self.subscription)
            self.subscription = None
    
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS
     
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        
        rclpy.shutdown()
        cv2.destroyAllWindows()
        
        return TransitionCallbackReturn.SUCCESS

    def listener_callback(self, msg):
        try:
            print("entered normal callback")
            self.frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame = cv2.resize(self.frame,(640,480))
            self.frame = cv2.rotate(self.frame,cv2.ROTATE_180)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def display_image(self):
        
        if self.frame is None:
            return
        
        if self.frame is not None:
            
            height, width = self.frame.shape[:2]
            print(f'heightl..................{height}')
            print(f'width....................{width}')
            img = self.frame
            results = self.model(img)[0]
            
            self.mp = (int(height/2), int(width/2))
            cv2.circle(img,(self.mp),10,(255,255,255),-1)

            for box in results.boxes:
                print("leaving now")
                if int(box.cls[0]) == 0:  # class 0 = person
                    print("here now")
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = box.conf[0]
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    self.cx = (x1 + x2) / 2
                    self.cy = (y1 + y2) / 2
                    cv2.circle(img, (int(self.cx),int(self.cy)), 1, (0,0,255), -1 )#midpoint of bounding box
                    
                    
                    cv2.putText(img, f'person {conf:.2f}', (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    print("here now")

                    self.align_bot()
                    
    
                    

                    # Show result
                    cv2.imshow("Detection", img)
                    key = cv2.waitKey(1)
                    if key == ord('q'):
                        rclpy.shutdown()
                        
    def is_point_inside_circle(self, point, center, radius):
        dx =  point[0] - center[0]   # world pointss - frame points 
        dy =  point[1] - center[1]  
        return dx, dy, dx*dx + dy*dy < radius*radius


    def align_bot(self):
        dx, dy, inside = self.is_point_inside_circle(self.mp, (int(self.cx), int(self.cy)), 10)
        print('inside the align*****************')
        if inside is False:
            print('performing aligning')
            self.new_twist.linear.x = -(dx * 0.01)
            self.new_twist.linear.y = (dy * 0.01)
            
            self.new_twist.angular.z = 0.0
            self.velocity_publisher.publish(self.new_twist)
        else:
            print('not need for the alignment')
            new_twist = Twist()
            new_twist.linear.x = 0.0
            new_twist.linear.y = 0.0
            

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
