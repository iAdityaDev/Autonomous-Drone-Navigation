#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
import time 
from sensor_msgs.msg import Image
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint, WaypointReached
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from collections import deque
import cv2 as cv 
import numpy as np 
from ultralytics import YOLO
import threading
from collections import deque

def adjust_image(image, brightness=70, contrast=1.0, saturation=1.0):
    """
    Adjust brightness, contrast, and saturation of an image.
    
    Args:
        image: Input BGR image
        brightness: Brightness adjustment (-100 to 100)
        contrast: Contrast adjustment (0.0 to 3.0)
        saturation: Saturation adjustment (0.0 to 3.0)
    
    Returns:
        Adjusted image
    """
    # Brightness adjustment (add/subtract value)
    adjusted = np.clip(image.astype(np.int16) + brightness, 0, 255).astype(np.uint8)
    
    # Contrast adjustment (multiply by factor)
    adjusted = np.clip(adjusted.astype(np.float32) * contrast, 0, 255).astype(np.uint8)
    
    # Saturation adjustment (convert to HSV and adjust S channel)
    hsv = cv.cvtColor(adjusted, cv.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * saturation, 0, 255)
    adjusted = cv.cvtColor(hsv.astype(np.uint8), cv.COLOR_HSV2BGR)
    
    return adjusted


class ImageOptimizer:
    def __init__(self, model, min_conf_threshold=0.15):
        self.model = model
        self.min_conf_threshold = min_conf_threshold
        self.best_params = {"brightness": 0, "contrast": 1.0, "saturation": 1.0}
        self.best_confidence = 0.0
        self.is_optimizing = False
        self.optimization_thread = None
        self.recent_confidences = deque(maxlen=10)  # Store recent confidence values
        self.param_history = deque(maxlen=20)  # Store recent parameter combinations
        
    def evaluate_params(self, frame, brightness, contrast, saturation):
        """Evaluate a parameter combination and return the max confidence for person detection"""
        adjusted = adjust_image(frame.copy(), brightness, contrast, saturation)
        results = self.model(adjusted, conf=self.min_conf_threshold, classes=[0])
        
        if len(results[0].boxes) == 0:
            return 0.0
        
        # Get the maximum confidence score for person detections
        confidences = [box.conf.item() for box in results[0].boxes]
        return max(confidences) if confidences else 0.0
    
    def optimize_params(self, frame):
        """Run optimization to find the best parameters"""
        # Define parameter ranges to search
        brightness_range = [-50, -25, 0, 25, 50]
        contrast_range = [0.8, 0.9, 1.0, 1.1, 1.2]
        saturation_range = [0.8, 0.9, 1.0, 1.1, 1.2]
        
        best_confidence = 0.0
        best_params = self.best_params.copy()
        
        # Check if we've already tried similar parameters recently
        param_set = set((b, c, s) for b, c, s, _ in self.param_history)
        
        # Grid search through parameter combinations
        for brightness in brightness_range:
            for contrast in contrast_range:
                for saturation in saturation_range:
                    # Skip if we've tried this combination recently
                    if (brightness, contrast, saturation) in param_set:
                        continue
                        
                    confidence = self.evaluate_params(frame, brightness, contrast, saturation)
                    
                    # Store this parameter combination and its result
                    self.param_history.append((brightness, contrast, saturation, confidence))
                    
                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_params = {
                            "brightness": brightness,
                            "contrast": contrast,
                            "saturation": saturation
                        }
        
        # Update best parameters if we found better ones
        if best_confidence > self.best_confidence:
            self.best_confidence = best_confidence
            self.best_params = best_params
            print(f"New best parameters found: {self.best_params}, confidence: {self.best_confidence:.2f}")
        
        self.is_optimizing = False
    
    def start_optimization(self, frame):
        """Start the optimization process in a separate thread"""
        if not self.is_optimizing:
            self.is_optimizing = True
            self.optimization_thread = threading.Thread(target=self.optimize_params, args=(frame.copy(),))
            self.optimization_thread.daemon = True
            self.optimization_thread.start()
    
    def update_confidence(self, confidence):
        """Update the recent confidence values"""
        self.recent_confidences.append(confidence)
        
        # If confidence is dropping, trigger optimization
        if len(self.recent_confidences) >= 5:
            avg_recent = sum(list(self.recent_confidences)[-5:]) / 5
            if avg_recent < self.best_confidence * 0.8 and not self.is_optimizing:
                return True  # Signal to start optimization
        return False
    
    def get_best_params(self):
        """Get the current best parameters"""
        return self.best_params


class Precise_Waypoint(Node):
    def __init__(self):
        print("script gonna start")
        super().__init__('precise_waypoints')
        
        # Load the YOLO model during initialization
        self.model = YOLO('src/drone_delivery/assets')
        self.optimizer = ImageOptimizer(self.model, min_conf_threshold=0.25)
        
        # Create trackbars window for parameter control
        cv.namedWindow("Person Detection")
        cv.createTrackbar("Brightness", "Person Detection", 100, 200, lambda x: None)  # 0-200 (100 is neutral)
        cv.createTrackbar("Contrast", "Person Detection", 100, 300, lambda x: None)    # 0-300 (100 is neutral)
        cv.createTrackbar("Saturation", "Person Detection", 100, 300, lambda x: None)  # 0-300 (100 is neutral)
        cv.createTrackbar("Confidence", "Person Detection", 25, 100, lambda x: None)   # 0-100 (25 = 0.25)
        # cv.createTrackbar("Auto Optimize", "Person Detection", 0, 1, lambda x: None)   # 0=off, 1=on
        
        # Variables for FPS calculation
        self.prev_time = time.time()
        self.auto_mode = False
        self.last_optimization_time = 0
        self.optimization_interval = 2  # seconds between optimizations
        
        # Subscribe to camera topic
        self.camera_sub = self.create_subscription(
            Image,
            '/drone/camera',
            self.callback, 
            10
        )
        
    def callback(self, msg):
        # Process incoming image
        self.get_logger().info(f"Image data size: {len(msg.data)} bytes")
        
        # Convert image data to numpy array
        image = np.frombuffer(msg.data, dtype=np.uint8)
        
        # Get image dimensions from message if available
        height = msg.height if hasattr(msg, 'height') else 480
        width = msg.width if hasattr(msg, 'width') else 640
        
        # Reshape and convert color space
        image = image.reshape((height, width, 3))
        image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
        
        # Run YOLO detection on this frame
        self.run_yolo_detection(image)
        
    def run_yolo_detection(self, frame):
        # Check if auto mode is enabled
        self.auto_mode = cv.getTrackbarPos("Auto Optimize", "Person Detection") == 0
        
        if self.auto_mode:
            # Use the best parameters from the optimizer
            params = self.optimizer.get_best_params()
            brightness_val = params["brightness"]
            contrast_val = params["contrast"]
            saturation_val = params["saturation"]
            
            # Update trackbars to reflect auto values
            cv.setTrackbarPos("Brightness", "Person Detection", int(brightness_val + 100))
            cv.setTrackbarPos("Contrast", "Person Detection", int(contrast_val * 100))
            cv.setTrackbarPos("Saturation", "Person Detection", int(saturation_val * 100))
        else:
            # Get current trackbar values
            brightness_val = cv.getTrackbarPos("Brightness", "Person Detection") - 100  # -100 to 100
            contrast_val = cv.getTrackbarPos("Contrast", "Person Detection") / 100.0    # 0.0 to 3.0
            saturation_val = cv.getTrackbarPos("Saturation", "Person Detection") / 100.0  # 0.0 to 3.0
        
        # Get confidence threshold
        conf_threshold = cv.getTrackbarPos("Confidence", "Person Detection") / 100.0  # 0.0 to 1.0
        
        # Apply adjustments to the frame
        adjusted_frame = adjust_image(
            frame, 
            brightness=brightness_val, 
            contrast=contrast_val, 
            saturation=saturation_val
        )
        
        # Calculate FPS
        new_time = time.time()
        fps = 1 / (new_time - self.prev_time) if (new_time - self.prev_time) > 0 else 0
        self.prev_time = new_time
        
        # Run YOLO detection (filter for person class only - class 0)
        results = self.model(adjusted_frame, conf=conf_threshold, classes=[0])
        annotated_frame = results[0].plot()
        
        # Count persons detected
        person_count = len(results[0].boxes)
        
        # Get max confidence if persons detected
        current_max_conf = 0
        if person_count > 0:
            confidences = [box.conf.item() for box in results[0].boxes]
            current_max_conf = max(confidences)
            
            # Update optimizer with current confidence
            if self.auto_mode:
                should_optimize = self.optimizer.update_confidence(current_max_conf)
                
                # Start optimization if needed and not already optimizing
                current_time = time.time()
                if should_optimize and not self.optimizer.is_optimizing and (current_time - self.last_optimization_time) > self.optimization_interval:
                    print("Starting parameter optimization...")
                    self.optimizer.start_optimization(frame)
                    self.last_optimization_time = current_time
        
        # Get frame dimensions
        height, width = frame.shape[:2]
        
        # Add information to the frame
        cv.putText(annotated_frame, f'FPS: {int(fps)}', (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(annotated_frame, f'Brightness: {brightness_val}', (20, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(annotated_frame, f'Contrast: {contrast_val:.1f}', (20, 90), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(annotated_frame, f'Saturation: {saturation_val:.1f}', (20, 120), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(annotated_frame, f'Confidence: {conf_threshold:.2f}', (20, 150), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(annotated_frame, f'Persons detected: {person_count}', (20, 180), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(annotated_frame, f'Max confidence: {current_max_conf:.2f}', (20, 210), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv.putText(annotated_frame, f'Mode: {"Auto" if self.auto_mode else "Manual"}', (20, 240), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if self.optimizer.is_optimizing:
            cv.putText(annotated_frame, "OPTIMIZING...", (width - 200, 30), 
                      cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Display the annotated frame
        cv.imshow("Person Detection", annotated_frame)
        cv.waitKey(1)  # Process GUI events


def main():
    rclpy.init()
    camera_subscriber = Precise_Waypoint()
    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Properly clean up resources
        camera_subscriber.destroy_node()  # Fixed: added parentheses
        cv.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()