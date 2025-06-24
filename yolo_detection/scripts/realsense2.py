#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
from yolo_detection.msg import BoundingBoxArray, BoundingBox
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Header, Bool, Float32
import math

class RealSenseTomatoDetector:
    def __init__(self):
        rospy.init_node('realsense_tomato_detector', anonymous=True)
        
        # Load YOLOv8 model
        model_path = rospy.get_param('~model_path', 'yolov8n.pt')
        self.yolo = YOLO(model_path, verbose=False)
        
        # Define classes that should be treated as tomatoes
        self.tomato_like_classes = rospy.get_param('~tomato_like_classes', [
            'tomato',
            'apple', 
            'orange',
            'sports ball',
            'banana',
            'tennis ball',
            'baseball',
            'basketball',
            'soccer ball'
        ])
        
        rospy.loginfo(f"Classes treated as tomatoes: {self.tomato_like_classes}")
        
        # RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure streams
        width = rospy.get_param('~image_width', 640)
        height = rospy.get_param('~image_height', 480)
        fps = rospy.get_param('~fps', 30)
        
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        # Start streaming
        try:
            self.profile = self.pipeline.start(self.config)
            rospy.loginfo("RealSense camera started successfully")
        except Exception as e:
            rospy.logerr(f"Failed to start RealSense camera: {e}")
            return
        
        # Get camera intrinsics
        self.depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
        self.color_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.color))
        self.depth_intrinsics = self.depth_profile.get_intrinsics()
        self.color_intrinsics = self.color_profile.get_intrinsics()
        
        # Align depth to color
        self.align = rs.align(rs.stream.color)
        
        # Create depth colorizer for visualization
        self.colorizer = rs.colorizer()
        
        # ROS setup
        self.bridge = CvBridge()
        
        # Publishers
        self.bbox_pub = rospy.Publisher('tomato_detections', BoundingBoxArray, queue_size=1)
        self.image_pub = rospy.Publisher('/realsense/annotated_image', Image, queue_size=1)
        self.depth_pub = rospy.Publisher('/realsense/depth_colorized', Image, queue_size=1)
        self.tomato_point_pub = rospy.Publisher('/realsense/tomato_3d_point', PointStamped, queue_size=1)
        
        # New publishers for enhanced functionality
        self.lowest_tomato_pub = rospy.Publisher('/realsense/lowest_tomato_point', PointStamped, queue_size=1)
        self.tomato_count_pub = rospy.Publisher('/realsense/tomato_count', Float32, queue_size=1)
        self.target_lost_pub = rospy.Publisher('/realsense/target_lost', Bool, queue_size=1)
        
        # Parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        
        # Camera transformation parameters
        self.camera_tilt_angle = rospy.get_param('~camera_tilt_angle', 0.0)
        self.camera_height = rospy.get_param('~camera_height', 0.0)
        self.camera_offset_x = rospy.get_param('~camera_offset_x', 0.0)
        
        # Convert to radians
        self.cam_angle_rad = math.radians(self.camera_tilt_angle)
        
        # Depth filtering parameters
        self.min_depth = rospy.get_param('~min_depth', 0.1)
        self.max_depth = rospy.get_param('~max_depth', 3.0)
        
        # Enhanced detection parameters
        self.min_tomato_size = rospy.get_param('~min_tomato_size', 20)  # minimum pixel size
        self.max_tomato_size = rospy.get_param('~max_tomato_size', 200)  # maximum pixel size
        self.aspect_ratio_tolerance = rospy.get_param('~aspect_ratio_tolerance', 0.3)
        
        # Target tracking for pickup detection
        self.previous_detections = []
        self.target_stable_threshold = rospy.get_param('~target_stable_threshold', 0.05)  # 5cm stability
        self.target_lost_threshold = rospy.get_param('~target_lost_threshold', 3)  # frames without detection
        self.frames_without_target = 0
        self.last_target_position = None
        
        # Selection strategy: 'lowest', 'closest', 'largest'
        self.selection_strategy = rospy.get_param('~selection_strategy', 'lowest')
        
        rospy.loginfo("Enhanced RealSense Tomato Detector initialized")
        
    def is_tomato_like(self, class_name):
        """Check if the detected class should be treated as a tomato"""
        return class_name.lower() in [cls.lower() for cls in self.tomato_like_classes]
        
    def get_3d_coordinates(self, center_x, center_y, depth_frame):
        """Get 3D coordinates from pixel coordinates and depth"""
        # Sample multiple points around center for more robust depth estimation
        sample_points = [
            (center_x, center_y),
            (center_x-2, center_y),
            (center_x+2, center_y),
            (center_x, center_y-2),
            (center_x, center_y+2)
        ]
        
        valid_depths = []
        for px, py in sample_points:
            if 0 <= px < self.depth_intrinsics.width and 0 <= py < self.depth_intrinsics.height:
                depth_value = depth_frame.get_distance(int(px), int(py))
                if self.min_depth <= depth_value <= self.max_depth:
                    valid_depths.append(depth_value)
        
        if not valid_depths:
            return None
            
        # Use median depth for robustness
        depth_value = np.median(valid_depths)
        
        # Deproject pixel to 3D point in camera frame
        point_3d = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [center_x, center_y], depth_value)
        
        return point_3d
    
    def transform_to_arm_coordinates(self, camera_coords):
        """Transform from camera coordinates to arm coordinate system"""
        if camera_coords is None:
            return None
            
        x_cam, y_cam, z_cam = camera_coords
        
        # Convert to centimeters
        x_cam_cm = x_cam * 100
        y_cam_cm = y_cam * 100  
        z_cam_cm = z_cam * 100
        
        # Apply camera tilt transformation
        x_arm = z_cam_cm * math.cos(self.cam_angle_rad) - y_cam_cm * math.sin(self.cam_angle_rad)
        y_arm = y_cam_cm * math.cos(self.cam_angle_rad) + z_cam_cm * math.sin(self.cam_angle_rad)
        z_arm = x_cam_cm
        
        # Apply camera offset
        x_arm += self.camera_offset_x * 100
        y_arm += self.camera_height * 100
        
        return [x_arm, y_arm, z_arm]
    
    def validate_tomato_detection(self, box, confidence):
        """Enhanced validation for tomato detections"""
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        width = x2 - x1
        height = y2 - y1
        
        # Size filtering
        avg_size = (width + height) / 2
        if avg_size < self.min_tomato_size or avg_size > self.max_tomato_size:
            return False
            
        # Aspect ratio check (tomatoes should be roughly circular)
        aspect_ratio = width / height if height > 0 else 0
        if abs(aspect_ratio - 1.0) > self.aspect_ratio_tolerance:
            return False
            
        return True
    
    def select_target_tomato(self, valid_detections):
        """Select the target tomato based on strategy"""
        if not valid_detections:
            return None
            
        if self.selection_strategy == 'lowest':
            # Select tomato with highest Y coordinate (lowest in image, assuming camera looks down)
            return max(valid_detections, key=lambda x: x['arm_coords'][1])
        elif self.selection_strategy == 'closest':
            # Select closest tomato
            return min(valid_detections, key=lambda x: x['distance'])
        elif self.selection_strategy == 'largest':
            # Select largest tomato
            return max(valid_detections, key=lambda x: x['size'])
        else:
            return valid_detections[0]
    
    def detect_target_stability(self, current_target):
        """Detect if target is stable (for pickup success detection)"""
        if current_target is None:
            self.frames_without_target += 1
            if self.frames_without_target > self.target_lost_threshold:
                return False, True  # unstable, target_lost
            return False, False
        
        self.frames_without_target = 0
        
        if self.last_target_position is None:
            self.last_target_position = current_target['arm_coords']
            return True, False
        
        # Calculate position change
        pos_change = np.linalg.norm(np.array(current_target['arm_coords']) - np.array(self.last_target_position))
        
        self.last_target_position = current_target['arm_coords']
        
        # Target is stable if position change is small
        is_stable = pos_change < self.target_stable_threshold
        
        return is_stable, False
    
    def detect_tomatoes(self, color_image, depth_frame):
        """Enhanced tomato detection with target selection and stability tracking"""
        # Run YOLO detection
        results = self.yolo(color_image, conf=self.confidence_threshold, verbose=False)
        
        # Create BoundingBoxArray message
        bbox_array = BoundingBoxArray()
        bbox_array.header.stamp = rospy.Time.now()
        bbox_array.header.frame_id = self.camera_frame
        
        annotated_image = color_image.copy()
        valid_detections = []
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Get class info
                    class_id = int(box.cls[0])
                    original_class_name = self.yolo.names[class_id]
                    confidence = float(box.conf[0])
                    
                    # Check if this is a tomato-like object
                    if not self.is_tomato_like(original_class_name) or confidence < self.confidence_threshold:
                        continue
                    
                    # Validate detection
                    if not self.validate_tomato_detection(box, confidence):
                        continue
                    
                    # Get bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    width = x2 - x1
                    height = y2 - y1
                    
                    # Get 3D coordinates
                    camera_coords = self.get_3d_coordinates(center_x, center_y, depth_frame)
                    if camera_coords is None:
                        continue
                        
                    arm_coords = self.transform_to_arm_coordinates(camera_coords)
                    if arm_coords is None:
                        continue
                    
                    # Calculate metrics for selection
                    distance = math.sqrt(sum(coord**2 for coord in camera_coords))
                    size = (width + height) / 2
                    
                    # Store valid detection
                    detection = {
                        'box': (x1, y1, x2, y2),
                        'center': (center_x, center_y),
                        'camera_coords': camera_coords,
                        'arm_coords': arm_coords,
                        'confidence': confidence,
                        'distance': distance,
                        'size': size
                    }
                    valid_detections.append(detection)
                    
                    # Create BoundingBox message
                    bbox_msg = BoundingBox()
                    bbox_msg.class_name = "tomato"
                    bbox_msg.confidence = confidence
                    bbox_msg.x_min = int(x1)
                    bbox_msg.y_min = int(y1)
                    bbox_msg.x_max = int(x2)
                    bbox_msg.y_max = int(y2)
                    bbox_array.bounding_boxes.append(bbox_msg)
        
        # Select target tomato
        target_tomato = self.select_target_tomato(valid_detections)
        
        # Check target stability and pickup success
        is_stable, target_lost = self.detect_target_stability(target_tomato)
        
        # Annotate image
        for i, detection in enumerate(valid_detections):
            x1, y1, x2, y2 = detection['box']
            arm_coords = detection['arm_coords']
            confidence = detection['confidence']
            
            # Different colors for target vs other tomatoes
            is_target = (detection == target_tomato)
            color = (0, 255, 0) if is_target else (0, 165, 255)  # Green for target, orange for others
            thickness = 3 if is_target else 2
            
            cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
            
            # Labels
            label_text = f"{'TARGET ' if is_target else ''}tomato {confidence:.2f}"
            cv2.putText(annotated_image, label_text, (int(x1), int(y1)-40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # 3D coordinates
            coord_text = f"3D: ({arm_coords[0]:.1f}, {arm_coords[1]:.1f}, {arm_coords[2]:.1f})cm"
            cv2.putText(annotated_image, coord_text, (int(x1), int(y1)-25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # Additional info for target
            if is_target:
                stability_text = f"Stable: {is_stable}"
                cv2.putText(annotated_image, stability_text, (int(x1), int(y1)-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # Add summary info
        info_text = f"Tomatoes: {len(valid_detections)} | Strategy: {self.selection_strategy}"
        cv2.putText(annotated_image, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if target_lost:
            cv2.putText(annotated_image, "TARGET LOST - PICKUP SUCCESS?", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        return bbox_array, annotated_image, target_tomato, len(valid_detections), target_lost
    
    def publish_results(self, bbox_array, target_tomato, tomato_count, target_lost):
        """Publish all detection results"""
        # Publish bounding boxes
        self.bbox_pub.publish(bbox_array)
        
        # Publish tomato count
        count_msg = Float32()
        count_msg.data = tomato_count
        self.tomato_count_pub.publish(count_msg)
        
        # Publish target lost status
        lost_msg = Bool()
        lost_msg.data = target_lost
        self.target_lost_pub.publish(lost_msg)
        
        # Publish target tomato points
        if target_tomato:
            # Closest tomato (backward compatibility)
            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()
            point_msg.header.frame_id = self.camera_frame
            point_msg.point.x = target_tomato['arm_coords'][0] / 100.0
            point_msg.point.y = target_tomato['arm_coords'][1] / 100.0
            point_msg.point.z = target_tomato['arm_coords'][2] / 100.0
            self.tomato_point_pub.publish(point_msg)
            
            # Target tomato (new topic)
            self.lowest_tomato_pub.publish(point_msg)
        
    def run(self):
        """Main detection loop"""
        rate = rospy.Rate(30)
        
        try:
            while not rospy.is_shutdown():
                # Get frames
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    continue
                
                # Convert to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                
                # Detect tomatoes
                bbox_array, annotated_image, target_tomato, tomato_count, target_lost = self.detect_tomatoes(color_image, depth_frame)
                
                # Publish results
                self.publish_results(bbox_array, target_tomato, tomato_count, target_lost)
                
                # Publish annotated image
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
                    image_msg.header.stamp = rospy.Time.now()
                    image_msg.header.frame_id = self.camera_frame
                    self.image_pub.publish(image_msg)
                except Exception as e:
                    rospy.logwarn(f"Failed to publish annotated image: {e}")
                
                # Publish colorized depth image
                try:
                    colorized_depth = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
                    depth_msg = self.bridge.cv2_to_imgmsg(colorized_depth, "bgr8")
                    depth_msg.header.stamp = rospy.Time.now()
                    depth_msg.header.frame_id = self.camera_frame
                    self.depth_pub.publish(depth_msg)
                except Exception as e:
                    rospy.logwarn(f"Failed to publish depth image: {e}")
                
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down Enhanced RealSense Tomato Detector")
        finally:
            self.pipeline.stop()

if __name__ == '__main__':
    try:
        detector = RealSenseTomatoDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in Enhanced RealSense Tomato Detector: {e}")