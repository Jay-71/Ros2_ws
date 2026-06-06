#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/r1_mini/camera/image_raw',
            self.image_callback,
            10
        )
        
        # --- MISSION MEMORY SETUP ---
        # The specific markers we care about
        self.target_ids = {24, 25, 26, 27}
        # Using a set to automatically ignore duplicates
        self.detected_memory = set() 

        self.get_logger().info('ArUco Detector Online.')

        # --- OPENCV ARUCO API SETUP ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            corners, ids, rejected = self.detector.detectMarkers(cv_image)

            if ids is not None:
                valid_corners = []
                valid_ids = []

                # 1. Filter out garbage/unwanted IDs
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    if marker_id in self.target_ids:
                        valid_corners.append(corners[i])
                        valid_ids.append([marker_id])

                # 2. Process valid IDs
                if len(valid_ids) > 0:
                    # Draw boxes only around the target markers
                    aruco.drawDetectedMarkers(cv_image, tuple(valid_corners), np.array(valid_ids))
                    
                    for m_id_array in valid_ids:
                        marker_id = m_id_array[0]
                        
                        # 3. Append to memory if it's new
                        if marker_id not in self.detected_memory:
                            self.detected_memory.add(marker_id)
                            self.get_logger().info(f'+++ NEW TARGET ARUCO SECURED: {marker_id} +++')
                            self.get_logger().info(f'Mission Progress: {len(self.detected_memory)}/4 -> {sorted(list(self.detected_memory))}')

            # --- HUD OVERLAY ---
            # Display the memory directly on the video feed
            memory_list = sorted(list(self.detected_memory))
            hud_text = f"Memory: {memory_list} ({len(memory_list)}/4)"
            
            # Green text if we found all 4, otherwise Orange
            text_color = (0, 255, 0) if len(self.detected_memory) == 4 else (0, 165, 255)
            
            cv2.putText(cv_image, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
            cv2.imshow("MINI_R1 ArUco Scanner", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final report on shutdown
        node.get_logger().info('--- SHUTDOWN ---')
        node.get_logger().info(f'Final Memory: {sorted(list(node.detected_memory))}')
        
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()