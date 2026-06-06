#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import math
import numpy as np

os.environ['QT_LOGGING_RULES'] = 'qt.qpa.fonts.warning=false'

try:
    from ultralytics import YOLO
except ImportError:
    raise ImportError("ultralytics is not installed. Run: pip install ultralytics")

CLASS_COLORS = {
    "LEFT":    (255, 165,   0), 
    "RIGHT":   ( 30, 144, 255), 
    "FORWARD": ( 50, 205,  50), 
    "STOP":    (  0,   0, 255), 
    "GOAL":    (255, 215,   0), 
    "ROTATE":  (238, 130, 238), 
}

class AutonomousCANBot(Node):
    def __init__(self):
        super().__init__('autonomous_canbot_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.hud_pub = self.create_publisher(Image, '/r1_mini/hud', 10) 
        
        self.create_subscription(LaserScan, '/r1_mini/lidar', self.lidar_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/r1_mini/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/r1_mini/camera/image_raw', self.image_cb, 10)

        cv2.namedWindow("CANBot Command Center", cv2.WINDOW_NORMAL)
        self.display_timer = self.create_timer(0.05, self.display_cb)
        self.current_hud = None

        self.bridge = CvBridge()
        self.is_processing_image = False
        self.latest_camera_frame = np.zeros((500, 500, 3), dtype=np.uint8) 
        self.conf = 0.85 
        
        self.model_path = '/home/jaydhake/ros2_ws/src/mini_r1_v1_gz/runs/detect/canbot_signs3/weights/best.pt'
        if os.path.exists(self.model_path):
            self.model = YOLO(self.model_path)
            self.get_logger().info('YOLOv8 AI Loaded.')

        self.MAX_SPEED = 0.6          
        self.MAX_TURN = 1.2           
        self.ROBOT_WIDTH_SECTORS = 3  
        self.TARGET_SECTOR = 24       
        self.last_chosen_sector = 24  

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.breadcrumbs = [] 
        self.stop_walls = [] 
        self.BREADCRUMB_SPACING = 0.4 
        self.BREADCRUMB_REPULSION = 3.0 

        self.sign_memory = []           
        self.pending_turn = None        
        self.pending_timeout = 0.0      
        self.armed_turn = None          
        self.armed_timeout = 0.0        
        self.executing_turn_until = 0.0 
        self.executing_turn_name = ""   
        self.is_stopped = False
        self.stop_until = 0.0
        self.stop_cooldown = 0.0
        
        self.current_visible_sign = None 
        self.blind_spin_until = 0.0      
        
        self.turn_cooldowns = {"LEFT": 0.0, "RIGHT": 0.0, "FORWARD": 0.0, "ROTATE": 0.0}

        self.get_logger().info("Monolithic Pipeline Online. Close-Approach Goal Braking Active!")

    def image_cb(self, msg: Image):
        if self.is_processing_image: return
        
        try:
            self.is_processing_image = True
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            current_time = self.get_clock().now().nanoseconds / 1e9

            results = self.model.predict(source=frame, conf=self.conf, imgsz=320, verbose=False, device=0)

            best_label = None
            best_conf = 0.0
            best_ratio = 0.0 

            frame_h = frame.shape[0]

            for result in results:
                if result.boxes is None: continue
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    label = self.model.names.get(cls_id, f'class_{cls_id}').upper()
                    
                    # ==========================================
                    # UPGRADE: Strict STOP Confidence (97%)
                    # ==========================================
                    if label == "STOP" and confidence < 0.98:
                        continue 
                        
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    b_w = x2 - x1
                    b_h = y2 - y1
                    
                    if b_h > 0:
                        aspect_ratio = b_w / b_h
                        if aspect_ratio < 0.50 or aspect_ratio > 2.00:
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (100, 100, 100), 2)
                            cv2.putText(frame, f'SKEW IGN: {aspect_ratio:.2f}', (x1, y1 - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 2)
                            continue 
                    
                    if confidence > best_conf:
                        best_conf = confidence
                        best_label = label
                        best_ratio = b_h / frame_h

                    color = CLASS_COLORS.get(label, (0, 255, 255))
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                    cv2.putText(frame, f'{label} {confidence:.2f}', (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            self.latest_camera_frame = cv2.resize(frame, (500, 500))
            self.current_visible_sign = best_label 

            if best_label:
                if current_time < self.turn_cooldowns.get(best_label, 0.0):
                    cv2.putText(self.latest_camera_frame, f"COOLDOWN: IGNORING {best_label}", (10, 80), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
                    best_label = None 

            if best_label:
                already_seen = False
                for sx, sy, slabel in self.sign_memory:
                    if slabel == best_label and math.hypot(self.current_x - sx, self.current_y - sy) < 6.0:
                        already_seen = True
                        break
                
                if not already_seen:
                    if best_label == "STOP":
                        if best_ratio > 0.26: 
                            if not self.is_stopped and current_time > self.stop_cooldown:
                                self.sign_memory.append((self.current_x, self.current_y, best_label))
                                
                                self.get_logger().info("🛑 STOP LINE REACHED! Erecting Virtual Wall.")
                                self.is_stopped = True
                                self.stop_until = current_time + 2.5 
                                self.stop_cooldown = current_time + 10.0 
                                wall_x = self.current_x + 1.0 * math.cos(self.current_yaw)
                                wall_y = self.current_y + 1.0 * math.sin(self.current_yaw)
                                self.stop_walls.append((wall_x, wall_y, current_time + 20.0))
                        else:
                            self.get_logger().info(f"👀 SPOTTED: {best_label} AHEAD. Driving closer...")
                            self.pending_turn = best_label
                            self.pending_timeout = current_time + 15.0
                            
                    elif best_label == "GOAL":
                        # ==========================================
                        # UPGRADE: Increased threshold to 0.40 to get much closer!
                        # ==========================================
                        if best_ratio > 0.40:
                            self.sign_memory.append((self.current_x, self.current_y, best_label))
                            self.get_logger().info("🏁 GOAL REACHED! Halting motors permanently.")
                            self.is_stopped = True
                            self.stop_until = current_time + 999999.0 
                            self.pending_turn = None
                        else:
                            self.get_logger().info(f"👀 SPOTTED: {best_label} AHEAD. Driving closer...")
                            self.pending_turn = best_label
                            self.pending_timeout = current_time + 15.0
                            
                    else:
                        if best_ratio > 0.22: 
                            self.sign_memory.append((self.current_x, self.current_y, best_label))
                            
                            self.get_logger().info(f"🎯 PROXIMITY REACHED! Armed for {best_label}.")
                            self.armed_turn = best_label
                            self.armed_timeout = current_time + 5.0 
                            self.pending_turn = None
                        else:
                            self.get_logger().info(f"👀 SPOTTED: {best_label} AHEAD. Driving closer...")
                            self.pending_turn = best_label
                            self.pending_timeout = current_time + 15.0

                else:
                    cv2.putText(self.latest_camera_frame, f"GPS BLOCK: {best_label}", (10, 110), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 215, 255), 2)
                                
                    if best_label != "STOP" and best_label != "GOAL":
                        if self.pending_turn == best_label:
                            self.pending_timeout = current_time + 15.0
                            if best_ratio > 0.22: 
                                self.get_logger().info(f"🎯 PROXIMITY REACHED! Armed for {best_label}.")
                                self.armed_turn = best_label
                                self.armed_timeout = current_time + 5.0
                                self.pending_turn = None
                        elif self.armed_turn == best_label:
                            self.armed_timeout = current_time + 5.0 

        except Exception as e:
            self.get_logger().error(f"Vision Error: {e}")
        finally:
            self.is_processing_image = False

    def odom_cb(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        angular_z = abs(msg.twist.twist.angular.z)
        
        if angular_z < 0.8: 
            if not self.breadcrumbs:
                self.breadcrumbs.append((self.current_x, self.current_y, current_time))
            else:
                last_x, last_y, _ = self.breadcrumbs[-1]
                if math.hypot(self.current_x - last_x, self.current_y - last_y) > self.BREADCRUMB_SPACING:
                    self.breadcrumbs.append((self.current_x, self.current_y, current_time))
                    
        self.breadcrumbs = [b for b in self.breadcrumbs if current_time - b[2] < 45.0]

    def lidar_cb(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges), 12.0, ranges)
        fov_scan = ranges[60:300] 
        self.process_navigation(fov_scan)

    def process_navigation(self, scan):
        twist = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9

        self.stop_walls = [w for w in self.stop_walls if current_time < w[2]]

        if self.pending_turn and current_time > self.pending_timeout:
            self.pending_turn = None
        if self.armed_turn and current_time > self.armed_timeout:
            self.armed_turn = None

        if self.is_stopped:
            if current_time < self.stop_until:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.build_hud(scan, None, current_time)
                return
            else:
                self.is_stopped = False

        if self.executing_turn_name == "ROTATE":
            twist.linear.x = 0.0
            twist.angular.z = self.MAX_TURN 
            
            if current_time > self.executing_turn_until:
                self.get_logger().warn("⚠️ ROTATE Timed Out! Escaping loop.")
                self.executing_turn_name = ""
                self.turn_cooldowns["ROTATE"] = current_time + 30.0 
            elif current_time > self.blind_spin_until:
                if self.current_visible_sign == "ROTATE":
                    self.get_logger().info("✅ Vision-Based 360 Completed! Sign Re-Acquired.")
                    self.executing_turn_name = ""
                    self.turn_cooldowns["ROTATE"] = current_time + 30.0 

            self.cmd_pub.publish(twist)
            self.build_hud(scan, None, current_time)
            return

        if current_time > self.executing_turn_until and self.armed_turn is None:
            self.TARGET_SECTOR = 24 

        if self.pending_turn:
            front_dist = np.min(scan[110:130]) 
            if front_dist < 1.0: 
                self.get_logger().info(f"⚠️ WALL APPROACHING! Auto-Arming {self.pending_turn} turn.")
                
                self.sign_memory.append((self.current_x, self.current_y, self.pending_turn))
                
                if self.pending_turn == "STOP":
                    self.is_stopped = True
                    self.stop_until = current_time + 2.5 
                    self.stop_cooldown = current_time + 10.0 
                    wall_x = self.current_x + 1.0 * math.cos(self.current_yaw)
                    wall_y = self.current_y + 1.0 * math.sin(self.current_yaw)
                    self.stop_walls.append((wall_x, wall_y, current_time + 20.0))
                elif self.pending_turn == "GOAL":
                    self.is_stopped = True
                    self.stop_until = current_time + 999999.0
                    self.get_logger().info("🏁 LiDAR triggered GOAL stop. Halting permanently.")
                else:
                    self.armed_turn = self.pending_turn
                    self.armed_timeout = current_time + 5.0
                
                self.pending_turn = None

        num_sectors = 48
        pod = np.zeros(num_sectors)

        for i, d in enumerate(scan):
            angle_rad = math.radians(i - 120)
            dynamic_safe_dist = 0.45 + (0.8 * max(0.0, math.cos(angle_rad)))
            if d < dynamic_safe_dist:
                sector = i // 5
                pod[sector] += 1.0 + ((dynamic_safe_dist - d) * 2.5)

        dilated_pod = np.copy(pod)
        for s in range(48):
            if pod[s] > 0.5: 
                if s > 0: dilated_pod[s-1] = max(dilated_pod[s-1], pod[s])
                if s < 47: dilated_pod[s+1] = max(dilated_pod[s+1], pod[s])
        pod = dilated_pod

        for wx, wy, w_expire in self.stop_walls:
            dist_to_wall = math.hypot(wx - self.current_x, wy - self.current_y)
            if dist_to_wall < 2.5: 
                global_angle = math.atan2(wy - self.current_y, wx - self.current_x)
                relative_angle = global_angle - self.current_yaw
                while relative_angle > math.pi: relative_angle -= 2 * math.pi
                while relative_angle < -math.pi: relative_angle += 2 * math.pi
                
                if -math.radians(120) <= relative_angle <= math.radians(120):
                    deg_angle = math.degrees(relative_angle)
                    sector = int((deg_angle + 120.0) / 5.0)
                    for s in range(max(0, sector - 3), min(48, sector + 4)):
                        pod[s] += 20.0 

        if len(self.breadcrumbs) > 10:
            for bx, by, b_time in self.breadcrumbs[:-10]:
                dist_to_crumb = math.hypot(bx - self.current_x, by - self.current_y)
                
                if dist_to_crumb < 1.5:
                    global_angle = math.atan2(by - self.current_y, bx - self.current_x)
                    relative_angle = global_angle - self.current_yaw
                    while relative_angle > math.pi: relative_angle -= 2 * math.pi
                    while relative_angle < -math.pi: relative_angle += 2 * math.pi

                    if -math.radians(120) <= relative_angle <= math.radians(120):
                        deg_angle = math.degrees(relative_angle)
                        lidar_idx = int(deg_angle + 120.0)
                        if 0 <= lidar_idx < 240:
                            if dist_to_crumb > (scan[lidar_idx] + 0.50):
                                continue 
                            sector = lidar_idx // 5
                            if 0 <= sector < 48:
                                age = current_time - b_time
                                dynamic_repulsion = max(0.0, self.BREADCRUMB_REPULSION * (1.0 - (age / 45.0)))
                                pod[sector] += dynamic_repulsion

        window = [0.1, 0.2, 0.4, 0.2, 0.1]
        smoothed_pod = np.convolve(pod, window, mode='same')

        valleys = []
        current_valley = []
        for i in range(num_sectors):
            if smoothed_pod[i] < 1.2:
                current_valley.append(i)
            else:
                if len(current_valley) > 0:
                    valleys.append(current_valley)
                    current_valley = []
        if len(current_valley) > 0:
            valleys.append(current_valley)

        if self.armed_turn == "LEFT":
            for v in valleys:
                if any(s > 34 for s in v): 
                    self.TARGET_SECTOR = 40 
                    self.executing_turn_name = "LEFT"
                    self.executing_turn_until = current_time + 4.5 
                    self.turn_cooldowns["LEFT"] = current_time + 30.0 
                    self.armed_turn = None
                    break
        elif self.armed_turn == "RIGHT":
            for v in valleys:
                if any(s < 14 for s in v): 
                    self.TARGET_SECTOR = 8 
                    self.executing_turn_name = "RIGHT"
                    self.executing_turn_until = current_time + 4.5 
                    self.turn_cooldowns["RIGHT"] = current_time + 30.0 
                    self.armed_turn = None
                    break
        elif self.armed_turn == "FORWARD":
            self.TARGET_SECTOR = 24 
            self.executing_turn_name = "FORWARD"
            self.executing_turn_until = current_time + 1.5 
            self.turn_cooldowns["FORWARD"] = current_time + 30.0 
            self.armed_turn = None
        elif self.armed_turn == "ROTATE":
            self.executing_turn_name = "ROTATE"
            self.blind_spin_until = current_time + 2.0 
            self.executing_turn_until = current_time + 15.0 
            self.armed_turn = None

        best_cost = float('inf')
        best_target = None
        
        heading_weight = 1.0
        if self.pending_turn or self.armed_turn:
            heading_weight = 3.5 
        elif current_time < self.executing_turn_until:
            if self.executing_turn_name == "FORWARD":
                heading_weight = 5.0 
            else:
                heading_weight = 0.5 

        for valley in valleys:
            if len(valley) < self.ROBOT_WIDTH_SECTORS: continue
            center_sector = sum(valley) / len(valley)
            if current_time > self.executing_turn_until:
                if self.TARGET_SECTOR in valley:
                    idx_in_valley = valley.index(self.TARGET_SECTOR)
                    if idx_in_valley >= 1 and (len(valley) - idx_in_valley) >= 2:
                        center_sector = self.TARGET_SECTOR
            
            heading_cost = abs(center_sector - self.TARGET_SECTOR)
            memory_cost = abs(center_sector - self.last_chosen_sector)
            total_cost = (heading_cost * heading_weight) + (memory_cost * 2.5)

            if total_cost < best_cost:
                best_cost = total_cost
                best_target = center_sector

        if best_target is not None:
            self.last_chosen_sector = best_target 
            angle_error = (best_target - 24) * 5.0 * (math.pi / 180.0) 
            twist.angular.z = np.clip(angle_error * 1.8, -self.MAX_TURN, self.MAX_TURN)
            speed_factor = max(0.15, 1.0 - (abs(twist.angular.z) / self.MAX_TURN))
            twist.linear.x = self.MAX_SPEED * speed_factor
        else:
            twist.linear.x = 0.0
            twist.angular.z = self.MAX_TURN

        self.cmd_pub.publish(twist)
        self.build_hud(scan, best_target, current_time)

    def build_hud(self, scan, best_target, current_time):
        lidar_img = np.zeros((500, 500, 3), dtype=np.uint8)
        cx, cy, scale = 250, 250, 50 

        for i, d in enumerate(scan):
            if d < 10.0:
                angle_rad = math.radians(i - 120)
                lx = d * math.cos(angle_rad)
                ly = d * math.sin(angle_rad)
                px = int(cx - (ly * scale))
                py = int(cy - (lx * scale))
                if 0 <= px < 500 and 0 <= py < 500:
                    cv2.circle(lidar_img, (px, py), 2, (0, 255, 0), -1)

        for bx, by, b_time in self.breadcrumbs:
            age = current_time - b_time
            intensity = int(max(50, 255 * (1.0 - (age / 45.0))))
            
            dx, dy = bx - self.current_x, by - self.current_y
            lx = dx * math.cos(-self.current_yaw) - dy * math.sin(-self.current_yaw)
            ly = dx * math.sin(-self.current_yaw) + dy * math.cos(-self.current_yaw)
            px, py = int(cx - (ly * scale)), int(cy - (lx * scale))
            if 0 <= px < 500 and 0 <= py < 500:
                cv2.circle(lidar_img, (px, py), int(0.4 * scale), (0, 0, intensity), 1)

        for wx, wy, w_expire in self.stop_walls:
            dx, dy = wx - self.current_x, wy - self.current_y
            lx = dx * math.cos(-self.current_yaw) - dy * math.sin(-self.current_yaw)
            ly = dx * math.sin(-self.current_yaw) + dy * math.cos(-self.current_yaw)
            px, py = int(cx - (ly * scale)), int(cy - (lx * scale))
            if 0 <= px < 500 and 0 <= py < 500:
                cv2.circle(lidar_img, (px, py), int(0.5 * scale), (0, 0, 255), -1)

        if best_target is not None:
            target_angle = (best_target - 24) * 5.0 * (math.pi / 180.0)
            tx = 2.0 * math.cos(target_angle)
            ty = 2.0 * math.sin(target_angle)
            px = int(cx - (ty * scale))
            py = int(cy - (tx * scale))
            cv2.line(lidar_img, (cx, cy), (px, py), (0, 255, 255), 2)

        cv2.circle(lidar_img, (cx, cy), 6, (255, 255, 255), -1)
        hud = np.hstack((lidar_img, self.latest_camera_frame))

        status_text = "STATUS: EXPLORING"
        text_color = (0, 255, 0)
        
        if self.is_stopped:
            if self.stop_until > current_time + 1000: 
                status_text = "STATUS: GOAL REACHED. MISSION ACCOMPLISHED."
                text_color = (0, 215, 255) 
            else:
                status_text = "STATUS: BLOCKED BY STOP SIGN"
                text_color = (0, 0, 255) 
        elif self.executing_turn_name == "ROTATE":
            if current_time < self.blind_spin_until:
                status_text = "STATUS: 360 SPIN (BLINDFOLD ACTIVE)"
                text_color = (255, 100, 100) 
            else:
                status_text = "STATUS: 360 SPIN (SEARCHING FOR SIGN...)"
                text_color = (255, 255, 0) 
        elif current_time < self.executing_turn_until:
            status_text = f"STATUS: EXECUTING {self.executing_turn_name} TURN!"
            text_color = (255, 255, 0) 
        elif self.armed_turn:
            status_text = f"STATUS: PROXIMITY REACHED. LOOKING FOR {self.armed_turn} GAP"
            text_color = (0, 165, 255) 
        elif self.pending_turn:
            status_text = f"STATUS: SPOTTED {self.pending_turn}. TUNNEL VISION ACTIVE."
            text_color = (0, 255, 255) 

        cv2.rectangle(hud, (0, 0), (1000, 50), (0, 0, 0), -1)
        cv2.putText(hud, status_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

        self.current_hud = hud

    def display_cb(self):
        if self.current_hud is not None:
            try:
                cv2.imshow("CANBot Command Center", self.current_hud)
                cv2.waitKey(1)
            except:
                pass
            
            try:
                hud_msg = self.bridge.cv2_to_imgmsg(self.current_hud, encoding="bgr8")
                self.hud_pub.publish(hud_msg)
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousCANBot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()