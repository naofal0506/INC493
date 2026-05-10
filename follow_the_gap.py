import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math

class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap_node')
        
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # ==========================================
        # 🏎️ SETTINGS (แก้บัคชนกำแพงโค้ง + กันวิ่งย้อนศร)
        # ==========================================
        self.bubble_radius = 0.2     # เผื่อระยะสีข้างนิดนึง
        self.clip_distance = 1.5       # 🌟 เพิ่มให้มองไกลขึ้น จะได้เห็นกำแพงตรงโค้งแต่เนิ่นๆ
        self.fov_degrees = 160.0       # 🌟 บีบสายตาลงมาที่ 140 องศา (ไม่ให้มองเห็นทางข้างหลัง)
        
        # 🚨 ระบบถอยหลัง (K-Turn)
        self.critical_dist = 0.35     
        self.clear_dist = 0.45          
        self.is_reversing = False     
        
        self.Kp = 0.6                  # เพิ่มความไวพวงมาลัย ให้เลี้ยวโค้งทัน
        self.Kd = 0.2                  
        self.prev_error = 0.0         
        
        self.max_speed = 0.8         
        self.min_speed = 0.4           
        self.reverse_speed = -0.4     
        self.max_steering = 1.0       
        self.steering_direction = 1.0  

        self.get_logger().info('🏁 F1TENTH: Anti-Loop & Active Cornering พร้อมลุย!!')

    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_inc = msg.angle_increment
        
        # --- Step 0: Pre-processing & Clipping ---
        start_angle = -self.fov_degrees / 2.0
        end_angle = self.fov_degrees / 2.0
        
        raw_data = []
        for i, r in enumerate(ranges):
            angle = i * angle_inc
            if angle > math.pi: angle -= 2 * math.pi
            angle_deg = math.degrees(angle)
            
            if start_angle <= angle_deg <= end_angle:
                if math.isinf(r) or math.isnan(r) or r == 0.0:
                    raw_data.append((angle, self.clip_distance)) 
                else:
                    raw_data.append((angle, min(r, self.clip_distance)))
        
        if not raw_data: return
        raw_data.sort(key=lambda x: x[0])
        proc_ranges = [item[1] for item in raw_data]
        angles = [item[0] for item in raw_data]

        # อ่านระยะรอบทิศทาง
        front_dists = [r for a, r in raw_data if -20 <= math.degrees(a) <= 20]
        left_diag_dists = [r for a, r in raw_data if 20 < math.degrees(a) <= 60]
        right_diag_dists = [r for a, r in raw_data if -60 <= math.degrees(a) < -20]
        
        front_dist = min(front_dists) if front_dists else self.clip_distance
        left_diag = min(left_diag_dists) if left_diag_dists else self.clip_distance
        right_diag = min(right_diag_dists) if right_diag_dists else self.clip_distance

        # --- 1. ระบบ K-Turn (เอาตัวรอดจากการชน) ---
        if self.is_reversing:
            if front_dist > self.clear_dist and left_diag > 0.4 and right_diag > 0.4:
                self.is_reversing = False
                self.get_logger().info('🟢 รอดแล้ว ลุยไปข้างหน้าต่อ!')
            else:
                # สะบัดหน้ารถไปทางที่กว้างกว่า
                rev_steer = -self.max_steering if left_diag > right_diag else self.max_steering
                self.publish_drive(self.reverse_speed, rev_steer * self.steering_direction)
                return 

        if front_dist < self.critical_dist or left_diag < 0.25 or right_diag < 0.25:
            self.get_logger().warning('🛑 จวนตัวแล้ว! ทิ่มกำแพง เริ่มถอย!')
            self.is_reversing = True
            return

        # --- 2. 🌟 Active Cornering (ชิงเลี้ยวโค้งก่อนชนกำแพง) 🌟 ---
        corner_assist = 0.0
        if front_dist < 1.2: 
            # ถ้าหน้ารถเริ่มตัน (เห็นกำแพงหน้าโค้ง) ให้เลี้ยวหาท่อลมที่เปิดกว้างทันที!
            if left_diag > right_diag + 0.3:
                corner_assist = 0.4  # ซ้ายโล่งกว่า (เป็นท่อลม) -> หักซ้ายช่วยแรงๆ
            elif right_diag > left_diag + 0.3:
                corner_assist = -0.4 # ขวาโล่งกว่า (เป็นท่อลม) -> หักขวาช่วยแรงๆ

        # --- 3. Safety Bubble ---
        min_dist = min(proc_ranges)
        if min_dist >= self.clip_distance - 0.05:
            self.publish_drive(self.max_speed, 0.0)
            return
            
        min_idx = proc_ranges.index(min_dist)
        
        try:
            bubble_angle = math.asin(self.bubble_radius / min_dist)
        except ValueError:
            bubble_angle = math.pi / 2 
            
        bubble_indices = int(bubble_angle / angle_inc)
        for i in range(len(proc_ranges)):
            if abs(i - min_idx) <= bubble_indices:
                proc_ranges[i] = 0.0
                
        # --- 4. หาช่องว่าง ---
        valid_gaps = []
        start_i = -1
        for i in range(len(proc_ranges)):
            if proc_ranges[i] > 0.1: 
                if start_i == -1: start_i = i
            else:
                if start_i != -1:
                    valid_gaps.append((start_i, i - 1))
                    start_i = -1
        if start_i != -1: valid_gaps.append((start_i, len(proc_ranges) - 1))

        if not valid_gaps:
            self.is_reversing = True
            return

        # --- 5. 🌟 Anti-U-Turn Filter (ป้องกันรถวิ่งย้อนศร) 🌟 ---
        best_gap = None
        max_width = -1
        for gap in valid_gaps:
            gap_start, gap_end = gap
            gap_width = gap_end - gap_start
            
            # หาจุดกึ่งกลางของช่องนี้
            mid_idx = int((gap_start + gap_end) / 2)
            gap_angle = math.degrees(angles[mid_idx])
            
            # 🚨 กรองช่องที่อยู่ด้านข้างเกิน 55 องศาทิ้งไป! (บังคับให้มองไปข้างหน้าเสมอ)
            if abs(gap_angle) > 55.0:
                continue

            if gap_width > max_width:
                max_width = gap_width
                best_gap = gap
            
        # ถ้าเผลอกรองทิ้งหมด (ทางตันชั่วคราว) ให้ใช้ช่องเดิมที่กว้างสุดไปก่อน
        if best_gap is None:
            best_gap = max(valid_gaps, key=lambda g: g[1] - g[0])

        goal_idx = int((best_gap[0] + best_gap[1]) / 2)
        target_angle = angles[goal_idx]
        
        # --- 6. PD Controller ---
        error = target_angle
        derivative = error - self.prev_error
        
        # ผสมพวงมาลัยปกติ เข้ากับ แรงดึงจาก Active Cornering
        raw_steering = (self.Kp * error) + (self.Kd * derivative) + corner_assist
        
        steering_angle = raw_steering * self.steering_direction 
        self.prev_error = error
        
        # Adaptive Speed
        adaptive_speed = self.max_speed - (abs(steering_angle) * 0.7)
        speed = max(adaptive_speed, self.min_speed)
        
        steering_angle = max(min(steering_angle, self.max_steering), -self.max_steering)
        self.publish_drive(speed, steering_angle)

    def publish_drive(self, speed, steering):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(steering)
        self.publisher.publish(drive_msg)

def main():
    rclpy.init()
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
