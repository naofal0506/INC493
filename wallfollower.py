import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        
        # 1. การเชื่อมต่อ Topic
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # 2. พารามิเตอร์ (ปรับจูนความเสถียรที่นี่)
        self.target_dist = 0.25       # ระยะเป้าหมายห่างจากกำแพงขวา
        self.Kp = 0.15                # ความนุ่มนวลพวงมาลัย
        self.deadband = 0.05         # ช่วงระยะ "นิ่ง" (5 ซม.)
        self.base_speed = 0.7        # ความเร็วปกติ
        
        # 3. ระยะในการตัดสินใจ
        self.safe_front_dist = 1.0   # ระยะเริ่มระวัง
        self.diag_limit = 0.3       # ระยะมองเฉียงเข้าโค้ง
        self.critical_dist = 0.5     # 🚨 ระยะต้องถอย!
        
        self.max_steering = 0.5      # ลิมิตพวงมาลัย

        self.get_logger().info('🚀 ระบบ Wall Follower (Reverse Ready) เริ่มทำงานแล้ว!')

    def scan_callback(self, msg):
        # --- เตรียมข้อมูล Lidar ---
        ranges = msg.ranges
        clean_ranges = [r if not math.isinf(r) and not math.isnan(r) else 10.0 for r in ranges]
        num_points = len(clean_ranges)
        
        def get_min_range(start_deg, end_deg):
            start_idx = int((start_deg / 360.0) * num_points)
            end_idx = int((end_deg / 360.0) * num_points)
            slice_data = clean_ranges[start_idx:end_idx]
            return min(slice_data) if len(slice_data) > 0 else 10.0

        # วัดระยะ 3 จุด
        front_dist = min(get_min_range(0, 15), get_min_range(345, 360))
        right_dist = get_min_range(260, 280)
        diag_dist = get_min_range(300, 330)

        # เตรียมส่งคำสั่ง
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'

        # --- ส่วนของ Logic และการโชว์ข้อความ (Loggers) ---

        # กรณีที่ 1: 🔴 วิกฤต (ถอยหลัง)
        if front_dist < self.critical_dist:
            drive_msg.drive.speed = -0.5
            drive_msg.drive.steering_angle = -0.5  # หักท้ายหนีกำแพง
            self.get_logger().error(f'🚨 CRITICAL: Dist {front_dist:.2f}m - REVERSING!')

        # กรณีที่ 2: 🟡 เจอโค้ง/สิ่งกีดขวาง (ชะลอและเลี้ยวหลบ)
        elif front_dist < self.safe_front_dist or diag_dist < self.diag_limit:
            drive_msg.drive.speed = 0.5
            drive_msg.drive.steering_angle = 0.5   # หักซ้ายสุด
            self.get_logger().warning(f'🔄 CORNERING: Front={front_dist:.2f}m, Diag={diag_dist:.2f}m')

        # กรณีที่ 3: 🟢 เกาะกำแพงปกติ
        else:
            error = self.target_dist - right_dist
            
            # เช็คช่วงนิ่ง (Deadband)
            if abs(error) < self.deadband:
                steering = 0.0
                status_msg = "✅ STABLE (Straight)"
            else:
                steering = self.Kp * error
                status_msg = f"✨ FOLLOWING: Dist={right_dist:.2f}m, Err={error:.2f}"
            
            # ปรับความเร็วตามการเลี้ยว
            adaptive_speed = self.base_speed - (abs(steering) * 0.7)
            drive_msg.drive.speed = max(adaptive_speed, 0.6)
            drive_msg.drive.steering_angle = max(min(steering, self.max_steering), -self.max_steering)
            
            self.get_logger().info(status_msg)

        # ยิงคำสั่งไปที่รถ
        self.publisher.publish(drive_msg)

def main():
    rclpy.init()
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
