import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescStateStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTachoFilteredNode(Node):
    def __init__(self):
        super().__init__('odom_tacho_filtered_node')

        # เปลี่ยนเป็น /imu/data ตามระบบของคุณแล้ว
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.vesc_sub = self.create_subscription(VescStateStamped, '/sensors/core', self.vesc_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/custom_odomtacho', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- 1. สเปกหุ่นยนต์ (Traxxas 3351R) ---
        self.latest_tachometer = 0.0
        self.prev_tachometer = None
        
        wheel_diameter = 0.10  # เมตร (อย่าลืมวัดล้อจริงแล้วมาแก้ตัวเลขนี้)
        pole_pairs = 2.0       # ของมอเตอร์ Traxxas 3351R
        gear_ratio = 9.81       # อัตราทดเกียร์ (เฟืองใหญ่ / เฟืองเล็ก)
        
        self.K = (math.pi * wheel_diameter) / (pole_pairs * gear_ratio * 6.0)

        # --- 2. ตัวแปรสำหรับ Low-Pass Filter (ลดแกว่ง) ---
        self.alpha_omega = 0.2  # ค่าความนิ่งของ IMU (0.0 - 1.0) ยิ่งน้อยยิ่งนิ่ง แต่ตอบสนองช้า
        self.alpha_v = 0.3      # ค่าความนิ่งของความเร็ว
        self.filtered_omega = 0.0
        
        self.latest_omega = 0.0  
        self.latest_v = 0.0 
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.last_time = self.get_clock().now()
        self.create_timer(0.02, self.timer_callback) 
        
        self.get_logger().info('🌟 เริ่มต้น Odometry (มีระบบกันแกว่ง Low-Pass Filter)')

    def imu_callback(self, msg):
        raw_omega = msg.angular_velocity.z 
        
        # 🛡️ ระบบกันแกว่งที่ 1: Low-Pass Filter (ทำสมูทตอนวิ่ง)
        self.filtered_omega = (self.alpha_omega * raw_omega) + ((1.0 - self.alpha_omega) * self.filtered_omega)
        
        # 🛑 ระบบกันแกว่งที่ 2: Deadband (ตัดให้เป็น 0 ตอนจอดนิ่ง)
        if abs(self.filtered_omega) < 0.03:
            self.latest_omega = 0.0
        else:
            self.latest_omega = self.filtered_omega

    def vesc_callback(self, msg):
        self.latest_tachometer = msg.state.displacement

    def get_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return [0.0, 0.0, qz, qw]

    def timer_callback(self):
        now_time = self.get_clock().now()
        dt = (now_time - self.last_time).nanoseconds / 1e9
        self.last_time = now_time

        if self.prev_tachometer is None:
            self.prev_tachometer = self.latest_tachometer
            return

        # --- Kinematics Calculation ---
        # อัปเดตมุม (ใช้ค่า IMU ที่ผ่านการกรองจนนิ่งแล้ว)
        self.theta = self.theta + (self.latest_omega * dt) 

        # คำนวณระยะทางจากล้อ (Tachometer)
        delta_tacho = self.latest_tachometer - self.prev_tachometer
        self.prev_tachometer = self.latest_tachometer 
        
        ds = -(delta_tacho * self.K)  

        # --- กรองความเร็ว v เพื่อโชว์อาจารย์ให้นิ่งๆ ---
        raw_v = ds / dt if dt > 0 else 0.0
        self.latest_v = (self.alpha_v * raw_v) + ((1.0 - self.alpha_v) * self.latest_v)
        
        # ถ้าความเร็วน้อยมาก ให้ถือว่าจอดนิ่ง
        if abs(self.latest_v) < 0.01:
            self.latest_v = 0.0

        # อัปเดตพิกัด
        self.x = self.x + (ds * math.cos(self.theta))
        self.y = self.y + (ds * math.sin(self.theta))

        q = self.get_quaternion(self.theta)

        # --- สร้างและส่งข้อมูล Odometry ---
        odom = Odometry()
        odom.header.stamp = now_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.latest_v
        odom.twist.twist.angular.z = self.latest_omega
        self.odom_pub.publish(odom)

        # --- ส่งข้อมูล TF ---
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf)

        # --- วาดเส้น Path ---
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path.poses.append(pose)
        if len(self.path.poses) > 1000: self.path.poses.pop(0) 
        self.path_pub.publish(self.path)

def main():
    rclpy.init()
    node = OdomTachoFilteredNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
