import rclpy
from rclpy.node import Node
import math

# --- [ใช้ Float32 สำหรับส่งค่าตัวเลขเดี่ยวๆ] ---
from std_msgs.msg import Float32

from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescStateStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class OdomERPMNode(Node):
    def __init__(self):
        super().__init__('odom_erpm_node')

        # 1. Subscribe Topics
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.vesc_sub = self.create_subscription(VescStateStamped, '/sensors/core', self.vesc_callback, 10)

        # 2. Publishers หลัก
        self.odom_pub = self.create_publisher(Odometry, '/custom_odomerpm', 10)
        
        # 🌟 เปลี่ยนชื่อ Topic รอยล้อเป็น /path_erpm จะได้ไม่ทับกับไฟล์อื่น
        self.path_pub = self.create_publisher(Path, '/path_erpm', 10)
        
        # --- [Publishers เสริม: ส่งค่าแบบเดี่ยวๆ ออกมาให้ดูง่ายและพล็อตกราฟได้] ---
        self.yaw_deg_pub = self.create_publisher(Float32, '/yaw_angle_deg', 10)       
        self.yaw_rad_pub = self.create_publisher(Float32, '/yaw_angle_rad', 10)       
        self.vel_pub = self.create_publisher(Float32, '/linear_velocity', 10)         
        self.omega_pub = self.create_publisher(Float32, '/angular_velocity', 10)      
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- [ระบบกรองสัญญาณ (Filter)] ---
        self.ds_buffer = []
        self.ds_window_size = 10
        self.omega_buffer = []
        self.omega_window_size = 15  
        self.deadband_value = 0.005   

        self.latest_v = 0.0      
        self.latest_omega = 0.0  
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.print_counter = 0

        self.last_time = self.get_clock().now()
        # ทำ Sampling Rate ที่ 50Hz (1 / 0.02 = 50)
        self.create_timer(0.02, self.timer_callback) 
        
        self.get_logger().info('🚀 Odometry Node: เริ่มทำงาน (เพิ่ม TF map -> odom ให้แล้ว!)')

    def imu_callback(self, msg):
        raw_omega = msg.angular_velocity.z
        if abs(raw_omega) < self.deadband_value:
            raw_omega = 0.0
            
        self.omega_buffer.append(raw_omega)
        if len(self.omega_buffer) > self.omega_window_size:
            self.omega_buffer.pop(0)
            
        self.latest_omega = sum(self.omega_buffer) / len(self.omega_buffer)

    def vesc_callback(self, msg):
        self.latest_v = msg.state.speed / 4076.51  

    def get_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return [0.0, 0.0, qz, qw]

    def timer_callback(self):
        now_time = self.get_clock().now()
        dt = (now_time - self.last_time).nanoseconds / 1e9
        self.last_time = now_time

        # --- 1. คำนวณ Kinematics ---
        self.theta = self.theta + (self.latest_omega * dt) 

        raw_ds = self.latest_v * dt
        self.ds_buffer.append(raw_ds)
        if len(self.ds_buffer) > self.ds_window_size:
            self.ds_buffer.pop(0)
        ds_filtered = sum(self.ds_buffer) / len(self.ds_buffer)

        self.x = self.x + (ds_filtered * math.cos(self.theta))
        self.y = self.y + (ds_filtered * math.sin(self.theta))

        # --- 2. แปลงมุมเป็นองศา ---
        heading_deg = math.degrees(self.theta) 

        # --- 3. [Publish ค่าต่างๆ ตรงๆ] ---
        yaw_deg_msg = Float32(); yaw_deg_msg.data = float(heading_deg)
        self.yaw_deg_pub.publish(yaw_deg_msg)

        yaw_rad_msg = Float32(); yaw_rad_msg.data = float(self.theta)
        self.yaw_rad_pub.publish(yaw_rad_msg)

        vel_msg = Float32(); vel_msg.data = float(self.latest_v)
        self.vel_pub.publish(vel_msg)

        omega_msg = Float32(); omega_msg.data = float(self.latest_omega)
        self.omega_pub.publish(omega_msg)

        # --- 4. ปริ้นท์ออกจอ (สำหรับดูประกอบ) ---
        self.print_counter += 1
        if self.print_counter >= 10:  
            self.get_logger().info(
                f"🎥 v: {self.latest_v:.2f} | w: {self.latest_omega:.2f} | "
                f"X: {self.x:.2f} | Y: {self.y:.2f} | Yaw: {heading_deg:.1f}°"
            )
            self.print_counter = 0

        q = self.get_quaternion(self.theta)

        # --- 5. ส่งข้อมูล Odometry ปกติ ---
        odom = Odometry()
        odom.header.stamp = now_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.latest_v
        odom.twist.twist.angular.z = self.latest_omega
        self.odom_pub.publish(odom)

        # ==========================================
        # 🌟 6. ส่งข้อมูล TF Tree (แบบมี Frame map) 🌟
        # ==========================================
        
        # 6.1 TF: odom -> base_link (รถขยับเมื่อเทียบกับจุดเริ่มต้น)
        tf_base = TransformStamped()
        tf_base.header = odom.header
        tf_base.child_frame_id = odom.child_frame_id
        tf_base.transform.translation.x = self.x
        tf_base.transform.translation.y = self.y
        tf_base.transform.rotation.z = q[2]
        tf_base.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf_base)

        # 6.2 TF: map -> odom (จุดกำเนิดของโลก ทับกันไปเลยที่ 0,0)
        tf_map = TransformStamped()
        tf_map.header.stamp = now_time.to_msg()
        tf_map.header.frame_id = 'map'       
        tf_map.child_frame_id = 'odom'       
        
        tf_map.transform.translation.x = 0.0
        tf_map.transform.translation.y = 0.0
        tf_map.transform.translation.z = 0.0
        
        tf_map.transform.rotation.x = 0.0
        tf_map.transform.rotation.y = 0.0
        tf_map.transform.rotation.z = 0.0
        tf_map.transform.rotation.w = 1.0    
        
        self.tf_broadcaster.sendTransform(tf_map)

        # --- 7. วาดเส้น Path ---
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path.poses.append(pose)
        if len(self.path.poses) > 1000: self.path.poses.pop(0) 
        self.path_pub.publish(self.path)

def main():
    rclpy.init()
    node = OdomERPMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
