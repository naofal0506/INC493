import rclpy
from rclpy.node import Node
import math

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

        # 2. Publishers
        self.odom_pub = self.create_publisher(Odometry, '/custom_odomerpm', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- [ส่วนที่เพิ่ม: ระบบกรองสัญญาณเพื่อความนิ่ง] ---
        # สำหรับระยะทาง (ds)
        self.ds_buffer = []
        self.ds_window_size = 10
        
        # สำหรับมุม (Heading/Omega)
        self.omega_buffer = []
        self.omega_window_size = 15  # เพิ่มขนาดหน้าต่างให้กว้างขึ้นเพื่อความนิ่ง
        self.deadband_value = 0.02   # ค่าความละเอียด (ถ้าหมุนน้อยกว่านี้ให้ถือว่าไม่หมุน)
        # --------------------------------------------

        self.latest_v = 0.0      
        self.latest_omega = 0.0  
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.last_time = self.get_clock().now()
        self.create_timer(0.02, self.timer_callback) 
        
        self.get_logger().info('🚀 Odometry Node: Heading & ds นิ่งๆ พร้อมแล้วเพื่อน!')

    def imu_callback(self, msg):
        # 1. รับค่า Raw และกรองด้วย Deadband ก่อน (กันไหลตอนจอด)
        raw_omega = msg.angular_velocity.z
        if abs(raw_omega) < self.deadband_value:
            raw_omega = 0.0
            
        # 2. ทำ Moving Average ให้ค่า Omega
        self.omega_buffer.append(raw_omega)
        if len(self.omega_buffer) > self.omega_window_size:
            self.omega_buffer.pop(0)
            
        self.latest_omega = sum(self.omega_buffer) / len(self.omega_buffer)

    def vesc_callback(self, msg):
        # แปลง ERPM เป็นความเร็ว (ใช้ค่า 4614.0 ตามเดิมของคุณ)
        self.latest_v = msg.state.speed / 3747.1 

    def get_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return [0.0, 0.0, qz, qw]

    def timer_callback(self):
        now_time = self.get_clock().now()
        dt = (now_time - self.last_time).nanoseconds / 1e9
        self.last_time = now_time

        # --- คำนวณ Kinematics ---
        
        # 1. อัปเดตมุม (ใช้ Omega ที่ผ่านการกรองจนนิ่งแล้ว)
        self.theta = self.theta + (self.latest_omega * dt) 

        # 2. คำนวณระยะทาง ds และกรอง Moving Average
        raw_ds = self.latest_v * dt
        self.ds_buffer.append(raw_ds)
        if len(self.ds_buffer) > self.ds_window_size:
            self.ds_buffer.pop(0)
        ds_filtered = sum(self.ds_buffer) / len(self.ds_buffer)

        # 3. อัปเดตพิกัด X, Y
        self.x = self.x + (ds_filtered * math.cos(self.theta))
        self.y = self.y + (ds_filtered * math.sin(self.theta))

        q = self.get_quaternion(self.theta)

        # --- ส่งข้อมูล Odometry ---
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

        # --- ส่งข้อมูล TF ---
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
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
    node = OdomERPMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
