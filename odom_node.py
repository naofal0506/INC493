import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        
        # Subscribe ค่าจาก IMU ที่คุณเพิ่งแคปจอไป
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # Publisher สำหรับ Odom และ Path
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        
        # TF Broadcaster (odom -> base_link)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ตัวแปรตำแหน่งพื้นฐาน
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.1  # ความเร็วคงที่ 0.1 m/s
        
        self.last_time = self.get_clock().now()
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

    def imu_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # ดึงค่าการหมุนแกน Z จาก IMU ของคุณ
        omega_z = msg.angular_velocity.z

        # --- คำนวณ Kinematics ตามสไลด์หน้า 7 ---
        self.theta += omega_z * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        # --- ส่ง TF (odom -> base_link) ---
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        
        # แปลงมุม theta เป็น Quaternion
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # --- ส่ง Path ให้ RViz วาดเส้น ---
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        self.path_msg.poses.append(pose)
        
        # เก็บแค่ 500 จุดพอ เดี๋ยวคอมค้าง
        if len(self.path_msg.poses) > 500:
            self.path_msg.poses.pop(0)
        
        self.path_pub.publish(self.path_msg)

def main():
    rclpy.init()
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
