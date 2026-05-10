import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from BMI160_i2c import Driver
import math

# 🌟 1. เปลี่ยน Import จาก Joy เป็น AckermannDriveStamped
from ackermann_msgs.msg import AckermannDriveStamped

class LabTaskNode(Node):
    def __init__(self):
        super().__init__('lab_task_node')
        
        try:
            self.sensor = Driver(0x69, 1)
            self.get_logger().info('✅ BMI160 Connected! เซนเซอร์พร้อม')
        except Exception as e:
            self.get_logger().error(f'❌ ไม่สามารถเชื่อมต่อเซนเซอร์ได้: {e}')

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 🌟 2. เปลี่ยน Subscriber ไปฟังช่อง /drive ที่ส่งไปหามอเตอร์ VESC
        self.drive_sub = self.create_subscription(AckermannDriveStamped, '/drive', self.drive_callback, 10)
        self.get_logger().info('🚗 รอรับค่าความเร็วจากช่อง /drive...')

        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.current_v = 0.0  
        
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        self.last_time = self.get_clock().now()
        self.create_timer(0.1, self.timer_callback)

    # 🌟 3. ฟังก์ชันใหม่: ดึงความเร็ว (m/s) จาก AckermannDrive 
    def drive_callback(self, msg):
        # ดึงความเร็วที่ถูกคำนวณแล้ว (เช่น กดเต็มแม็กซ์จะได้ 2.0 m/s) มาใช้เลย
        self.current_v = msg.drive.speed

    def get_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def timer_callback(self):
        try:
            data = self.sensor.getMotion6() 
        except Exception:
            return

        now_time = self.get_clock().now()
        now = now_time.to_msg()
        dt = (now_time - self.last_time).nanoseconds / 1e9
        self.last_time = now_time

        gx, gy, gz = [math.radians(d / 131.0) for d in data[:3]]
        ax, ay, az = [d / 16384.0 * 9.81 for d in data[3:]]

        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        
        # การหมุนซ้าย/ขวาใน RViz ยังคงใช้ IMU จริงที่ติดอยู่บนรถ
        self.yaw += gz * dt 
        q = self.get_quaternion(roll, pitch, self.yaw)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'base_link' 
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = q
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gx, gy, gz
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = ax, ay, az
        self.imu_pub.publish(imu)

        # อัปเดตตำแหน่งตามความเร็วรถจริง + ทิศทางที่หันจริง
        self.x += self.current_v * math.cos(self.yaw) * dt
        self.y += self.current_v * math.sin(self.yaw) * dt

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.current_v
        odom.twist.twist.angular.z = gz
        self.odom_pub.publish(odom)

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

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path.poses.append(pose)
        if len(self.path.poses) > 500: self.path.poses.pop(0) 
        self.path_pub.publish(self.path)

def main():
    rclpy.init()
    node = LabTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
