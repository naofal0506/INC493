import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import smbus2 as smbus
import time
import math

class BMI160FullNode(Node):
    def __init__(self):
        super().__init__('bmi160_full_node')
        
        # 1. Publisher สำหรับ Topic /imu
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        
        # 2. TF Broadcaster (base_link -> imu_link)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Setup I2C (Address 0x69)
        self.bus = smbus.SMBus(1)
        self.addr = 0x69
        self.init_bmi160()
        
        # Timer รันที่ 20Hz (0.05s)
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info('BMI160 Reading ALL 6 AXES (XYZ)...')

    def init_bmi160(self):
        # เปิดโหมดทำงานปกติทั้ง Accel และ Gyro
        try:
            self.bus.write_byte_data(self.addr, 0x7E, 0x11) # Accel Normal Mode
            time.sleep(0.05)
            self.bus.write_byte_data(self.addr, 0x7E, 0x15) # Gyro Normal Mode
            time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f'I2C Error: {e}')

    def read_word(self, reg):
        # อ่านค่า 16-bit และจัดการค่าติดลบ (Two's Complement)
        low = self.bus.read_byte_data(self.addr, reg)
        high = self.bus.read_byte_data(self.addr, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000: value -= 65536
        return value

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = 'imu_link'

        # --- อ่านค่า Gyroscope (XYZ) ---
        # Register: 0x0C (X), 0x0E (Y), 0x10 (Z)
        gx = self.read_word(0x0C)
        gy = self.read_word(0x0E)
        gz = self.read_word(0x10)
        
        # แปลงเป็น rad/s (สำหรับช่วง +/- 2000 deg/s)
        msg.angular_velocity.x = (gx / 16.4) * (math.pi / 180.0)
        msg.angular_velocity.y = (gy / 16.4) * (math.pi / 180.0)
        msg.angular_velocity.z = (gz / 16.4) * (math.pi / 180.0)

        # --- อ่านค่า Accelerometer (XYZ) ---
        # Register: 0x12 (X), 0x14 (Y), 0x16 (Z)
        ax = self.read_word(0x12)
        ay = self.read_word(0x14)
        az = self.read_word(0x16)
        
        # แปลงเป็น m/s^2 (สำหรับช่วง +/- 2g)
        msg.linear_acceleration.x = (ax / 16384.0) * 9.81
        msg.linear_acceleration.y = (ay / 16384.0) * 9.81
        msg.linear_acceleration.z = (az / 16384.0) * 9.81

        # Publish ข้อมูลออกไป
        self.imu_pub.publish(msg)

        # --- ส่ง TF (base_link -> imu_link) ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.05
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = BMI160FullNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
