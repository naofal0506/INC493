import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class F1TenthTeleop(Node):
    def __init__(self):
        super().__init__('f1tenth_joy_teleop')

        # แนะนำให้ใช้ /drive เพื่อให้ตรงกับมาตรฐาน F1TENTH ส่วนใหญ่
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)     

        # Subscriber รับค่าจาก Joy Node
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # การตั้งค่าปุ่ม (Index ของจอย Xbox/PS4 ปกติ)
        self.RB_BUTTON = 5   # ปุ่ม RB สำหรับเปิดใช้งาน
        self.STEER_AXIS = 3  # แกน Analog ซ้าย (ซ้าย-ขวา)
        self.SPEED_AXIS = 1  # แกน Analog ซ้าย (บน-ล่าง)

        # ขีดจำกัดความเร็ว
        self.max_speed = 2.0  # m/s
        self.max_steer = 0.34 # rad
        
        # [แก้แล้ว] ย่อหน้าเข้ามาให้อยู่ใน __init__
        self.get_logger().info("F1Tenth Joy Teleop Started! Hold RB to drive.")

    def joy_callback(self, msg):
        drive_msg = AckermannDriveStamped()
        
        # เพิ่ม stamp ให้สมบูรณ์
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"

        # ป้องกันกรณีจอยส่งปุ่มมาไม่ครบ index
        if len(msg.buttons) > self.RB_BUTTON:
            if msg.buttons[self.RB_BUTTON] == 1:
                # คำนวณความเร็วและการเลี้ยว
                drive_msg.drive.speed = msg.axes[self.SPEED_AXIS] * self.max_speed
                drive_msg.drive.steering_angle = msg.axes[self.STEER_AXIS] * self.max_steer
            else:
                drive_msg.drive.speed = 0.0
                drive_msg.drive.steering_angle = 0.0

        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = F1TenthTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

# [แก้แล้ว] ต้องใช้ __ (ขีดล่าง 2 อัน)
if __name__ == '__main__':
    main()
