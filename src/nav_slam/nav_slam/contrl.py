
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import termios, sys, tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = 1.0  # 初始线速度 (m/s)
        self.angular_speed = 0.9  # 初始角速度 (rad/s)
        self.twist = Twist()
        self.get_logger().info("Keyboard Teleop Node Started. Use WASD/Arrow Keys, 1/2 (speed), 4/5 (turn).")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            # 方向控制
            if key == 'w' or key == '\x1b[A':  # 上箭头
                self.twist.linear.x = self.linear_speed
                self.twist.angular.z = 0.0
            elif key == 's' or key == '\x1b[B':  # 下箭头
                self.twist.linear.x = -self.linear_speed
                self.twist.angular.z = 0.0
            elif key == 'a' or key == '\x1b[D':  # 左箭头
                self.twist.angular.z = self.angular_speed
                self.twist.linear.x = 0.0
            elif key == 'd' or key == '\x1b[C':  # 右箭头
                self.twist.angular.z = -self.angular_speed
                self.twist.linear.x = 0.0
            # 速度调节
            elif key == '1':
                self.linear_speed += 0.2
                self.get_logger().info(f"Linear Speed: {self.linear_speed:.1f} m/s")
            elif key == '2':
                self.linear_speed = max(0.1, self.linear_speed - 0.1)
                self.get_logger().info(f"Linear Speed: {self.linear_speed:.1f} m/s")
            # 转角调节
            elif key == '4':
                self.angular_speed += 0.1
                self.get_logger().info(f"Angular Speed: {self.angular_speed:.1f} rad/s")
            elif key == '5':
                self.angular_speed = max(0.1, self.angular_speed - 0.1)
                self.get_logger().info(f"Angular Speed: {self.angular_speed:.1f} rad/s")
            # 停止
            elif key == ' ':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
            # 退出
            elif key == 'q':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.publisher.publish(self.twist)
                self.get_logger().info("Exiting...")
                break
            # 发布指令
            self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()