"""joystickをsubscribeして、twistをpublishする"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy, Imu
import math
import numpy as np
import tf_transformations
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import *
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


def rot(vec, theta):
    vec = vec.reshape(-1, 1)

    cos, sin = np.cos(theta), np.sin(theta)

    R = np.array([
        [cos, -sin],
        [sin, cos],
    ])

    return R @ vec


def calc_p_value(target, current, p_gain):
    output = (target - current) * p_gain

    return output


class TwistPublisher(Node):
    """subscribe joystick and publish twist message

    Attributes:
        subscription_joy: instance of subscribe
        twist_publisher:  instance of publish
    """

    def __init__(self):
        super().__init__("twist_publisher")

        self.subscription_joy = self.create_subscription(
            Joy,  # メッセージの型
            "/joy",  # 購読するトピック名
            self.joy_callback,  # 呼び出すコールバック関数
            10,
        )  # キューサイズ(溜まっていく)
        self.subscription_joy

        self.subscription_imu = self.create_subscription(
            Imu,
            "imu/data",
            self.imu_callback,
            10,
        )
        self.subscription_imu

        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel_joy", 10)

        self.yaw_rad = 0

    def joy_callback(self, msg):

        axes_values = msg.axes

        twist = Twist()

        if abs(axes_values[1]) < 0.1:
            axes_values[1] = 0
        if abs(axes_values[0]) < 0.1:
            axes_values[0] = 0
        if abs(axes_values[2]) < 0.1:
            axes_values[2] = 0

        v = np.array([-axes_values[0] * 2, -axes_values[1] * 2])
        Rv = rot(v, self.yaw_rad)
        twist.linear.y = float(Rv[0])
        twist.linear.x = float(Rv[1])
        twist.angular.z = -axes_values[2]

        self.twist_publisher.publish(twist)

    def imu_callback(self, msg):
        q = msg.orientation

        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_rad = euler[2]
        self.get_logger().info(f"値:{self.yaw_rad}")


def main():
    rclpy.init()  # rclpyライブラリの初期化

    twist_publisher_node = TwistPublisher()

    rclpy.spin(twist_publisher_node)  # ノードをスピンさせる
    twist_publisher_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
