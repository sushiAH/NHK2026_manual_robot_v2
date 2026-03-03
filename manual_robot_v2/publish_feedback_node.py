"""IMU,計測輪から取得したデータをpublishする"""

import os
import sys
import rclpy
from rclpy.node import Node
import math
import numpy as np
from tf2_ros import TransformBroadcaster

# メッセージ型
from std_msgs.msg import String
from sensor_msgs.msg import Joy, Imu
from geometry_msgs.msg import TransformStamped, Twist, Quaternion
from nav_msgs.msg import Odometry

#自作ライブラリ
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from recv_feedback import *


def calc_delta_odometry(x_vel, y_vel, theta, ang_z_vel, dt):
    delta_x = (x_vel * math.cos(theta) - y_vel * math.sin(theta)) * dt
    delta_y = (x_vel * math.sin(theta) + y_vel * math.cos(theta)) * dt
    delta_theta = ang_z_vel * dt

    return (delta_x, delta_y, delta_theta)


class FeedbackPublisher(Node):

    def __init__(self):
        super().__init__("feedback_publisher")

        # --- パラメータ設定 ----
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("wheel_radius", 0.1)

        self.imu_frame_id = self.get_parameter("imu_frame_id").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.wheel_radius = self.get_parameter("wheel_radius").value

        # --- publisherの設定 ---
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

        # recv_feedbackの割り込み設定
        self.recv_feedback_timer = self.create_timer(0.005, self.recv_feedback)
        self.publish_timer = self.create_timer(0.005, self.publish_feedback)

        # esp32 serialの設定
        self.ser = serial.Serial(port="/dev/ttyACM0", baudrate=115200)

        # メンバ変数初期化
        self.mcu_timestamp_millis = 0

        self.enc_x_vel = 0.0
        self.enc_y_vel = 0.0

        self.q_w = 1.0
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0

        self.ang_x_vel = 0.0
        self.ang_y_vel = 0.0
        self.ang_z_vel = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

    def recv_feedback(self):
        struct_format = "<BIfffffffffB"
        packet = receive_packet(struct_format, self.ser)

        if packet != None:
            self.mcu_timestamp_millis = packet[1]

            self.q_w = packet[2]
            self.q_x = packet[3]
            self.q_y = packet[4]
            self.q_z = packet[5]

            self.ang_x_vel = packet[6]
            self.ang_y_vel = packet[7]
            self.ang_z_vel = packet[8]

            self.enc_x_vel = packet[9]
            self.enc_y_vel = packet[10]

    def publish_feedback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # odomの配信
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        # odomの計算
        delta_x, delta_y, delta_theta = calc_delta_odometry(
            self.enc_x_vel, self.enc_y_vel, self.theta, self.ang_z_vel, dt)

        self.x += delta_x * (2 * math.pi / 60.0) * self.wheel_radius
        self.y += delta_y * (2 * math.pi / 60.0) * self.wheel_radius
        self.theta += delta_theta

        # 位置情報
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # 共分散(位置)
        odom.pose.covariance = [
            1e-3,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,  # x
            0.0,
            1e-3,
            0.0,
            0.0,
            0.0,
            0.0,  # y
            0.0,
            0.0,
            1e-6,
            0.0,
            0.0,
            0.0,  # z
            0.0,
            0.0,
            0.0,
            1e-6,
            0.0,
            0.0,  # roll
            0.0,
            0.0,
            0.0,
            0.0,
            1e-6,
            0.0,  # pitch
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1e-3,  # yaw
        ]

        # 速度情報
        odom.twist.twist.linear.x = self.enc_x_vel
        odom.twist.twist.linear.y = self.enc_y_vel
        odom.twist.twist.angular.z = self.ang_z_vel

        # 共分散(Twist)
        odom.twist.covariance = [
            1e-3,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,  # x速度の信頼度
            0.0,
            1e-3,
            0.0,
            0.0,
            0.0,
            0.0,  # y速度の信頼度
            0.0,
            0.0,
            1e-6,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1e-6,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1e-6,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1e-3,  # yaw角速度の信頼度
        ]

        self.odom_pub.publish(odom)

        # imuの配信
        imu = Imu()
        imu.header.stamp = current_time.to_msg()
        imu.header.frame_id = self.imu_frame_id

        # 姿勢(Quaternion)
        imu.orientation.w = self.q_w
        imu.orientation.x = self.q_x
        imu.orientation.y = self.q_y
        imu.orientation.z = self.q_z

        # 共分散行列
        # 本来はセンサーのスペックから設定
        imu.orientation_covariance = [
            1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6
        ]

        # 角速度(Aungular Vel)
        imu.angular_velocity.x = self.ang_x_vel
        imu.angular_velocity.y = self.ang_y_vel
        imu.angular_velocity.z = self.ang_z_vel
        imu.angular_velocity_covariance = [
            1e-6,
            0.0,
            0.0,
            0.0,
            1e-6,
            0.0,
            0.0,
            0.0,
            1e-6,
        ]

        self.imu_pub.publish(imu)

        # tfはsensor fusionパッケージで出力する


def main():
    rclpy.init()  # rclpyライブラリの初期化

    feedback_publisher_node = FeedbackPublisher()
    rclpy.spin(feedback_publisher_node)  # ノードをスピンさせる
    feedback_publisher_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
