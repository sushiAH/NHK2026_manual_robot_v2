"""twistをsubscribeして、足回りesp32にモーター指令値を送信する
id 1 右下 正回転
id 2 左下 逆回転
id 3 右上 正回転
id 4 左上 逆回転

タイヤ直径100[mm]"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import atexit

#自作ライブラリ
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from ah_python_can import *


def from_twist_to_motor_vel(vx, vy, w, L, fy):
    V_1 = (vx - vy + 2 * math.sqrt(2) * w * L) / (4 * math.pi * fy)
    V_2 = (-vx - vy + 2 * math.sqrt(2) * w * L) / (4 * math.pi * fy)
    V_3 = (vx + vy + 2 * math.sqrt(2) * w * L) / (4 * math.pi * fy)
    V_4 = (-vx + vy + 2 * math.sqrt(2) * w * L) / (4 * math.pi * fy)

    return (V_1, V_2, V_3, V_4)


bus = can.interface.Bus(bustype="socketcan",
                        channel="can0",
                        asynchronous=True,
                        bitrate=1000000)


class TwistSubscriber(Node):

    def __init__(self):
        super().__init__("TwistSubscriber")

        # can立ち上げ
        set_enc_vel_mode(0x020, bus)
        set_enc_vel_mode(0x021, bus)
        set_enc_vel_mode(0x022, bus)
        set_enc_vel_mode(0x023, bus)

        #set_vel_pid_gain(0x020,0,0,0,bus)
        #set_vel_pid_gain(0x021,0,0,0,bus)
        #set_vel_pid_gain(0x022,0,0,0,bus)
        #set_vel_pid_gain(0x023,0,0,0,bus)

        self.subscription_twist_joy = self.create_subscription(
            Twist,  # メッセージの型
            "/cmd_vel_joy",  # 購読するトピック名
            self.twist_by_joy_callback,  # 呼び出すコールバック関数
            10,
        )  # キューサイズ(溜まっていく)
        self.subscription_twist_joy

        # --- Config ---
        # 車体横の長さ
        self.L = 0.8  #[m]
        # 車体中心からタイヤまでの距離
        self.fy = 0.55
        self.wheel_r = 0.05  #[mm]
        self.twist_gain = 2 * math.pi * self.wheel_r

        # メンバーの初期化
        self.joy_linear_x = 0
        self.joy_linear_y = 0
        self.joy_w = 0

        timer_period = 0.01
        # wirte_to_motorの割り込み設定
        self.timer = self.create_timer(timer_period, self.write_to_motor)

    def twist_by_joy_callback(self, msg):
        """subscribe twist message, store twist in member value

        Args:
            msg (Twist): [twist message]
        """
        self.joy_linear_x = msg.linear.x
        self.joy_linear_y = msg.linear.y
        self.joy_w = msg.angular.z

    def write_to_motor(self):
        """Twistをメカナムホイール逆運動学で、各モーターの速度指令値に分解。4つの速度指令値を一つのパケットでesp32に送信する"""
        vx = self.joy_linear_x
        vy = self.joy_linear_y
        w = self.joy_w

        V_1, V_2, V_3, V_4 = from_twist_to_motor_vel(vx, vy, w, self.L, self.fy)

        set_goal_vel(0x020, float(V_1 / self.twist_gain), bus)
        set_goal_vel(0x021, float(V_2 / self.twist_gain), bus)
        set_goal_vel(0x022, float(V_3 / self.twist_gain), bus)
        set_goal_vel(0x023, float(V_4 / self.twist_gain), bus)


def main():
    rclpy.init()  # rclpyライブラリの初期化

    twist_subscriber_node = TwistSubscriber()

    rclpy.spin(twist_subscriber_node)  # ノードをスピンさせる
    twist_subscriber_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


def stop():
    send_packet_1byte(0x020, 0, 0, bus)
    send_packet_1byte(0x021, 0, 0, bus)
    send_packet_1byte(0x022, 0, 0, bus)
    send_packet_1byte(0x023, 0, 0, bus)


atexit.register(stop)

if __name__ == "__main__":
    main()
