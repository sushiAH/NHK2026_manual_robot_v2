from rclpy.node import Node
import rclpy
from std_msgs.msg import String
import math
import numpy as np
import atexit
from sensor_msgs.msg import Joy

#自作ライブラリ
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from ah_python_can import *
from dyna_interfaces.msg import DynaFeedback, DynaTarget

bus = can.interface.Bus(bustype="socketcan",
                        channel="can0",
                        asynchronous=True,
                        bitrate=1000000)


class BoxController(Node):

    def __init__(self):
        super().__init__("box_spear_controller")

        self.subscription_joy = self.create_subscription(
            Joy,  # メッセージの型
            "/joy",  # 購読するトピック名
            self.joy_callback,  # 呼び出すコールバック関数
            10,
        )  # キューサイズ(溜まっていく)
        self.subscription_joy

        # publisherの設定
        self.dyna_pos_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_pos", 10)

        # box hand up down
        send_packet_1byte(0x020, 0, 2, bus)
        send_packet_4byte(0x020, 6, 5, bus)

        self.state_counter = 0
        self.last_button_state = 0
        self.now_button_state = 0

    def publish_dyna_pos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_pos_publisher.publish(msg)

    def update_box_arm_state(self):
        # 取る状態のパターン
        state_length = 5
        """やり回収機構の状態を更新"""
        if self.now_button_state == 1 and self.last_button_state == 0:
            self.state_counter += 1

        self.state_counter = self.state_counter % state_length
        self.last_button_state = self.now_button_state

    def move_box_arm(self):
        """state_counterに従って動作"""

        if self.state_counter == 0:
            self.publish_dyna_pos(3, 3000)  # 縮む
            self.publish_dyna_pos(4, 2100)  # ハンド開く
            send_packet_4byte(0x020, 1, 1300, bus)  # 昇降　下

        elif self.state_counter == 1:
            self.publish_dyna_pos(3, 1500)  # 伸びる

        elif self.state_counter == 2:
            self.publish_dyna_pos(4, 1400)  # ハンド閉じる

        elif self.state_counter == 3:
            send_packet_4byte(0x020, 1, 800, bus)  #昇降　上

        elif self.state_counter == 4:
            self.publish_dyna_pos(4, 2100)  # ハンド閉じる

    def joy_callback(self, msg):
        """joyを受取、各機構を動作

        Args:
            msg (Joy): joy_stick_message
        """
        self.now_button_state = msg.buttons[1]
        self.update_box_arm_state()
        self.move_box_arm()


def main():
    rclpy.init()  # rclpyライブラリの初期化

    box_controller_node = BoxController()

    rclpy.spin(box_controller_node)  # ノードをスピンさせる
    box_controller_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


def stop():
    send_packet_1byte(0x020, 0, 0, bus)
    send_packet_1byte(0x021, 0, 0, bus)
    send_packet_1byte(0x022, 0, 0, bus)
    send_packet_1byte(0x023, 0, 0, bus)


atexit.register(stop)

if __name__ == "__main__":
    main()
