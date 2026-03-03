import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import numpy as np

import atexit
from sensor_msgs.msg import Joy
import time

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


class SpearController(Node):

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

        # air
        send_packet_1byte(0x021, 0, 5, bus)  # set_operating

        self.state_counter = 0
        self.last_button_state = 0
        self.now_button_state = 0

        # 取る状態のパターン
        self.state_length = 4

    def publish_dyna_pos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_pos_publisher.publish(msg)

    def update_spear_state(self):
        """やり回収機構の状態を更新"""
        if self.now_button_state == 1 and self.last_button_state == 0:
            self.state_counter += 1

        self.state_counter = self.state_counter % self.state_length
        self.last_button_state = self.now_button_state

    def move_spear(self):
        """state_counterに従って動作"""
        if self.state_counter == 0:
            self.publish_dyna_pos(0, 3100)  #横
            self.publish_dyna_pos(1, 3300)  #開ける
            self.publish_dyna_pos(2, 2800)  #開ける
            send_packet_1byte(0x021, 12, 0, bus)  # air 閉じる

        elif self.state_counter == 1:
            send_packet_1byte(0x021, 12, 1, bus)  # air 開く
            self.publish_dyna_pos(0, 2100)  #縦

        elif self.state_counter == 2:
            self.publish_dyna_pos(1, 3000)  #ハンド閉じる
            self.publish_dyna_pos(2, 3200)  #ハンド閉じる
            send_packet_1byte(0x021, 12, 0, bus)  # air 閉じる

        elif self.state_counter == 3:
            self.publish_dyna_pos(0, 3100)  #横

    def joy_callback(self, msg):
        """joyを受取、各機構を動作

        Args:
            msg (Joy): joy_stick_message
        """
        self.now_button_state = msg.buttons[0]
        self.update_spear_state()
        self.move_spear()


def main():
    rclpy.init()  # rclpyライブラリの初期化

    spear_controller_node = SpearController()

    rclpy.spin(spear_controller_node)  # ノードをスピンさせる
    spear_controller_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
