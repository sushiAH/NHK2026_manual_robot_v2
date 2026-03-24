"""
機構仕様
モーター4つとポテンショメーターで昇降(同時に動く) とりあえず4パターン
モーター2つとポテンショメーターで直動(同時に動く) とりあえず3パターン
ロボマスモーター2つでハンド(同時に動く) 2パターン
エアシリ2つで機体を掴む(同時に動く) 2パターン
ダイナミクセル2つでボックスを回収する 2パターン

"""

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


def update_state(now_button_state, last_button_state, state_counter,
                 state_length):
    "ステータス更新関数"
    if now_button_state == 1 and last_button_state == 0:
        state_counter += 1

    elif now_button_state == -1 and last_button_state == 0:
        state_counter += -1

    state_counter = (state_counter + state_length) % state_length
    last_button_state = now_button_state

    return state_counter, last_button_state


class BoxArmController(Node):

    def __init__(self):
        super().__init__("box_arm_controller")

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

        self.now_button_state = [0, 0, 0, 0, 0]  #[昇降、直動、ハンド、ボックス保持、機体把持]
        self.last_button_state = [0, 0, 0, 0, 0]

        self.now_state_counter = [0, 0, 0, 0, 0]
        self.last_state_counter = [0, 0, 0, 0, 0]

        self.timer = self.create_timer(0.01, self.timer_callback)

        #dc立ち上げ
        #昇降
        set_potentio_pos_mode()
        set_potentio_pos_mode()
        set_potentio_pos_mode()
        set_potentio_pos_mode()
        #直動
        set_potentio_pos_mode()
        set_potentio_pos_mode()
        #機体保持エアシリ
        set_air_mode()
        set_air_mode()

        #pidゲイン設定
        set_pos_pid_gain()
        set_pos_pid_gain()
        set_pos_pid_gain()
        set_pos_pid_gain()
        set_pos_pid_gain()
        set_pos_pid_gain()

    def publish_dyna_pos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_pos_publisher.publish(msg)

    def joy_callback(self, msg):
        """joyを受取、各機構を動作

        Args:
            msg (Joy): joy_stick_message
        """

        #受取部分
        self.now_button_state[0] = msg.axes[7]  #昇降: 十字上下
        self.now_button_state[1] = msg.axes[5]  #直動: R2
        self.now_button_state[2] = msg.buttons[5]  #ハンド: R1
        self.now_button_state[3] = msg.buttons[4]  #回転: L1
        self.now_button_state[4] = msg.axes[6]  #機体把持L2

    def timer_callback(self):

        #ステータス更新
        #昇降
        self.now_state_counter[0], self.last_button_state[0] = update_state(
            self.now_button_state[0], self.last_button_state[0],
            self.now_state_counter[0], 4)

        #直動
        self.now_state_counter[1], self.last_button_state[1] = update_state(
            self.now_button_state[1], self.last_button_state[1],
            self.now_state_counter[1], 3)

        #ハンド
        self.now_state_counter[2], self.last_button_state[2] = update_state(
            self.now_button_state[2], self.last_button_state[2],
            self.now_state_counter[2], 2)

        #ボックス保持
        self.now_state_counter[3], self.last_button_state[3] = update_state(
            self.now_button_state[3], self.last_button_state[3],
            self.now_state_counter[3], 2)

        #機体把持
        self.now_state_counter[4], self.last_button_state[4] = update_state(
            self.now_button_state[4], self.last_button_state[4],
            self.now_state_counter[4], 2)

        #動作部分
        #ハンド
        if (self.now_state_counter[2] == 0 and self.last_state_counter[2] == 0):
            set_goal_pos()
            set_goal_pos()
            self.last_state_counter[2] = 1

        elif (self.now_state_counter[2] == 1 and
              self.last_state_counter[2] == 1):
            set_goal_pos()
            set_goal_pos()
            self.last_state_counter[2] = 0

        #ボックス保持
        if (self.now_state_counter[3] == 0 and self.last_state_counter[3] == 0):
            self.publish_dyna_pos()
            self.publish_dyna_pos()
            self.last_state_counter[3] = 1

        elif (self.now_state_counter[3] == 1 and
              self.last_state_counter[3] == 1):
            self.publish_dyna_pos()
            self.publish_dyna_pos()
            self.last_state_counter[3] = 0

        #機体把持
        if (self.now_state_counter[4] == 0 and self.last_state_counter[4] == 0):
            set_air()
            set_air()
            self.last_state_counter[4] = 1

        elif (self.now_state_counter[4] == 1 and
              self.last_state_counter[4] == 1):
            set_air()
            set_air()
            self.last_state_counter[4] = 0

        #昇降
        if (self.now_state_counter[0] == 0):
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
        elif (self.now_state_counter[0] == 1):
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
        elif (self.now_state_counter[0] == 2):
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
        elif (self.now_state_counter[0] == 3):
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()

        #直動
        if (self.now_state_counter[1] == 0):
            set_goal_pos()
            set_goal_pos()
        elif (self.now_state_counter[1] == 1):
            set_goal_pos()
            set_goal_pos()
        elif (self.now_state_counter[1] == 2):
            set_goal_pos()
            set_goal_pos()


def main():
    rclpy.init()  # rclpyライブラリの初期化

    box_arm_controller_node = BoxArmController()

    rclpy.spin(box_arm_controller_node)  # ノードをスピンさせる
    box_arm_controller_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


def stop():
    set_stop_mode()
    set_stop_mode()
    set_stop_mode()
    set_stop_mode()
    set_stop_mode()
    set_stop_mode()
    set_stop_mode()
    set_stop_mode()


atexit.register(stop)

if __name__ == "__main__":
    main()
