"""
機構仕様
モーター4つとポテンショメーターで昇降(同時に動く) とりあえず4パターン
モーター2つとポテンショメーターで直動(同時に動く) とりあえず3パターン
ロボマスモーター2つでハンド(同時に動く) 2パターン
エアシリ2つで機体を掴む(同時に動く) 2パターン
ダイナミクセル2つでボックスを回収する 2パターン

"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
import time
import threading
import os
import sys
import atexit

# 自作ライブラリ
target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from ah_python_can import *
from dyna_interfaces.msg import DynaFeedback, DynaTarget


# ステータス更新関数（既存のまま）
def update_state(now_button_state, last_button_state, state_counter,
                 state_length):
    if now_button_state == 1 and last_button_state == 0:
        state_counter += 1
    elif now_button_state == -1 and last_button_state == 0:
        state_counter -= 1
    state_counter = (state_counter + state_length) % state_length
    last_button_state = now_button_state
    return state_counter, last_button_state


class BoxArmController(Node):

    def __init__(self):
        super().__init__("box_arm_controller")

        # --- 並列実行のための設定 ---
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()  # データの整合性用

        # 既存の変数（そのまま）
        self.now_button_state = [0, 0, 0, 0, 0]
        self.last_button_state = [0, 0, 0, 0, 0]
        self.now_state_counter = [0, 0, 0, 0, 0]
        self.last_state_counter = [0, 0, 0, 0, 0]

        # 二重実行（スレッド爆発）防止フラグ
        self.is_working = [False] * 5

        # 購読設定（callback_groupを追加）
        self.subscription_joy = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10, callback_group=self.group)

        # パブリッシャー
        self.dyna_pos_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_pos", 10)

        # --- タイマーを分割 ---
        self.create_timer(0.01,
                          self.status_monitor_callback,
                          callback_group=self.group)
        self.create_timer(0.1,
                          self.lift_timer_callback,
                          callback_group=self.group)
        self.create_timer(0.1,
                          self.extend_timer_callback,
                          callback_group=self.group)
        self.create_timer(0.1,
                          self.hand_timer_callback,
                          callback_group=self.group)
        self.create_timer(0.1,
                          self.box_timer_callback,
                          callback_group=self.group)
        self.create_timer(0.1,
                          self.grip_timer_callback,
                          callback_group=self.group)

        # 初期化処理（既存のまま）
        set_potentio_pos_mode()
        set_potentio_pos_mode()
        set_potentio_pos_mode()
        set_potentio_pos_mode()
        set_potentio_pos_mode()
        set_potentio_pos_mode()
        set_air_mode()
        set_air_mode()
        set_pos_pid_gain()
        set_pos_pid_gain()
        set_pos_pid_gain()
        set_pos_pid_gain()
        set_pos_pid_gain()
        set_pos_pid_gain()

    def publish_dyna_pos(self, id=0, target=0):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_pos_publisher.publish(msg)

    def joy_callback(self, msg):
        with self.lock:
            self.now_button_state[0] = msg.axes[7]
            self.now_button_state[1] = msg.axes[5]
            self.now_button_state[2] = msg.buttons[5]
            self.now_button_state[3] = msg.buttons[4]
            self.now_button_state[4] = msg.axes[6]

    def status_monitor_callback(self):
        """カウンターの更新（ロック内で一括処理）"""
        with self.lock:
            self.now_state_counter[0], self.last_button_state[0] = update_state(
                self.now_button_state[0], self.last_button_state[0],
                self.now_state_counter[0], 4)
            self.now_state_counter[1], self.last_button_state[1] = update_state(
                self.now_button_state[1], self.last_button_state[1],
                self.now_state_counter[1], 3)
            self.now_state_counter[2], self.last_button_state[2] = update_state(
                self.now_button_state[2], self.last_button_state[2],
                self.now_state_counter[2], 2)
            self.now_state_counter[3], self.last_button_state[3] = update_state(
                self.now_button_state[3], self.last_button_state[3],
                self.now_state_counter[3], 2)
            self.now_state_counter[4], self.last_button_state[4] = update_state(
                self.now_button_state[4], self.last_button_state[4],
                self.now_state_counter[4], 2)

    def lift_timer_callback(self):
        with self.lock:
            if self.is_working[0]:
                return
            state = self.now_state_counter[0]

        self.is_working[0] = True
        if state == 0:
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
        elif state == 1:
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
        elif state == 2:
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
        elif state == 3:
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
            set_goal_pos()
        self.is_working[0] = False

    def extend_timer_callback(self):
        with self.lock:
            if self.is_working[1]:
                return
            state = self.now_state_counter[1]

        self.is_working[1] = True
        if state == 0:
            set_goal_pos()
            set_goal_pos()
        elif state == 1:
            set_goal_pos()
            set_goal_pos()
        elif state == 2:
            set_goal_pos()
            set_goal_pos()
        self.is_working[1] = False

    def hand_timer_callback(self):
        with self.lock:
            if self.is_working[2]:
                return
            state = self.now_state_counter[2]
            last_state = self.last_state_counter[2]

        if state == 0 and last_state == 0:
            self.is_working[2] = True
            set_goal_pos()
            set_goal_pos()
            with self.lock:
                self.last_state_counter[2] = 1
            self.is_working[2] = False
        elif state == 1 and last_state == 1:
            self.is_working[2] = True
            set_goal_pos()
            set_goal_pos()
            with self.lock:
                self.last_state_counter[2] = 0
            self.is_working[2] = False

    def box_timer_callback(self):
        with self.lock:
            if self.is_working[3]:
                return
            state = self.now_state_counter[3]
            last_state = self.last_state_counter[3]

        if state == 0 and last_state == 0:
            self.is_working[3] = True
            self.publish_dyna_pos()
            self.publish_dyna_pos()
            with self.lock:
                self.last_state_counter[3] = 1
            self.is_working[3] = False
        elif state == 1 and last_state == 1:
            self.is_working[3] = True
            self.publish_dyna_pos()
            self.publish_dyna_pos()
            with self.lock:
                self.last_state_counter[3] = 0
            self.is_working[3] = False

    def grip_timer_callback(self):
        with self.lock:
            if self.is_working[4]:
                return
            state = self.now_state_counter[4]
            last_state = self.last_state_counter[4]

        if state == 0 and last_state == 0:
            self.is_working[4] = True
            set_air()
            set_air()
            with self.lock:
                self.last_state_counter[4] = 1
            self.is_working[4] = False
        elif state == 1 and last_state == 1:
            self.is_working[4] = True
            set_air()
            set_air()
            with self.lock:
                self.last_state_counter[4] = 0
            self.is_working[4] = False


def main():
    rclpy.init()
    box_arm_controller_node = BoxArmController()
    executor = MultiThreadedExecutor()
    executor.add_node(box_arm_controller_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        box_arm_controller_node.destroy_node()
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
