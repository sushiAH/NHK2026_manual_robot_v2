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

# CANバスの初期化
bus = can.interface.Bus(bustype="socketcan",
                        channel="can0",
                        asynchronous=True,
                        bitrate=1000000)


# ステータス更新関数（BoxArmControllerと同じもの）
def update_state(now_button_state, last_button_state, state_counter,
                 state_length):
    if now_button_state == 1 and last_button_state == 0:
        state_counter += 1
    elif now_button_state == -1 and last_button_state == 0:
        state_counter -= 1
    state_counter = (state_counter + state_length) % state_length
    last_button_state = now_button_state
    return state_counter, last_button_state


class SpearController(Node):

    def __init__(self):
        super().__init__("box_spear_controller")

        # --- 並列実行・排他制御の設定 ---
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        # 状態管理変数（リスト形式に統一）
        self.now_button_state = [0]
        self.last_button_state = [0]
        self.now_state_counter = [0]
        self.last_state_counter = [-1]  # 最初は必ず実行されるように -1 で初期化

        # 二重実行防止フラグ
        self.is_working = [False]

        # 購読設定
        self.subscription_joy = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10, callback_group=self.group)

        # パブリッシャー
        self.dyna_pos_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_pos", 10)

        # --- タイマー設定（監視用と実行用を分離） ---
        self.create_timer(0.01,
                          self.status_monitor_callback,
                          callback_group=self.group)
        self.create_timer(0.1,
                          self.spear_timer_callback,
                          callback_group=self.group)

        # 初期設定（既存のまま）
        send_packet_1byte(0x021, 0, 5, bus)  # set_operating
        self.get_logger().info("Spear Controller Initialized")

    def publish_dyna_pos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_pos_publisher.publish(msg)

    def joy_callback(self, msg):
        """Joy入力をロックして保存"""
        with self.lock:
            # msg.buttons[0] を監視
            self.now_button_state[0] = msg.buttons[0]

    def status_monitor_callback(self):
        """カウンターの更新（100Hz）"""
        with self.lock:
            self.now_state_counter[0], self.last_button_state[0] = update_state(
                self.now_button_state[0],
                self.last_button_state[0],
                self.now_state_counter[0],
                5  # state_length
            )

    def spear_timer_callback(self):
        """動作実行ロジック（sleep使用可能）"""
        with self.lock:
            if self.is_working[0]:
                return

            state = self.now_state_counter[0]
            last_state = self.last_state_counter[0]

        # 状態が変わった瞬間だけ実行する判定
        if state != last_state:
            self.is_working[0] = True

            # --- 既存の move_spear ロジックをそのまま配置 ---
            if state == 0:
                self.publish_dyna_pos(0, 3100)  # 横
                self.publish_dyna_pos(1, 3300)  # 閉じる
                self.publish_dyna_pos(2, 2800)  # 閉じる
                send_packet_1byte(0x021, 12, 0, bus)  # air 閉じる

            elif state == 1:
                self.publish_dyna_pos(0, 2100)  # 縦
                self.publish_dyna_pos(1, 3300)  # 開く
                self.publish_dyna_pos(2, 2800)  # 開く
                send_packet_1byte(0x021, 12, 1, bus)  # air 開く

            elif state == 2:
                self.publish_dyna_pos(1, 3000)  # ハンド閉じる
                self.publish_dyna_pos(2, 3200)  # ハンド閉じる
                send_packet_1byte(0x021, 12, 0, bus)  # air 閉じる

            elif state == 3:
                self.publish_dyna_pos(0, 3100)  # 横

            elif state == 4:
                self.publish_dyna_pos(0, 3100)  # 縦

            # ---------------------------------------------

            with self.lock:
                self.last_state_counter[0] = state
            self.is_working[0] = False


def main():
    rclpy.init()
    node = SpearController()

    # マルチスレッド実行
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# 終了時処理（必要に応じて追加）
def stop_all():
    # 槍機構の停止パケットなどがあればここに記述
    pass


atexit.register(stop_all)

if __name__ == "__main__":
    main()
