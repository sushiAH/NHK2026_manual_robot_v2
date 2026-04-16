"""
機構仕様
モーター4つとポテンショメーターで昇降(同時に動く) とりあえず4パターン
モーター2つとポテンショメーターで直動(同時に動く) とりあえず3パターン
ロボマスモーター2つでハンド(同時に動く) 2パターン
エアシリ2つで機体を掴む(同時に動く) 2パターン
ダイナミクセル2つでボックスを回収する 2パターン

0x010 ハンド手先左
0x011 ハンド手先右

0x020 足回り右下
0x021 足回り左下
0x022 足回り右上
0x023 足回り左上

0x030 直動 normal [152,3297] (縮、伸) (3147の移動量)
0x031 昇降奥 reverse [3147,197](上、下)
0x032 昇降手前 reverse [325,3275](上、下)(3043の移動量)

0x040 normal [3190,43]
0x041 normal [634,3584]
0x042 normal [2900,0]

エアシリ
0x050
0x051 vゴール左
0x052 vゴール右

ダイナミクセル
箱台左 0
箱台右 1
やり回転 2
やりハンド左 3
やりハンド右 4

ベルトを固定する際は
手前は一番上まで上げる
奥は、横線が引いてあるところにつめの上を合わせる
位置を記録する



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

bus = can.interface.Bus(bustype="socketcan",
                        channel="can0",
                        asynchronous=True,
                        bitrate=1000000)


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


def ratio_to_pot_pos(ratio, origin_pos_list):
    resolution = origin_pos_list[1] - origin_pos_list[0]
    pot_pos = (ratio * resolution) + origin_pos_list[0]
    return pot_pos


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

        # ----Params----
        self.origin_pos_dict = {
            0x030: [152, 3297],
            0x031: [3222, 529],  # [縮む、伸びる]
            0x032: [293, 2881],
            0x040: [3190, 43],
            0x041: [378, 3054],
            0x042: [3418, 616],
        }

        # ----init----

        #set_potentio_pos_mode(0x030, bus)
        set_potentio_pos_mode(0x031, bus)
        set_potentio_pos_mode(0x032, bus)
        #set_potentio_pos_mode(0x040, bus)
        set_potentio_pos_mode(0x041, bus)
        set_potentio_pos_mode(0x042, bus)
        time.sleep(1.0)

        set_air_mode(0x051, bus)
        set_air_mode(0x052, bus)

        #モーターの回転方向決定
        set_motor_rot_dir(0x030, 0, bus)
        set_motor_rot_dir(0x031, 1, bus)
        set_motor_rot_dir(0x032, 1, bus)
        set_motor_rot_dir(0x040, 0, bus)
        set_motor_rot_dir(0x041, 0, bus)
        set_motor_rot_dir(0x042, 0, bus)
        time.sleep(0.1)

        #pidゲイン設定
        set_pos_pid_gain(0x030, 5.0, 0, 0, bus)
        set_pos_pid_gain(0x031, 5.0, 0, 0, bus)
        set_pos_pid_gain(0x032, 5.0, 0, 0, bus)
        set_pos_pid_gain(0x040, 5.0, 0, 0, bus)
        set_pos_pid_gain(0x041, 5.0, 0, 0, bus)
        set_pos_pid_gain(0x042, 5.0, 0, 0, bus)
        time.sleep(0.1)

        #profile vel設定
        set_profile_vel(0x030, 1000, bus)
        set_profile_vel(0x031, 1000, bus)
        set_profile_vel(0x032, 1000, bus)
        set_profile_vel(0x040, 1000, bus)
        set_profile_vel(0x041, 1000, bus)
        set_profile_vel(0x042, 1000, bus)
        time.sleep(0.1)

        #profile accel設定
        set_profile_accel(0x030, 1000, bus)
        set_profile_accel(0x031, 1000, bus)
        set_profile_accel(0x032, 1000, bus)
        set_profile_accel(0x040, 1000, bus)
        set_profile_accel(0x041, 1000, bus)
        set_profile_accel(0x042, 1000, bus)
        time.sleep(0.1)

        self.set_ratio(0x030, 0.2)
        self.set_ratio(0x031, 0.9)
        self.set_ratio(0x032, 0.9)
        self.set_ratio(0x040, 0.2)
        self.set_ratio(0x041, 0.9)
        self.set_ratio(0x042, 0.9)

    def publish_dyna_pos(self, id=0, target=0):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_pos_publisher.publish(msg)

    def set_ratio(self, id, target_ratio):
        target_pos = ratio_to_pot_pos(target_ratio, self.origin_pos_dict[id])
        set_goal_pos(id, target_pos, bus)

    def joy_callback(self, msg):
        with self.lock:
            self.now_button_state[0] = msg.axes[7]
            self.now_button_state[1] = msg.buttons[4]
            self.now_button_state[2] = msg.buttons[5]
            self.now_button_state[3] = msg.buttons[1]
            self.now_button_state[4] = msg.buttons[2]

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
            last_state = self.last_state_counter[0]

        if (state != last_state):
            self.is_working[0] = True

            if state == 0:
                self.set_ratio(0x031, 0.87)
                self.set_ratio(0x032, 0.8)
                self.set_ratio(0x041, 0.8)
                self.set_ratio(0x042, 0.85)
            elif state == 1:
                self.set_ratio(0x031, 0.5)
                self.set_ratio(0x032, 0.5)
                self.set_ratio(0x041, 0.5)
                self.set_ratio(0x042, 0.5)
            elif state == 2:
                self.set_ratio(0x031, 0.3)
                self.set_ratio(0x032, 0.3)
                self.set_ratio(0x041, 0.3)
                self.set_ratio(0x042, 0.3)
            elif state == 3:
                self.set_ratio(0x031, 0.1)
                self.set_ratio(0x032, 0.1)
                self.set_ratio(0x041, 0.1)
                self.set_ratio(0x042, 0.1)

            with self.lock:
                self.last_state_counter[0] = state
            self.is_working[0] = False

    def extend_timer_callback(self):
        with self.lock:
            if self.is_working[1]:
                return
            state = self.now_state_counter[1]
            last_state = self.last_state_counter[1]

        if (state != last_state):
            self.is_working[1] = True

            if state == 0:
                self.set_ratio(0x030, 0.2)
                self.set_ratio(0x040, 0.2)

            elif state == 1:
                self.set_ratio(0x030, 0.55)
                self.set_ratio(0x040, 0.55)

            elif state == 2:
                self.set_ratio(0x030, 0.9)
                self.set_ratio(0x040, 0.95)

            with self.lock:
                self.last_state_counter[1] = state
            self.is_working[1] = False

    def hand_timer_callback(self):
        with self.lock:
            if self.is_working[2]:
                return
            state = self.now_state_counter[2]
            last_state = self.last_state_counter[2]

        if (state != last_state):
            self.is_working[2] = True

            if state == 0:
                set_goal_pos(0x010, 0, bus)
                set_goal_pos(0x011, 0, bus)

            elif state == 1:
                set_goal_pos(0x010, 1000, bus)
                set_goal_pos(0x011, -1000, bus)

            with self.lock:
                self.last_state_counter[2] = state
            self.is_working[2] = False

    def box_timer_callback(self):
        with self.lock:
            if self.is_working[3]:
                return
            state = self.now_state_counter[3]
            last_state = self.last_state_counter[3]

        if (state != last_state):
            self.is_working[3] = True

            if state == 0:
                self.publish_dyna_pos(0, 0)
                self.publish_dyna_pos(1, 0)

            elif state == 1:
                self.publish_dyna_pos(0, 0)
                self.publish_dyna_pos(1, 0)

            with self.lock:
                self.last_state_counter[3] = state
            self.is_working[3] = False

    def grip_timer_callback(self):
        with self.lock:
            if self.is_working[4]:
                return
            state = self.now_state_counter[4]
            last_state = self.last_state_counter[4]

        if (state != last_state):
            self.is_working[4] = True

            if state == 0:
                set_air(0x051, 0, bus)
                set_air(0x052, 0, bus)

            elif state == 1:
                set_air(0x051, 1, bus)
                set_air(0x052, 1, bus)

            with self.lock:
                self.last_state_counter[4] = state
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
    set_stop_mode(0x010, bus)
    set_stop_mode(0x011, bus)

    set_stop_mode(0x030, bus)
    set_stop_mode(0x031, bus)
    set_stop_mode(0x032, bus)

    set_stop_mode(0x040, bus)
    set_stop_mode(0x041, bus)
    set_stop_mode(0x042, bus)

    set_stop_mode(0x050, bus)
    set_stop_mode(0x051, bus)
    set_stop_mode(0x052, bus)


atexit.register(stop)

if __name__ == "__main__":
    main()
