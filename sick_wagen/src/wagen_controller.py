#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

class WagenController(Node):
    def __init__(self):
        super().__init__('wagen_controller')

        # ===== Parameters =====
        self.declare_parameter('linearGain', 0.5)
        self.declare_parameter('angularGain', 0.6)
        self.declare_parameter('speedUpRate', 1.6)

        # Twist -> Joy 正規化用の上限値（m/s, rad/s）
        self.declare_parameter('maxLinear', 1.0)
        self.declare_parameter('maxAngular', 1.0)

        # 起動直後監視用（秒）とデッドバンド
        self.declare_parameter('startupGuardSec', 5.0)   # ← ここを変えれば監視時間を変更可
        self.declare_parameter('neutralDeadband', 0.10)  # |axis| <= 0.10 をニュートラル扱い

        self.linear_gain = float(self.get_parameter('linearGain').value)
        self.angular_gain = float(self.get_parameter('angularGain').value)
        self.speed_up_rate = float(self.get_parameter('speedUpRate').value)

        self.max_linear = float(self.get_parameter('maxLinear').value)
        self.max_angular = float(self.get_parameter('maxAngular').value)

        self.startup_guard_sec = float(self.get_parameter('startupGuardSec').value)
        self.neutral_deadband = float(self.get_parameter('neutralDeadband').value)

        # ===== Publishers =====
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.whill_cmd_pub = self.create_publisher(Joy, '/whill/controller/joy', 10)

        # ===== Subscribers =====
        self.create_subscription(Twist, '/cmd_vel_move_base', self.nav_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_subscription(Joy, '/whill/states/joy', self.whill_callback, 10)

        # ===== Storage =====
        self.cmd_vel = Twist()
        self.sub_cmd_vel = Twist()
        self.cmd_vel_joy = Twist()
        self.cmd_vel_whill = Twist()

        # ===== Safety lockout state =====
        self.start_time = self.get_clock().now()
        self.locked = False
        self._lock_logged = False  # 1回だけログするため

        # ===== Timer (10Hz) =====
        self.create_timer(0.1, self.update_loop)

        self.get_logger().info(
            f"Params: linearGain={self.linear_gain}, angularGain={self.angular_gain}, "
            f"speedUpRate={self.speed_up_rate}, maxLinear={self.max_linear}, "
            f"maxAngular={self.max_angular}, startupGuardSec={self.startup_guard_sec}, "
            f"neutralDeadband={self.neutral_deadband}"
        )

    # Nav stack から
    def nav_callback(self, msg: Twist):
        self.sub_cmd_vel.linear.x  = msg.linear.x
        self.sub_cmd_vel.angular.z = msg.angular.z

    # ゲームパッドの生Joyから Twist を作る
    def joy_callback(self, msg: Joy):
        # 軸割り当ては環境に合わせて調整してください
        RStickX = msg.axes[2]
        RStickY = msg.axes[3]
        LB_btn  = msg.buttons[4] if len(msg.buttons) > 4 else 0

        # ---- 起動直後のロックアウト判定 ----
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if not self.locked and t <= self.startup_guard_sec:
            # 非ニュートラル入力（どちらかの軸がデッドバンド外 or 何かボタン押下）が来たらロック
            non_neutral = (abs(RStickX) > self.neutral_deadband) or (abs(RStickY) > self.neutral_deadband) \
                          or any(b != 0 for b in msg.buttons)
            if non_neutral:
                self.locked = True

        # ロック中は常に 0
        if self.locked:
            if not self._lock_logged:
                self.get_logger().error(
                    f"[LOCKED] 起動後 {self.startup_guard_sec:.1f}s 以内に /joy の非ニュートラル入力を検出。"
                    f" 以後は安全のため速度を常に 0 に固定します。 (X={RStickX:.2f}, Y={RStickY:.2f})"
                )
                self._lock_logged = True
            vx, wz = 0.0, 0.0
        else:
            # 通常の変換
            vx = math.copysign(RStickY ** 2, RStickY) * self.linear_gain
            if LB_btn == 1:
                vx *= self.speed_up_rate
            wz = RStickX * self.angular_gain

        self.cmd_vel_joy.linear.x  = vx
        self.cmd_vel_joy.angular.z = wz

    # WHILL 本体のJoy状態（参考/優先入力）
    def whill_callback(self, msg: Joy):
        self.cmd_vel_whill.linear.x  = msg.axes[1]
        self.cmd_vel_whill.angular.z = msg.axes[0]

    # Twist -> Joy 変換（/whill/controller/joy 出力）
    def publish_whill_joy(self, twist: Twist):
        lin = 0.0 if self.max_linear  <= 0.0 else clamp(twist.linear.x  / self.max_linear,  -1.0, 1.0)
        ang = 0.0 if self.max_angular <= 0.0 else clamp(twist.angular.z / self.max_angular, -1.0, 1.0)

        if abs(lin) < 1e-3: lin = 0.0
        if abs(ang) < 1e-3: ang = 0.0

        joy = Joy()
        joy.axes = [0.0, 0.0, 0.0, 0.0]
        joy.buttons = [0]*8

        joy.axes[1] = lin
        joy.axes[0] = ang

        self.whill_cmd_pub.publish(joy)

    def update_loop(self):
        # ロック中は無条件で出力 0（ナビやWHILL入力も含め全て無効化）
        if self.locked:
            zero = Twist()
            self.cmd_vel = zero
            self.cmd_pub.publish(zero)
            self.publish_whill_joy(zero)
            return

        # 優先順位: WHILLの手元操作 > 手動ゲームパッド > ナビ
        if self.cmd_vel_whill.linear.x != 0.0 or self.cmd_vel_whill.angular.z != 0.0:
            self.cmd_vel = self.cmd_vel_whill
        elif self.cmd_vel_joy.linear.x != 0.0 or self.cmd_vel_joy.angular.z != 0.0:
            self.cmd_vel = self.cmd_vel_joy
        else:
            self.cmd_vel = self.sub_cmd_vel

        self.cmd_pub.publish(self.cmd_vel)
        self.publish_whill_joy(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = WagenController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



