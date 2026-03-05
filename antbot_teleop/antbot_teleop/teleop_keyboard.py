# Copyright 2026 ROBOTIS AI CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Daun Jeong

from select import select
import signal
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

HELP_MSG = """
ANTBot Teleop Keyboard (hold key to move, release to stop)
----------------------------------------------------------

   q    w    e
   a         d
        x

w/x : forward / backward
a/d : strafe left / right
q/e       : rotate CCW / CW
1~9       : speed level
ESC/Ctrl+C: quit
----------------------------------------------------------
"""


class TeleopKeyboardNode(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')

        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('speed_level', 5)
        self.declare_parameter('publish_rate', 10.0)

        self.max_linear_vel = self.get_parameter(
            'max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter(
            'max_angular_vel').get_parameter_value().double_value
        self.speed_level = self.get_parameter(
            'speed_level').get_parameter_value().integer_value
        publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publish_period = 1.0 / publish_rate

        # Terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)

        # Signal handlers for safe shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGHUP, self._signal_handler)

    def _signal_handler(self, signum, frame):
        self._publish_stop()
        self._restore_terminal()
        sys.exit(0)

    def _publish_stop(self):
        self.publisher.publish(Twist())

    def _restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def _get_key(self, timeout):
        tty.setraw(sys.stdin.fileno())
        ready, _, _ = select([sys.stdin], [], [], timeout)
        if ready:
            key = sys.stdin.read(1)
            # ESC or escape sequence (arrow keys etc.) → quit.
            # Flush trailing bytes so they don't leak into terminal.
            if key == '\x1b':
                while select([sys.stdin], [], [], 0.01)[0]:
                    sys.stdin.read(1)
                return '\x03'
            return key
        return None

    def _get_speed(self):
        ratio = self.speed_level / 9.0
        return self.max_linear_vel * ratio, self.max_angular_vel * ratio

    def _print_status(self, twist):
        lin_speed, ang_speed = self._get_speed()
        status = (
            f'\rSpeed: {self.speed_level}/9 '
            f'(lin: {lin_speed:.2f} m/s, ang: {ang_speed:.2f} rad/s) | '
            f'x: {twist.linear.x:+.2f}  '
            f'y: {twist.linear.y:+.2f}  '
            f'az: {twist.angular.z:+.2f}  '
        )
        sys.stdout.write('\x1b[2K' + status)
        sys.stdout.flush()

    def run(self):
        print(HELP_MSG)
        last_twist = Twist()
        idle_count = 0
        # Tolerate up to max_idle consecutive timeouts before stopping.
        # Bridges the gap between initial key press and OS key-repeat onset
        # (~200-500ms), preventing spurious zero-velocity publishes.
        max_idle = 4

        try:
            while rclpy.ok():
                key = self._get_key(self.publish_period)

                twist = Twist()
                lin_speed, ang_speed = self._get_speed()

                if key is None:
                    idle_count += 1
                    if idle_count <= max_idle:
                        twist = last_twist
                elif key == '\x03':
                    # ESC or Ctrl+C
                    break
                elif key == 'w':
                    twist.linear.x = lin_speed
                elif key == 'x':
                    twist.linear.x = -lin_speed
                elif key == 'a':
                    twist.linear.y = lin_speed
                elif key == 'd':
                    twist.linear.y = -lin_speed
                elif key == 'q':
                    twist.angular.z = ang_speed
                elif key == 'e':
                    twist.angular.z = -ang_speed
                elif key in [str(i) for i in range(1, 10)]:
                    self.speed_level = int(key)

                if key is not None:
                    idle_count = 0
                    last_twist = twist

                self.publisher.publish(twist)
                self._print_status(twist)

        except KeyboardInterrupt:
            pass
        finally:
            self._publish_stop()
            self._restore_terminal()
            print('\n')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
