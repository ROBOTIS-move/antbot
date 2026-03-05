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

import glob
import math

from antbot_interfaces.srv import CargoCommand
from antbot_interfaces.srv import WiperOperation
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool

# Controller axis/button mappings
# Left stick X(0) / Y(1) are the same across both controllers.
CONTROLLERS = {
    'ds4': {
        'name': 'DualShock 4',
        'axis_l2': 2, 'axis_r2': 5,
        'btn_cross': 0, 'btn_triangle': 2,
        'btn_square': 3, 'btn_circle': 1,
        'btn_l1': 4, 'btn_r1': 5,
    },
    'dualsense': {
        'name': 'DualSense',
        'axis_l2': 3, 'axis_r2': 4,
        'btn_cross': 1, 'btn_triangle': 3,
        'btn_square': 0, 'btn_circle': 2,
        'btn_l1': 4, 'btn_r1': 5,
    },
}
DETECT_MAX_ATTEMPTS = 10

V_THRESHOLD = 0.05      # [m/s] curve driving activation threshold


class TeleopJoystickNode(Node):

    def __init__(self):
        super().__init__('teleop_joystick')

        # Declare parameters
        self.declare_parameter('max_linear_vel', 1.5)
        self.declare_parameter('max_spin_vel', 1.5)
        self.declare_parameter('speed_level', 3)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('module_x', 0.265)
        self.declare_parameter('module_y', 0.256)
        self.declare_parameter('steering_limit_deg', 60.0)
        self.declare_parameter('safety_factor', 0.95)
        self.declare_parameter('w_abs_max', 2.0)

        # Load parameters
        self.max_linear_vel = self.get_parameter(
            'max_linear_vel').get_parameter_value().double_value
        self.max_spin_vel = self.get_parameter(
            'max_spin_vel').get_parameter_value().double_value
        self.speed_level = self.get_parameter(
            'speed_level').get_parameter_value().integer_value
        self.deadzone = self.get_parameter(
            'deadzone').get_parameter_value().double_value
        module_x = self.get_parameter(
            'module_x').get_parameter_value().double_value
        module_y = self.get_parameter(
            'module_y').get_parameter_value().double_value
        steering_limit_deg = self.get_parameter(
            'steering_limit_deg').get_parameter_value().double_value
        safety_factor = self.get_parameter(
            'safety_factor').get_parameter_value().double_value
        self.w_abs_max = self.get_parameter(
            'w_abs_max').get_parameter_value().double_value

        # Compute minimum turning radius from robot geometry
        effective_limit_rad = math.radians(steering_limit_deg * safety_factor)
        self.r_min = module_x / math.tan(effective_limit_rad) + module_y

        self.get_logger().info(
            f'Effective steering limit: {steering_limit_deg * safety_factor:.1f} deg, '
            f'R_min: {self.r_min:.3f} m, '
            f'W_ABS_MAX: {self.w_abs_max:.1f} rad/s')

        # Controller detection state
        self.mapping = None
        self.last_joy_time = None
        self.detect_attempts = 0

        # Button edge detection state (shared dict for all buttons)
        self.prev_buttons = {}
        self.was_active = False

        # Toggle states
        self.headlight_on = False
        self.wiper_on = False

        # ROS pub/sub/clients
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cargo_client = self.create_client(CargoCommand, 'cargo/command')
        self.headlight_client = self.create_client(
            SetBool, 'headlight/operation')
        self.wiper_client = self.create_client(
            WiperOperation, 'wiper/operation')
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info(
            f'Teleop Joystick ready. Speed level: {self.speed_level}/9')

    def _detect_from_usb(self):
        """Identify Sony controller by USB product ID from sysfs."""
        DUALSENSE_PIDS = {'0ce6', '0df2'}
        DS4_PIDS = {'05c4', '09cc'}
        for path in glob.glob('/sys/bus/usb/devices/*/idVendor'):
            try:
                vid = open(path).read().strip()
                if vid == '054c':
                    pid = open(
                        path.replace('idVendor', 'idProduct')).read().strip()
                    if pid in DUALSENSE_PIDS:
                        return CONTROLLERS['dualsense']
                    elif pid in DS4_PIDS:
                        return CONTROLLERS['ds4']
            except IOError:
                continue
        return None

    def _detect_from_axes(self, msg):
        """Fallback: distinguish controller by axis rest values."""
        if len(msg.axes) < 6:
            return None
        a2, a4 = msg.axes[2], msg.axes[4]
        # DS4:       axis[2]=L2(+1.0 at rest), axis[4]=RS_Y(~0.0)
        # DualSense: axis[2]=RS_X(~0.0),       axis[4]=L2(+1.0 at rest)
        if a2 > 0.5 and a4 < 0.5:
            return CONTROLLERS['ds4']
        elif a4 > 0.5 and a2 < 0.5:
            return CONTROLLERS['dualsense']
        return None

    def _button_pressed(self, msg, key):
        """Return True on rising edge (0->1) for the mapped button."""
        idx = self.mapping[key]
        curr = msg.buttons[idx]
        prev = self.prev_buttons.get(key, 0)
        self.prev_buttons[key] = curr
        return curr == 1 and prev == 0

    def _call_cargo(self, operation):
        """Send async cargo command service call."""
        req = CargoCommand.Request()
        req.operation = operation
        future = self.cargo_client.call_async(req)
        future.add_done_callback(self._service_response_callback)
        op_name = 'LOCK' if operation == \
            CargoCommand.Request.OPERATION_LOCK else 'UNLOCK'
        self.get_logger().info(f'Cargo {op_name} requested')

    def _toggle_headlight(self):
        self.headlight_on = not self.headlight_on
        req = SetBool.Request()
        req.data = self.headlight_on
        future = self.headlight_client.call_async(req)
        future.add_done_callback(self._service_response_callback)
        state = 'ON' if self.headlight_on else 'OFF'
        self.get_logger().info(f'Headlight {state} requested')

    def _toggle_wiper(self):
        self.wiper_on = not self.wiper_on
        req = WiperOperation.Request()
        req.mode = WiperOperation.Request.REPEAT if self.wiper_on \
            else WiperOperation.Request.OFF
        future = self.wiper_client.call_async(req)
        future.add_done_callback(self._service_response_callback)
        state = 'REPEAT' if self.wiper_on else 'OFF'
        self.get_logger().info(f'Wiper {state} requested')

    def _service_response_callback(self, future):
        try:
            resp = future.result()
            if not resp.success:
                self.get_logger().warn(
                    f'Service call failed: {resp.message}')
        except Exception as e:
            self.get_logger().warn(f'Service call failed: {e}')

    def _print_help(self):
        name = self.mapping['name'] if self.mapping else 'Unknown'
        self.get_logger().info(
            f'\n'
            f'--- {name} Joystick Teleop ---\n'
            f'Left stick Y : forward / backward\n'
            f'Left stick X : curve turning (while moving)\n'
            f'L2 trigger   : in-place rotate CCW\n'
            f'R2 trigger   : in-place rotate CW\n'
            f'Triangle     : speed level UP\n'
            f'Cross        : speed level DOWN\n'
            f'Square       : cargo lock\n'
            f'Circle       : cargo open\n'
            f'L1           : headlight toggle\n'
            f'R1           : wiper toggle\n'
            f'---------------------------')

    def _apply_deadzone(self, val):
        return 0.0 if abs(val) < self.deadzone else val

    def _trigger_to_value(self, raw):
        """Convert L2/R2 trigger axis (+1=released, -1=pressed) to 0.0~1.0."""
        val = (1.0 - raw) / 2.0
        return 0.0 if val < self.deadzone else val

    def _get_speed_ratio(self):
        return self.speed_level / 9.0

    def joy_callback(self, msg):
        now = self.get_clock().now()

        # Hot-plug detection: message gap > 1s indicates controller swap
        if self.last_joy_time is not None:
            gap = (now - self.last_joy_time).nanoseconds / 1e9
            if gap > 1.0:
                self.mapping = None
                self.detect_attempts = 0
        self.last_joy_time = now

        # Detect controller type (USB first, then axis fallback)
        if self.mapping is None:
            self.mapping = self._detect_from_usb() or \
                self._detect_from_axes(msg)
            if self.mapping is None:
                self.detect_attempts += 1
                if self.detect_attempts >= DETECT_MAX_ATTEMPTS:
                    self.mapping = CONTROLLERS['ds4']
                    self.get_logger().warn(
                        'Unknown controller, '
                        'using DualShock 4 mapping as default')
                else:
                    return
            else:
                self.get_logger().info(
                    f"Detected controller: {self.mapping['name']}")
            self.detect_attempts = 0
            self._print_help()

        # Bounds check based on current mapping
        m = self.mapping
        min_axes = max(1, m['axis_l2'], m['axis_r2']) + 1
        btn_indices = [v for k, v in m.items() if k.startswith('btn_')]
        min_btns = max(btn_indices) + 1
        if len(msg.axes) < min_axes or len(msg.buttons) < min_btns:
            return

        # Read inputs using detected mapping
        stick_y = self._apply_deadzone(msg.axes[1])
        stick_x = self._apply_deadzone(msg.axes[0])
        l2_val = self._trigger_to_value(msg.axes[m['axis_l2']])
        r2_val = self._trigger_to_value(msg.axes[m['axis_r2']])

        # Speed level control
        if self._button_pressed(msg, 'btn_triangle'):
            self.speed_level = min(self.speed_level + 1, 9)
            self.get_logger().info(f'Speed level UP: {self.speed_level}/9')
        if self._button_pressed(msg, 'btn_cross'):
            self.speed_level = max(self.speed_level - 1, 1)
            self.get_logger().info(f'Speed level DOWN: {self.speed_level}/9')

        # Cargo control
        if self._button_pressed(msg, 'btn_square'):
            self._call_cargo(CargoCommand.Request.OPERATION_LOCK)
        if self._button_pressed(msg, 'btn_circle'):
            self._call_cargo(CargoCommand.Request.OPERATION_UNLOCK)

        # Headlight / Wiper toggle
        if self._button_pressed(msg, 'btn_l1'):
            self._toggle_headlight()
        if self._button_pressed(msg, 'btn_r1'):
            self._toggle_wiper()

        # Compute velocities
        speed_ratio = self._get_speed_ratio()
        vx = self.max_linear_vel * speed_ratio * stick_y
        wz = 0.0

        if abs(vx) >= V_THRESHOLD:
            # Curve driving mode: left stick X controls angular velocity
            w_max = min(abs(vx) / self.r_min, self.w_abs_max)
            wz = w_max * stick_x
            # Backward steering inversion
            if vx < 0.0:
                wz = -wz
        else:
            # In-place rotation mode: L2(CCW) - R2(CW)
            spin_input = l2_val - r2_val
            wz = self.max_spin_vel * speed_ratio * spin_input

        # Publish Twist only when there is actual input,
        # plus one zero Twist when input stops (to brake immediately)
        is_active = abs(vx) >= V_THRESHOLD or abs(wz) > 0.0
        if is_active or self.was_active:
            twist = Twist()
            twist.linear.x = vx
            twist.angular.z = wz
            self.cmd_vel_pub.publish(twist)
        self.was_active = is_active


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before shutdown
        if node.context.ok():
            node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
