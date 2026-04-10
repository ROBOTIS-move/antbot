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

import math
import threading

from geometry_msgs.msg import Twist
import pygame
import rclpy
from rclpy.node import Node

# Window settings
WINDOW_W = 900
WINDOW_H = 700
FPS = 30
PIXELS_PER_METER = 150

# Robot geometry (meters)
MODULE_X = [0.265, 0.265, -0.265, -0.265]   # FL, FR, RL, RR
MODULE_Y = [0.256, -0.256, 0.256, -0.256]
MODULE_NAMES = ['FL', 'FR', 'RL', 'RR']
BODY_LENGTH = 0.53    # 2 * 0.265
BODY_WIDTH = 0.512    # 2 * 0.256
STEERING_LIMIT_DEG = 60.0
EFFECTIVE_LIMIT_DEG = 57.0   # 60 * 0.95

# Colors
BG_COLOR = (30, 30, 30)
GRID_COLOR = (50, 50, 50)
BODY_COLOR = (120, 120, 120)
BODY_OUTLINE = (180, 180, 180)
TRAIL_COLOR = (60, 120, 220)
MODULE_COLOR = (220, 220, 220)
LIMIT_COLOR = (200, 60, 60)
GAUGE_GREEN = (50, 200, 80)
GAUGE_YELLOW = (230, 200, 40)
HUD_BG = (20, 20, 20, 200)
HUD_TEXT = (220, 220, 220)
HUD_LABEL = (150, 150, 150)
SHORTCUT_COLOR = (130, 130, 130)


class SwerveSimNode(Node):

    def __init__(self):
        super().__init__('swerve_sim')
        self.vx = 0.0
        self.wz = 0.0
        self.lock = threading.Lock()

        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        with self.lock:
            self.vx = msg.linear.x
            self.wz = msg.angular.z

    def get_cmd(self):
        with self.lock:
            return self.vx, self.wz


def compute_steering_angles(vx, wz):
    """Compute steering angles for all 4 modules using swerve IK."""
    angles = []
    for i in range(4):
        if abs(vx) < 1e-6 and abs(wz) < 1e-6:
            angles.append(0.0)
        else:
            a = wz * MODULE_X[i]
            b = vx - wz * MODULE_Y[i]
            angles.append(math.atan2(a, b))
    return angles


def rotate_point(px, py, angle):
    """Rotate point (px, py) by angle around origin."""
    c = math.cos(angle)
    s = math.sin(angle)
    return px * c - py * s, px * s + py * c


def world_to_screen(wx, wy, cam_x, cam_y):
    """Convert world coords (meters) to screen pixels."""
    sx = WINDOW_W // 2 + (wx - cam_x) * PIXELS_PER_METER
    sy = WINDOW_H // 2 - (wy - cam_y) * PIXELS_PER_METER
    return int(sx), int(sy)


def draw_robot(surface, x, y, theta, steering_angles, cam_x, cam_y):
    """Draw the robot body and swerve modules."""
    hl = BODY_LENGTH / 2 * PIXELS_PER_METER
    hw = BODY_WIDTH / 2 * PIXELS_PER_METER

    # Body corners (local frame)
    corners_local = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    corners_screen = []
    for lx, ly in corners_local:
        rx, ry = rotate_point(lx, ly, theta)
        sx = WINDOW_W // 2 + (x - cam_x) * PIXELS_PER_METER + rx
        sy = WINDOW_H // 2 - (y - cam_y) * PIXELS_PER_METER - ry
        corners_screen.append((int(sx), int(sy)))

    pygame.draw.polygon(surface, BODY_COLOR, corners_screen)
    pygame.draw.polygon(surface, BODY_OUTLINE, corners_screen, 2)

    # Draw heading indicator (front line)
    front_mid = (
        (corners_screen[1][0] + corners_screen[2][0]) // 2,
        (corners_screen[1][1] + corners_screen[2][1]) // 2)
    center = world_to_screen(x, y, cam_x, cam_y)
    pygame.draw.line(surface, (100, 200, 100), center, front_mid, 2)

    # Draw each module
    module_len = 20  # pixels
    for i in range(4):
        # Module position in world frame
        mx_local = MODULE_X[i] * PIXELS_PER_METER
        my_local = MODULE_Y[i] * PIXELS_PER_METER
        rx, ry = rotate_point(mx_local, my_local, theta)
        mx_screen = WINDOW_W // 2 + (x - cam_x) * PIXELS_PER_METER + rx
        my_screen = WINDOW_H // 2 - (y - cam_y) * PIXELS_PER_METER - ry

        # Wheel direction line (module steering + robot heading)
        wheel_angle = theta + steering_angles[i]
        dx = module_len * math.cos(wheel_angle)
        dy = module_len * math.sin(wheel_angle)

        start = (int(mx_screen - dx), int(my_screen + dy))
        end = (int(mx_screen + dx), int(my_screen - dy))
        pygame.draw.line(surface, MODULE_COLOR, start, end, 3)
        pygame.draw.circle(surface, MODULE_COLOR, (int(mx_screen), int(my_screen)), 4)

        # Steering limit arc (effective limit)
        limit_len = module_len + 6
        for sign in [1, -1]:
            limit_angle = theta + sign * math.radians(EFFECTIVE_LIMIT_DEG)
            ldx = limit_len * math.cos(limit_angle)
            ldy = limit_len * math.sin(limit_angle)
            lend = (int(mx_screen + ldx), int(my_screen - ldy))
            pygame.draw.line(surface, LIMIT_COLOR, (int(mx_screen), int(my_screen)), lend, 1)


def draw_grid(surface, cam_x, cam_y):
    """Draw a meter-based grid."""
    grid_spacing = PIXELS_PER_METER
    # Compute offset so grid follows camera
    offset_x = (WINDOW_W // 2 - int(cam_x * PIXELS_PER_METER)) % grid_spacing
    offset_y = (WINDOW_H // 2 + int(cam_y * PIXELS_PER_METER)) % grid_spacing

    for gx in range(offset_x, WINDOW_W, grid_spacing):
        pygame.draw.line(surface, GRID_COLOR, (gx, 0), (gx, WINDOW_H))
    for gy in range(offset_y, WINDOW_H, grid_spacing):
        pygame.draw.line(surface, GRID_COLOR, (0, gy), (WINDOW_W, gy))


def draw_trail(surface, trail, cam_x, cam_y):
    """Draw robot trajectory as dots."""
    for i in range(0, len(trail), 2):
        sx, sy = world_to_screen(trail[i][0], trail[i][1], cam_x, cam_y)
        if 0 <= sx < WINDOW_W and 0 <= sy < WINDOW_H:
            pygame.draw.circle(surface, TRAIL_COLOR, (sx, sy), 2)


def draw_hud(surface, font, font_small, vx, wz, steering_angles):
    """Draw heads-up display with telemetry and steering gauge."""
    hud_x = 10
    hud_y = WINDOW_H - 170
    hud_w = 340
    hud_h = 160

    # Semi-transparent background
    hud_surface = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
    hud_surface.fill((20, 20, 20, 200))
    surface.blit(hud_surface, (hud_x, hud_y))

    # Determine mode
    if abs(vx) < 0.05 and abs(wz) < 0.01:
        mode = 'STOP'
    elif abs(vx) < 0.05:
        mode = 'SPIN'
    else:
        mode = 'CURVE'

    # Turning radius
    r_str = f'{abs(vx / wz):.2f}m' if abs(wz) > 0.01 and abs(vx) > 0.01 else '---'

    y = hud_y + 8
    pad = hud_x + 10

    # Line 1: velocities
    text = font.render(f'vx: {vx:+.2f} m/s    wz: {wz:+.2f} rad/s', True, HUD_TEXT)
    surface.blit(text, (pad, y))
    y += 24

    # Line 2: mode and radius
    text = font.render(f'Mode: {mode}    R: {r_str}', True, HUD_TEXT)
    surface.blit(text, (pad, y))
    y += 28

    # Steering gauges for each module
    gauge_w = 140
    gauge_h = 14

    for i in range(4):
        angle_deg = math.degrees(abs(steering_angles[i]))
        # Normalize for gauge: when reversing, atan2 returns ~180°;
        # fold into [0°, 90°] to show actual steering offset.
        if angle_deg > 90.0:
            angle_deg = 180.0 - angle_deg
        ratio = min(angle_deg / EFFECTIVE_LIMIT_DEG, 1.0)

        label_text = f'{MODULE_NAMES[i]}: {angle_deg:4.1f}/{EFFECTIVE_LIMIT_DEG:.0f}\u00b0'
        label = font_small.render(label_text, True, HUD_LABEL)
        surface.blit(label, (pad, y))

        bar_x = pad + 130
        # Bar background
        pygame.draw.rect(surface, (60, 60, 60), (bar_x, y + 1, gauge_w, gauge_h))
        # Bar fill
        fill_w = int(gauge_w * ratio)
        color = GAUGE_YELLOW if ratio >= 0.9 else GAUGE_GREEN
        if fill_w > 0:
            pygame.draw.rect(surface, color, (bar_x, y + 1, fill_w, gauge_h))
        # Bar border
        pygame.draw.rect(surface, (100, 100, 100), (bar_x, y + 1, gauge_w, gauge_h), 1)

        # Percentage
        pct = font_small.render(f'{ratio * 100:.0f}%', True, HUD_LABEL)
        surface.blit(pct, (bar_x + gauge_w + 5, y))

        y += 20


def draw_shortcuts(surface, font_small):
    """Draw keyboard shortcuts overlay (always visible)."""
    shortcuts = ['R: Reset  |  G: Grid  |  T: Trail  |  ESC: Quit']
    text = font_small.render(shortcuts[0], True, SHORTCUT_COLOR)
    surface.blit(text, (WINDOW_W // 2 - text.get_width() // 2, 8))


def run_sim(node):
    """Run pygame simulation loop."""
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption('ANTBot Swerve Sim')
    clock = pygame.time.Clock()

    font = pygame.font.SysFont('monospace', 16)
    font_small = pygame.font.SysFont('monospace', 13)

    # Robot state
    robot_x = 0.0
    robot_y = 0.0
    robot_theta = math.pi / 2   # facing up initially

    trail = []
    show_grid = True
    show_trail = True
    trail_interval = 0
    running = True

    while running:
        dt = 1.0 / FPS

        # Process pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    robot_x = 0.0
                    robot_y = 0.0
                    robot_theta = math.pi / 2
                    trail.clear()
                elif event.key == pygame.K_g:
                    show_grid = not show_grid
                elif event.key == pygame.K_t:
                    show_trail = not show_trail

        # Process ROS callbacks
        rclpy.spin_once(node, timeout_sec=0)

        # Get current command
        vx, wz = node.get_cmd()

        # Integrate robot pose (RK2 for better accuracy)
        half_dtheta = wz * dt * 0.5
        mid_theta = robot_theta + half_dtheta
        robot_x += vx * math.cos(mid_theta) * dt
        robot_y += vx * math.sin(mid_theta) * dt
        robot_theta += wz * dt

        # Record trail
        trail_interval += 1
        if trail_interval >= 3:
            trail.append((robot_x, robot_y))
            trail_interval = 0
            # Limit trail length
            if len(trail) > 2000:
                trail = trail[-1500:]

        # Compute steering angles
        steering_angles = compute_steering_angles(vx, wz)

        # Camera follows robot
        cam_x = robot_x
        cam_y = robot_y

        # Draw
        screen.fill(BG_COLOR)

        if show_grid:
            draw_grid(screen, cam_x, cam_y)

        if show_trail:
            draw_trail(screen, trail, cam_x, cam_y)

        draw_robot(screen, robot_x, robot_y, robot_theta, steering_angles, cam_x, cam_y)
        draw_hud(screen, font, font_small, vx, wz, steering_angles)
        draw_shortcuts(screen, font_small)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = SwerveSimNode()
    try:
        run_sim(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
