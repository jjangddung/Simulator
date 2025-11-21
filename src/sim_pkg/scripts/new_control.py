#!/usr/bin/env python3
# coding: utf-8

import math
import json

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String
import numpy as np


class PathFollowerCmd(Node):
    def __init__(self):
        super().__init__('path_follower_cmd')

        # ===== 파라미터 =====
        # model.sdf 기준 축간거리: rear wheel(-0.152) ~ steering(0.112) → 0.264 m
        self.declare_parameter('wheelbase', 0.264)          # [m]
        self.declare_parameter('lookahead', 1.0)            # [m] Pure Pursuit lookahead
        self.declare_parameter('max_steer_deg', 20.5)       # [deg] RcBrain 기본 maxSteerAngle 근처
        # RcBrainThread에서 speed/100.0 이 실제 명령 값이므로,
        # 여기서는 그 값 자체를 직접 보내는 normalized 속도 (0.0 ~ 0.3 정도)
        self.declare_parameter('speed_cmd', 0.15)           # [unitless] ex) 0.15 → 15%

        self.L = float(self.get_parameter('wheelbase').value)
        self.Ld = float(self.get_parameter('lookahead').value)
        self.max_steer_deg = float(self.get_parameter('max_steer_deg').value)
        self.max_steer_rad = math.radians(self.max_steer_deg)
        self.speed_cmd = float(self.get_parameter('speed_cmd').value)

        # ===== 상태 =====
        self.path_array = None   # Nx2 (x,y in map)
        self.odom = None         # 마지막 odometry

        # ===== 구독 =====
        self.path_sub = self.create_subscription(
            Path,
            '/track_path',
            self.path_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            50
        )

        # ===== 퍼블리셔 (/automobile/command에 JSON String) =====
        self.cmd_pub = self.create_publisher(String, '/automobile/command', 10)

        # ===== 제어 루프 (20 Hz) =====
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'PathFollowerCmd started. L={self.L:.3f} m, Ld={self.Ld:.2f} m, '
            f'max_steer={self.max_steer_deg:.1f} deg, speed_cmd={self.speed_cmd:.2f}'
        )

    # ---------------- Path 콜백 ----------------
    def path_callback(self, msg: Path):
        pts = []
        for ps in msg.poses:
            pts.append([ps.pose.position.x, ps.pose.position.y])
        if pts:
            self.path_array = np.array(pts)  # shape (N,2)
            self.get_logger().info(f'Received path with {len(pts)} waypoints.')
        else:
            self.path_array = None
            self.get_logger().warn('Received empty path.')

    # ---------------- Odometry 콜백 ----------------
    def odom_callback(self, msg: Odometry):
        self.odom = msg

    # ---------------- 쿼터니언 → yaw ----------------
    def quat_to_yaw(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ---------------- 제어 루프 ----------------
    def control_loop(self):
        # path 또는 odom이 없으면 아무것도 안 함
        if self.path_array is None or self.odom is None:
            return

        # ==== 1. 현재 로봇 pose (map frame) ====
        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        yaw = self.quat_to_yaw(self.odom.pose.pose.orientation)

        # ==== 2. path 상에서 현재 위치에 가장 가까운 점 찾기 ====
        dx = self.path_array[:, 0] - px
        dy = self.path_array[:, 1] - py
        d2 = dx * dx + dy * dy

        idx_near = int(np.argmin(d2))

        # path 끝에 가까워지면 브레이크
        if idx_near >= len(self.path_array) - 2:
            self.publish_brake()
            self.get_logger().info('Reached end of path. Braking.')
            return

        # ==== 3. near 이후의 점들만 대상으로 lookahead 후보 만든다 ====
        candidates_map = self.path_array[idx_near:]

        # map → base_link 변환
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        bl_points = []
        for p in candidates_map:
            dx = p[0] - px
            dy = p[1] - py
            # base_link: x 앞으로, y 왼쪽(+)
            x_bl = cos_y * dx + sin_y * dy
            y_bl = -sin_y * dx + cos_y * dy
            bl_points.append([x_bl, y_bl])

        bl_points = np.array(bl_points)

        # 앞쪽 (x>0)만 사용
        forward_mask = bl_points[:, 0] > 0.1
        bl_forward = bl_points[forward_mask]

        if bl_forward.shape[0] == 0:
            # 앞에 포인트가 없으면 안전하게 브레이크
            self.get_logger().warn('No forward points found, braking.')
            self.publish_brake()
            return

        # ==== 4. lookahead 거리와 가장 가까운 포인트 선택 ====
        dists = np.sqrt(bl_forward[:, 0] ** 2 + bl_forward[:, 1] ** 2)
        idx_Ld = int(np.argmin(np.abs(dists - self.Ld)))

        xL = float(bl_forward[idx_Ld, 0])
        yL = float(bl_forward[idx_Ld, 1])
        Ld = max(float(dists[idx_Ld]), 0.01)

        # ==== 5. Pure Pursuit 조향각 계산 (rad) ====
        # δ = atan(2 L y / Ld^2)
        delta_rad = math.atan2(2.0 * self.L * yL, Ld ** 2)

        # 제한 (rad)
        if delta_rad > self.max_steer_rad:
            delta_rad = self.max_steer_rad
        elif delta_rad < -self.max_steer_rad:
            delta_rad = -self.max_steer_rad

        # deg로 변환 (RcBrainThread는 deg 기반)
        delta_deg = math.degrees(delta_rad)

        # ==== 6. 속도는 고정 speed_cmd 에서 lateral 오차에 따라 살짝 조절 ====
        lat_err = abs(yL)
        speed_val = self.speed_cmd / (1.0 + 2.0 * lat_err)  # y 커지면 살짝 감속
        speed_val = max(0.0, min(speed_val, 0.3))  # 상한선 0.3 정도로 클램프

        # ==== 7. /automobile/command 로 JSON 전송 ====
        # RcBrainThread._stateDict() 형식에 맞춤:
        #   - 속도 명령: {"action":"1","speed":<float>}
        #   - 조향 명령: {"action":"2","steerAngle":<deg>}
        self.publish_speed(speed_val)
        self.publish_steering(-delta_deg)

        self.get_logger().debug(
            f'near_idx={idx_near}, target_bl=({xL:.2f},{yL:.2f}), '
            f'Ld={Ld:.2f}, steer={delta_deg:.2f} deg, speed={speed_val:.2f}'
        )

    # ---------------- command helper들 ----------------
    def publish_speed(self, speed_val: float):
        cmd = {
            "action": "1",
            "speed": float(speed_val)  # RcBrainThread 기준 speed/100.0
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)

    def publish_steering(self, steer_deg: float):
        cmd = {
            "action": "2",
            "steerAngle": float(steer_deg)  # RcBrainThread.steerAngle (deg)
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)

    def publish_brake(self):
        cmd = {
            "action": "3",
            "steerAngle": 0.0
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
