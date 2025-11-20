#!/usr/bin/env python3
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


def rpy_to_rotmat(roll, pitch, yaw):
    """R = Rz(yaw) * Ry(pitch) * Rx(roll)"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])

    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]])

    Rx = np.array([[1,  0,   0],
                   [0, cr, -sr],
                   [0, sr,  cr]])

    return Rz @ Ry @ Rx


class DepthColorBEV(Node):
    def __init__(self):
        super().__init__('depth_color_bev')

        self.bridge = CvBridge()

        # ================== 카메라 내파라 (초기값, 필요하면 launch에서 바꾸기) ==================
        # RealSense D455 기준 대략 값. 정확하게 쓰려면 /camera/color/camera_info 참고해서 세팅.
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 424.0)
        self.declare_parameter('cy', 240.0)

        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        # ================== extrinsic: camera -> base_link (SDF 기준) ==================
        # <pose>0 0 0.3 0 0.436332 0</pose>  (x, y, z, roll, pitch, yaw)
        self.declare_parameter('tx', 0.0)
        self.declare_parameter('ty', 0.0)
        self.declare_parameter('tz', 0.3)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.436332)  # 약 25 deg
        self.declare_parameter('yaw', 0.0)

        tx = self.get_parameter('tx').value
        ty = self.get_parameter('ty').value
        tz = self.get_parameter('tz').value
        roll = self.get_parameter('roll').value
        pitch = self.get_parameter('pitch').value
        yaw = self.get_parameter('yaw').value

        self.R = rpy_to_rotmat(roll, pitch, yaw)
        self.t = np.array([tx, ty, tz])

        # ================== BEV 영역 (base_link 기준) ==================
        self.declare_parameter('x_min', 0.0)
        self.declare_parameter('x_max', 8.0)
        self.declare_parameter('y_min', -4.0)
        self.declare_parameter('y_max', 4.0)
        self.declare_parameter('resolution', 0.05)

        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.res = self.get_parameter('resolution').value

        # 지면으로 쓸 Z 범위 (base_link 기준)
        self.declare_parameter('z_min', -0.2)
        self.declare_parameter('z_max', 0.05)
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value

        # BEV 이미지 크기 (픽셀)
        self.W = int(math.ceil((self.x_max - self.x_min) / self.res))  # cols
        self.H = int(math.ceil((self.y_max - self.y_min) / self.res))  # rows

        self.get_logger().info(
            f"BEV size = {self.W} x {self.H}, res={self.res}, "
            f"X:[{self.x_min},{self.x_max}], Y:[{self.y_min},{self.y_max}]"
        )
        self.get_logger().info(
            f"Extrinsic (camera->base_link): t={self.t}, roll={roll}, pitch={pitch}, yaw={yaw}"
        )

        # ================== QoS: RealSense/Gazebo 센서 토픽에 맞춰 BEST_EFFORT ==================
        self.declare_parameter('qos_reliability', 'best_effort')
        qos_reliability = self.get_parameter('qos_reliability').value.lower()
        if qos_reliability not in ('reliable', 'best_effort'):
            self.get_logger().warn(
                f"Unknown qos_reliability '{qos_reliability}', defaulting to 'reliable'"
            )
            qos_reliability = 'reliable'
        reliability_policy = (
            QoSReliabilityPolicy.RELIABLE
            if qos_reliability == 'reliable'
            else QoSReliabilityPolicy.BEST_EFFORT
        )

        qos_sensor = QoSProfile(
            reliability=reliability_policy,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ================== depth + color 동기화 구독 ==================
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            qos_profile=qos_sensor
        )
        self.color_sub = message_filters.Subscriber(
            self,
            Image,
            "/camera/color/image_raw",
            qos_profile=qos_sensor
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.color_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        # ================== BEV 컬러 이미지 퍼블리셔 ==================
        self.pub_bev = self.create_publisher(Image, "/bev_color_image", 10)

    def sync_callback(self, depth_msg: Image, color_msg: Image):
        # depth: 보통 16UC1(mm) 또는 32FC1(m)
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')

        # depth를 meter 단위 float32로 맞추기
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0  # mm -> m
        else:
            depth_m = depth.astype(np.float32)           # 이미 m 라고 가정

        h, w = depth_m.shape

        # 픽셀 좌표 그리드 (u,v)
        u = np.arange(w, dtype=np.float32)
        v = np.arange(h, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)

        Zc = depth_m
        # 유효 depth 마스크
        valid = (Zc > 0.1) & np.isfinite(Zc) & (Zc < 15.0)
        if not np.any(valid):
            return

        Zc_valid = Zc[valid]
        uu_valid = uu[valid]
        vv_valid = vv[valid]

        # === camera frame 3D 좌표 ===
        Xc = (uu_valid - self.cx) / self.fx * Zc_valid
        Yc = (vv_valid - self.cy) / self.fy * Zc_valid

        Pc = np.stack([Xc, Yc, Zc_valid], axis=0)    # (3, N)

        # === camera -> base_link ===
        Pb = self.R @ Pc + self.t.reshape(3, 1)
        Xb, Yb, Zb = Pb[0, :], Pb[1, :], Pb[2, :]

        # === 관심 영역 필터링 (X, Y, Z 범위) ===
        mask_roi = (
            (Xb >= self.x_min) & (Xb <= self.x_max) &
            (Yb >= self.y_min) & (Yb <= self.y_max) &
            (Zb >= self.z_min) & (Zb <= self.z_max)
        )

        if not np.any(mask_roi):
            # 아무것도 없으면 그냥 리턴
            return

        Xb = Xb[mask_roi]
        Yb = Yb[mask_roi]
        Zb = Zb[mask_roi]
        uu_valid = uu_valid[mask_roi]
        vv_valid = vv_valid[mask_roi]

        # === BEV 인덱스 계산 ===
        # X -> col (i), Y -> row (j)
        i = ((Xb - self.x_min) / self.res).astype(np.int32)
        j = ((Yb - self.y_min) / self.res).astype(np.int32)

        valid_idx = (
            (i >= 0) & (i < self.W) &
            (j >= 0) & (j < self.H)
        )
        if not np.any(valid_idx):
            return

        i = i[valid_idx]
        j = j[valid_idx]
        uu_valid = uu_valid[valid_idx]
        vv_valid = vv_valid[valid_idx]

        # === BEV 컬러 이미지 생성 (BGR) ===
        bev = np.zeros((self.H, self.W, 3), dtype=np.uint8)

        # 여러 점이 같은 cell에 들어가면 나중 값이 덮어씀
        bev[j, i] = color[vv_valid.astype(np.int32), uu_valid.astype(np.int32)]

        # ========= OpenCV로 바로 확인 =========
        cv2.imshow("BEV_color", bev)
        cv2.waitKey(1)

        # ========= ROS Image로 퍼블리시 =========
        bev_msg = self.bridge.cv2_to_imgmsg(bev, encoding="bgr8")
        bev_msg.header.stamp = self.get_clock().now().to_msg()
        bev_msg.header.frame_id = "base_link"
        self.pub_bev.publish(bev_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthColorBEV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
