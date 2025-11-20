#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
import numpy as np
import math


class LanePointProjector(Node):
    def __init__(self):
        super().__init__('lane_point_projector')

        # 🔧 실제 토픽 이름에 맞게 수정하세요
        color_image_topic = '/camera/color/image_raw'
        caminfo_topic = '/camera/color/camera_info'
        cloud_topic = '/camera/depth/color/points'

        qos_sensor = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.bridge = CvBridge()

        # 카메라 내부 파라미터
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # 최신 이미지 / 흰색 마스크
        self.latest_image = None
        self.white_mask = None

        # 로깅 플래그
        self.image_logged = False
        self.cloud_logged = False

        # SDF 기반 카메라→base_link 변환 (고정 값)
        # <pose>0 0 0.3 0 0.436332 0</pose>  (x y z roll pitch yaw)
        self.cam_tx = 0.0
        self.cam_ty = 0.0
        self.cam_tz = 0.3       # 카메라 높이
        self.cam_roll = 0.0
        self.cam_pitch = 0.436332   # 약 25도 (앞으로 숙인 각도)
        self.cam_yaw = 0.0

        self.get_logger().info('LanePointProjector (SDF-based) node started.')

        # 구독
        self.image_sub = self.create_subscription(
            Image, color_image_topic, self.image_callback, qos_sensor
        )
        self.caminfo_sub = self.create_subscription(
            CameraInfo, caminfo_topic, self.caminfo_callback, qos_sensor
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_callback, qos_sensor
        )

        # 퍼블리셔
        self.debug_img_pub = self.create_publisher(
            Image, '/debug/lane_points_image', 10
        )
        # camera_depth_optical_frame 기준 차선 포인트
        self.lane_cloud_cam_pub = self.create_publisher(
            PointCloud2, '/camera/depth/lane_points_camera', 10
        )
        # base_link 기준 차선 포인트
        self.lane_cloud_base_pub = self.create_publisher(
            PointCloud2, '/camera/depth/lane_points_base_sdf', 10
        )
        # base_link 기준 중앙선 포인트
        self.center_cloud_base_pub = self.create_publisher(
            PointCloud2, '/camera/depth/lane_center_base_sdf', 10
        )

    # ---------------- CameraInfo ----------------
    def caminfo_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]
        self.get_logger().info(
            f'[caminfo_callback] fx={self.fx:.2f}, fy={self.fy:.2f}, '
            f'cx={self.cx:.2f}, cy={self.cy:.2f}'
        )

    # ---------------- 이미지 콜백: 흰 차선 마스크 ----------------
    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = img

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 🔧 흰 차선 threshold (필요시 조정)
        lower_white = np.array([0, 0, 180], dtype=np.uint8)
        upper_white = np.array([180, 60, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        self.white_mask = mask

        if not self.image_logged:
            self.get_logger().info('[image_callback] First image & white mask generated.')
            self.image_logged = True

    # ---------------- PointCloud 콜백 ----------------
    def cloud_callback(self, msg: PointCloud2):
        if self.fx is None or self.latest_image is None or self.white_mask is None:
            self.get_logger().debug('[cloud_callback] Waiting for camera info / image / mask...')
            return

        if not self.cloud_logged:
            self.get_logger().info('[cloud_callback] First pointcloud received.')
            self.cloud_logged = True

        img = self.latest_image.copy()
        mask = self.white_mask
        h, w = img.shape[:2]

        lane_points_cam = []   # camera_depth_optical_frame 기준
        lane_points_base = []  # base_link 기준

        points_iter = pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )

        step = 5  # 샘플링 간격 (필요하면 1로 줄이기)
        for i, p in enumerate(points_iter):
            if i % step != 0:
                continue

            Xc, Yc, Zc = p  # frame_id = camera_depth_optical_frame (optical frame)

            # 너무 가까운/먼 포인트 제거 (필요시 조정)
            if Zc <= 0.6 or Zc > 2.0 or Xc > 0.4 or Xc < -0.4:
                continue

            # 카메라 intrinsics로 이미지 좌표 투영
            u = self.fx * (Xc / Zc) + self.cx
            v = self.fy * (Yc / Zc) + self.cy

            u_i = int(round(u))
            v_i = int(round(v))

            if 0 <= u_i < w and 0 <= v_i < h:
                if mask[v_i, u_i] > 0:
                    # 이미지에 초록 점 (차선 위 포인트)
                    cv2.circle(img, (u_i, v_i), 2, (0, 255, 0), -1)

                    # 1) optical frame 기준 포인트 저장
                    lane_points_cam.append((Xc, Yc, Zc))

                    # 2) optical → camera_link 변환 (REP-103 optical 규약)
                    # optical: X-right, Y-down, Z-forward
                    # camera_link: X-forward, Y-left, Z-up
                    Xl =  Zc
                    Yl = -Xc
                    Zl = -Yc

                    # 3) camera_link → base_link (SDF pitch 적용)
                    theta = self.cam_pitch  # 0.436332 rad

                    # pitch는 camera_link의 Y축 기준 회전이라고 가정
                    Xb =  math.cos(theta) * Xl + math.sin(theta) * Zl
                    Yb =  Yl
                    Zb = -math.sin(theta) * Xl + math.cos(theta) * Zl

                    # 카메라 높이만큼 z 이동
                    Zb += self.cam_tz  # 0.3 m

                    lane_points_base.append((Xb, Yb, Zb))

        # ---------------- base_link 기준 좌/우 차선 분리 ----------------
        left_lane = []
        right_lane = []
        for Xb, Yb, Zb in lane_points_base:
            # 조금의 여유 margin (노이즈 제거)
            if Yb > 0.05:
                left_lane.append((Xb, Yb, Zb))
            elif Yb < -0.05:
                right_lane.append((Xb, Yb, Zb))

        # ---------------- 중앙선 계산 ----------------
        center_points = []
        if left_lane and right_lane:
            # X 방향으로 binning (가까운 곳부터 일정 간격으로)
            x_min = 0.2
            x_max = 4.0    # 앞 5m 까지 중앙선 사용 (필요시 조정)
            bin_size = 0.2 # 20cm 간격

            bins = np.arange(x_min, x_max, bin_size)

            left_arr = np.array(left_lane)   # shape (N,3)
            right_arr = np.array(right_lane) # shape (M,3)

            for xb in bins:
                # 현재 bin 구간 [xb, xb+bin_size)
                l_mask = (left_arr[:, 0] >= xb) & (left_arr[:, 0] < xb + bin_size)
                r_mask = (right_arr[:, 0] >= xb) & (right_arr[:, 0] < xb + bin_size)

                if not np.any(l_mask) or not np.any(r_mask):
                    continue

                # 해당 bin 안에서 평균점 사용 (혹은 np.argmin 등으로 가장 가까운 점만 사용할 수도 있음)
                l_mean = left_arr[l_mask].mean(axis=0)
                r_mean = right_arr[r_mask].mean(axis=0)

                cx = 0.5 * (l_mean[0] + r_mean[0])
                cy = 0.5 * (l_mean[1] + r_mean[1])
                cz = 0.5 * (l_mean[2] + r_mean[2])

                center_points.append((cx, cy, cz))

            # 가장 가까운 중앙점 (차량 기준)
            if center_points:
                center_arr = np.array(center_points)
                dists = np.sqrt(center_arr[:, 0]**2 + center_arr[:, 1]**2)
                idx_min = int(np.argmin(dists))
                closest_center = center_arr[idx_min]
                self.get_logger().info(
                    f'Closest center point (base_link) = '
                    f'({closest_center[0]:.2f}, {closest_center[1]:.2f}, {closest_center[2]:.2f})'
                )

        # ---------------- camera frame 기준 lane cloud publish ----------------
        if lane_points_cam:
            header_cam = Header()
            header_cam.stamp = msg.header.stamp
            header_cam.frame_id = msg.header.frame_id  # camera_depth_optical_frame

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]

            lane_cloud_cam = pc2.create_cloud(header_cam, fields, lane_points_cam)
            self.lane_cloud_cam_pub.publish(lane_cloud_cam)

        # ---------------- base_link 기준 lane cloud publish ----------------
        if lane_points_base:
            header_base = Header()
            header_base.stamp = msg.header.stamp
            header_base.frame_id = 'base_link'  # 🔧 실제 base 프레임 이름에 맞게 변경 가능

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]

            lane_cloud_base = pc2.create_cloud(header_base, fields, lane_points_base)
            self.lane_cloud_base_pub.publish(lane_cloud_base)

        # ---------------- base_link 기준 centerline cloud publish ----------------
        if center_points:
            header_center = Header()
            header_center.stamp = msg.header.stamp
            header_center.frame_id = 'base_link'

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]

            center_cloud = pc2.create_cloud(header_center, fields, center_points)
            self.center_cloud_base_pub.publish(center_cloud)

        # 디버그 이미지 (차선 포인트만 시각화)
        debug_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_img_pub.publish(debug_msg)

        try:
            cv2.imshow('lane points on image', img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f'cv2.imshow error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LanePointProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
            pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
