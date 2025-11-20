#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
import numpy as np


class CamPointsProjector(Node):
    def __init__(self):
        super().__init__('cam_points_projector')

        # ==========================
        # 🔧 토픽 이름 꼭 확인하세요
        # 터미널에서:
        #   ros2 topic list | grep camera
        # 같은 걸로 실제 이름 보고 맞추기
        # ==========================
        image_topic = '/camera/color/image_raw'
        caminfo_topic = '/camera/color/camera_info'
        cloud_topic = '/camera/depth/color/points'  # 실제 pointcloud 토픽 이름으로 변경 필요

        # Gazebo Realsense는 보통 SensorData QoS (BEST_EFFORT) 사용
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

        # 최신 이미지 저장용
        self.latest_image = None

        # 로그를 한 번만 찍기 위한 플래그
        self.image_logged = False
        self.cloud_logged = False

        # 구독자들
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_sensor
        )
        self.caminfo_sub = self.create_subscription(
            CameraInfo, caminfo_topic, self.caminfo_callback, qos_sensor
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_callback, qos_sensor
        )

        # rqt_image_view에서 확인할 디버그 이미지 퍼블리셔
        self.debug_img_pub = self.create_publisher(
            Image, '/debug/cam_points_image', 10
        )

        self.get_logger().info('CamPointsProjector node started.')

    # -------------------------------
    # CameraInfo 콜백: K 행렬에서 fx, fy, cx, cy 읽기
    # -------------------------------
    def caminfo_callback(self, msg: CameraInfo):
        # CameraInfo.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]

        # 계속 들어와도 크게 문제는 없지만, 많아지면 시끄러우니 한 번만 출력
        self.get_logger().info(
            f'[caminfo_callback] fx={self.fx:.2f}, fy={self.fy:.2f}, '
            f'cx={self.cx:.2f}, cy={self.cy:.2f}'
        )

    # -------------------------------
    # 이미지 콜백: 최신 이미지 저장
    # -------------------------------
    def image_callback(self, msg: Image):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if not self.image_logged:
            self.get_logger().info('[image_callback] First image received.')
            self.image_logged = True

    # -------------------------------
    # PointCloud 콜백: 포인트를 이미지 위에 사영
    # -------------------------------
    def cloud_callback(self, msg: PointCloud2):
        # 카메라 파라미터나 이미지가 아직 준비 안 됐으면 패스
        if self.fx is None or self.latest_image is None:
            self.get_logger().debug('[cloud_callback] Waiting for camera info or image...')
            return

        if not self.cloud_logged:
            self.get_logger().info('[cloud_callback] First pointcloud received.')
            self.cloud_logged = True

        img = self.latest_image.copy()
        h, w = img.shape[:2]

        # PointCloud2 -> (x, y, z) 반복자
        # field_names는 실제 pointcloud 구조에 맞게 조정 가능
        points_iter = pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )

        # 성능을 위해 subsample
        step = 10  # 10개 중 1개만 그림
        for i, p in enumerate(points_iter):
            if i % step != 0:
                continue

            Xc, Yc, Zc = p  # 이미 camera_optical_frame 기준이라고 가정

            # 카메라 앞에 있고 너무 멀지 않은 점만 사용
            if Zc <= 0.1 or Zc > 2.0:
                continue

            # pinhole projection
            u = self.fx * (Xc / Zc) + self.cx
            v = self.fy * (Yc / Zc) + self.cy

            u_i = int(round(u))
            v_i = int(round(v))

            if 0 <= u_i < w and 0 <= v_i < h:
                # 빨간 점 표시
                cv2.circle(img, (u_i, v_i), 2, (0, 0, 255), -1)

        # rqt_image_view용 디버그 이미지 퍼블리시
        debug_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        # 시간 동기화용으로 header 복사 (옵션)
        debug_msg.header = msg.header
        self.debug_img_pub.publish(debug_msg)

        # GUI가 있는 환경이라면 imshow로도 표시
        try:
            cv2.imshow('points on image', img)
            cv2.waitKey(1)
        except Exception as e:
            # DISPLAY 없는 환경(ssh, Docker 등)에서 imshow 에러 방지
            self.get_logger().warn(f'cv2.imshow error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CamPointsProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
