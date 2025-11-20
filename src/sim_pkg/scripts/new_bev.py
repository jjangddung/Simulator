#!/usr/bin/env python3
import rclpy
import numpy as np
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)


class IPMNode(Node):
    def __init__(self):
        super().__init__('ipm_node')

        self.bridge = CvBridge()
        self.K = None  # camera intrinsic matrix

        # ----------------- 카메라/지면 기하 설정 -----------------
        # Gazebo SDF:
        #   - realsense_link pose: z = 0.3
        #   - wheelradius = 0.0325
        #   => 지면 기준 카메라 높이 ≈ 0.3 + 0.0325 = 0.3325 m
        self.camera_height = 0.3325      # [m] ground( z=0 )로부터 카메라까지 높이

        # SDF 에서 pitch ≈ +0.436332 rad (≈ 25도, 아래로 기울어짐)
        # 여기서는 world 좌표계에서 x:앞, y:좌, z:위라고 두고
        # 카메라를 "아래로" 기울이기 위해 pitch를 음수로 둠.
        self.pitch_deg = -25.0           # [deg] 아래를 보는 방향
        self.pitch = np.deg2rad(self.pitch_deg)

        # ----------------- BEV 이미지 설정 -----------------
        # 전방 X: 0 ~ 5 m, 좌우 Y: -2 ~ 2 m 범위를 IPM으로 펼침
        self.X_min, self.X_max = 0.0, 5.0
        self.Y_min, self.Y_max = -4.0, 4.0

        # BEV 출력 해상도 (원하는 대로 조절 가능)
        self.bev_width = 640            # X 방향 (앞뒤)
        self.bev_height = 480           # Y 방향 (좌우)
        # 참고용: 실제 해상도(m/pixel)
        self.res_x = (self.X_max - self.X_min) / self.bev_width
        self.res_y = (self.Y_max - self.Y_min) / self.bev_height

        self.get_logger().info(
            f"IPM world range X:[{self.X_min},{self.X_max}], "
            f"Y:[{self.Y_min},{self.Y_max}], "
            f"res ≈ ({self.res_x:.3f}, {self.res_y:.3f}) m/pixel"
        )

        # ----------------- QoS 설정 (sensor data: BEST_EFFORT) -----------------
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 카메라 이미지 / 카메라 파라미터 구독
        self.sub_img = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.img_callback,
            qos_profile=qos_sensor,
        )

        self.sub_info = self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.info_callback,
            qos_profile=qos_sensor,
        )

        # BEV 이미지 퍼블리셔
        self.pub_bev = self.create_publisher(
            Image,
            "/camera/color/bev_ipm",
            10,
        )

        self.get_logger().info("IPM node started (BEST_EFFORT QoS, ground z=0, cam_height=0.3325).")

    # ---------------- CameraInfo 콜백 ----------------
    def info_callback(self, msg: CameraInfo):
        if self.K is not None:
            return

        # msg.k 는 길이 9짜리 배열
        self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.get_logger().info(f"Camera K loaded:\n{self.K}")

    # ---------------- Image 콜백 ----------------
    def img_callback(self, msg: Image):
        if self.K is None:
            # 아직 camera_info 못 받았으면 스킵
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        bev = self.compute_ipm(frame)

        # 디버깅용 OpenCV 창
        cv2.imshow("camera", frame)
        cv2.imshow("bev_ipm", bev)
        cv2.waitKey(1)

        # ROS Image 퍼블리시
        bev_msg = self.bridge.cv2_to_imgmsg(bev, encoding="bgr8")
        bev_msg.header.stamp = msg.header.stamp
        bev_msg.header.frame_id = "bev_frame"
        self.pub_bev.publish(bev_msg)

    # ---------------- IPM 계산 ----------------
    def compute_ipm(self, img):
        h, w = img.shape[:2]

        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        # world 좌표계:
        #   X: 차량 앞 방향
        #   Y: 차량 좌측
        #   Z: 위쪽
        #
        # 카메라는 (0, 0, camera_height)에 있고,
        # pitch_deg 만큼 "아래"로 기울어져 있다고 가정.
        #
        # world -> camera 회전 (x축 기준 pitch)
        R_x = np.array([
            [1, 0,                  0],
            [0, np.cos(self.pitch), -np.sin(self.pitch)],
            [0, np.sin(self.pitch),  np.cos(self.pitch)],
        ], dtype=np.float32)

        # world 에서 카메라 위치 (지면 z=0 으로부터 camera_height 위)
        C_world = np.array([0.0, 0.0, self.camera_height], dtype=np.float32).reshape(3, 1)

        # world 점 Pw를 camera 로: Pc = R * (Pw - C)
        # 동차좌표계로 쓰면:
        #   [Pc] = [R | -R*C] [Pw; 1]
        t = -R_x @ C_world   # (3x1)
        Rt = np.hstack([R_x, t])  # (3x4)

        # image = K * [R|t] * [X,Y,Z,1]^T
        # 지면 평면 Z=0에서 world -> image homography:
        #   H_world2img = K * [ r1 r2 t ], r1,r2는 R의 첫 두 열
        H_world2img = self.K @ np.hstack([R_x[:, :2], t])  # (3x3)

        # ---------------- world 평면 grid 생성 (BEV pixel -> world X,Y) ----------------
        bev_W = self.bev_width
        bev_H = self.bev_height

        # X: 앞 방향 (0 ~ 5m), Y: 좌우 (-2 ~ 2m)
        xs = np.linspace(self.X_max, self.X_min, bev_W)
        ys = np.linspace(self.Y_max, self.Y_min, bev_H)  # 위쪽이 Y_min, 아래쪽이 Y_max (원하는대로 바꿀 수 있음)

        xv, yv = np.meshgrid(xs, ys)  # xv,yv shape: (bev_H, bev_W)

        ones = np.ones_like(xv, dtype=np.float32)
        world_pts = np.stack([xv, yv, ones], axis=-1).reshape(-1, 3).T  # (3, N)

        # ---------------- world -> image 투영 ----------------
        uvw = H_world2img @ world_pts  # (3, N)
        uv = uvw[:2, :] / uvw[2:3, :]  # 정규화 (u,v)

        map_u = uv[0, :].reshape(bev_H, bev_W).astype(np.float32)
        map_v = uv[1, :].reshape(bev_H, bev_W).astype(np.float32)

        # OpenCV remap 으로 IPM 결과 생성
        bev = cv2.remap(
            img,
            map_u,
            map_v,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

        return bev


def main(args=None):
    rclpy.init(args=args)
    node = IPMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
