#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np







class BevTunerNode(Node):
    def __init__(self):
        super().__init__('bev_tuner_node')

        # 파라미터
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('bev_width', 640)
        self.declare_parameter('bev_height', 480)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.bev_width = self.get_parameter('bev_width').get_parameter_value().integer_value
        self.bev_height = self.get_parameter('bev_height').get_parameter_value().integer_value

        # 이미지 구독 (sensor_data QoS)
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.bridge = CvBridge()
        self.last_image = None
        self.last_header = None
        self.first_image_logged = False  # 첫 이미지 들어왔는지 확인용

        # OpenCV 창 & 트랙바 설정
        self.setup_windows()

        # 30Hz 타이머로 update_view 호출
        self.timer = self.create_timer(1.0 / 30.0, self.update_view)

        self.get_logger().info(f"BEV tuner started. Subscribing: {self.image_topic}")

    def setup_windows(self):
        cv2.namedWindow("original", cv2.WINDOW_NORMAL)
        cv2.namedWindow("bev", cv2.WINDOW_NORMAL)
        cv2.namedWindow("BEV_Config", cv2.WINDOW_NORMAL)

        def nothing(x):
            pass

        # 57
        # 85
        # 25
        # 71
        # 5
        # 89

        #시야각 90 
        # 65
        # 100
        # 28
        # 70
        # 10
        # 86


        #49
        #84
        #27
        #73
        #0
        #100


        cv2.createTrackbar("top_y", "BEV_Config", 53, 100, nothing)
        cv2.createTrackbar("bottom_y", "BEV_Config", 76, 100, nothing)

        cv2.createTrackbar("left_top_x", "BEV_Config", 36, 100, nothing)
        cv2.createTrackbar("right_top_x", "BEV_Config", 63, 100, nothing)

        cv2.createTrackbar("left_bottom_x", "BEV_Config", 26, 100, nothing)
        cv2.createTrackbar("right_bottom_x", "BEV_Config", 72, 100, nothing)

    def image_callback(self, msg: Image):
        try:
            # cv_image = self.bridge.to_cv2(msg, desired_encoding='bgr8')
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        self.last_image = cv_image
        self.last_header = msg.header

        if not self.first_image_logged:
            h, w = cv_image.shape[:2]
            self.get_logger().info(f"First image received: {w}x{h}")
            self.first_image_logged = True

    def update_view(self):
        # ESC 눌러도 timer는 계속 돌기 때문에, 여기에서 종료 처리
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            self.get_logger().info("ESC pressed, shutting down node...")
            # node를 destroy하면 main 쪽 spin이 빠져나옴
            self.destroy_node()
            return

        if self.last_image is None:
            # 아직 이미지 안 들어오면 텍스트만 표시
            blank = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(blank, "No image yet...",
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            cv2.imshow("original", blank)
            cv2.imshow("bev", blank)
            return

        frame = self.last_image.copy()
        h, w = frame.shape[:2]

        # 트랙바 값 읽기 (0~100% → 픽셀)
        top_y_percent = cv2.getTrackbarPos("top_y", "BEV_Config")
        bottom_y_percent = cv2.getTrackbarPos("bottom_y", "BEV_Config")
        left_top_x_percent = cv2.getTrackbarPos("left_top_x", "BEV_Config")
        right_top_x_percent = cv2.getTrackbarPos("right_top_x", "BEV_Config")
        left_bottom_x_percent = cv2.getTrackbarPos("left_bottom_x", "BEV_Config")
        right_bottom_x_percent = cv2.getTrackbarPos("right_bottom_x", "BEV_Config")

        top_y = int(h * top_y_percent / 100.0)
        bottom_y = int(h * bottom_y_percent / 100.0)
        left_top_x = int(w * left_top_x_percent / 100.0)
        right_top_x = int(w * right_top_x_percent / 100.0)
        left_bottom_x = int(w * left_bottom_x_percent / 100.0)
        right_bottom_x = int(w * right_bottom_x_percent / 100.0)

        # src 사다리꼴 4점 (좌하, 우하, 우상, 좌상)
        src_points = np.float32([
            [left_bottom_x, bottom_y],
            [right_bottom_x, bottom_y],
            [right_top_x, top_y],
            [left_top_x, top_y]
        ])

        dst_points = np.float32([
            [0, self.bev_height],
            [self.bev_width, self.bev_height],
            [self.bev_width, 0],
            [0, 0]
        ])

        # Homography 계산
        try:
            H, status = cv2.findHomography(src_points, dst_points)
        except Exception as e:
            self.get_logger().error(f"findHomography error: {e}")
            return

        if H is None:
            # H 잘못되면 그냥 원본만 보여줌
            self.get_logger().warn("Homography is None, check src_points")
            cv2.imshow("original", frame)
            cv2.imshow("bev", np.zeros((self.bev_height, self.bev_width, 3), dtype=np.uint8))
            return

        bev = cv2.warpPerspective(frame, H, (self.bev_width, self.bev_height))
        # bev = cv2.warpPerspective(frame, H, (1080, 720))
        

        # 원본에 사다리꼴 그리기
        pts = src_points.astype(int)
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        # 디버그 텍스트 (이미지 크기 등)
        cv2.putText(frame, f"{w}x{h}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("original", frame)
        cv2.imshow("bev", bev)


def main(args=None):
    rclpy.init(args=args)
    node = BevTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
