#!/usr/bin/env python3
import math
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__('initial_pose_publisher')

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('initial_roll', 0.0)
        self.declare_parameter('initial_pitch', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('covariance_diagonal', 0.01)
        self.declare_parameter('service_name', '/set_pose')

        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_z = float(self.get_parameter('initial_z').value)
        self.initial_roll = float(self.get_parameter('initial_roll').value)
        self.initial_pitch = float(self.get_parameter('initial_pitch').value)
        self.initial_yaw = float(self.get_parameter('initial_yaw').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)
        self.cov_diag = float(self.get_parameter('covariance_diagonal').value)
        self.service_name = str(self.get_parameter('service_name').value)

        self._client = self.create_client(SetPose, self.service_name)
        self._sent = False
        self._future: Optional[rclpy.task.Future] = None

        self._timer = self.create_timer(0.5, self._try_send_request)
        self.get_logger().info(
            f"Waiting to set initial pose at ({self.initial_x:.3f}, "
            f"{self.initial_y:.3f}, {self.initial_z:.3f}) yaw={self.initial_yaw:.3f} "
            f"on service {self.service_name}"
        )

    def _try_send_request(self) -> None:
        if self._sent:
            return

        if not self._client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(f'Service {self.service_name} not available, waiting...')
            return

        request = SetPose.Request()
        request.pose = self._build_pose_msg()
        self._future = self._client.call_async(request)
        self._future.add_done_callback(self._handle_result)
        self._sent = True
        self.get_logger().info(f'Initial pose request sent to {self.service_name}')

    def _build_pose_msg(self) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.position.z = self.initial_z

        q = self._quaternion_from_rpy(self.initial_roll, self.initial_pitch, self.initial_yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        cov = [0.0] * 36
        diag = self.cov_diag
        for idx in (0, 7, 14, 21, 28, 35):
            cov[idx] = diag
        msg.pose.covariance = cov
        return msg

    @staticmethod
    def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def _handle_result(self, future: rclpy.task.Future) -> None:
        try:
            future.result()
            self.get_logger().info('Initial pose successfully set.')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('Failed to set initial pose: %s', exc)
        finally:
            self._timer.cancel()
            self.destroy_node()
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Initial pose publisher interrupted.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
