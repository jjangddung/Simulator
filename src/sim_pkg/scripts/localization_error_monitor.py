#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from utils.msg import Localisation
import tf_transformations


class LocalizationErrorMonitor(Node):
    """Compare EKF odometry with GPS ground truth and publish the error."""

    def __init__(self) -> None:
        super().__init__('localization_error_monitor')

        # self.declare_parameter('ground_truth_source', 'gps')
        self.declare_parameter('ground_truth_source', 'gazebo')
        self.declare_parameter('gps_topic', '/automobile/localization')
        self.declare_parameter('model_state_topic', 'model_states')
        self.declare_parameter('model_state_model_name', 'automobile')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('error_topic', '/debug/localization_error')
        self.declare_parameter('log_period', 0.5)

        self.ground_truth_source = str(self.get_parameter('ground_truth_source').value).lower()
        self.gps_topic = str(self.get_parameter('gps_topic').value)
        self.model_state_topic = str(self.get_parameter('model_state_topic').value)
        self.model_state_model_name = str(self.get_parameter('model_state_model_name').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.error_topic = str(self.get_parameter('error_topic').value)
        self.log_period = float(self.get_parameter('log_period').value)

        self.last_ground_truth: Optional[tuple[float, float, float]] = None
        self.last_odom: Optional[tuple[float, float, float]] = None
        self.model_state_missing_warned = False

        self.error_pub = self.create_publisher(Vector3Stamped, self.error_topic, 10)
        if self.ground_truth_source == 'gazebo':
            self.create_subscription(ModelStates, self.model_state_topic, self.on_model_states, 10)
        else:
            self.create_subscription(Localisation, self.gps_topic, self.on_gps, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.timer = self.create_timer(self.log_period, self.report_error)

        self.get_logger().info(
            self._comparison_message()
        )

    def on_gps(self, msg: Localisation) -> None:
        self.last_ground_truth = (float(msg.pos_a), float(msg.pos_b), float(msg.rot_a))

    def on_model_states(self, msg: ModelStates) -> None:
        try:
            idx = msg.name.index(self.model_state_model_name)
        except ValueError:
            if not self.model_state_missing_warned:
                self.get_logger().warning(
                    "Model '%s' not found in '%s'. Waiting for ground truth.",
                    self.model_state_model_name,
                    self.model_state_topic,
                )
                self.model_state_missing_warned = True
            return

        pose = msg.pose[idx]
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)
        self.model_state_missing_warned = False
        self.last_ground_truth = (pose.position.x, pose.position.y, yaw)

    def on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)
        self.last_odom = (pose.position.x, pose.position.y, yaw)

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def report_error(self) -> None:
        if not (self.last_ground_truth and self.last_odom):
            return

        odom_x, odom_y, odom_yaw = self.last_odom
        gt_x, gt_y, gt_yaw = self.last_ground_truth

        dx = odom_x - gt_x
        dy = odom_y - gt_y
        dyaw = self.wrap_angle(odom_yaw - gt_yaw)

        error_vec = Vector3Stamped()
        error_vec.header.stamp = self.get_clock().now().to_msg()
        error_vec.header.frame_id = 'odom'
        error_vec.vector.x = dx
        error_vec.vector.y = dy
        error_vec.vector.z = dyaw
        self.error_pub.publish(error_vec)

        distance_error = math.hypot(dx, dy)
        self.get_logger().info(
            f"Localization error -> dx: {dx:.3f} m, dy: {dy:.3f} m, |e|: {distance_error:.3f} m, dyaw: {math.degrees(dyaw):.2f} deg"
        )

    def _comparison_message(self) -> str:
        if self.ground_truth_source == 'gazebo':
            return (
                f"Comparing EKF odom '{self.odom_topic}' against Gazebo model state "
                f"'{self.model_state_topic}' (model='{self.model_state_model_name}')."
            )
        return f"Comparing EKF odom '{self.odom_topic}' against GPS '{self.gps_topic}'."


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationErrorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
