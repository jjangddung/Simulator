#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf_transformations
import tf2_ros


class WheelImuOdom(Node):
    """Integrate wheel speed with IMU yaw to publish odometry."""

    def __init__(self) -> None:
        super().__init__('wheel_imu_odom')

        # Parameters
        self.declare_parameter('wheel_twist_topic', '/automobile/wheel_twist')
        self.declare_parameter('imu_topic', '/automobile/imu_raw')
        self.declare_parameter('odom_topic', '/automobile/wheel_imu_odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('covariance_diagonal', 0.5)
        self.declare_parameter('speed_deadband', 0.01)
        self.declare_parameter('yaw_deadband_deg', 2.0)

        # State
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.latest_yaw: Optional[float] = None
        self.last_yaw_for_rate: Optional[float] = None
        self.last_stamp: Optional[rclpy.time.Time] = None
        self.last_wall_time: Optional[rclpy.time.Time] = None
        self.current_speed = 0.0
        self.current_yaw_rate = 0.0

        # Topics and frames
        self.odom_frame = str(self.get_parameter('frame_id').value)
        self.child_frame = str(self.get_parameter('child_frame_id').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cov_diag = float(self.get_parameter('covariance_diagonal').value)
        self.speed_deadband = float(self.get_parameter('speed_deadband').value)
        self.yaw_deadband = math.radians(float(self.get_parameter('yaw_deadband_deg').value))

        # Publishers / TF
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_br = tf2_ros.TransformBroadcaster(self)

        # Subscriptions
        wheel_topic = str(self.get_parameter('wheel_twist_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        self.create_subscription(TwistWithCovarianceStamped, wheel_topic, self.wheel_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)

        self.get_logger().info(
            f"WheelImuOdom listening to {wheel_topic} and {imu_topic}, publishing {self.odom_topic}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def imu_callback(self, msg: Imu) -> None:
        quat = msg.orientation
        q = (quat.x, quat.y, quat.z, quat.w)
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        self.latest_yaw = yaw

    def wheel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        current_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        now_wall = self.get_clock().now()
        if self.last_stamp is None:
            self.last_stamp = current_stamp
            self.current_speed = msg.twist.twist.linear.x # 그냥 차량의 속도임
            if self.latest_yaw is not None:
                self.yaw = self.latest_yaw
                self.last_yaw_for_rate = self.latest_yaw
            self.last_wall_time = now_wall
            return

        dt = (current_stamp - self.last_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            if self.last_wall_time is not None:
                dt = (now_wall - self.last_wall_time).nanoseconds * 1e-9
            if dt <= 0.0:
                self.last_stamp = current_stamp
                self.last_wall_time = now_wall
                return

        v = msg.twist.twist.linear.x
        if abs(v) < self.speed_deadband:
            v = 0.0
        yaw_updated = False
        if self.latest_yaw is not None:
            yaw_diff = self.wrap_angle(self.latest_yaw - self.yaw)
            if abs(yaw_diff) >= self.yaw_deadband:
                yaw_for_update = self.latest_yaw
                yaw_updated = True
            else:
                yaw_for_update = self.yaw
        else:
            yaw_for_update = self.yaw

        dx = v * dt * math.cos(yaw_for_update)
        dy = v * dt * math.sin(yaw_for_update)

        self.x += dx
        self.y += dy
        self.current_speed = v
        if yaw_updated:
            self.yaw = yaw_for_update
            if self.last_yaw_for_rate is not None:
                dyaw = self.wrap_angle(self.yaw - self.last_yaw_for_rate)
                self.current_yaw_rate = dyaw / dt
            else:
                self.current_yaw_rate = 0.0
            self.last_yaw_for_rate = self.yaw
        else:
            self.current_yaw_rate = 0.0

        self.last_stamp = current_stamp
        self.last_wall_time = now_wall
        self.publish_odom(current_stamp)

    # ------------------------------------------------------------------
    def publish_odom(self, stamp: rclpy.time.Time) -> None:
        odom = Odometry()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.child_frame
        odom.header.stamp = stamp.to_msg()

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        cov = [0.0] * 36
        for idx in (0, 7, 14, 21, 28, 35):
            cov[idx] = self.cov_diag
        odom.pose.covariance = cov
        odom.twist.covariance = cov

        odom.twist.twist.linear.x = self.current_speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.current_yaw_rate

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.child_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = quat[0]
            tf_msg.transform.rotation.y = quat[1]
            tf_msg.transform.rotation.z = quat[2]
            tf_msg.transform.rotation.w = quat[3]
            self.tf_br.sendTransform(tf_msg)

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelImuOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
