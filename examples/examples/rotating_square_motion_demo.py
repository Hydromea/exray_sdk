#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Quaternion, Vector3
from exray_ros_messages.msg import PositionTarget, AttitudeTarget, ImuOrientation, ExternalPressure
from scipy.spatial.transform import Rotation as R

"""
This node demonstrates a simple motion sequence while stabilizing depth using position control and maintaining
the initial orientation of an underwater robot, rotating at the corners of a square. The motion sequence is:

1. Move forward for 2 seconds.
2. Turn 90° in yaw and move forward for 2 seconds.
3. Repeat the above two steps three more times to complete a square.

Throughout this sequence:
- The robot maintains its initial orientation using IMU feedback.
- The robot stabilizes its depth using feedback from a pressure sensor.

Key features:
- ROS 2 (rclpy) for node creation, subscriptions, and publications.
- Waits for initial orientation and depth before starting.
- Publishes velocity and attitude targets at a regular interval during each phase.
"""


class RotatingSquareMotionDemo(Node):
    def __init__(self):
        super().__init__('rotating_square_motion_demo')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Use Best Effort reliability
            history=HistoryPolicy.KEEP_LAST,            # Keep the last N messages
            depth=5                                     # Set queue size to 5
        )

        # --- Publishers ---
        # Publish velocity targets for position and attitude
        self.pos_pub = self.create_publisher(PositionTarget, '/exray/control/target/position', best_effort_qos)
        self.att_pub = self.create_publisher(AttitudeTarget, '/exray/control/target/attitude', best_effort_qos)

        # --- Subscribers ---
        # Get orientation feedback from IMU
        self.create_subscription(
            ImuOrientation,
            '/exray/imu_main_control/orientation',
            self.orientation_callback,
            best_effort_qos)
        # Get depth feedback from external pressure sensor
        self.create_subscription(ExternalPressure, '/exray/external_pressure', self.pressure_callback, best_effort_qos)

        # --- Message Initialization ---
        self.pos_msg = PositionTarget()
        self.att_msg = AttitudeTarget()

        # Set frame IDs if needed
        self.pos_msg.header.frame_id = "base_link"
        self.att_msg.header.frame_id = "base_link"

        self.pos_msg.type_mask = 4  # b00000100:  # Velocity mode for x and y, position control for z.

        # Set attitude control to attitude mode
        self.att_msg.type_mask = AttitudeTarget.TARGET_ATTITUDE_MASK
        self.att_msg.attitude_delta = Quaternion()
        self.att_msg.attitude_delta.w = 1.0
        self.att_msg.attitude_delta.x = 0.0
        self.att_msg.attitude_delta.y = 0.0
        self.att_msg.attitude_delta.z = 0.0
        self.att_msg.angular_rate = Vector3()
        self.att_msg.angular_rate.x = 0.0
        self.att_msg.angular_rate.y = 0.0
        self.att_msg.angular_rate.z = 0.0

        # --- State Variables ---
        self.initial_orientation = None
        self.initial_depth = None
        self.current_orientation = None
        self.current_depth = None
        self.q_corr = Quaternion()  # Quaternion correction to maintain initial orientation
        self.q_corr.w = 1.0
        self.q_corr.x = 0.0
        self.q_corr.y = 0.0
        self.q_corr.z = 0.0

        # Commanded planar velocities
        self.command_vx = 0.0
        self.command_vy = 0.0

        # Target quaternion for 90-degree yaw rotations
        self.target_quaternion = None

    def orientation_callback(self, msg: ImuOrientation):
        self.current_orientation = msg.quaternion

        if self.initial_orientation is None:
            self.initial_orientation = self.current_orientation
            self.target_quaternion = self.initial_orientation

    def pressure_callback(self, msg: ExternalPressure):
        """
        Callback that receives the current depth from the external pressure sensor.
        If we do not have an initial depth yet, store it as the reference.
        """
        self.current_depth = msg.depth
        if self.initial_depth is None:
            self.initial_depth = self.current_depth

    def run(self):
        """
        Main execution sequence:
        1. Wait for initial orientation and depth.
        2. Execute four phases of motion: forward, left, backward, and right.
        3. Stop at the end.
        """
        self.get_logger().info("Waiting for initial orientation and depth...")

        # Wait until we have initial orientation and depth
        while rclpy.ok() and (self.initial_orientation is None or self.initial_depth is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.get_logger().info("Initial orientation and depth acquired. Starting movement sequence...")

        for i in range(4):  # Four sides of the square
            self.set_planar_velocity(0.2, 0.0)  # Move forward
            self.execute_phase(2.0)  # For 2 seconds

            self.set_planar_velocity(0.0, 0.0)  # Stop before rotating
            self.rotate_90_degrees()
            self.execute_phase(5.0)

        self.set_planar_velocity(0.0, 0.0)  # Stop at the end
        self.execute_phase(0.5)

        self.get_logger().info("Motion sequence completed.")

    def execute_phase(self, duration: float):
        """
        Execute a movement phase for the specified duration.
        During this time, continuously publish commands to maintain orientation and depth.
        """
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < duration:
            # Spin once to process subscription callbacks
            rclpy.spin_once(self, timeout_sec=0.0)
            self.publish_messages()
            time.sleep(0.04)

    def set_planar_velocity(self, vx: float, vy: float):
        """
        Set the desired planar (x,y) velocities. Vertical velocity (z) will be
        handled by the depth stabilization logic.
        """
        self.command_vx = vx
        self.command_vy = vy

    def rotate_90_degrees(self):
        self.get_logger().info("Rotating 90 degrees...")

        # Convert current orientation to a rotation object
        current_quat = [
            self.target_quaternion.x,
            self.target_quaternion.y,
            self.target_quaternion.z,
            self.target_quaternion.w
        ]
        current_rot = R.from_quat(current_quat)

        # Apply a 90-degree yaw rotation
        yaw_rotation = R.from_euler('z', 90, degrees=True)
        new_rot = current_rot * yaw_rotation

        # Update target quaternion
        new_quat = new_rot.as_quat()
        self.target_quaternion.w = new_quat[3]
        self.target_quaternion.x = new_quat[0]
        self.target_quaternion.y = new_quat[1]
        self.target_quaternion.z = new_quat[2]

    def publish_messages(self):
        """
        Publish the current commands for position and attitude.
        This function:
        - Applies the stored quaternion correction to maintain orientation.
        - Publishes the combined commands at the current timestamp.
        """
        # compute delta quaternion between current and target
        q_target = [
            self.target_quaternion.x,
            self.target_quaternion.y,
            self.target_quaternion.z,
            self.target_quaternion.w
        ]
        q_current = [
            self.current_orientation.x,
            self.current_orientation.y,
            self.current_orientation.z,
            self.current_orientation.w
        ]

        R_target = R.from_quat(q_target)
        R_current = R.from_quat(q_current)
        R_diff = R_target * R_current.inv()

        q_corr = R_diff.as_quat()
        self.q_corr.w = q_corr[3]
        self.q_corr.x = q_corr[0]
        self.q_corr.y = q_corr[1]
        self.q_corr.z = q_corr[2]

        # Set attitude correction
        self.att_msg.attitude_delta = self.q_corr

        # Set velocities with vertical correction
        self.pos_msg.velocity = Vector3()
        self.pos_msg.velocity.x = self.command_vx
        self.pos_msg.velocity.y = self.command_vy
        self.pos_msg.velocity.z = 0.0  # Not used because of type mask

        self.pos_msg.position = Vector3()
        self.pos_msg.position.x = 0.0  # Not used because of type mask
        self.pos_msg.position.y = 0.0  # Not used because of type mask
        self.pos_msg.position.z = -self.initial_depth

        # Update headers with the current time
        now = self.get_clock().now().to_msg()
        self.pos_msg.header.stamp = now
        self.att_msg.header.stamp = now

        # Publish messages
        self.pos_pub.publish(self.pos_msg)
        self.att_pub.publish(self.att_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RotatingSquareMotionDemo()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
