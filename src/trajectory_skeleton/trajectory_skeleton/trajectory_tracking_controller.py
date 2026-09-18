import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Transform, Twist
from tf2_ros import (
    TransformListener, Buffer, LookupException, ConnectivityException,
    ExtrapolationException)


def normalize_angle(angle):
    """Wrap an angle into the range (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class TrajectoryTrackingController(Node):
    """Drives the LIMO to the waypoints published on /waypoint_cmd.

    Three control laws are implemented and selected with the `controller`
    parameter:

        controller := on_off        On/Off (bang-bang) controller
        controller := proportional  Proportional (P) controller
        controller := kinematic     Kinematic position (rho/alpha/beta) controller

    Gains and thresholds are all ROS parameters, so they can be changed
    without editing this file, e.g.

        ros2 run trajectory_skeleton trajectory_tracking_controller \
            --ros-args -p controller:=kinematic -p kinematic_k_rho:=0.8
    """

    def __init__(self):
        super().__init__('smart_driver')
        self.waypoint = None
        self.robot_pose = None

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        # Which control law to run: 'on_off', 'proportional' or 'kinematic'.
        self.declare_parameter('controller', 'on_off')
        # Control loop period [s]. The skeleton used 1.0 s, which is far too
        # slow for smooth path following.
        self.declare_parameter('control_period', 0.1)

        # --- On/Off controller -------------------------------------------------
        self.declare_parameter('on_off_position_threshold', 0.25)  # [m]
        self.declare_parameter('on_off_heading_threshold', 0.10)   # [rad]
        self.declare_parameter('on_off_linear_speed', 0.15)         # [m/s]
        self.declare_parameter('on_off_angular_speed', 0.80)        # [rad/s]

        # --- Proportional controller ------------------------------------------
        self.declare_parameter('p_kp_distance', 0.60)
        self.declare_parameter('p_kp_angle', 1.50)
        self.declare_parameter('p_angle_deadband', 0.02)  # [rad]

        # --- Kinematic controller ---------------------------------------------
        self.declare_parameter('kinematic_k_rho', 0.60)
        self.declare_parameter('kinematic_k_alpha', 1.50)
        self.declare_parameter('kinematic_k_beta', -0.30)
        self.declare_parameter('kinematic_k_heading', 1.50)
        self.declare_parameter('kinematic_goal_tolerance', 0.15)  # [m]

        # --- Limits applied to every controller --------------------------------
        self.declare_parameter('max_linear_speed', 0.30)   # [m/s]
        self.declare_parameter('max_angular_speed', 1.50)  # [rad/s]

        self.controller = self.get_parameter('controller').value
        self.control_period = self.get_parameter('control_period').value

        self.subscription = self.create_subscription(
            Transform,
            '/waypoint_cmd',
            self.waypoint_callback,
            10)
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(self.control_period, self.timer_callback)

        self.get_logger().info(f"Using controller: '{self.controller}'")

    def waypoint_callback(self, msg):
        self.waypoint = msg

    def timer_callback(self):
        # Obtain current robot pose
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time())
            self.robot_pose = trans
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().error(f"Transform error: {ex}")
            return

        # Nothing to do until we have both a pose and a waypoint: stay stopped.
        if self.robot_pose is None or self.waypoint is None:
            self.publish_command(0.0, 0.0)
            return

        # Print current robot pose
        x = self.robot_pose.transform.translation.x
        y = self.robot_pose.transform.translation.y
        q = self.robot_pose.transform.rotation
        theta = self.quaternion_to_yaw(q)
        self.get_logger().info(
            f"Robot is believed to be at (x,y): ({x:.2f},{y:.2f})",
            throttle_duration_sec=1.0)
        self.get_logger().info(
            f"Robot is believed to have orientation (theta): ({theta:.2f})",
            throttle_duration_sec=1.0)

        # Print current destination
        wx = self.waypoint.translation.x
        wy = self.waypoint.translation.y
        wq = self.waypoint.rotation
        wtheta = self.quaternion_to_yaw(wq)
        self.get_logger().info(
            f"Current waypoint (x,y): ({wx:.2f},{wy:.2f})",
            throttle_duration_sec=1.0)
        self.get_logger().info(
            f"Current waypoint (theta): ({wtheta:.2f})",
            throttle_duration_sec=1.0)

        # ------------------------------------------------------------------
        # Errors, expressed in the robot frame
        # ------------------------------------------------------------------
        dx = wx - x
        dy = wy - y

        # rho: straight-line distance to the goal
        rho = math.hypot(dx, dy)
        # alpha: heading error -- the direction of the goal relative to the
        # robot's own heading. Zero when the robot points at the goal.
        alpha = normalize_angle(math.atan2(dy, dx) - theta)
        # beta: goal orientation error -- how much the goal's heading differs
        # from the direction of travel to the goal.
        beta = normalize_angle(wtheta - theta - alpha)
        # heading_error: direct error between robot heading and goal heading.
        # Used by the kinematic controller once it is sitting on the goal,
        # where alpha and beta are no longer meaningful.
        heading_error = normalize_angle(wtheta - theta)

        self.get_logger().info(
            f"rho: {rho:.2f}, alpha: {alpha:.2f}, beta: {beta:.2f}",
            throttle_duration_sec=1.0)

        # ------------------------------------------------------------------
        # DRIVE THE ROBOT HERE
        # ------------------------------------------------------------------
        if self.controller == 'proportional':
            v, omega = self.proportional_control(rho, alpha)
        elif self.controller == 'kinematic':
            v, omega = self.kinematic_control(rho, alpha, beta, heading_error)
        else:
            v, omega = self.on_off_control(rho, alpha)

        self.publish_command(v, omega)

    # ----------------------------------------------------------------------
    # Controller 1: On/Off (bang-bang)
    # ----------------------------------------------------------------------
    def on_off_control(self, rho, alpha):
        """Drive at a fixed speed while the error exceeds a threshold.

        Position and heading are handled independently, so the robot can
        turn on the spot while the distance error is small, and keep its
        heading while it drives a long straight leg.
        """
        position_threshold = self.get_parameter('on_off_position_threshold').value
        heading_threshold = self.get_parameter('on_off_heading_threshold').value
        linear_speed = self.get_parameter('on_off_linear_speed').value
        angular_speed = self.get_parameter('on_off_angular_speed').value

        v = linear_speed if rho > position_threshold else 0.0

        if abs(alpha) > heading_threshold:
            omega = math.copysign(angular_speed, alpha)
        else:
            omega = 0.0

        return v, omega

    # ----------------------------------------------------------------------
    # Controller 2: Proportional (P)
    # ----------------------------------------------------------------------
    def proportional_control(self, rho, alpha):
        """Command speed proportionally to the error magnitude.

        v     = Kp_distance * rho
        omega = Kp_angle * alpha

        Gains are clamped by the shared speed limits, which is what stops a
        large gain from asking for an impossible wheel speed.
        """
        kp_distance = self.get_parameter('p_kp_distance').value
        kp_angle = self.get_parameter('p_kp_angle').value
        deadband = self.get_parameter('p_angle_deadband').value

        v = kp_distance * rho

        # A small deadband keeps the robot from twitching at the goal, where
        # alpha is dominated by odometry noise.
        omega = 0.0 if abs(alpha) < deadband else kp_angle * alpha

        return v, omega

    # ----------------------------------------------------------------------
    # Controller 3: Kinematic position controller
    # ----------------------------------------------------------------------
    def kinematic_control(self, rho, alpha, beta, heading_error):
        """Differential-drive kinematic controller.

        v     = K_rho * rho
        omega = K_alpha * alpha + K_beta * beta

        Stable with K_rho > 0, K_beta < 0 and K_alpha > K_rho. Once the robot
        is within `kinematic_goal_tolerance` of the goal, alpha and beta are
        ill-conditioned (the direction to the goal is undefined at zero
        distance), so the robot stops translating and rotates onto the goal
        orientation using K_heading.
        """
        k_rho = self.get_parameter('kinematic_k_rho').value
        k_alpha = self.get_parameter('kinematic_k_alpha').value
        k_beta = self.get_parameter('kinematic_k_beta').value
        k_heading = self.get_parameter('kinematic_k_heading').value
        goal_tolerance = self.get_parameter('kinematic_goal_tolerance').value

        if rho < goal_tolerance:
            v = 0.0
            omega = k_heading * heading_error
        else:
            v = k_rho * rho
            omega = k_alpha * alpha + k_beta * beta

        return v, omega

    # ----------------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------------
    def publish_command(self, v, omega):
        """Clamp and publish a velocity command on /cmd_vel."""
        max_v = self.get_parameter('max_linear_speed').value
        max_omega = self.get_parameter('max_angular_speed').value

        motor_command = Twist()
        motor_command.linear.x = max(-max_v, min(max_v, v))
        motor_command.angular.z = max(-max_omega, min(max_omega, omega))
        self.publisher.publish(motor_command)

    @staticmethod
    def quaternion_to_yaw(q):
        # Convert quaternion to yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTrackingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
