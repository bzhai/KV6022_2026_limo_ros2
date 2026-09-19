# file: closest_scan_marker_node.py
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class ClosestScanMarker(Node):
    def __init__(self):
        super().__init__('closest_scan_marker')

        self.marker_pub = self.create_publisher(Marker, '/closest_obstacle_marker', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.get_logger().info('closest_scan_marker node started (publishing in odom).')

    def on_scan(self, msg: LaserScan):

        #################  START WRITING YOUR CODE HERE #################

        # Find closest range reading and its angle.
        # The ranges array can hold NaN (no echo) and inf (nothing within
        # range_max), and anything outside [range_min, range_max] is not
        # trustworthy, so those readings are skipped.
        closest_range = float('inf')
        closest_index = -1

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            if r < closest_range:
                closest_range = r
                closest_index = i

        if closest_index < 0:
            self.get_logger().warning('No valid range readings in this scan.')
            return

        # ranges[] is polar and compressed: only the first angle and the step
        # between beams are stored, so the bearing of beam i is recovered as
        closest_angle = msg.angle_min + closest_index * msg.angle_increment

        # Convert to (x, y) in the scan frame (polar -> Cartesian)
        x = closest_range * math.cos(closest_angle)
        y = closest_range * math.sin(closest_angle)
        z = 0.0

        ################# END OF WRITING YOUR CODE  #################

        point = PointStamped()
        point.header = msg.header  # source frame = scan's frame
        point.point.x = x
        point.point.y = y
        point.point.z = z

        target_frame = 'odom'
        source_frame = msg.header.frame_id  # 'laser_link' on the LIMO

        try:
            # get the transform laser point in laser link frame to odom frame
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            # here we directly transform the pose into another pose for the
            # given frame of reference
            point = do_transform_point(point, transform)
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return  # don't publish a laser-frame point as if it were odom

        self.get_logger().info(
            f"closest: {closest_range:.3f} m at {math.degrees(closest_angle):+.1f} deg "
            f"in {source_frame} -> odom ({point.point.x:.3f}, {point.point.y:.3f})",
            throttle_duration_sec=1.0)

        # Build marker (in odom)
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'odom'
        marker.ns = 'closest_scan'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = point.point.x
        marker.pose.position.y = point.point.y
        marker.pose.position.z = point.point.z
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 0.9
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = ClosestScanMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
