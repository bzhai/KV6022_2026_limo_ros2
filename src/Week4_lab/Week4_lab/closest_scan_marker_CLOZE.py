# file: closest_scan_marker_CLOZE.py
#
# =============================================================================
#  EXERCISE: transform the closest laser point into the odom frame
# =============================================================================
#
#  Everything in this file already works EXCEPT the nine blanks marked
#  ___BLANK_n___ inside the student section. Your job is to replace each
#  ___BLANK_n___ with a correct Python expression.
#
#  The file will NOT run until every blank is filled: each one raises a
#  NotImplementedError on purpose, so you get a clear error naming the blank
#  you still owe rather than a silent wrong answer.
#
#  Work through them in order (1 -> 9). See the WORKSHEET for the maths, a
#  worked numerical example, and three levels of hint per blank.
# =============================================================================

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


def TODO(n):
    """Placeholder. Delete the call and write the real expression instead."""
    print(f"BLANK {n} is not filled in yet.")
    return 0.0


# Convenience aliases so the blanks read naturally in the code below.
___BLANK_1a___ = lambda: TODO("1a")
___BLANK_1b___ = lambda: TODO("1b")


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

        # =====================================================================
        #  START WRITING YOUR CODE HERE
        # =====================================================================

        # ---------------------------------------------------------------------
        # STEP 1 - Initialise the "best so far" variables.
        #
        # You are about to run a linear scan for a minimum. Before the loop
        # starts you need a starting value for the smallest range seen so far,
        # and a starting value for the index at which it was seen.
        #
        # The starting range must be a value that ANY real measurement will
        # beat, and the starting index must be a value that can never be a
        # valid array index, so that later you can tell "found nothing" apart
        # from "found something at index 0".
        #
        # BLANK 1: two initial values.
        # ---------------------------------------------------------------------
        closest_range = ___BLANK_1a___()      # smallest range seen so far  [m]
        closest_index = ___BLANK_1b___()      # index of that range         [-]

        # ---------------------------------------------------------------------
        # STEP 2 - Loop over every beam and reject the unusable readings.
        #
        # msg.ranges is a list of N floats, one per laser beam. Not all of them
        # are real measurements:
        #
        #   * NaN  means the beam returned nothing interpretable at all.
        #   * inf  means nothing was found inside the sensor's maximum range.
        #   * a value below msg.range_min or above msg.range_max is outside the
        #     range the manufacturer will vouch for, so it is not trustworthy.
        #
        # If you do not reject these, min() will happily return NaN, or your
        # marker will jump to a garbage position.
        #
        # BLANK 2: the condition that is True for a NaN or infinite reading.
        # BLANK 3: the condition that is True for a reading outside the
        #          sensor's physically valid interval.
        # ---------------------------------------------------------------------
        for i, r in enumerate[float](msg.ranges):

            if TODO(2):                       # BLANK 2: NaN or infinite?
                continue

            if TODO(3):                       # BLANK 3: outside [range_min, range_max]?
                continue

            # -----------------------------------------------------------------
            # STEP 3 - Keep the running minimum.
            #
            # At this point r is a trustworthy distance in metres. Compare it
            # against the best you have seen so far, and if it wins, record
            # BOTH the distance and the index it came from. The index is what
            # tells you the direction later, so forgetting to store it is the
            # single most common mistake here.
            #
            # BLANK 4: the comparison that decides whether r is a new minimum.
            # BLANK 5: the two values to store when it is.
            # -----------------------------------------------------------------
            if TODO(4):                       # BLANK 4: is r better than the best so far?
                closest_range = TODO("5a")    # BLANK 5a
                closest_index = TODO("5b")    # BLANK 5b

        # ---------------------------------------------------------------------
        # STEP 4 - Guard against an empty scan.
        #
        # It is possible that EVERY beam was rejected above: the robot is in
        # the middle of a large open space, or the simulator has not warmed up
        # yet. In that case closest_index still holds the impossible value you
        # chose in BLANK 1b. Detect that and leave the callback early, because
        # there is no point to publish.
        #
        # BLANK 6: the condition meaning "no valid reading was found".
        # ---------------------------------------------------------------------
        if TODO(6):                           # BLANK 6
            self.get_logger().warning('No valid range readings in this scan.')
            return

        # ---------------------------------------------------------------------
        # STEP 5 - Recover the bearing of the closest beam.
        #
        # A LaserScan does NOT store one angle per beam; that would double the
        # message size for no reason. It stores only the angle of the first
        # beam (msg.angle_min) and the constant angular step between
        # consecutive beams (msg.angle_increment). The beams are evenly spaced,
        # so the bearing of beam i is an arithmetic sequence in i.
        #
        # Work it out for beam 0, then beam 1, then beam i, and write it down.
        #
        # BLANK 7: the bearing of the closest beam, in radians.
        # ---------------------------------------------------------------------
        closest_angle = TODO(7)               # BLANK 7  [rad]

        # ---------------------------------------------------------------------
        # STEP 6 - Convert from polar to Cartesian, in the LASER's own frame.
        #
        # You now have the closest obstacle described as (range, bearing),
        # which is a polar coordinate pair. TF cannot transform polar
        # coordinates; it transforms points. So convert to (x, y, z).
        #
        # Mind the convention: in a ROS frame, +x points forward, +y points to
        # the LEFT, +z points up, and a positive angle is measured
        # anticlockwise from +x. A 2-D scan lies flat in the sensor's own
        # plane, so one of the three coordinates is trivially zero.
        #
        # BLANK 8: x and y from range and bearing.
        # BLANK 9: z.
        # ---------------------------------------------------------------------
        x = TODO("8a")                        # BLANK 8a  [m]
        y = TODO("8b")                        # BLANK 8b  [m]
        z = TODO(9)                           # BLANK 9   [m]

        # =====================================================================
        #  END OF WRITING YOUR CODE
        # =====================================================================

        # ---- Nothing below this line needs changing. ------------------------

        # Wrap the point up with the frame it was measured in. The header is
        # copied from the scan, so header.frame_id is the laser's frame and
        # header.stamp is the moment of measurement. TF needs both to know
        # "where" and "when" this point lives.
        point = PointStamped()
        point.header = msg.header
        point.point.x = x
        point.point.y = y
        point.point.z = z

        target_frame = 'odom'
        source_frame = msg.header.frame_id    # 'laser_link' on the LIMO

        try:
            # Ask TF for the rigid-body transform that expresses laser_link
            # coordinates in odom coordinates, then apply it to our point.
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            point = do_transform_point(point, transform)
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return

        self.get_logger().info(
            f"closest: {closest_range:.3f} m at "
            f"{math.degrees(closest_angle):+.1f} deg (beam {closest_index}) "
            f"-> odom ({point.point.x:+.3f}, {point.point.y:+.3f})",
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
