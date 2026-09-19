# file: safe_stop_SOLUTION.py
#
# =============================================================================
#  REFERENCE SOLUTION - Workshop task 3: safe-stop behaviour
# =============================================================================
#
#  Every blank from safe_stop_CLOZE.py is filled. The filled expressions are
#  marked  # <-- BLANK n  so this file lines up with the cloze and the
#  worksheet answer key.
#
#  Blank 10 (hysteresis) IS included here, so the behaviour is the polished
#  version: stop at d_s, and do not resume until the obstacle is at
#  d_s + resume_margin.
#
#  Behaviour summary:
#      front distance >= d_c   ->  cruise at v_max
#      d_s <= front < d_c      ->  linear ramp down to zero
#      front < d_s             ->  stop, and stay stopped until d_s + margin
#      no usable front data    ->  stop (fail-safe)
# =============================================================================

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SafeStop(Node):

    def __init__(self):
        super().__init__('safe_stop')

        self.declare_parameter('safety_radius', 0.30)       # d_s   [m]
        self.declare_parameter('caution_radius', 0.60)      # d_c   [m]
        self.declare_parameter('cruise_speed', 0.22)        # v_max [m/s]
        self.declare_parameter('front_half_angle', 0.5236)  # 30 deg [rad]
        self.declare_parameter('resume_margin', 0.05)       # delta [m]

        self.d_s = self.get_parameter('safety_radius').value
        self.d_c = self.get_parameter('caution_radius').value
        self.v_max = self.get_parameter('cruise_speed').value
        self.half_angle = self.get_parameter('front_half_angle').value
        self.resume_margin = self.get_parameter('resume_margin').value

        if self.d_c <= self.d_s:
            self.get_logger().error(
                'caution_radius must be larger than safety_radius; '
                'the MIDDLE zone would be empty or inverted.')

        self.stopped = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f'safe_stop started: stop < {self.d_s} m, '
            f'slow < {self.d_c} m, cruise {self.v_max} m/s, '
            f'front sector +/- {math.degrees(self.half_angle):.0f} deg')

    # =========================================================================
    #  PART A - which beams count as "in front"?
    # =========================================================================
    def front_sector_bounds(self, msg: LaserScan):
        a_lo = -self.half_angle
        a_hi = +self.half_angle

        # Inverse of  theta_i = angle_min + i * angle_increment,
        # rounded to the nearest whole beam. int() matters: range() rejects floats.
        i_lo = int(round((a_lo - msg.angle_min) / msg.angle_increment))   # <-- BLANK 1a
        i_hi = int(round((a_hi - msg.angle_min) / msg.angle_increment))   # <-- BLANK 1b

        n = len(msg.ranges)

        def clamp(index):
            return max(0, min(n - 1, index))                              # <-- BLANK 2

        return clamp(i_lo), clamp(i_hi)

    # =========================================================================
    #  PART B - how far is the nearest obstacle in that sector?
    # =========================================================================
    def min_front_range(self, msg: LaserScan):
        i_lo, i_hi = self.front_sector_bounds(msg)

        best = float('inf')

        for i in range(i_lo, i_hi + 1):
            r = msg.ranges[i]

            # Beam failed: no information about this direction, so ignore it.
            if math.isnan(r):                                             # <-- BLANK 3
                continue

            # Beam worked and found nothing within range: the direction is
            # CLEAR. Record that as the furthest trustworthy distance rather
            # than discarding it, or the robot refuses to move in an open room.
            if math.isinf(r) or r > msg.range_max:                        # <-- BLANK 4a
                r = msg.range_max                                         # <-- BLANK 4b

            # Inside the blind shell: the number exists but is not trustworthy.
            elif r < msg.range_min:                                       # <-- BLANK 5
                continue

            if r < best:                                                  # <-- BLANK 6a
                best = r                                                  # <-- BLANK 6b

        # Nothing usable anywhere in the sector. A safety check with no data
        # must assume the worst, so report "unknown" and let the caller stop.
        if math.isinf(best):
            return None                                                   # <-- BLANK 7

        return best

    # =========================================================================
    #  PART C - which zone are we in?
    # =========================================================================
    def classify(self, d):
        if d < self.d_s:                                                  # <-- BLANK 8a
            return 'CLOSE'
        elif d < self.d_c:                                                # <-- BLANK 8b
            return 'MIDDLE'
        else:
            return 'FAR'

    # =========================================================================
    #  PART D - what speed does that zone imply?
    # =========================================================================
    def speed_for(self, zone, d):
        if zone == 'CLOSE':
            return 0.0                                                    # <-- BLANK 9a

        if zone == 'MIDDLE':
            # Straight line through (d_s, 0) and (d_c, v_max), so the command
            # is continuous at both boundaries. This is a proportional
            # controller on the error (d - d_s) with gain v_max/(d_c - d_s).
            return self.v_max * (d - self.d_s) / (self.d_c - self.d_s)     # <-- BLANK 9b

        return self.v_max                                                 # <-- BLANK 9c

    # =========================================================================
    #  Glue
    # =========================================================================
    def on_scan(self, msg: LaserScan):

        d = self.min_front_range(msg)

        if d is None:
            self.publish(0.0)
            self.get_logger().warning(
                'No usable readings in the front sector - holding still.',
                throttle_duration_sec=2.0)
            return

        zone = self.classify(d)
        v = self.speed_for(zone, d)

        # ---- BLANK 10: hysteresis ------------------------------------------
        # Two thresholds instead of one. Stop at d_s, but do not release until
        # the obstacle is at d_s + resume_margin. Without this, millimetre
        # noise either side of d_s makes the robot twitch forward and stop
        # repeatedly. Same deadband idea as a thermostat.
        if self.stopped:
            if d > self.d_s + self.resume_margin:
                self.stopped = False
        elif d < self.d_s:
            self.stopped = True

        if self.stopped:
            v = 0.0
        # --------------------------------------------------------------------

        self.publish(v)

        self.get_logger().info(
            f'front {d:.3f} m | {zone:<6} | '
            f'{"LATCHED" if self.stopped else "       "} | v = {v:.3f} m/s',
            throttle_duration_sec=0.5)

    def publish(self, v):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = 0.0          # task 3 only stops; task 4 turns
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = SafeStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Never leave a robot driving after the controller dies.
        try:
            node.publish(0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
