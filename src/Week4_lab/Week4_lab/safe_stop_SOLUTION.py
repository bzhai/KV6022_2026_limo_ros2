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

        # Make the tuning numbers configurable so they can be overridden from
        # the command line or a launch file without touching the code.
        self.declare_parameter('safety_radius', 0.30)       # d_s   [m]  stop if closer than this
        self.declare_parameter('caution_radius', 0.60)      # d_c   [m]  begin slowing below this
        self.declare_parameter('cruise_speed', 0.22)        # v_max [m/s]  speed when the path is clear
        self.declare_parameter('front_half_angle', 0.5236)  # 30 deg [rad]  half-width of the front sector
        self.declare_parameter('resume_margin', 0.05)       # delta [m]  extra gap before moving again

        # Read each parameter once and store it under a short, readable name.
        self.d_s = self.get_parameter('safety_radius').value
        self.d_c = self.get_parameter('caution_radius').value
        self.v_max = self.get_parameter('cruise_speed').value
        self.half_angle = self.get_parameter('front_half_angle').value
        self.resume_margin = self.get_parameter('resume_margin').value

        # Sanity-check the geometry: caution must sit beyond safety, otherwise
        # the "slow down" zone would have zero (or negative) width.
        if self.d_c <= self.d_s:
            self.get_logger().error(
                'caution_radius must be larger than safety_radius; '
                'the MIDDLE zone would be empty or inverted.')

        # Hysteresis flag: are we currently latched into a stop?
        self.stopped = False

        # Where the robot's speed command is published, and where scans arrive.
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
        # The "front" is a wedge centred on straight ahead (bearing 0), so its
        # edges are symmetric: minus the half-angle and plus the half-angle.
        a_lo = -self.half_angle
        a_hi = +self.half_angle

        # Inverse of  theta_i = angle_min + i * angle_increment,
        # rounded to the nearest whole beam. int() matters: range() rejects floats.
        # Turn each edge angle into a beam number: which beam points at that angle?
        i_lo = int(round((a_lo - msg.angle_min) / msg.angle_increment))   # <-- BLANK 1a
        i_hi = int(round((a_hi - msg.angle_min) / msg.angle_increment))   # <-- BLANK 1b

        # Beams are numbered 0 ... n-1, so n-1 is the last valid index.
        n = len(msg.ranges)

        def clamp(index):
            # Keep the index inside [0, n-1] in case rounding overshoots the
            # first or last beam.
            return max(0, min(n - 1, index))                              # <-- BLANK 2

        # The front sector is every beam between the two edges, inclusive.
        return clamp(i_lo), clamp(i_hi)

    # =========================================================================
    #  PART B - how far is the nearest obstacle in that sector?
    # =========================================================================
    def min_front_range(self, msg: LaserScan):
        # Which beams make up the front sector?
        i_lo, i_hi = self.front_sector_bounds(msg)

        # Start with infinity: any real distance is smaller, so the first
        # usable beam will always replace it.
        best = float('inf')

        # Look at every beam in the sector, including the last one.
        for i in range(i_lo, i_hi + 1):
            # The distance this particular beam measured.
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

            # Is this the closest obstacle we have found so far? If so, keep it.
            if r < best:                                                  # <-- BLANK 6a
                best = r                                                  # <-- BLANK 6b

        # Nothing usable anywhere in the sector. A safety check with no data
        # must assume the worst, so report "unknown" and let the caller stop.
        if math.isinf(best):
            return None                                                   # <-- BLANK 7

        # Otherwise return the distance to the nearest obstacle in front.
        return best

    # =========================================================================
    #  PART C - which zone are we in?
    # =========================================================================
    def classify(self, d):
        # Put the front distance into one of three bands:
        #   CLOSE  = too near, must stop
        #   MIDDLE = near, slow down
        #   FAR    = clear, cruise
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
        # Too near: stop completely.
        if zone == 'CLOSE':
            return 0.0                                                    # <-- BLANK 9a

        # Near: slow down smoothly from cruise at d_c to zero at d_s.
        if zone == 'MIDDLE':
            # Straight line through (d_s, 0) and (d_c, v_max), so the command
            # is continuous at both boundaries. This is a proportional
            # controller on the error (d - d_s) with gain v_max/(d_c - d_s).
            return self.v_max * (d - self.d_s) / (self.d_c - self.d_s)     # <-- BLANK 9b

        # Clear: cruise at full speed.
        return self.v_max                                                 # <-- BLANK 9c

    # =========================================================================
    #  Glue
    # =========================================================================
    def on_scan(self, msg: LaserScan):
        # Each scan: measure the nearest obstacle ahead, decide a speed, send it.

        # How far away is the nearest obstacle in the front sector?
        d = self.min_front_range(msg)

        # No usable front data -> fail-safe: stop and do not move.
        if d is None:
            self.publish(0.0)
            self.get_logger().warning(
                'No usable readings in the front sector - holding still.',
                throttle_duration_sec=2.0)
            return

        # Which band is that distance in, and what speed does it imply?
        zone = self.classify(d)
        v = self.speed_for(zone, d)

        # ---- BLANK 10: hysteresis ------------------------------------------
        # Two thresholds instead of one. Stop at d_s, but do not release until
        # the obstacle is at d_s + resume_margin. Without this, millimetre
        # noise either side of d_s makes the robot twitch forward and stop
        # repeatedly. Same deadband idea as a thermostat.
        if self.stopped:
            # We are latched in a stop: only resume once the obstacle is
            # clearly beyond d_s + margin.
            if d > self.d_s + self.resume_margin:
                self.stopped = False
        elif d < self.d_s:
            # We are driving and the obstacle came inside the stop distance:
            # latch a stop.
            self.stopped = True

        # While latched, override whatever speed the zone wanted with zero.
        if self.stopped:
            v = 0.0
        # --------------------------------------------------------------------

        # Send the chosen speed to the motors.
        self.publish(v)

        # One status line per half-second so the behaviour can be traced.
        self.get_logger().info(
            f'front {d:.3f} m | {zone:<6} | '
            f'{"LATCHED" if self.stopped else "       "} | v = {v:.3f} m/s',
            throttle_duration_sec=0.5)

    def publish(self, v):
        cmd = Twist()
        cmd.linear.x = float(v)      # forward speed only
        cmd.angular.z = 0.0          # task 3 only stops; task 4 turns
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = SafeStop()
    try:
        rclpy.spin(node)             # keep processing scans until interrupted
    except KeyboardInterrupt:
        pass                         # Ctrl-C is a normal way to stop
    finally:
        # Never leave a robot driving after the controller dies.
        try:
            node.publish(0.0)        # send a stop command before shutting down
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
