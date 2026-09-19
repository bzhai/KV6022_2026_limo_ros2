# file: safe_stop_CLOZE.py
#
# =============================================================================
#  EXERCISE (Workshop task 3): stop the robot at a safe distance
# =============================================================================
#
#  Goal: drive forward at a cruise speed, slow down as an obstacle approaches,
#  and come to a complete stop while there is still clear space in front of
#  the bumper. Three zones: FAR (cruise), MIDDLE (slow), CLOSE (stop).
#
#  Everything is written for you EXCEPT the blanks marked TODO(n), which hold
#  the whole control algorithm. Each unfilled blank raises NotImplementedError
#  naming itself, so you always know which one you still owe.
#
#  Fill them in this order. Blank 10 is a stretch task - the node works
#  without it, just not very gracefully.
#
#  Angular velocity stays at zero throughout: this node only stops. Turning
#  away from the obstacle is task 4.
# =============================================================================

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def TODO(n):
    """Placeholder. Delete the call and write the real expression instead."""
    print(f"BLANK {n} is not filled in yet.")   
    return 0.0


class SafeStop(Node):

    def __init__(self):
        super().__init__('safe_stop')

        # ---------------------------------------------------------------------
        #  Tunable parameters. Override at launch, e.g.
        #     ros2 run example_codes safe_stop --ros-args -p safety_radius:=0.5
        #
        #  Do NOT hard-code these numbers inside your algorithm. Part of the
        #  exercise is keeping the policy (these values) separate from the
        #  mechanism (your code).
        # ---------------------------------------------------------------------
        self.declare_parameter('safety_radius', 0.30)     # d_s: stop by here      [m]
        self.declare_parameter('caution_radius', 0.60)    # d_c: start slowing here [m]
        self.declare_parameter('cruise_speed', 0.22)      # v_max                  [m/s]
        self.declare_parameter('front_half_angle', 0.5236)  # half-width of the front sector, 30 deg [rad]
        self.declare_parameter('resume_margin', 0.05)     # hysteresis band, blank 10 [m]

        self.d_s = self.get_parameter('safety_radius').value
        self.d_c = self.get_parameter('caution_radius').value
        self.v_max = self.get_parameter('cruise_speed').value
        self.half_angle = self.get_parameter('front_half_angle').value
        self.resume_margin = self.get_parameter('resume_margin').value

        if self.d_c <= self.d_s:
            self.get_logger().error(
                'caution_radius must be larger than safety_radius; '
                'the MIDDLE zone would be empty or inverted.')

        # Latched stop flag, used only by blank 10.
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
        """Return (i_lo, i_hi), the inclusive index range of the front sector.

        The scan covers about +/-114.6 deg, but an obstacle at -110 deg is
        beside the wheels, not in the path. Braking for it would leave the
        robot frozen in any corridor. So you first need to convert an ANGLE
        limit into an INDEX limit.

        In task 2 you went from index to angle:

            theta_i = angle_min + i * angle_increment

        Here you need the inverse: given a bearing, which beam is it?
        Rearrange that equation for i, and round to the nearest whole beam.
        """
        a_lo = -self.half_angle          # right edge of the front sector [rad]
        a_hi = +self.half_angle          # left edge  of the front sector [rad]

        # ---------------------------------------------------------------------
        # BLANK 1 - invert the bearing equation.
        # Both expressions must evaluate to an int (round, then cast).
        # ---------------------------------------------------------------------
        i_lo = TODO('1a')
        i_hi = TODO('1b')

        # ---------------------------------------------------------------------
        # BLANK 2 - clamp.
        #
        # If a user sets front_half_angle to 2.5 rad, wider than the scan
        # itself, the indices above fall outside the array and you get an
        # IndexError at runtime. Force any index into the legal interval
        # [0, n - 1]. Two nested built-in functions do this in one line.
        # ---------------------------------------------------------------------
        n = len(msg.ranges)

        def clamp(index):
            return TODO(2)

        return clamp(i_lo), clamp(i_hi)

    # =========================================================================
    #  PART B - how far is the nearest obstacle in that sector?
    # =========================================================================
    def min_front_range(self, msg: LaserScan):
        """Smallest trustworthy range inside the front sector, or None.

        Same running-minimum pattern as task 2, but over a slice of the array
        instead of all of it, and with one important difference in how the
        special values are handled. Read the three cases carefully - they are
        NOT all rejections:

          * NaN  -> the beam failed. You know nothing about that direction, so
                    skip it and rely on the others.
          * inf  -> the beam worked and found NOTHING within range_max. That is
                    not missing data, it is GOOD news: the direction is clear.
                    Rejecting it would make the robot refuse to move in an open
                    room, which is the classic bug in this task. Substitute the
                    largest distance the sensor can vouch for.
          * r < range_min -> inside the blind shell, so the number exists but
                    cannot be trusted. Skip it.
        """
        i_lo, i_hi = self.front_sector_bounds(msg)

        best = float('inf')

        for i in range(i_lo, i_hi + 1):
            r = msg.ranges[i]

            # -----------------------------------------------------------------
            # BLANK 3 - is this beam a failed reading (NaN)? If so, skip it.
            # -----------------------------------------------------------------
            if TODO(3):
                continue

            # -----------------------------------------------------------------
            # BLANK 4 - the "clear direction" case.
            #
            # Condition: the reading is infinite, or finite but beyond what the
            # sensor will vouch for. Action: replace r with the furthest
            # trustworthy distance (a field of msg - do not hard-code 8.0).
            # -----------------------------------------------------------------
            if TODO('4a'):
                r = TODO('4b')

            # -----------------------------------------------------------------
            # BLANK 5 - reading inside the blind zone. Skip it.
            # -----------------------------------------------------------------
            elif TODO(5):
                continue

            # -----------------------------------------------------------------
            # BLANK 6 - keep the running minimum.
            #
            # Note you do NOT need the index this time. Ask yourself why task 2
            # needed it and this task does not.
            # -----------------------------------------------------------------
            if TODO('6a'):
                best = TODO('6b')

        # ---------------------------------------------------------------------
        # BLANK 7 - the fail-safe return.
        #
        # If every beam in the sector was skipped, `best` is still infinite and
        # you have NO information about the space ahead. Return a value that
        # signals "unknown" rather than a distance, and make sure on_scan
        # treats it as a reason to stop. When a safety check has no data, the
        # safe assumption is the pessimistic one.
        # ---------------------------------------------------------------------
        if math.isinf(best):
            return TODO(7)

        return best

    # =========================================================================
    #  PART C - which zone are we in?
    # =========================================================================
    def classify(self, d):
        """Map a front distance in metres onto 'CLOSE', 'MIDDLE' or 'FAR'.

        Two thresholds cut the number line into three intervals:

            0 ........ d_s ........ d_c ........ infinity
              CLOSE       MIDDLE       FAR

        Use self.d_s and self.d_c. Order your tests so that each distance
        matches exactly one branch, and decide deliberately which side of each
        boundary is inclusive.
        """
        # ---------------------------------------------------------------------
        # BLANK 8 - the two boundary tests.
        # ---------------------------------------------------------------------
        if TODO('8a'):
            return 'CLOSE'
        elif TODO('8b'):
            return 'MIDDLE'
        else:
            return 'FAR'

    # =========================================================================
    #  PART D - what speed does that zone imply?
    # =========================================================================
    def speed_for(self, zone, d):
        """Forward speed in m/s for the given zone and distance.

        CLOSE:  the whole point of the task.

        FAR:    nothing nearby, so run at the cruise speed.

        MIDDLE: do not just pick a fixed slow speed - ramp linearly so that
                the command is continuous. At d = d_c the speed should still be
                v_max, and at d = d_s it should have fallen to exactly zero.
                That means finding the straight line through the two points

                    (d_s, 0)  and  (d_c, v_max)

                Write the fraction of the way across the MIDDLE band first,
                then scale v_max by it. The fraction is dimensionless: metres
                divided by metres.
        """
        # ---------------------------------------------------------------------
        # BLANK 9 - one speed per zone.
        # ---------------------------------------------------------------------
        if zone == 'CLOSE':
            return TODO('9a')

        if zone == 'MIDDLE':
            return TODO('9b')

        return TODO('9c')

    # =========================================================================
    #  Glue - already written, apart from blank 10.
    # =========================================================================
    def on_scan(self, msg: LaserScan):

        d = self.min_front_range(msg)

        if d is None:
            # No usable data in the front sector: refuse to move.
            self.publish(0.0)
            self.get_logger().warning(
                'No usable readings in the front sector - holding still.',
                throttle_duration_sec=2.0)
            return

        zone = self.classify(d)
        v = self.speed_for(zone, d)

        # ---------------------------------------------------------------------
        # BLANK 10 (STRETCH) - hysteresis.
        #
        # Try the node without this first. Park the robot so the obstacle sits
        # almost exactly at d_s and watch: sensor noise of a few millimetres
        # flips the distance either side of the threshold, so the robot twitches
        # forward and stops, forward and stops. This is threshold chattering,
        # and it is a real problem on real hardware.
        #
        # The cure is to use two different thresholds depending on which state
        # you are already in - stop at d_s, but do not resume until the obstacle
        # is at d_s + resume_margin. That asymmetry is what self.stopped is for.
        #
        # Replace the line below with that logic: set self.stopped True when the
        # robot should hold, False when it is safe to release, and force v to
        # zero for as long as the flag is set.
        # ---------------------------------------------------------------------
        self.stopped = (v == 0.0)      # <-- naive version; blank 10 replaces this

        self.publish(v)

        self.get_logger().info(
            f'front {d:.3f} m | {zone:<6} | v = {v:.3f} m/s',
            throttle_duration_sec=0.5)

    def publish(self, v):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = 0.0            # task 3 only stops; task 4 turns
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
