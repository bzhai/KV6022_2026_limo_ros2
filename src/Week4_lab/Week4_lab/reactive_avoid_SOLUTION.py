# file: reactive_avoid_SOLUTION.py
#
# =============================================================================
#  REFERENCE SOLUTION - Workshop task 4: reactive obstacle avoidance
# =============================================================================
#
#  Every blank from reactive_avoid_CLOZE.py is filled and marked
#  # <-- BLANK n  so this file lines up with the cloze and the worksheet key.
#
#  Blank 10 (commitment / hysteresis) IS included, which changes two things
#  relative to the cloze skeleton:
#
#    1. front_is_blocked() now reads self.avoiding, so the blocked threshold
#       is d_s when cruising and d_s + resume_margin once a manoeuvre has
#       started. ORDER MATTERS: the blocked test must run BEFORE self.avoiding
#       is updated, otherwise the state lags the decision by one scan.
#
#    2. The dead-end case commands the full turn rate rather than the
#       proportional one. In a dead end the two sides are nearly equal, so the
#       normalised asymmetry is tiny and a proportional rate would be far too
#       slow to rotate out.
#
#  Behaviour summary:
#      front clear                    ->  drive on, speed ramped by task 3 law
#      front blocked, one side open   ->  creep + turn towards the open side
#      front blocked, both sides shut ->  stop translating, rotate on the spot
#      no usable front data           ->  hold still (fail-safe)
#
#  Sign convention: linear.x > 0 forwards, angular.z > 0 anticlockwise = LEFT,
#  bearing > 0 to the LEFT. All three agree (REP 103).
# =============================================================================

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ReactiveAvoid(Node):

    def __init__(self):
        super().__init__('reactive_avoid')

        # --- distances -------------------------------------------------------
        self.declare_parameter('safety_radius', 0.35)       # d_s        [m]
        self.declare_parameter('caution_radius', 0.70)      # d_c        [m]
        self.declare_parameter('side_blocked', 0.45)        # d_side     [m]
        self.declare_parameter('resume_margin', 0.08)       # delta      [m]

        # --- speeds ----------------------------------------------------------
        self.declare_parameter('cruise_speed', 0.22)        # v_max      [m/s]
        self.declare_parameter('turn_speed_max', 0.9)       # w_max      [rad/s]
        self.declare_parameter('creep_speed', 0.05)         # v_creep    [m/s]

        # --- sector geometry -------------------------------------------------
        self.declare_parameter('front_half_angle', 0.4363)  # 25 deg     [rad]
        self.declare_parameter('side_outer_angle', 1.3090)  # 75 deg     [rad]

        self.d_s = self.get_parameter('safety_radius').value
        self.d_c = self.get_parameter('caution_radius').value
        self.d_side = self.get_parameter('side_blocked').value
        self.resume_margin = self.get_parameter('resume_margin').value
        self.v_max = self.get_parameter('cruise_speed').value
        self.w_max = self.get_parameter('turn_speed_max').value
        self.v_creep = self.get_parameter('creep_speed').value
        self.a_front = self.get_parameter('front_half_angle').value
        self.a_side = self.get_parameter('side_outer_angle').value

        # Latched avoidance state.
        self.avoiding = False        # is a manoeuvre in progress?
        self.turn_sign = 0.0         # +1.0 left, -1.0 right, 0.0 uncommitted

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f'reactive_avoid started: front +/-{math.degrees(self.a_front):.0f} deg, '
            f'sides out to {math.degrees(self.a_side):.0f} deg, '
            f'block < {self.d_s} m, cruise {self.v_max} m/s')

    # =========================================================================
    #  Carried over from task 3 - no blanks
    # =========================================================================
    def sector_bounds(self, msg: LaserScan, a_lo, a_hi):
        """Inclusive index range of beams with bearing in [a_lo, a_hi]."""
        n = len(msg.ranges)

        def to_index(angle):
            i = int(round((angle - msg.angle_min) / msg.angle_increment))
            return max(0, min(n - 1, i))

        return to_index(a_lo), to_index(a_hi)

    def min_in_sector(self, msg: LaserScan, a_lo, a_hi):
        """Smallest trustworthy range in the sector, or None if no usable beam."""
        i_lo, i_hi = self.sector_bounds(msg, a_lo, a_hi)
        best = float('inf')

        for i in range(i_lo, i_hi + 1):
            r = msg.ranges[i]

            if math.isnan(r):
                continue
            if math.isinf(r) or r > msg.range_max:
                r = msg.range_max
            elif r < msg.range_min:
                continue

            if r < best:
                best = r

        return None if math.isinf(best) else best

    # =========================================================================
    #  PART A - carve the scan into three sectors
    # =========================================================================
    def sector_windows(self):
        """Three (a_lo, a_hi) bearing pairs: right, front, left.

        Negative bearings are to the robot's RIGHT, so the right-hand window is
        the negative one. Swapping these two is the classic bug: the robot then
        swerves into obstacles instead of away from them.

        With the defaults and the LIMO's scan (angle_min = -2.0 rad,
        angle_increment = 0.011142061, N = 360) these map to beams
        62-140 (right), 140-219 (front) and 219-297 (left).
        """
        right = (-self.a_side, -self.a_front)                             # <-- BLANK 1a
        front = (-self.a_front, self.a_front)                             # <-- BLANK 1b
        left = (self.a_front, self.a_side)                                # <-- BLANK 1c

        return right, front, left

    # =========================================================================
    #  PART B - is the way ahead blocked?
    # =========================================================================
    def front_is_blocked(self, d_f):
        """Blocked test, with the blank-10 deadband built in.

        Commit at d_s. Once avoiding, require d_s + resume_margin before
        declaring the way clear again, so the decision cannot chatter.
        """
        limit = self.d_s + self.resume_margin if self.avoiding else self.d_s
        return d_f < limit                                                # <-- BLANK 2 (+ 10)

    # =========================================================================
    #  PART C - which way do we turn?
    # =========================================================================
    def choose_turn_sign(self, d_l, d_r):
        """+1.0 to turn left, -1.0 to turn right: towards the more open side."""
        if d_l > d_r:                                                     # <-- BLANK 3a
            return 1.0                                                    # <-- BLANK 3b
        if d_r > d_l:                                                     # <-- BLANK 3c
            return -1.0                                                   # <-- BLANK 3d

        # Exact tie. Never let floating-point luck decide, or the robot
        # oscillates. Policy: keep the direction already committed to; if there
        # is none, prefer left so the behaviour is repeatable.
        return self.turn_sign if self.turn_sign != 0.0 else 1.0           # <-- BLANK 4

    def both_sides_shut(self, d_l, d_r):
        """True when neither side offers a way out - a dead end or a corner."""
        return d_l < self.d_side and d_r < self.d_side                    # <-- BLANK 5

    # =========================================================================
    #  PART D - how hard do we turn, and how fast do we drive?
    # =========================================================================
    def angular_for(self, sign, d_l, d_r):
        """Turn rate, proportional to the normalised asymmetry.

            A = (d_l - d_r) / (d_l + d_r)   in [-1, +1], dimensionless

        A already carries the correct sign, so `sign` is needed only in the two
        degenerate cases below, where A is uninformative.
        """
        total = d_l + d_r

        if total < 1e-6:
            # Both sides read nothing usable: no asymmetry to measure, so fall
            # back on the policy sign at full rate.
            w = sign * self.w_max                                         # <-- BLANK 6 (degenerate)
        else:
            w = self.w_max * (d_l - d_r) / total                          # <-- BLANK 6
            if abs(w) < 1e-3:
                # Near-symmetric: a near-zero rate would leave the robot
                # dithering, so commit to the policy sign instead.
                w = sign * self.w_max

        return max(-self.w_max, min(self.w_max, w))                       # <-- BLANK 7

    def linear_for(self, blocked, dead_end, d_f):
        """Forward speed in m/s."""
        if dead_end:
            # Nowhere to go: rotate on the spot. Any forward component would
            # advance into space that has not been cleared.
            return 0.0                                                    # <-- BLANK 8a

        if blocked:
            # Slight headway makes a differential-drive robot trace a useful
            # arc; the instantaneous turn radius is R = v / w, so v = 0 would
            # be a pure spin.
            return self.v_creep                                           # <-- BLANK 8b

        # Task 3's ramp, clipped.
        ramp = self.v_max * (d_f - self.d_s) / (self.d_c - self.d_s)      # <-- BLANK 8c
        return max(0.0, min(self.v_max, ramp))

    # =========================================================================
    #  Glue
    # =========================================================================
    def on_scan(self, msg: LaserScan):

        right, front, left = self.sector_windows()

        d_r = self.min_in_sector(msg, *right)
        d_f = self.min_in_sector(msg, *front)
        d_l = self.min_in_sector(msg, *left)

        # Blind where it matters most: hold still.
        if d_f is None:
            self.publish(0.0, 0.0)
            self.get_logger().warning(
                'No usable readings in the front sector - holding still.',
                throttle_duration_sec=2.0)
            return

        # A missing side reading is treated as "that side is shut", the
        # pessimistic assumption.
        if d_l is None:
            d_l = 0.0
        if d_r is None:
            d_r = 0.0

        # ORDER MATTERS: test with the CURRENT value of self.avoiding, then
        # update it below.
        blocked = self.front_is_blocked(d_f)
        dead_end = blocked and self.both_sides_shut(d_l, d_r)

        # ---- BLANKS 9 and 10 ------------------------------------------------
        if blocked:
            if not self.avoiding:
                # Decide ONCE, at the start of the manoeuvre. Recomputing every
                # scan is what makes the robot wobble head-on into a wall:
                # turning left changes what the left sector sees, which flips
                # the decision to right, which flips it back.
                self.avoiding = True
                self.turn_sign = self.choose_turn_sign(d_l, d_r)          # <-- BLANK 9a

            if dead_end:
                # Sides are near-equal here, so the proportional rate would be
                # far too slow. Rotate at full rate.
                w = self.turn_sign * self.w_max
            else:
                w = self.angular_for(self.turn_sign, d_l, d_r)            # <-- BLANK 9b
        else:
            self.avoiding = False
            self.turn_sign = 0.0                                          # <-- BLANK 9c
            w = 0.0                                                       # <-- BLANK 9d
        # ---------------------------------------------------------------------

        v = self.linear_for(blocked, dead_end, d_f)

        self.publish(v, w)

        state = 'DEAD-END' if dead_end else ('AVOID' if blocked else 'CRUISE')
        self.get_logger().info(
            f'R {d_r:.2f} | F {d_f:.2f} | L {d_l:.2f} m  ->  '
            f'{state:<8} v={v:+.3f} w={w:+.3f}',
            throttle_duration_sec=0.5)

    def publish(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ReactiveAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
