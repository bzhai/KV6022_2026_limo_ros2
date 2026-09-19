# file: reactive_avoid_CLOZE.py
#
# =============================================================================
#  EXERCISE (Workshop task 4): reactive obstacle avoidance
# =============================================================================
#
#  Goal: instead of stopping in front of an obstacle, slice the scan into three
#  sectors - front-right, front, front-left - and turn towards whichever side
#  has more open space. Keep driving.
#
#  What is GIVEN: `sector_bounds` and `min_in_sector`. You wrote both in task 3
#  and there is nothing new in them, so they are complete here. Read them once
#  to remind yourself of the special-value handling, then leave them alone.
#
#  What is BLANK: the sector geometry (blank 1) and the whole decision layer
#  (blanks 2 to 10). Each unfilled blank raises NotImplementedError naming
#  itself. Blank 10 is a stretch task; the node runs without it, but it will
#  dither in a symmetric corridor.
#
#  Sign convention, worth pinning down before you start:
#      linear.x  > 0  =  forwards
#      angular.z > 0  =  anticlockwise  =  turn LEFT
#      bearings  > 0  =  to the robot's LEFT
#  All three agree, which is what makes the arithmetic in blank 6 come out
#  with the right sign if you are careful.
# =============================================================================

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def TODO(n):
    """Placeholder. Delete the call and write the real expression instead."""
    raise NotImplementedError(f"BLANK {n} is not filled in yet.")


class ReactiveAvoid(Node):

    def __init__(self):
        super().__init__('reactive_avoid')

        # --- distances --------------------------------------------------------
        self.declare_parameter('safety_radius', 0.35)      # d_s: front blocked below this   [m]
        self.declare_parameter('caution_radius', 0.70)      # d_c: start slowing here         [m]
        self.declare_parameter('side_blocked', 0.45)        # a side counts as shut below this [m]
        self.declare_parameter('resume_margin', 0.08)       # hysteresis on "front clear"     [m]

        # --- speeds -----------------------------------------------------------
        self.declare_parameter('cruise_speed', 0.22)        # v_max                           [m/s]
        self.declare_parameter('turn_speed_max', 0.9)       # w_max                           [rad/s]
        self.declare_parameter('creep_speed', 0.05)         # forward speed while swerving    [m/s]

        # --- sector geometry --------------------------------------------------
        self.declare_parameter('front_half_angle', 0.4363)  # 25 deg: edge of the front sector [rad]
        self.declare_parameter('side_outer_angle', 1.3090)  # 75 deg: outer edge of each side  [rad]

        self.d_s = self.get_parameter('safety_radius').value
        self.d_c = self.get_parameter('caution_radius').value
        self.d_side = self.get_parameter('side_blocked').value
        self.resume_margin = self.get_parameter('resume_margin').value
        self.v_max = self.get_parameter('cruise_speed').value
        self.w_max = self.get_parameter('turn_speed_max').value
        self.v_creep = self.get_parameter('creep_speed').value
        self.a_front = self.get_parameter('front_half_angle').value
        self.a_side = self.get_parameter('side_outer_angle').value

        # Latched avoidance state, used by blank 10.
        self.avoiding = False       # are we currently swerving?
        self.turn_sign = 0.0        # +1.0 = left, -1.0 = right, 0.0 = not committed

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f'reactive_avoid started: front +/-{math.degrees(self.a_front):.0f} deg, '
            f'sides out to {math.degrees(self.a_side):.0f} deg, '
            f'block < {self.d_s} m, cruise {self.v_max} m/s')

    # =========================================================================
    #  GIVEN - you wrote these in task 3. No blanks here.
    # =========================================================================
    def sector_bounds(self, msg: LaserScan, a_lo, a_hi):
        """Inclusive index range of the beams whose bearing lies in [a_lo, a_hi]."""
        n = len(msg.ranges)

        def to_index(angle):
            i = int(round((angle - msg.angle_min) / msg.angle_increment))
            return max(0, min(n - 1, i))

        return to_index(a_lo), to_index(a_hi)

    def min_in_sector(self, msg: LaserScan, a_lo, a_hi):
        """Smallest trustworthy range in the sector, or None if no usable beam.

        Reminder of the three special cases: NaN means the beam failed, so skip
        it; inf means nothing was found within range, which is GOOD news, so
        substitute range_max; below range_min is inside the blind shell, so skip.
        """
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
        """Return three (a_lo, a_hi) angle pairs: right, front, left.

        You have two parameters to work with:

            self.a_front   the bearing of the edge of the FRONT sector  (25 deg)
            self.a_side    the outer edge of each SIDE sector           (75 deg)

        Lay them out on the bearing axis and read off the six numbers:

            -a_side        -a_front          +a_front        +a_side
               |--------------|-----------------|---------------|
               |  FRONT-RIGHT |      FRONT      |  FRONT-LEFT   |
               (negative bearings)          (positive bearings)

        Two things to get right. First, the sectors must not overlap - a beam
        counted in both FRONT and FRONT-LEFT is double-braked. Second, remember
        which sign of bearing is which side: a NEGATIVE bearing is to the
        robot's RIGHT, so the right-hand sector is the one with the negative
        numbers. Getting this backwards makes the robot swerve INTO obstacles,
        which is the single most common bug in this task.

        Ignore anything outside +/-a_side. Those beams look sideways or
        backwards and cannot tell you anything about where you are heading.
        """
        # ---------------------------------------------------------------------
        # BLANK 1 - three (low, high) bearing pairs, in radians.
        # ---------------------------------------------------------------------
        right = TODO('1a')      # e.g. (-something, -something)
        front = TODO('1b')
        left = TODO('1c')

        return right, front, left

    # =========================================================================
    #  PART B - is the way ahead blocked?
    # =========================================================================
    def front_is_blocked(self, d_f):
        """True when the FRONT sector distance means we must not carry straight on.

        Without hysteresis this is a single comparison. With it (blank 10) the
        threshold depends on whether we are already avoiding: commit at d_s,
        but do not declare the way clear again until there is a bit more room
        than that. Write the simple version first and test it.
        """
        # ---------------------------------------------------------------------
        # BLANK 2 - the blocked test.
        # ---------------------------------------------------------------------
        return TODO(2)

    # =========================================================================
    #  PART C - which way do we turn?
    # =========================================================================
    def choose_turn_sign(self, d_l, d_r):
        """Return +1.0 to turn left, -1.0 to turn right.

        The whole idea of the task in one line: turn towards the side with more
        open space. Compare the two side distances and return the sign that
        steers towards the larger one.

        Then the awkward case. In a symmetric dead end the two distances are
        equal, or near enough that noise decides for you, and the robot sits
        oscillating left-right-left-right forever. You must break the tie
        deliberately rather than let floating-point luck do it. Two workable
        policies:

          * always prefer one fixed side (simple, and it makes the robot's
            behaviour repeatable, which matters when you are marking it);
          * keep whichever direction you already committed to, if any -
            self.turn_sign holds it.

        Pick one, implement it, and be ready to justify the choice.
        """
        # ---------------------------------------------------------------------
        # BLANK 3 - the comparison that picks the more open side.
        # BLANK 4 - the tie-break, for when the two sides are equally open.
        # ---------------------------------------------------------------------
        if TODO('3a'):
            return TODO('3b')       # left is more open
        if TODO('3c'):
            return TODO('3d')       # right is more open

        return TODO(4)              # tie

    def both_sides_shut(self, d_l, d_r):
        """True when neither side offers a way out - a dead end or a corner.

        Both side distances are below self.d_side. The response is not to pick
        the marginally better one and creep into it; it is to stop translating
        and rotate on the spot until something opens up.
        """
        # ---------------------------------------------------------------------
        # BLANK 5 - the dead-end test.
        # ---------------------------------------------------------------------
        return TODO(5)

    # =========================================================================
    #  PART D - how hard do we turn, and how fast do we drive?
    # =========================================================================
    def angular_for(self, sign, d_l, d_r):
        """Angular velocity in rad/s.

        The crudest version is bang-bang: sign * w_max, full lock either way.
        It works, and it looks terrible - the robot lurches.

        Better is to make the turn rate proportional to HOW asymmetric the two
        sides are. Use the normalised asymmetry

            A = (d_l - d_r) / (d_l + d_r)

        which is dimensionless and lies in [-1, +1] whatever the units or the
        room size, then scale w_max by it. A big difference gives a hard turn,
        a slight difference a gentle correction. Note that A already carries
        the correct sign, so think about whether you still need `sign` here -
        and about what the denominator does if both readings are tiny.

        Whichever you write, saturate the result to +/- w_max so a parameter
        change cannot command a rate the robot cannot deliver.
        """
        # ---------------------------------------------------------------------
        # BLANK 6 - the raw turn rate (bang-bang, or proportional to A).
        # BLANK 7 - saturate it into [-w_max, +w_max].
        # ---------------------------------------------------------------------
        w = TODO(6)
        return TODO(7)

    def linear_for(self, blocked, dead_end, d_f):
        """Forward speed in m/s.

        Three cases, in order of how constrained you are:

          * dead end   - do not translate at all. Rotating on the spot is the
                         only safe move when you have nowhere to go.
          * blocked    - creep forward at self.v_creep while swerving. Zero
                         also works and is safer; a slow creep makes the
                         manoeuvre smoother because a differential-drive robot
                         turns more usefully while it has some headway.
          * clear      - the ramp you already wrote in task 3:

                             v = v_max * (d_f - d_s) / (d_c - d_s)

                         clipped into [0, v_max] so that a front distance past
                         d_c gives full cruise and one at d_s gives zero.
        """
        # ---------------------------------------------------------------------
        # BLANK 8 - the three speeds.
        # ---------------------------------------------------------------------
        if dead_end:
            return TODO('8a')

        if blocked:
            return TODO('8b')

        return TODO('8c')

    # =========================================================================
    #  Glue - mostly written, apart from blanks 9 and 10.
    # =========================================================================
    def on_scan(self, msg: LaserScan):

        right, front, left = self.sector_windows()

        d_r = self.min_in_sector(msg, *right)
        d_f = self.min_in_sector(msg, *front)
        d_l = self.min_in_sector(msg, *left)

        # Fail-safe: if the FRONT sector told us nothing, we are blind where it
        # matters most. Hold still. A missing side reading is less serious - we
        # treat that side as shut, which is the pessimistic assumption.
        if d_f is None:
            self.publish(0.0, 0.0)
            self.get_logger().warning(
                'No usable readings in the front sector - holding still.',
                throttle_duration_sec=2.0)
            return

        if d_l is None:
            d_l = 0.0
        if d_r is None:
            d_r = 0.0

        blocked = self.front_is_blocked(d_f)
        dead_end = blocked and self.both_sides_shut(d_l, d_r)

        # ---------------------------------------------------------------------
        # BLANK 9 - decide the turn.
        #
        # When the front is clear there is nothing to avoid, so the robot
        # should drive straight: no turn at all, and no committed direction
        # left over from the last obstacle.
        #
        # When it is blocked, choose a sign and work out the rate from it.
        # Store the sign in self.turn_sign, because blank 10 and the tie-break
        # in blank 4 both read it.
        # ---------------------------------------------------------------------
        if blocked:
            self.turn_sign = TODO('9a')
            w = TODO('9b')
        else:
            self.turn_sign = TODO('9c')
            w = TODO('9d')

        v = self.linear_for(blocked, dead_end, d_f)

        # ---------------------------------------------------------------------
        # BLANK 10 (STRETCH) - commitment.
        #
        # Run the node without this first, then drive it at a wall head-on. The
        # robot decides "left", turns a few degrees, which changes what the
        # sectors see, which flips the decision to "right", and so on. It
        # wobbles up to the wall instead of going round it.
        #
        # The cure is to commit: once you have started avoiding, keep the SAME
        # turn_sign until the front is properly clear - not merely past d_s,
        # but past d_s + resume_margin. self.avoiding is the flag that records
        # "a manoeuvre is in progress"; use it to keep the earlier sign rather
        # than recomputing one every scan.
        #
        # Replace the line below with that logic.
        # ---------------------------------------------------------------------
        self.avoiding = blocked        # <-- naive version; blank 10 replaces this

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
