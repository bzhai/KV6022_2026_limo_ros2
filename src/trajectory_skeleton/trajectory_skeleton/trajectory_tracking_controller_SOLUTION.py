#!/usr/bin/env python3
"""
KV6022 - Trajectory Tracking Controller  [INSTRUCTOR SOLUTION]

=============================================================================
  COMPLETE REFERENCE SOLUTION. Do not distribute to students.
  Generated from the student skeleton, so docstrings, hints and task
  numbering are identical - only the five TODO blocks are filled in.
=============================================================================

Read CONTROLLER_WORKSHEET.md alongside this file. It contains the derivations,
the staged tasks, and three tiers of hints per task.

TASK MAP
--------
  TASK 0   normalize_angle()          ~1 line     scaffolding: HIGH
  TASK 1   compute_errors()           ~6 lines    scaffolding: HIGH
  TASK 2   control_on_off()           ~8 lines    scaffolding: MEDIUM
  TASK 3   control_proportional()     ~8 lines    scaffolding: LOW
  TASK 4   control_kinematic()        ~8 lines    scaffolding: MINIMAL

Do them in order. Each task depends on the ones before it, and TASKs 0-1 can
be verified without the simulator:

    python3 check_my_math.py

Everything outside the TODO blocks (ROS plumbing, TF lookup, parameters,
saturation, logging) is already complete. You should not need to edit it.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Transform, Twist
from tf2_ros import (
    TransformListener,
    Buffer,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)


# =============================================================================
#  TASK 0 - Angle wrapping                                  [HIGH scaffolding]
# =============================================================================
def normalize_angle(angle):
    r"""Wrap an angle into the interval [-pi, +pi].

    WHY THIS MATTERS
    ----------------
    Every angle in this workshop is a *difference* between two headings, and
    differences of angles do not behave like differences of numbers. Suppose
    the robot faces theta = +175 deg and the waypoint lies at +185 deg. The raw
    subtraction gives an error of +10 deg, which is correct. But if the robot
    faces -175 deg instead, the raw subtraction gives

        185 - (-175) = 360 deg

    when the true error is 0 deg. Feed that into a proportional controller and
    it commands a violent spin to correct an error that does not exist.

    We need a function that maps any real angle to the equivalent angle in
    [-pi, pi]. The trick is that sin and cos are already periodic, so they
    discard the excess revolutions for us:

        wrap(phi) = atan2( sin(phi), cos(phi) )

    Both sin(phi) and cos(phi) are unchanged by adding 2*pi*k to phi, and
    atan2 always returns a value in [-pi, pi]. So the composition throws away
    whole revolutions and keeps the part we care about.

    TODO (TASK 0)
    -------------
    Replace the line below with the one-line implementation.
    Requirements: normalize_angle(4*pi) must return approximately 0.0, and
    normalize_angle(-3*pi/2) must return approximately +pi/2. Note that
    normalize_angle(3*pi) is +pi (or equivalently -pi), not 0 - three
    half-turns is a full turn plus a half-turn, and only the full turn is
    discarded.

    HINT 1: You need exactly one call to math.atan2, with two arguments.
    HINT 2: math.atan2(y, x) takes the *vertical* component first.
    """
    # --- SOLUTION (TASK 0) --------------------------------------------------
    return math.atan2(math.sin(angle), math.cos(angle))
    # ------------------------------------------------------------------------


def clamp(value, limit):
    """Saturate value to [-limit, +limit].  (Provided - no work needed.)"""
    return max(-limit, min(limit, value))


class TrajectoryTrackingController(Node):

    def __init__(self):
        super().__init__('smart_driver')

        self.waypoint = None
        self.robot_pose = None
        self._last_state = None

        # ---------------- parameters (all provided) ----------------
        self.declare_parameter('controller', 'onoff')          # onoff | p | kinematic
        self.declare_parameter('match_orientation', False)     # True -> 'dor' mode
        self.declare_parameter('control_rate', 20.0)           # Hz
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')

        # Tolerances and actuator limits
        self.declare_parameter('dist_tol', 0.10)   # m   - position "reached"
        self.declare_parameter('ang_tol', 0.10)    # rad - orientation "reached"
        self.declare_parameter('v_max', 0.25)      # m/s
        self.declare_parameter('w_max', 1.2)       # rad/s

        # TASK 2: on/off gains
        self.declare_parameter('v_on', 0.15)
        self.declare_parameter('w_on', 0.5)
        self.declare_parameter('onoff_align_tol', 0.15)

        # TASK 3: proportional gains
        self.declare_parameter('kp_lin', 0.6)
        self.declare_parameter('kp_ang', 1.5)
        self.declare_parameter('p_turn_first', 0.7)

        # TASK 4: kinematic gains
        self.declare_parameter('k_rho', 0.5)
        self.declare_parameter('k_alpha', 1.5)
        self.declare_parameter('k_beta', -0.6)

        # ---------------- ROS plumbing (all provided) ----------------
        self.subscription = self.create_subscription(
            Transform, '/waypoint_cmd', self.waypoint_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        rate = float(self.get_parameter('control_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(
            f"Controller '{self.get_parameter('controller').value}' at {rate:.0f} Hz "
            f"(match_orientation={self.get_parameter('match_orientation').value})")

    # ------------------------------------------------------------------
    #  Callbacks (provided)
    # ------------------------------------------------------------------
    def waypoint_callback(self, msg):
        self.waypoint = msg

    def timer_callback(self):
        """Main control loop. Runs at control_rate Hz. Provided - do not edit.

        Note the shape of this loop, because it is the shape of every feedback
        controller you will ever write:

            1. MEASURE    read the robot's pose from TF
            2. COMPARE    turn (pose, goal) into error terms   <- TASK 1
            3. DECIDE     turn error terms into (v, omega)     <- TASKs 2,3,4
            4. ACT        saturate and publish to /cmd_vel

        You are implementing steps 2 and 3. Steps 1 and 4 are done.
        """
        self._read_params()

        # ---- 1. MEASURE --------------------------------------------
        try:
            self.robot_pose = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(f"Transform error: {ex}", throttle_duration_sec=2.0)
            self.stop_robot()
            return

        if self.waypoint is None:
            self.get_logger().info("Waiting for a waypoint on /waypoint_cmd...",
                                   throttle_duration_sec=2.0)
            self.stop_robot()
            return

        x = self.robot_pose.transform.translation.x
        y = self.robot_pose.transform.translation.y
        theta = self.quaternion_to_yaw(self.robot_pose.transform.rotation)

        wx = self.waypoint.translation.x
        wy = self.waypoint.translation.y
        wtheta = self.quaternion_to_yaw(self.waypoint.rotation)

        # ---- 2. COMPARE --------------------------------------------
        rho, alpha, beta, theta_err = self.compute_errors(
            x, y, theta, wx, wy, wtheta, self.match_orientation)

        # ---- 3. DECIDE ---------------------------------------------
        if self.controller == 'onoff':
            v, w, state = self.control_on_off(rho, alpha, theta_err)
        elif self.controller == 'p':
            v, w, state = self.control_proportional(rho, alpha, theta_err)
        elif self.controller == 'kinematic':
            v, w, state = self.control_kinematic(rho, alpha, beta, theta_err)
        else:
            self.get_logger().error(
                f"Unknown controller '{self.controller}'. Use onoff | p | kinematic.",
                throttle_duration_sec=5.0)
            self.stop_robot()
            return

        # ---- 4. ACT ------------------------------------------------
        motor_command = Twist()
        motor_command.linear.x = clamp(v, self.v_max)
        motor_command.angular.z = clamp(w, self.w_max)
        self.publisher.publish(motor_command)

        # ---- Debug output ------------------------------------------
        self.get_logger().info(
            f"[{self.controller}:{state}] pose=({x:+.2f},{y:+.2f},{math.degrees(theta):+6.1f}d) "
            f"goal=({wx:+.2f},{wy:+.2f},{math.degrees(wtheta):+6.1f}d) "
            f"rho={rho:.3f} alpha={math.degrees(alpha):+6.1f}d beta={math.degrees(beta):+6.1f}d "
            f"-> v={motor_command.linear.x:+.3f} w={motor_command.angular.z:+.3f}",
            throttle_duration_sec=0.5)

        if state != self._last_state:
            self.get_logger().info(f"state -> {state}")
            self._last_state = state

    # =================================================================
    #  TASK 1 - The error terms                       [HIGH scaffolding]
    # =================================================================
    @staticmethod
    def compute_errors(x, y, theta, wx, wy, wtheta, match_orientation):
        r"""Convert an absolute pose and an absolute goal into four error terms.

        This is the conceptual heart of the whole workshop. All three
        controllers consume these four numbers and nothing else. Get this
        right and the controllers are three or four lines each; get it wrong
        and no amount of gain tuning will save you.

        THE IDEA
        --------
        Both the robot pose (x, y, theta) and the waypoint (wx, wy, wtheta) are
        expressed in the fixed `odom` frame. But a differential-drive robot
        cannot act on absolute coordinates: it has exactly two controls, a
        forward speed v and a turn rate omega. So we re-express "where the goal
        is" in terms the robot can act on - a distance to close and some angles
        to null. This is a change of coordinates from Cartesian error to polar
        error.

        THE FOUR TERMS
        --------------
        Let the Cartesian offset to the goal be

            \Delta x = wx - x,        \Delta y = wy - y

        1. rho  -  distance remaining, always >= 0:

               \rho = \sqrt{\Delta x^2 + \Delta y^2}

           Drives the *linear* velocity. When rho = 0 we have arrived.

        2. alpha  -  bearing of the goal in the robot's own frame:

               \alpha = \mathrm{wrap}\big(\mathrm{atan2}(\Delta y, \Delta x) - \theta\big)

           atan2(dy, dx) is the absolute compass direction from robot to goal.
           Subtracting theta rotates that into the robot's frame, so:
               alpha =  0      goal is dead ahead
               alpha = +pi/2   goal is directly to the left
               alpha = +/-pi   goal is directly behind
           Drives the *angular* velocity. Note alpha says nothing about which
           way the robot will be facing when it arrives.

        3. theta_err  -  mismatch between current heading and the goal's
           required final orientation:

               \theta_e = \mathrm{wrap}(w\theta - \theta)

           Only meaningful in `dor` mode, and only once rho is small. This is
           what you null while spinning on the spot at the end.

        4. beta  -  mismatch between the goal orientation and the direction
           from which you are approaching:

               \beta = \mathrm{wrap}(w\theta - \theta - \alpha)
                     = \mathrm{wrap}(\theta_e - \alpha)

           Harder to picture than the others, so read it this way: alpha tells
           you where the goal *is*, beta tells you how *wrongly angled* your
           approach is. If beta = 0 you are travelling straight down the
           goal's own heading and will arrive already correctly oriented. If
           beta is large you are approaching side-on and will need to curve.
           Only the kinematic controller uses beta.

           In `dis` mode the goal orientation is not graded, so beta carries no
           useful information and we set it to 0.0. This is why the kinematic
           controller degenerates gracefully into a P controller in `dis` mode.

        TODO (TASK 1)
        -------------
        Replace the four stubs below. Two rules, and nearly every bug in this
        workshop is a violation of one of them:

          RULE A: rho is a length. Never wrap it. Never let it go negative.
          RULE B: every angular term must pass through normalize_angle().
                  Every one. Including beta, even though it is built from two
                  quantities that were each already wrapped - wrapping is not
                  preserved by subtraction.

        HINT 1: math.hypot(dx, dy) computes sqrt(dx*dx + dy*dy) and is more
                numerically robust than writing it out.
        HINT 2: For alpha, work out the absolute bearing first, then subtract
                the robot's heading, then wrap the result. Three steps, one
                line.
        HINT 3: beta can be written from theta_err and alpha in one short
                expression - look at the second form given above. Remember to
                wrap it, and remember the match_orientation switch.

        Returns:
            (rho, alpha, beta, theta_err)
        """
        dx = wx - x
        dy = wy - y

        # --- SOLUTION (TASK 1) -------------------------------------------
        rho = math.hypot(dx, dy)
        alpha = normalize_angle(math.atan2(dy, dx) - theta)
        theta_err = normalize_angle(wtheta - theta)

        if match_orientation:
            beta = normalize_angle(theta_err - alpha)
        else:
            beta = 0.0
        # -----------------------------------------------------------------

        return rho, alpha, beta, theta_err

    # =================================================================
    #  TASK 2 - On/Off controller                   [MEDIUM scaffolding]
    # =================================================================
    def control_on_off(self, rho, alpha, theta_err):
        r"""Bang-bang control: every output is either zero or a fixed magnitude.

        THE CONTROL LAW
        ---------------
        There is no proportionality here at all. The controller asks a yes/no
        question about each error and responds with a constant:

        \[
        (v, \omega) =
        \begin{cases}
          (0,\; \omega_0 \operatorname{sgn}\alpha)
              & \rho > \epsilon_d \;\text{and}\; |\alpha| > \epsilon_a
                  \quad\text{(turn on the spot)}\\[4pt]
          (v_0,\; 0)
              & \rho > \epsilon_d \;\text{and}\; |\alpha| \le \epsilon_a
                  \quad\text{(drive straight)}\\[4pt]
          (0,\; \omega_0 \operatorname{sgn}\theta_e)
              & \rho \le \epsilon_d \;\text{and}\; |\theta_e| > \epsilon_a
                  \quad\text{(spin to final heading, 'dor' only)}\\[4pt]
          (0,\; 0) & \text{otherwise (stop)}
        \end{cases}
        \]

        where \(v_0\) = `self.v_on`, \(\omega_0\) = `self.w_on`,
        \(\epsilon_d\) = `self.dist_tol`, \(\epsilon_a\) = `self.onoff_align_tol`.

        The sgn term only sets the *direction* of the turn; the magnitude is
        always the same. That is what "on/off" means.

        WHAT YOU SHOULD OBSERVE
        -----------------------
        Because the command does not shrink as the error shrinks, the robot
        arrives at the tolerance boundary still travelling at full speed v_0.
        It therefore overshoots, the error flips sign, and it corrects at full
        speed in the other direction. Expect limit-cycle behaviour: a
        permanent oscillation whose amplitude is set by v_0, by the tolerance
        band, and by the loop rate. Shrinking dist_tol makes this *worse*, not
        better. Predict why before you test it - that prediction is the point
        of this exercise.

        TODO (TASK 2)
        -------------
        Implement the four cases above. Return a 3-tuple (v, w, state) where
        state is one of the strings 'turn', 'drive', 'spin', 'stop' - it drives
        the debug log, so use exactly those spellings.

        HINT 1: The structure is a chain of if / elif / else on rho and the
                absolute values of the angles. Handle "not yet arrived"
                (rho > dist_tol) first, and within that branch decide whether
                you are pointing the right way.
        HINT 2: math.copysign(magnitude, error) gives you
                magnitude * sgn(error) in one call. Use it rather than writing
                an if/else on the sign.
        HINT 3: Guard the third case with `self.match_orientation`, or your
                robot will refuse to leave each waypoint in `dis` mode - it
                will sit there trying to null an orientation error that the
                referee is not even grading.
        """
        # --- SOLUTION (TASK 2) -------------------------------------------
        if rho > self.dist_tol:
            if abs(alpha) > self.onoff_align_tol:
                # Badly aimed: rotate only, direction from the sign of alpha.
                return 0.0, math.copysign(self.w_on, alpha), 'turn'
            # Aimed well enough: constant speed ahead, no steering correction.
            return self.v_on, 0.0, 'drive'

        if self.match_orientation and abs(theta_err) > self.ang_tol:
            return 0.0, math.copysign(self.w_on, theta_err), 'spin'

        return 0.0, 0.0, 'stop'
        # -----------------------------------------------------------------

    # =================================================================
    #  TASK 3 - Proportional controller                [LOW scaffolding]
    # =================================================================
    def control_proportional(self, rho, alpha, theta_err):
        r"""Commands proportional to the errors that produce them.

        THE CONTROL LAW
        ---------------
        \[
            v = K_p^{lin} \, \rho \cos\alpha,
            \qquad
            \omega = K_p^{ang} \, \alpha
        \]

        Gains: `self.kp_lin`, `self.kp_ang`.

        WHY THE \(\cos\alpha\) FACTOR
        -----------------------------
        The bare law \(v = K_p \rho\) has a defect: it commands full forward
        speed even when the goal is off to the side, or behind. The robot then
        drives a long arc away from the waypoint before curving back.
        Multiplying by \(\cos\alpha\) projects the command onto the robot's
        heading, so speed falls to zero as the goal reaches 90 degrees off the
        bow and goes *negative* (reverse) when the goal is behind.

        There is a formal reason to like this factor. Take the candidate
        Lyapunov function \(V = \tfrac{1}{2}\rho^2 \ge 0\). For a
        differential-drive robot \(\dot\rho = -v\cos\alpha\), so substituting
        the control law gives

        \[
            \dot V = \rho \dot\rho = -K_p^{lin} \rho^2 \cos^2\alpha \le 0 .
        \]

        \(V\) can never increase, so the distance to the goal is
        non-increasing for *any* alpha - the robot can never be driven further
        away. That is a stability guarantee the bare law does not give you.

        You will still want a "turn on the spot first" branch when
        \(|\alpha|\) exceeds `self.p_turn_first`: cos(alpha) makes large-alpha
        motion harmless, but not efficient.

        TODO (TASK 3)
        -------------
        Implement the law. Same 3-tuple return contract as TASK 2, same four
        states.

        HINT 1: Reuse the case structure you built in TASK 2 - not yet
                arrived / arrived but mis-oriented / done. Only the
                *expressions* inside each branch change.
        HINT 2: omega can be computed once before the branching, since the
                same expression applies whether or not you are also moving
                forward.
        HINT 3: For the final spin-in-place case, apply the angular gain to
                theta_err instead of alpha. When rho is tiny, alpha is
                atan2 of two nearly-zero numbers and is therefore numerical
                noise - do not steer with it.
        """
        # --- SOLUTION (TASK 3) -------------------------------------------
        if rho > self.dist_tol:
            w = self.kp_ang * alpha

            if abs(alpha) > self.p_turn_first:
                return 0.0, w, 'turn'

            return self.kp_lin * rho * math.cos(alpha), w, 'drive'

        if self.match_orientation and abs(theta_err) > self.ang_tol:
            return 0.0, self.kp_ang * theta_err, 'spin'

        return 0.0, 0.0, 'stop'
        # -----------------------------------------------------------------

    # =================================================================
    #  TASK 4 - Kinematic position controller      [MINIMAL scaffolding]
    # =================================================================
    def control_kinematic(self, rho, alpha, beta, theta_err):
        r"""Full differential-drive position controller.

        THE CONTROL LAW
        ---------------
        \[
            v = K_\rho \, \rho,
            \qquad
            \omega = K_\alpha \, \alpha + K_\beta \, \beta
        \]

        Gains: `self.k_rho`, `self.k_alpha`, `self.k_beta`.

        The angular command now does two jobs at once. \(K_\alpha \alpha\)
        steers toward the waypoint; \(K_\beta \beta\), with \(K_\beta < 0\),
        bends the trajectory so the robot arrives already pointing along the
        goal's own heading. The result is a single smooth curve into the
        waypoint rather than the drive-then-pirouette of TASKs 2 and 3.

        STABILITY (worth understanding before you tune)
        -----------------------------------------------
        In polar error coordinates the plant is

        \[
            \dot\rho = -v\cos\alpha, \qquad
            \dot\alpha = \frac{v \sin\alpha}{\rho} - \omega, \qquad
            \dot\beta = -\frac{v \sin\alpha}{\rho}.
        \]

        Substituting the control law and linearising about the origin
        (\(\cos\alpha \approx 1\), \(\sin\alpha \approx \alpha\)) gives

        \[
            \dot\rho = -K_\rho \rho, \qquad
            \dot\alpha = -(K_\alpha - K_\rho)\alpha - K_\beta \beta, \qquad
            \dot\beta = -K_\rho \alpha,
        \]

        with characteristic polynomial

        \[
            (\lambda + K_\rho)\big(\lambda^2 + (K_\alpha - K_\rho)\lambda
              - K_\rho K_\beta\big) = 0 .
        \]

        Applying the Routh-Hurwitz conditions to that polynomial yields three
        inequalities your gains must satisfy:

        \[
            K_\rho > 0, \qquad K_\beta < 0, \qquad K_\alpha - K_\rho > 0 .
        \]

        Violate the third and the robot spirals instead of converging.
        Violate the second and it arrives at the right place facing the wrong
        way, persistently. These are not arbitrary rules of thumb - they fall
        straight out of the polynomial above, and you can cite the derivation
        in your report.

        Suggested starting point: \(K_\rho = 0.5\), \(K_\alpha = 1.5\),
        \(K_\beta = -0.6\). Verify it satisfies all three inequalities.

        TODO (TASK 4)
        -------------
        Implement it. You have seen the pattern twice; this time work out the
        case structure yourself.

        HINT 1: The law as written has a singularity you must handle. Look at
                \(\dot\alpha\) above and ask what happens to the alpha term as
                rho approaches zero. Then ask what your code should do once
                rho is inside dist_tol.
        HINT 2: If \(|\alpha| > \pi/2\) the waypoint is behind the robot and
                \(v = K_\rho \rho\) is positive, so it drives forward while
                turning hard - a wide arc away from the goal. Consider
                suppressing v in that case and letting omega do the work
                first. (The cos(alpha) trick from TASK 3 is an alternative
                answer; either is defensible, and comparing them is a good
                thing to write up.)
        HINT 3: In `dis` mode beta is 0.0, so omega collapses to
                \(K_\alpha \alpha\). If your `dis` runs behave exactly like
                your TASK 3 runs, that is expected and correct - it is
                evidence your TASK 1 is right, not a bug.
        """
        # --- SOLUTION (TASK 4) -------------------------------------------
        if rho > self.dist_tol:
            v = self.k_rho * rho
            w = self.k_alpha * alpha + self.k_beta * beta

            # Waypoint behind the robot: suppress v so omega can swing the
            # nose round first, instead of sweeping a wide arc away.
            if abs(alpha) > math.pi / 2.0:
                return 0.0, w, 'turn'

            return v, w, 'drive'

        # Inside dist_tol alpha is numerical noise, so steer on theta_err.
        if self.match_orientation and abs(theta_err) > self.ang_tol:
            return 0.0, self.k_alpha * theta_err, 'spin'

        return 0.0, 0.0, 'stop'
        # -----------------------------------------------------------------

    # ------------------------------------------------------------------
    #  Helpers (all provided)
    # ------------------------------------------------------------------
    def _read_params(self):
        """Refresh gains every cycle so they can be tuned with `ros2 param set`."""
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.controller = str(g('controller')).lower()
        self.match_orientation = bool(g('match_orientation'))
        self.base_frame = str(g('base_frame'))
        self.odom_frame = str(g('odom_frame'))

        self.dist_tol = float(g('dist_tol'))
        self.ang_tol = float(g('ang_tol'))
        self.v_max = float(g('v_max'))
        self.w_max = float(g('w_max'))

        self.v_on = float(g('v_on'))
        self.w_on = float(g('w_on'))
        self.onoff_align_tol = float(g('onoff_align_tol'))

        self.kp_lin = float(g('kp_lin'))
        self.kp_ang = float(g('kp_ang'))
        self.p_turn_first = float(g('p_turn_first'))

        self.k_rho = float(g('k_rho'))
        self.k_alpha = float(g('k_alpha'))
        self.k_beta = float(g('k_beta'))

    def stop_robot(self):
        self.publisher.publish(Twist())

    @staticmethod
    def quaternion_to_yaw(q):
        """Convert quaternion to yaw (rotation about z).

        Provided. For the curious: a rotation of theta about z has quaternion
        (w, x, y, z) = (cos(theta/2), 0, 0, sin(theta/2)), and inverting that
        via the half-angle identities gives the atan2 form below. It is written
        in the general form so that it still returns the correct yaw when the
        robot has a small amount of roll or pitch.
        """
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
    finally:
        try:
            node.stop_robot()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
