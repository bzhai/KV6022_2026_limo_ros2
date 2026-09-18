#!/usr/bin/env python3
"""
KV6022 - Trajectory Tracking Controller  [STUDENT SKELETON - HIGH SCAFFOLDING]

=============================================================================
  THIS FILE RUNS AS-IS. The robot will not move.

  Every control structure is already written for you: all the if/elif
  branches, all the return statements, all the state strings. You do not
  need to design any logic.

  Your job is to fill in 21 numbered BLANKS, each of which is a single
  expression on a single line. Every blank has:
     - a FILL-IN TABLE entry in the docstring above it, in plain English
     - the variable names you need, listed
     - the equation it comes from
=============================================================================

Read CONTROLLER_WORKSHEET.md alongside this file for the derivations.

TASK MAP  (all tasks HIGH scaffolding)
--------------------------------------
  TASK 0   normalize_angle()          2 blanks   (0a - 0b)
  TASK 1   compute_errors()           5 blanks   (1a - 1e)
  TASK 2   control_on_off()           6 blanks   (2a - 2f)
  TASK 3   control_proportional()     4 blanks   (3a - 3d)
  TASK 4   control_kinematic()        4 blanks   (4a - 4d)

Do them in order - later tasks use earlier ones. Check your work at any
point, without the simulator, with:

    python3 check_my_math.py

HOW TO FILL A BLANK
-------------------
Each blank looks like this:

    rho = 0.0          # BLANK 1a  <-- replace the 0.0

Replace only the value. Leave the variable name and the comment. So

    rho = 0.0          # BLANK 1a
becomes
    rho = math.hypot(dx, dy)          # BLANK 1a

Everything outside the BLANK lines is complete and correct. If you find
yourself wanting to add an if statement, re-read the structure - it is
already there.
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
#  TASK 0 - Angle wrapping                                   2 blanks (0a-0b)
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

    THE METHOD
    ----------
    We want to map any real angle to the equivalent angle in [-pi, pi]. The
    trick is that sin and cos are periodic, so they *discard* whole
    revolutions for us - sin(phi) and cos(phi) are unchanged by adding
    2*pi*k to phi. Then atan2 rebuilds an angle from those two components,
    and atan2 always returns a value in [-pi, pi]. So:

        wrap(phi) = atan2( sin(phi), cos(phi) )

    Careful: atan2 takes the VERTICAL component first. atan2(y, x), not
    atan2(x, y). The sine is the vertical one.

    WORKED SANITY CHECK
    -------------------
    wrap(4*pi)     = 0        two full revolutions is no rotation at all
    wrap(3*pi)     = +pi      3*pi = 2*pi + pi, so one full turn is discarded
                              and a HALF turn remains. Not zero.
    wrap(-3*pi/2)  = +pi/2    270 deg one way is 90 deg the other way
    wrap(0.7)      = 0.7      small angles pass through untouched

    FILL-IN TABLE
    -------------
      BLANK 0a   sin_part   the sine of `angle`                 use math.sin
      BLANK 0b   cos_part   the cosine of `angle`               use math.cos

    (The atan2 call is already written for you on the return line, so you
    cannot get the argument order wrong. Note which of the two variables it
    passes first, and why.)
    """
    sin_part = 0.0          # BLANK 0a
    cos_part = 1.0          # BLANK 0b

    return math.atan2(sin_part, cos_part)


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

        You are filling in the expressions inside steps 2 and 3. Steps 1 and 4
        are done.
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
    #  TASK 1 - The error terms                        5 blanks (1a-1e)
    # =================================================================
    @staticmethod
    def compute_errors(x, y, theta, wx, wy, wtheta, match_orientation):
        r"""Convert an absolute pose and an absolute goal into four error terms.

        This is the conceptual heart of the whole workshop. All three
        controllers consume these four numbers and nothing else. Get this
        right and the controllers are a handful of lines each; get it wrong
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
        The Cartesian offset to the goal is computed for you below:

            dx = wx - x        dy = wy - y

        1. rho - distance remaining, always >= 0:

               \rho = \sqrt{dx^2 + dy^2}

           Drives the *linear* velocity. rho = 0 means arrived.

        2. alpha - bearing of the goal in the robot's own frame. Build it in
           two steps. First the absolute compass bearing from robot to goal:

               \text{bearing} = \mathrm{atan2}(dy, dx)

           then rotate it into the robot's frame by subtracting the robot's
           own heading, and wrap:

               \alpha = \mathrm{wrap}(\text{bearing} - \theta)

           So:
               alpha =  0      goal is dead ahead
               alpha = +pi/2   goal is directly to the LEFT
               alpha = -pi/2   goal is directly to the RIGHT
               alpha = +/-pi   goal is directly BEHIND
           Drives the *angular* velocity. Note alpha says nothing about which
           way the robot will be facing when it arrives.

        3. theta_err - mismatch between current heading and the goal's
           required final orientation:

               \theta_e = \mathrm{wrap}(w\theta - \theta)

           Only graded in `dor` mode, and only useful once rho is small. This
           is what you null while spinning on the spot at the end.

        4. beta - mismatch between the goal orientation and the direction from
           which you are approaching:

               \beta = \mathrm{wrap}(\theta_e - \alpha)

           Read it this way: alpha tells you where the goal *is*, beta tells
           you how *wrongly angled* your approach is. If beta = 0 you are
           travelling straight down the goal's own heading and will arrive
           already correctly oriented. If beta is large you are approaching
           side-on and will need to curve. Only TASK 4 uses beta.

        THE TWO RULES
        -------------
          RULE A: rho is a length. Never wrap it. Never let it go negative.
          RULE B: every angular term passes through normalize_angle().
                  EVERY one - including beta, even though theta_err and alpha
                  were each already wrapped. Wrapping does not survive
                  subtraction: two wrapped angles can differ by more than pi.

        Nearly every bug in this workshop breaks one of those two rules.

        FILL-IN TABLE
        -------------
          BLANK 1a   rho         distance from (x,y) to (wx,wy)
                                 available: dx, dy      use: math.hypot
                                 do NOT wrap this one (RULE A)

          BLANK 1b   bearing     absolute compass bearing robot -> goal
                                 available: dx, dy      use: math.atan2
                                 remember atan2 takes the vertical first

          BLANK 1c   alpha       bearing rotated into the robot frame
                                 available: bearing, theta
                                 = wrap(bearing - theta)
                                 use: normalize_angle

          BLANK 1d   theta_err   goal heading minus robot heading, wrapped
                                 available: wtheta, theta
                                 use: normalize_angle

          BLANK 1e   beta        approach-angle error, wrapped
                                 available: theta_err, alpha
                                 = wrap(theta_err - alpha)
                                 use: normalize_angle

        The `if match_orientation` branch is already written. The `else` side
        is already correct: in `dis` mode the goal orientation is not graded,
        so beta must be exactly 0.0 and carry no information. This is why the
        kinematic controller degenerates gracefully into a P controller in
        `dis` mode.

        Returns:
            (rho, alpha, beta, theta_err)
        """
        dx = wx - x
        dy = wy - y

        rho = 0.0                # BLANK 1a
        bearing = 0.0            # BLANK 1b
        alpha = 0.0              # BLANK 1c
        theta_err = 0.0          # BLANK 1d

        if match_orientation:
            beta = 0.0           # BLANK 1e
        else:
            beta = 0.0           # already correct - do not change

        return rho, alpha, beta, theta_err

    # =================================================================
    #  TASK 2 - On/Off controller                      6 blanks (2a-2f)
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

        The sgn term sets only the *direction* of the turn; the magnitude is
        always the same. That is what "on/off" means. In Python,
        `math.copysign(magnitude, error)` gives you
        magnitude * sgn(error) in a single call - use it rather than writing
        an if/else on the sign.

        THE FOUR CASES ARE ALREADY WRITTEN as branches below, in the same
        order as the equation above. You only supply the two numbers in each.

        FILL-IN TABLE
        -------------
          case 1: too far away AND badly aimed -> turn on the spot
            BLANK 2a   v   zero - no forward motion until aligned
            BLANK 2b   w   fixed magnitude self.w_on, signed by alpha
                           use: math.copysign(self.w_on, alpha)

          case 2: too far away but aimed well enough -> drive straight
            BLANK 2c   v   the fixed forward speed self.v_on
            BLANK 2d   w   zero - no steering correction while driving

          case 3: arrived, but heading wrong, and we are in 'dor' mode
            BLANK 2e   v   zero - spin on the spot, do not translate
            BLANK 2f   w   fixed magnitude self.w_on, signed by theta_err

          case 4: arrived and satisfied -> already written, returns 0, 0

        Note that case 3 is already guarded by `self.match_orientation`.
        Without that guard the robot would reach a waypoint in `dis` mode and
        sit there spinning forever, trying to null an orientation error the
        referee is not even grading.

        WHAT YOU SHOULD OBSERVE (predict before you run it)
        ---------------------------------------------------
        Because the command does not shrink as the error shrinks, the robot
        arrives at the tolerance boundary still travelling at full speed v_0.
        It overshoots, the error flips sign, and it corrects at full speed the
        other way. Expect a limit cycle - permanent oscillation. Shrinking
        dist_tol makes this WORSE, not better. Work out why before you test:
        at 20 Hz and v_0 = 0.15 m/s, how far does the robot move between two
        consecutive decisions? Compare that with dist_tol.
        """
        if rho > self.dist_tol:

            if abs(alpha) > self.onoff_align_tol:
                v = 0.0          # BLANK 2a
                w = 0.0          # BLANK 2b
                return v, w, 'turn'

            v = 0.0              # BLANK 2c
            w = 0.0              # BLANK 2d
            return v, w, 'drive'

        if self.match_orientation and abs(theta_err) > self.ang_tol:
            v = 0.0              # BLANK 2e
            w = 0.0              # BLANK 2f
            return v, w, 'spin'

        return 0.0, 0.0, 'stop'

    # =================================================================
    #  TASK 3 - Proportional controller                4 blanks (3a-3d)
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

        The command now shrinks as the error shrinks, which is exactly the
        cure for the TASK 2 limit cycle: v -> 0 as rho -> 0, so the robot
        eases into the waypoint instead of charging the boundary.

        WHY THE \(\cos\alpha\) FACTOR
        -----------------------------
        The bare law \(v = K_p \rho\) has a defect: it commands full forward
        speed even when the goal is off to the side, or behind. The robot then
        drives a long arc away from the waypoint before curving back.
        Multiplying by \(\cos\alpha\) projects the command onto the robot's
        heading, so speed falls to zero as the goal reaches 90 degrees off the
        bow and goes *negative* (reverse) when the goal is behind.

        There is a formal reason to like this factor. Take the candidate
        Lyapunov function \(V = \tfrac{1}{2}\rho^2 \ge 0\), a scalar "energy"
        that is zero only at the goal. For a differential-drive robot
        \(\dot\rho = -v\cos\alpha\), so substituting the control law:

        \[
            \dot V = \rho \dot\rho = -K_p^{lin} \rho^2 \cos^2\alpha \le 0 .
        \]

        \(V\) can never increase, for *any* value of alpha - the robot can
        never be driven further from the goal than it started. The bare law
        gives you no such guarantee. Reproduce this derivation in your report.

        FILL-IN TABLE
        -------------
          BLANK 3a   w    the angular command, used by every branch below
                          = self.kp_ang * alpha
                          Computed once before the branching, because the
                          same expression applies whether or not the robot
                          is also moving forward.

          BLANK 3b   v    zero. This is the "turn on the spot first" branch,
                          taken when |alpha| exceeds self.p_turn_first. The
                          cosine factor already makes large-alpha motion
                          harmless, but not efficient - so we suppress v.

          BLANK 3c   v    the full law: self.kp_lin * rho * cos(alpha)
                          use: math.cos

          BLANK 3d   w    the final spin-in-place command. Apply the angular
                          gain to theta_err, NOT to alpha:
                          = self.kp_ang * theta_err
                          Why not alpha? When rho is tiny, alpha is atan2 of
                          two nearly-zero numbers and is therefore pure
                          numerical noise. Never steer on it at close range.

        PREDICT BEFORE YOU RUN
        ----------------------
        1. Compared with TASK 2, what happens in the last 20 cm?
        2. You raise kp_ang from 1.5 to 5.0. What is the failure mode?
        3. In `dor` mode this controller reaches the right position but not
           generally the right heading. Which error term is missing from the
           law above? (That absence is the entire motivation for TASK 4.)
        """
        w = 0.0                  # BLANK 3a

        if rho > self.dist_tol:

            if abs(alpha) > self.p_turn_first:
                v = 0.0          # BLANK 3b
                return v, w, 'turn'

            v = 0.0              # BLANK 3c
            return v, w, 'drive'

        if self.match_orientation and abs(theta_err) > self.ang_tol:
            w = 0.0              # BLANK 3d
            return 0.0, w, 'spin'

        return 0.0, 0.0, 'stop'

    # =================================================================
    #  TASK 4 - Kinematic position controller          4 blanks (4a-4d)
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

        THE ERROR DYNAMICS
        ------------------
        In polar error coordinates the plant is

        \[
            \dot\rho = -v\cos\alpha, \qquad
            \dot\alpha = \frac{v \sin\alpha}{\rho} - \omega, \qquad
            \dot\beta = -\frac{v \sin\alpha}{\rho}.
        \]

        Look hard at the \(\dot\alpha\) equation: there is a \(\rho\) in a
        denominator. That is a SINGULARITY. As rho -> 0 the term blows up, and
        alpha itself becomes atan2 of two nearly-zero numbers - noise. This is
        why the structure below stops using alpha once rho is inside
        dist_tol, and switches to theta_err instead.

        STABILITY
        ---------
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

        Applying Routh-Hurwitz to that polynomial (the quadratic factor
        \(\lambda^2 + b\lambda + c\) needs \(b>0\) and \(c>0\); the linear
        factor needs \(K_\rho>0\)) yields three inequalities:

        \[
            K_\rho > 0, \qquad K_\beta < 0, \qquad K_\alpha - K_\rho > 0 .
        \]

        Break the third and the robot spirals instead of converging. Break the
        second and it arrives in the right place persistently facing the wrong
        way. These are not rules of thumb - they fall straight out of the
        polynomial, and you should reproduce the derivation in your report
        rather than quoting the result.

        Suggested starting point: \(K_\rho = 0.5\), \(K_\alpha = 1.5\),
        \(K_\beta = -0.6\). Check it satisfies all three before running.

        FILL-IN TABLE
        -------------
          BLANK 4a   w    the two-term angular law, used by both of the
                          not-yet-arrived branches:
                          = self.k_alpha * alpha + self.k_beta * beta

          BLANK 4b   v    zero. This branch is taken when |alpha| > pi/2,
                          i.e. the waypoint is BEHIND the robot. There
                          v = k_rho * rho would still be positive, so the
                          robot would drive forward while turning hard and
                          sweep a wide arc away from the goal. Suppress v and
                          let omega swing the nose round first.

          BLANK 4c   v    the linear law: self.k_rho * rho
                          Note: no cosine factor here, unlike TASK 3. Applying
                          one instead of the |alpha| > pi/2 branch is a
                          defensible alternative design - implementing both and
                          comparing them is good report material.

          BLANK 4d   w    the final spin-in-place command, driven by
                          theta_err and not by alpha (see the singularity
                          note above):
                          = self.k_alpha * theta_err

        A NOTE ON TESTING THIS ONE
        --------------------------
        In `dis` mode beta is 0.0, so omega collapses to k_alpha * alpha and
        this controller behaves just like TASK 3. If your `dis` runs look
        identical to your TASK 3 runs, that is EXPECTED AND CORRECT - it is
        evidence your TASK 1 is right, not a bug. Test this controller in
        `dor` mode, where beta actually carries information.
        """
        w = 0.0                  # BLANK 4a

        if rho > self.dist_tol:

            if abs(alpha) > math.pi / 2.0:
                v = 0.0          # BLANK 4b
                return v, w, 'turn'

            v = 0.0              # BLANK 4c
            return v, w, 'drive'

        if self.match_orientation and abs(theta_err) > self.ang_tol:
            w = 0.0              # BLANK 4d
            return 0.0, w, 'spin'

        return 0.0, 0.0, 'stop'

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
