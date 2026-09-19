# Worksheet — Closest laser point as a marker in `odom`

Guidance to `closest_scan_marker_CLOZE.py`. Read sections 1–3, fill the blanks, then use section 5 only when you are stuck and section 7 only after a real attempt.

---

## 1. What you are building

The pipeline has four stages, and each blank in the cloze file belongs to exactly one of them.

```
 /scan                                                           /closest_obstacle_marker
   │                                                                        ▲
   │  ranges[] : 360 distances                                              │
   ▼                                                                        │
 [A] pick the nearest trustworthy beam   →  (r*, i*)                        │
 [B] recover its bearing from i*         →  (r*, θ*)   polar, laser frame   │
 [C] polar → Cartesian                   →  p^L = (x,y,0)                   │
 [D] rigid-body transform via TF          →  p^O = T^O_L · p^L  ────────────┘
```

Stage D is already written for you. Stages A, B and C are the blanks.

The reason stage D cannot be skipped: the laser measures in its own frame, which moves and rotates with the robot. A point that is "0.3 m to my right" is a *different* place in the world every time the robot moves. The marker is published in `odom`, a frame fixed to the world, so a stationary wall must produce a stationary marker even while the robot drives past it. That is your visual test that the maths is right.

---

## 2. The mathematics

### 2.1 The compressed polar encoding

A `LaserScan` message stores $N$ distances in `ranges[]` but **not** the $N$ matching angles. Storing them would be redundant, because the beams are evenly spaced in angle. Only two numbers are needed:

- `angle_min` $= \theta_{\min}$, the bearing of beam $0$,
- `angle_increment` $= \Delta\theta$, the constant angular step between consecutive beams.

The bearings therefore form an arithmetic sequence:

$$\theta_i = \theta_{\min} + i\,\Delta\theta, \qquad i = 0, 1, 2, \dots, N-1$$

Sanity check this on your own robot. The workshop trace gives $\theta_{\min} = -2.0$ rad, $\Delta\theta = 0.011142061091959476$ rad and $N = 360$. The last beam is beam $N-1 = 359$, not beam $360$:

$$\theta_{359} = -2.0 + 359 \times 0.011142061091959476 = -2.0 + 4.0 = 2.0 \text{ rad} = +114.59^\circ$$

which matches the advertised `angle_max`. So the full fan spans $[-114.59^\circ, +114.59^\circ]$, about $229^\circ$.

> **Note on the workshop handout.** Task 4 of the handout prints `msg.ranges[360]` and calls it the leftmost beam. With $N = 360$ that is one past the end of the array and will raise `IndexError`; the leftmost beam is `ranges[359]`. The handout also labels `ranges[180]` as "90 degree", but $\theta_{180} = -2.0 + 180(0.011142) = 0.0056$ rad $\approx 0.3^\circ$, i.e. the beam pointing essentially straight ahead. Recompute these yourself with the formula rather than trusting the printed indices — that recomputation *is* the learning objective.

### 2.2 Which readings are usable

Define the set of valid beam indices

$$\mathcal{V} = \left\{\, i \;\middle|\; r_i \in \mathbb{R},\ \ r_{\min} \le r_i \le r_{\max} \,\right\}$$

where $r_{\min} =$ `range_min` $= 0.12$ m and $r_{\max} =$ `range_max` $= 8.0$ m for the LIMO.

Three kinds of entry fail this test:

| Entry | Meaning | Why it must go |
|---|---|---|
| `nan` | the beam produced no interpretable echo | every comparison with NaN is `False`, so NaN silently poisons a naive minimum search |
| `inf` | nothing found within $r_{\max}$ | `max()` would return $\infty$ forever |
| $r < r_{\min}$ or $r > r_{\max}$ | outside the calibrated interval | the number exists but is not trustworthy |

The $r_{\min}$ bound is also the answer to the earlier workshop question about contact detection: an object nearer than $0.12$ m falls outside $\mathcal{V}$ and is invisible to the sensor. There is a blind shell around the LiDAR.

### 2.3 The nearest obstacle

$$i^{\star} = \operatorname*{arg\,min}_{i \in \mathcal{V}} r_i, \qquad r^{\star} = r_{i^{\star}}, \qquad \theta^{\star} = \theta_{\min} + i^{\star}\,\Delta\theta$$

You are implementing $\arg\min$ by hand as a single pass over the array, keeping a running best. The invariant to hold at the top of each iteration is:

> `closest_range` is the smallest valid range among beams $0 \dots i-1$, and `closest_index` is where it occurred.

To make the invariant true *before* the first iteration — when you have examined nothing — the initial value must lose to every possible real measurement. Since ranges are finite and positive, $+\infty$ is the natural identity element for a minimum:

$$\min(\infty, r) = r \quad \text{for all finite } r$$

This is why you must not initialise to $0$. With `closest_range = 0` the test $r < 0$ is never true, the loop never updates anything, and your marker sits permanently at the origin. That failure mode is quiet, which makes it worse than a crash.

### 2.4 Polar to Cartesian, in the laser frame

ROS uses a right-handed frame convention (REP 103): $+x$ forward, $+y$ to the **left**, $+z$ up, and angles measured anticlockwise from $+x$. So

$$\mathbf{p}^{L} = \begin{bmatrix} x^{L} \\ y^{L} \\ z^{L} \end{bmatrix} = \begin{bmatrix} r^{\star}\cos\theta^{\star} \\ r^{\star}\sin\theta^{\star} \\ 0 \end{bmatrix}$$

The superscript $L$ reads "expressed in the laser frame". $z^{L} = 0$ because a 2-D LiDAR sees only its own horizontal plane; it cannot report height, so claiming any other value would be inventing data.

Check the signs against the convention. A beam at $\theta^{\star} = 0$ gives $(r, 0)$ — straight ahead, correct. A beam at $\theta^{\star} = +\pi/2$ gives $(0, +r)$ — to the left, correct for a positive (anticlockwise) angle. A beam at $\theta^{\star} = -\pi/2$ gives $(0, -r)$ — to the right. If you swap $\sin$ and $\cos$ your marker will appear rotated $90^\circ$ from the obstacle, which is easy to spot in RViz.

### 2.5 The rigid-body transform (stage D, provided)

TF returns a transform $T^{O}_{L} \in SE(3)$, a rotation $R^{O}_{L} \in SO(3)$ plus a translation $\mathbf{t}^{O}_{L} \in \mathbb{R}^3$. In homogeneous coordinates

$$\begin{bmatrix} \mathbf{p}^{O} \\ 1 \end{bmatrix} = \underbrace{\begin{bmatrix} R^{O}_{L} & \mathbf{t}^{O}_{L} \\ \mathbf{0}^{\top} & 1 \end{bmatrix}}_{T^{O}_{L}} \begin{bmatrix} \mathbf{p}^{L} \\ 1 \end{bmatrix} \qquad \Longleftrightarrow \qquad \mathbf{p}^{O} = R^{O}_{L}\,\mathbf{p}^{L} + \mathbf{t}^{O}_{L}$$

`do_transform_point` performs exactly this line. The rotation is delivered as a quaternion $\mathbf{q} = (q_x, q_y, q_z, q_w)$ rather than a matrix, but the operation is the same.

Note the index convention: $T^{O}_{L}$ maps *from* $L$ *to* $O$, and matches the call `lookup_transform(target='odom', source='laser_link', ...)`. Swapping those two arguments gives you $T^{L}_{O} = (T^{O}_{L})^{-1}$ and a marker that flies off in the wrong direction as you drive — a good deliberate experiment once your code works.

---

## 3. Worked numerical example

Do this by hand with a calculator before you run anything. If your code disagrees with these numbers, the fault is in your code.

**Given:** $i^{\star} = 40$, $r^{\star} = 0.300$ m, $\theta_{\min} = -2.0$ rad, $\Delta\theta = 0.011142061$ rad.

**Stage B — bearing:**

$$\theta^{\star} = -2.0 + 40 \times 0.011142061 = -2.0 + 0.445682 = -1.554318 \text{ rad} = -89.06^{\circ}$$

Negative, so the obstacle is on the robot's right — almost exactly abeam.

**Stage C — Cartesian in `laser_link`:**

$$x^{L} = 0.300\cos(-1.554318) = 0.300 \times 0.016475 = +0.00494 \text{ m}$$
$$y^{L} = 0.300\sin(-1.554318) = 0.300 \times (-0.999864) = -0.29996 \text{ m}$$
$$z^{L} = 0$$

Almost no forward component and nearly the full $0.3$ m to the right, exactly as $-89^{\circ}$ demands.

**Stage D — into `odom`.** Suppose TF reports the laser at $\mathbf{t}^{O}_{L} = (1.0,\ 2.0,\ 0.15)^{\top}$ with yaw $\psi = 90^{\circ}$, so

$$R^{O}_{L} = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1\end{bmatrix} = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

Then

$$\mathbf{p}^{O} = R^{O}_{L}\begin{bmatrix} 0.00494 \\ -0.29996 \\ 0 \end{bmatrix} + \begin{bmatrix} 1.0 \\ 2.0 \\ 0.15 \end{bmatrix} = \begin{bmatrix} 0.29996 \\ 0.00494 \\ 0 \end{bmatrix} + \begin{bmatrix} 1.0 \\ 2.0 \\ 0.15 \end{bmatrix} = \begin{bmatrix} 1.29996 \\ 2.00494 \\ 0.15 \end{bmatrix}$$

Interpret it: the robot sits at $(1, 2)$ facing north (yaw $90^{\circ}$), the obstacle is on its right, and the answer places the obstacle due east of the robot. Consistent. This interpretation step is worth more than the arithmetic — always ask whether the number you computed could be true.

---

## 4. The blanks at a glance

| Blank | Quantity | Units | Section |
|---|---|---|---|
| 1a | initial `closest_range` | m | 2.3 |
| 1b | initial `closest_index` | — | 2.3 |
| 2 | reject NaN / infinite | — | 2.2 |
| 3 | reject outside $[r_{\min}, r_{\max}]$ | m | 2.2 |
| 4 | "is this a new minimum?" | — | 2.3 |
| 5a, 5b | store new best range and index | m, — | 2.3 |
| 6 | "no valid reading found" | — | 2.3 |
| 7 | $\theta^{\star}$ | rad | 2.1 |
| 8a, 8b | $x^{L}$, $y^{L}$ | m | 2.4 |
| 9 | $z^{L}$ | m | 2.4 |

---

## 5. Tiered hints

Use hint 1 first. Only move to hint 2 if you are still stuck after trying something.

### Blank 1a — initial `closest_range`
1. What single value is guaranteed to lose a `<` comparison against every real distance?
2. Ranges are finite. Python can represent a non-finite number.
3. Look up `float('inf')` and `math.inf`.

### Blank 1b — initial `closest_index`
1. You need a value that could never be a legitimate index into `ranges[]`.
2. Valid indices run $0 \dots N-1$, so any negative number is impossible.
3. Use $-1$, and remember that blank 6 must test for exactly this value.

### Blank 2 — reject NaN / inf
1. The `math` module has two predicates for precisely these two cases.
2. You need both, combined so that *either* one triggers the `continue`.
3. `math.isnan(r)` and `math.isinf(r)`, joined with `or`.

### Blank 3 — reject out-of-bounds
1. The valid interval is not hard-coded; the message carries it. Which two fields?
2. You want the *complement* of $r_{\min} \le r \le r_{\max}$.
3. Two comparisons against `msg.range_min` and `msg.range_max`, joined with `or`. Python also allows the chained form `not (a <= r <= b)`.

### Blank 4 — new minimum?
1. Restate the invariant in section 2.3 in Python.
2. Compare the candidate against the best so far.
3. Strict `<`, so ties keep the first (lower-index) beam. Using `<=` would keep the last instead — decide which you want and be able to justify it.

### Blanks 5a / 5b — store the new best
1. Two assignments, not one.
2. What does blank 7 need that `closest_range` alone cannot supply?
3. The loop variables are already named `r` and `i`.

### Blank 6 — nothing found
1. After the loop, what do the variables hold if $\mathcal{V} = \varnothing$?
2. They still hold your blank-1 initial values.
3. Test the index against $-1$. Testing `closest_range == float('inf')` also works; testing the index is clearer about intent.

### Blank 7 — the bearing
1. Write out $\theta_0$, then $\theta_1$, then $\theta_2$. What is the pattern?
2. It is an arithmetic sequence: a starting angle plus a whole number of steps.
3. Equation in section 2.1, with $i = $ `closest_index`. Both quantities are fields of `msg`.

### Blanks 8a / 8b — Cartesian
1. This is the standard polar-to-Cartesian conversion. Which function gives the component along the axis the angle is measured *from*?
2. At $\theta = 0$ the point must come out as $(r, 0)$. Which assignment satisfies that?
3. Section 2.4. `math.cos` and `math.sin`, both taking radians — do not convert to degrees.

### Blank 9 — the z coordinate
1. What does a 2-D scanner know about height?
2. Nothing, so the only honest value is the one in its own plane.
3. Write `0.0`, not `0`. The ROS message field is a float64; an int can raise an `AssertionError` on assignment.

---

## 6. Check your work

Run the node with the simulation and RViz (fixed frame `odom`, Marker display on `/closest_obstacle_marker`), then test these in order.

| Test | What to do | Pass criterion |
|---|---|---|
| Sign of bearing | Place an obstacle on the robot's left only | Logged angle is **positive** |
| Axis convention | Place an obstacle directly ahead | $y^{L} \approx 0$, $x^{L} \approx r^{\star}$ |
| Frame correctness | Leave obstacle still; teleop the robot around it | Marker stays **glued to the obstacle**, does not move with the robot |
| Magnitude | Measure the obstacle distance in the simulator | Logged range matches to a few cm |
| Blind zone | Drive to touch the obstacle | Below $\approx 0.12$ m the reading is rejected and the warning fires |
| Empty scan | Face the robot at open space beyond 8 m | Warning from blank 6, no marker published |

The third test is the decisive one. A marker that drifts along with the robot means your point never really left the laser frame.

### Common failure modes

| Symptom | Likely cause |
|---|---|
| Marker pinned at the origin, never moves | `closest_range` initialised to `0`, so blank 4 is never true |
| `closest_range` is `nan` | blank 2 missing or using `and` instead of `or` |
| Marker $90^{\circ}$ away from the obstacle | `sin`/`cos` swapped in blank 8 |
| Marker mirrored left/right | sign error, or you converted the angle to degrees before the trig call |
| Marker always 8.0 m away | you searched for the maximum, or forgot the `inf` rejection |
| `IndexError` | you used $N$ somewhere a valid index was required, e.g. `ranges[360]` |
| Marker never appears at all | fixed frame not `odom`, or the topic name in RViz does not match the publisher |

---

## 7. Answer key

Only read this after a genuine attempt — the diagnostic value of section 6 depends on you having made your own mistakes first.

```python
        # BLANK 1
        closest_range = float('inf')
        closest_index = -1

        for i, r in enumerate(msg.ranges):

            # BLANK 2
            if math.isnan(r) or math.isinf(r):
                continue

            # BLANK 3
            if r < msg.range_min or r > msg.range_max:
                continue

            # BLANK 4 / 5
            if r < closest_range:
                closest_range = r
                closest_index = i

        # BLANK 6
        if closest_index < 0:
            self.get_logger().warning('No valid range readings in this scan.')
            return

        # BLANK 7
        closest_angle = msg.angle_min + closest_index * msg.angle_increment

        # BLANK 8 / 9
        x = closest_range * math.cos(closest_angle)
        y = closest_range * math.sin(closest_angle)
        z = 0.0
```

Remember to delete the `TODO()` helper and the two `___BLANK_1*___` lambda aliases from the top of the cloze file once every blank is filled.

---

## 8. Extensions towards tasks 3 and 4

These reuse the same machinery, so attempt them once the marker behaves.

1. **Restrict the search to a front sector.** For safe stopping, an obstacle at $-110^{\circ}$ is behind the wheels and irrelevant. Invert the bearing equation to convert an angle limit into an index limit: $i = \lfloor (\theta - \theta_{\min}) / \Delta\theta \rfloor$. For a $\pm 30^{\circ}$ front window, $\theta = \pm 0.5236$ rad gives $i \in [132, 226]$. Verify that arithmetic yourself.
2. **Three sectors.** Split $\mathcal{V}$ into front-left, front and front-right index bands and compute a minimum per band. Compare the left and right minima to choose a turn direction — this is the core of task 4.
3. **Use the minimum, not the mean.** Averaging a sector hides a thin obstacle such as a chair leg among many long readings. Ask yourself which statistic is safe for collision avoidance and why.
4. **Latency.** The node currently asks TF for the newest available transform rather than the transform at `msg.header.stamp`. At what robot speed does that approximation start to matter, and what would you pass instead?
