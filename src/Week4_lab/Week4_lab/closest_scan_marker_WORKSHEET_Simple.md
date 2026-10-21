# Worksheet — Closest laser point as a marker in `odom`

Guidance to `closest_scan_marker_CLOZE.py`. Read sections 1–3, fill the blanks, then use section 5 only when you are stuck, and section 7 only after a real attempt.

---

## 1. What you are building

The pipeline has four stages, and each blank in the cloze file belongs to exactly one of them.

```
 /scan                                                         /closest_obstacle_marker
   │                                                                     ▲
   │  ranges[] : 360 distances                                           │
   ▼                                                                     │
 [A] find the nearest trustworthy beam   →  (r*, i*)                     │
 [B] work out its angle from i*          →  (r*, theta*)  polar          │
 [C] polar → x, y                        →  (x, y, 0)                    │
 [D] move the point into odom via TF     →  (X, Y, Z)   ─────────────────┘
```

Stage D is already written for you. Stages A, B and C are the blanks.

Why stage D is needed: the laser measures things in its own frame of reference, which turns and moves with the robot. A wall that is "0.3 m to my right" is at a different place in the world every time the robot turns. The marker is published in `odom`, a frame fixed to the world, so a stationary wall must produce a stationary marker while the robot drives past it. If the marker stays glued to the wall as you drive around it, your maths is right.

---

## 2. The maths, step by step

### 2.1 The angles are not stored — you can work them out

A `LaserScan` message gives you all 360 distances in `ranges[]`, but it does **not** store 360 matching angles. That would be wasteful, because the beams are evenly spaced. You only need two numbers:

- `msg.angle_min` = the angle of beam 0, the first beam in the fan. On the LIMO this is `-2.0` radians.
- `msg.angle_increment` = the angle between one beam and the next. On the LIMO this is `0.011142061091959476` radians.

A radian is just another way of measuring an angle, like degrees. There are about `57.3` degrees in 1 radian, so `2.0` rad is the same as `114.59°`. ROS always works in radians, so keep your calculator in radian mode.

The angle of beam number `i` is:

```
angle_i = angle_min + i * angle_increment
```

So beam 0 is at `angle_min`, beam 1 is at `angle_min + 1 * angle_increment`, beam 2 is at `angle_min + 2 * angle_increment`, and so on.

With 360 beams they are numbered `0, 1, 2, ..., 359`. The last beam is `359`, **not** `360`.

Sanity check: beam 359 is at `-2.0 + 359 * 0.011142061091959476 = -2.0 + 4.0 = 2.0` rad = `+114.59°`, which matches `msg.angle_max`. So the fan of beams runs from `-114.59°` to `+114.59°`, about `229°` in total — the sensor sees a bit more than a semicircle in front of the robot.

Angles have a sign convention worth remembering:

- `0` points straight ahead,
- **negative** angles point to the robot's **right**,
- **positive** angles point to the robot's **left**.

> **Note on the workshop handout.** Task 4 of the handout prints `msg.ranges[360]` and calls it the leftmost beam. With 360 beams there is no index 360 — that is one past the end of the array and raises `IndexError`; the leftmost beam is `ranges[359]`. The handout also calls `ranges[180]` "90 degrees", but beam 180 is at `-2.0 + 180 * 0.011142061 = 0.0056` rad ≈ `0.3°`, i.e. almost straight ahead. Work the angles out with the formula instead of trusting the printed indices.

### 2.2 Which readings can you trust?

Some entries in `ranges[]` are not real distances. A reading is usable only if it lies between two limits that the message itself tells you:

- `msg.range_min` = `0.12` m — anything closer than this cannot be measured reliably.
- `msg.range_max` = `8.0` m — this is as far as the sensor can see.

So a reading `r` is usable only when `msg.range_min <= r <= msg.range_max`.

Three kinds of entry fail this test:

| Entry | What it means | Why you must skip it |
|---|---|---|
| `nan` | the beam got no echo at all | comparing anything with `nan` is `False`, so `nan` silently breaks a "find the smallest" search |
| `inf` | nothing was found within 8 m | `max()` would be stuck at `inf` forever |
| a number below `0.12` or above `8.0` | outside the sensor's trusted range | the number exists but is not reliable |

The `0.12` m limit is also the answer to the earlier question about detecting contact: an object closer than `0.12` m is inside the sensor's blind zone and cannot be seen at all.

### 2.3 Find the nearest obstacle (stage A)

You want the beam with the smallest distance, because that is the nearest obstacle. You are finding the minimum by hand: walk through the list once and keep the best one found so far.

The rule to keep true on every pass of the loop is:

> `closest_range` is the smallest **usable** distance among beams `0 ... i-1`, and `closest_index` is where it was found.

Before the first beam is looked at, you have looked at nothing, so the starting value must be a number that any real distance will beat. Real distances are positive and finite, so start `closest_range` at `inf` (infinity). Infinity is the ideal starting value for a minimum search, because `min(inf, r) = r` for any real `r`.

Do **not** start at `0`: then the test `r < 0` is never true, the loop never updates anything, and the marker sits at the origin forever. That failure is silent, which makes it worse than a crash.

### 2.4 Turn a distance + angle into x, y (stage C)

ROS uses the standard robot coordinate convention:

- `+x` is **forward**,
- `+y` is to the **left**,
- `+z` is **up**,
- angles are measured anticlockwise, starting from `+x`.

If a beam is at distance `r` and angle `theta`, the obstacle is at:

```
x = r * cos(theta)
y = r * sin(theta)
z = 0.0
```

Why `z = 0.0`? A 2-D LiDAR only sees its own flat horizontal plane. It cannot measure height, so the only honest value is `0`.

Check the formula against the convention:

- `theta = 0` → `(x, y) = (r, 0)` → straight ahead. Correct.
- `theta = +pi/2` (90°, the left) → `(x, y) = (0, r)` → to the left. Correct.
- `theta = -pi/2` (−90°, the right) → `(x, y) = (0, -r)` → to the right. Correct.

If you swap `cos` and `sin`, the marker appears rotated 90° away from the real obstacle — easy to spot in RViz.

### 2.5 Move the point into the world frame (stage D, provided)

This stage is already written for you, so you only need to understand roughly what it does, not reproduce it.

The laser measures `(x, y, 0)` in its own frame of reference, attached to the robot. To put that point into the world frame `odom`, the code looks up the robot's position and heading from TF, then rotates and shifts the point to match. You can picture it as:

```
point_in_odom = rotate(point_in_laser, robot_heading) + robot_position
```

The rotation is delivered as a quaternion — a four-number way of describing a 3-D rotation — rather than the three angles you might expect. You do not need to do anything with it; the provided `do_transform_point` handles it.

One thing to get right if you ever call `lookup_transform` yourself: the order of the two frame names matters. Asking for the transform from `laser_link` **to** `odom` gives the version that maps a laser point into the world. Swap the two names and you get the reverse, which sends your marker flying off in the wrong direction as you drive. That is a good deliberate experiment once your code works.

---

## 3. Worked numerical example

Do this by hand with a calculator before running anything. If your code disagrees with these numbers, the bug is in your code.

**Given:** beam `i = 40`, distance `r = 0.300` m, `angle_min = -2.0` rad, `angle_increment = 0.011142061` rad.

**Stage B — the angle:**

```
theta = -2.0 + 40 * 0.011142061 = -2.0 + 0.445682 = -1.554318 rad = -89.06°
```

Negative, so the obstacle is on the robot's right — almost exactly sideways.

**Stage C — x and y in the laser frame:**

```
x = 0.300 * cos(-1.554318) = 0.300 * 0.016475 = +0.00494 m
y = 0.300 * sin(-1.554318) = 0.300 * (-0.999864) = -0.29996 m
z = 0
```

Almost no forward movement and nearly the full 0.3 m to the right — exactly what −89° should give.

**Stage D — into `odom`.** Suppose the robot is at position `(1.0, 2.0, 0.15)` and facing north (heading 90°). A 90° anticlockwise turn maps `(x, y)` to `(-y, x)`, so the point `(0.00494, -0.29996, 0)` becomes `(0.29996, 0.00494, 0)`. Adding the robot's position gives `(1.29996, 2.00494, 0.15)` — the obstacle due east of the robot, which is to its right when facing north. Consistent.

Always ask whether the number you computed could be true before trusting it.

---

## 4. The blanks at a glance

| Blank | Quantity | Units | Section |
|---|---|---|---|
| 1a | initial `closest_range` | m | 2.3 |
| 1b | initial `closest_index` | — | 2.3 |
| 2 | reject `nan` / `inf` | — | 2.2 |
| 3 | reject readings outside the trusted range | m | 2.2 |
| 4 | "is this a new minimum?" | — | 2.3 |
| 5a, 5b | store the new best distance and index | m, — | 2.3 |
| 6 | "no usable reading found" | — | 2.3 |
| 7 | the angle of the closest beam | rad | 2.1 |
| 8a, 8b | `x`, `y` | m | 2.4 |
| 9 | `z` | m | 2.4 |

---

## 5. Tiered hints

Use hint 1 first. Only move to hint 2 if you are still stuck after trying something.

### Blank 1a — initial `closest_range`
1. What single value is guaranteed to lose a `<` comparison against every real distance?
2. Ranges are finite. Python can represent a non-finite number.
3. Look up `float('inf')` and `math.inf`.

### Blank 1b — initial `closest_index`
1. You need a value that could never be a real index into `ranges[]`.
2. Valid indices run `0 ... 359`, so any negative number is impossible.
3. Use `-1`, and remember that blank 6 must test for exactly this value.

### Blank 2 — reject `nan` / `inf`
1. The `math` module has two predicates for precisely these two cases.
2. You need both, combined so that *either* one triggers the `continue`.
3. `math.isnan(r)` and `math.isinf(r)`, joined with `or`.

### Blank 3 — reject out-of-bounds
1. The valid range is not hard-coded; the message carries it. Which two fields?
2. You want the *opposite* of `msg.range_min <= r <= msg.range_max`.
3. Two comparisons against `msg.range_min` and `msg.range_max`, joined with `or`. Python also allows the chained form `not (a <= r <= b)`.

### Blank 4 — new minimum?
1. Restate the rule in section 2.3 in Python.
2. Compare the candidate against the best so far.
3. Strict `<`, so ties keep the first (lower-index) beam. Using `<=` would keep the last instead — decide which you want and be able to justify it.

### Blanks 5a / 5b — store the new best
1. Two assignments, not one.
2. What does blank 7 need that `closest_range` alone cannot supply?
3. The loop variables are already named `r` and `i`.

### Blank 6 — nothing found
1. After the loop, what do the variables hold if there are no usable readings at all?
2. They still hold your blank-1 starting values.
3. Test the index against `-1`. Testing `closest_range == float('inf')` also works; testing the index is clearer about intent.

### Blank 7 — the bearing
1. Write out the angle of beam 0, then beam 1, then beam 2. What is the pattern?
2. It is an arithmetic sequence: a starting angle plus a whole number of steps.
3. The formula in section 2.1, with `i` set to `closest_index`. Both quantities are fields of `msg`.

### Blanks 8a / 8b — x and y
1. This is the standard polar-to-x,y conversion. Which function gives the component along the axis the angle is measured *from*?
2. At `theta = 0` the point must come out as `(r, 0)`. Which assignment satisfies that?
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
| Sign of bearing | place an obstacle on the robot's left only | logged angle is **positive** |
| Axis convention | place an obstacle directly ahead | `y` ≈ 0 and `x` ≈ the range |
| Frame correctness | leave the obstacle still; teleop the robot around it | marker stays **glued to the obstacle**, does not move with the robot |
| Magnitude | measure the obstacle distance in the simulator | logged range matches to a few cm |
| Blind zone | drive until you touch the obstacle | below about `0.12` m the reading is rejected and the warning fires |
| Empty scan | face the robot at open space beyond 8 m | warning from blank 6, no marker published |

The third test is the decisive one. A marker that drifts along with the robot means your point never really left the laser frame.

### Common failure modes

| Symptom | Likely cause |
|---|---|
| Marker pinned at the origin, never moves | `closest_range` initialised to `0`, so blank 4 is never true |
| `closest_range` is `nan` | blank 2 missing, or using `and` instead of `or` |
| Marker 90° away from the obstacle | `sin`/`cos` swapped in blank 8 |
| Marker mirrored left/right | sign error, or you converted the angle to degrees before the trig call |
| Marker always 8.0 m away | you searched for the maximum, or forgot the `inf` rejection |
| `IndexError` | you used an invalid index, e.g. `ranges[360]` |
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

1. **Restrict the search to a front sector.** For safe stopping, an obstacle at −110° is behind the wheels and irrelevant. To turn an angle limit into an index limit, rearrange the bearing formula: `i = (theta - angle_min) / angle_increment`, then round down to a whole number. For a ±30° front window, `theta = ±0.5236` rad gives `i` from 132 to 226. Check that arithmetic yourself.
2. **Three sectors.** Split the usable beams into front-left, front and front-right index bands and find the minimum distance in each band. Compare the left and right minima to choose a turn direction — this is the core of task 4.
3. **Use the minimum, not the mean.** Averaging a sector hides a thin obstacle such as a chair leg among many long readings. Ask yourself which statistic is safe for collision avoidance and why.
4. **Latency.** The node currently asks TF for the newest available transform rather than the transform at `msg.header.stamp`. At what robot speed does that approximation start to matter, and what would you pass instead?
