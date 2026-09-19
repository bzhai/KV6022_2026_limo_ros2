# Worksheet — Safe-stop behaviour (workshop task 3)

Guidance to `safe_stop_CLOZE.py`. Read sections 1–4, fill the ten blanks, then use the hints in section 6 and the key in section 8.

---

## 1. What you are building

A reactive controller with no memory of the world and no map. Every scan produces one command, and the loop is:

```
 /scan ──▶ [A] select front beams ──▶ [B] nearest distance d ──▶ [C] zone ──▶ [D] speed v ──▶ /cmd_vel
```

Each blank belongs to exactly one stage, and each stage is one method in the file:

| Stage | Method | Blanks | Question it answers |
|---|---|---|---|
| A | `front_sector_bounds` | 1, 2 | which beams are "in front"? |
| B | `min_front_range` | 3–7 | how far away is the nearest thing there? |
| C | `classify` | 8 | is that close, middling or far? |
| D | `speed_for` | 9 | how fast may I drive? |
| — | `on_scan` | 10 | how do I avoid twitching at the threshold? |

The important design idea is that C and D are separated. Stage C turns a continuous measurement into a discrete *situation*; stage D turns a situation into an *action*. Keeping them apart is what lets you extend the node for task 4 by changing only stage D.

---

## 2. Why a sector, not the whole scan

The LIMO's scan spans about $[-114.6^\circ, +114.6^\circ]$. If you minimise over all 360 beams you are braking for obstacles beside the wheels, and in any corridor the robot never moves at all.

Define a front sector of half-width $\alpha$ and use only the beams inside it:

$$\mathcal{F} = \{\, i \;|\; -\alpha \le \theta_i \le +\alpha \,\}$$

In task 2 you mapped index to bearing. Now you need the inverse. From

$$\theta_i = \theta_{\min} + i\,\Delta\theta$$

solve for $i$ and round to the nearest whole beam:

$$i(\theta) = \left\lfloor \frac{\theta - \theta_{\min}}{\Delta\theta} + \tfrac{1}{2} \right\rfloor$$

With $\alpha = 30^\circ = 0.5236$ rad, $\theta_{\min} = -2.0$ rad and $\Delta\theta = 0.011142061$ rad:

$$i_{lo} = \text{round}\!\left(\frac{-0.5236 + 2.0}{0.011142061}\right) = \text{round}(132.50) = 132$$
$$i_{hi} = \text{round}\!\left(\frac{+0.5236 + 2.0}{0.011142061}\right) = \text{round}(226.49) = 226$$

So the front sector is beams 132 to 226 inclusive — 95 of the 360 beams. Check that against your own scan before trusting it.

**Choosing $\alpha$.** Too narrow and the robot clips obstacles with its shoulders; too wide and it brakes for walls it is driving parallel to. A sensible lower bound comes from the robot's width $w$ and the stopping distance $d_s$: an obstacle at the edge of the footprint, $w/2$ to the side and $d_s$ ahead, lies at

$$\alpha_{\min} = \arctan\!\frac{w/2}{d_s}$$

For the LIMO, $w \approx 0.22$ m and $d_s = 0.30$ m give $\alpha_{\min} = \arctan(0.367) = 20.1^\circ$. The default $30^\circ$ leaves margin. Justify whatever you choose in these terms rather than by guessing.

---

## 3. The three special values in `ranges[]`

This is where most attempts go wrong, because the three cases need three *different* actions — not three rejections.

| Entry | What it means | Correct action |
|---|---|---|
| `nan` | beam failed, direction unknown | skip the beam, rely on its neighbours |
| `inf` (or finite $> r_{\max}$) | beam worked, found nothing within range | **treat as clear at $r_{\max}$** |
| $r < r_{\min}$ | inside the blind shell, untrustworthy | skip the beam |
| otherwise | a real measurement | use it |

The `inf` row is the one that catches people. In task 2 you rejected infinities because you were hunting for the *closest* point and an infinity could never be it. Here, rejecting infinities means that in an open room every beam gets skipped, the sector comes back empty, the fail-safe fires, and the robot refuses to move. Semantically `inf` is not missing data: it is a confident report that nothing is near. Substituting $r_{\max}$ records that correctly.

Then

$$d = \min_{i \in \mathcal{F} \cap \mathcal{V}} \tilde r_i, \qquad \tilde r_i = \begin{cases} r_{\max} & r_i = \infty \text{ or } r_i > r_{\max} \\ r_i & \text{otherwise} \end{cases}$$

**Minimum, never mean.** Averaging the sector hides a chair leg among 94 long readings and the robot drives into it. The minimum is the only statistic that is safe here, because the constraint you are enforcing is about the *worst* obstacle, not the typical one.

**The empty-sector case.** If $\mathcal{F} \cap \mathcal{V} = \varnothing$ — say the LiDAR has dropped out — you have no information. A safety check with no data must assume the worst, so return `None` and let the controller command zero. Returning a large distance instead would make a sensor failure look like an open corridor, and the robot would accelerate into whatever is actually there.

---

## 4. Zones and the speed law

### 4.1 Where to put $d_s$

$d_s$ is not arbitrary. The robot cannot stop instantly, so the distance you need is

$$d_s \;\ge\; \underbrace{v\,t_{\text{lat}}}_{\text{reaction}} \;+\; \underbrace{\frac{v^{2}}{2a}}_{\text{braking}} \;+\; \underbrace{d_{\text{bumper}}}_{\text{laser to front edge}} \;+\; \epsilon$$

With $v = 0.22$ m/s, a scan at 10 Hz so $t_{\text{lat}} \approx 0.15$ s, deceleration $a \approx 0.5$ m/s², a laser-to-bumper offset of about $0.12$ m and $\epsilon = 0.05$ m:

$$d_s \ge 0.22(0.15) + \frac{0.22^{2}}{2(0.5)} + 0.12 + 0.05 = 0.033 + 0.048 + 0.12 + 0.05 = 0.25 \text{ m}$$

So the handout's suggestion of 30 cm is comfortable and 50 cm is conservative. Note the $v^2$ term: doubling the cruise speed quadruples the braking component, so a fixed $d_s$ is only valid up to a particular speed. Recompute this for whatever `cruise_speed` you set — that recomputation is the real content of the task.

Remember also that $d$ is measured from `laser_link`, not from the front bumper. The $d_{\text{bumper}}$ term above is what accounts for that, and you can read the offset off `tf2_echo base_link laser_link`.

### 4.2 The piecewise speed law

$$v(d) = \begin{cases} 0 & d < d_s \\[4pt] v_{\max}\,\dfrac{d - d_s}{d_c - d_s} & d_s \le d < d_c \\[8pt] v_{\max} & d \ge d_c \end{cases}$$

Two properties are worth noticing, because they are the reason to prefer this over three fixed speeds.

It is **continuous**. At $d = d_s$ the middle branch gives $v_{\max} \cdot 0 = 0$, matching the first branch. At $d = d_c$ it gives $v_{\max} \cdot 1 = v_{\max}$, matching the third. No jolts at the boundaries.

The middle branch is a **proportional controller** on the error $e = d - d_s$, with gain $K = v_{\max}/(d_c - d_s)$. Written as $v = K e$, it is the simplest member of the PID family, and the robot decelerates smoothly rather than slamming to a halt.

The fraction $(d - d_s)/(d_c - d_s)$ is dimensionless — metres over metres. If your expression has stray units it is wrong, and checking dimensions is a fast way to catch an inverted fraction.

---

## 5. Worked numerical example

$d_s = 0.30$ m, $d_c = 0.60$ m, $v_{\max} = 0.22$ m/s. Verify these by hand before running.

| $d$ [m] | Zone | Fraction $(d-d_s)/(d_c-d_s)$ | $v$ [m/s] |
|---|---|---|---|
| 0.90 | FAR | — | 0.220 |
| 0.60 | FAR (boundary) | 1.00 | 0.220 |
| 0.45 | MIDDLE | 0.50 | 0.110 |
| 0.36 | MIDDLE | 0.20 | 0.044 |
| 0.30 | MIDDLE (boundary) | 0.00 | 0.000 |
| 0.25 | CLOSE | — | 0.000 |
| `None` | — | — | 0.000 |

The $d = 0.45$ row is the arithmetic to check: $(0.45 - 0.30)/(0.60 - 0.30) = 0.15/0.30 = 0.5$, so $v = 0.22 \times 0.5 = 0.11$ m/s.

---

## 6. Tiered hints

Try hint 1 first, and only escalate after attempting something.

### Blank 1 — index from angle
1. You already have the forward equation in section 2. Do the algebra to make $i$ the subject.
2. Both `angle_min` and `angle_increment` are fields of `msg`; the numerator is a difference of two angles.
3. `int(round((a_lo - msg.angle_min) / msg.angle_increment))`. The `int()` matters because `range()` rejects floats.

### Blank 2 — clamp an index
1. You want the value if it is inside $[0, n-1]$, and the nearer end point otherwise.
2. Two built-ins nested: one enforces the floor, the other the ceiling.
3. `max(0, min(n - 1, index))`. Write it out and check what it returns for $-5$ and for $n + 5$.

### Blank 3 — failed beam
1. One predicate from `math` tests for this value specifically.
2. Not `isinf` — that case is handled separately in blank 4, and for a different reason.
3. `math.isnan(r)`.

### Blank 4 — the clear direction
1. Two sub-blanks: the test, then the replacement value.
2. The test catches infinities *and* finite readings past the trustworthy limit. The replacement is a field of `msg`, not the literal 8.0.
3. Test: `math.isinf(r) or r > msg.range_max`. Replacement: `msg.range_max`.

### Blank 5 — blind zone
1. Which `msg` field is the near limit?
2. A reading below it is a number, but not one you can believe.
3. `r < msg.range_min`.

### Blank 6 — running minimum
1. Identical pattern to task 2, minus one of the two assignments.
2. Compare the candidate against the best so far; on success store only the distance.
3. `r < best`, then `best = r`. You do not need the index because stage C cares only about *how far*, not *which way* — that changes in task 4.

### Blank 7 — nothing usable
1. What would happen downstream if you returned `0.0` here? What about `msg.range_max`?
2. `on_scan` already has a branch for one particular return value. Look at it.
3. `None`. Returning `range_max` would make a dead LiDAR indistinguishable from an open corridor — the most dangerous possible failure mode.

### Blank 8 — the zone boundaries
1. Two tests against `self.d_s` and `self.d_c`, in an order that makes each distance match exactly one branch.
2. The first test runs before the second, so the second does not need to re-check the lower bound.
3. `d < self.d_s` for CLOSE, then `d < self.d_c` for MIDDLE. Be able to say what happens at exactly $d = d_s$ and why that is consistent with section 4.2.

### Blank 9 — the three speeds
1. Two of the three are a single literal or a single attribute.
2. For MIDDLE, build the dimensionless fraction from section 4.2 first, then multiply.
3. `0.0`; then `self.v_max * (d - self.d_s) / (self.d_c - self.d_s)`; then `self.v_max`. Sanity-check by substituting $d = d_s$ and $d = d_c$ into your middle expression.

### Blank 10 — hysteresis (stretch)
1. You need two thresholds, not one: a stop threshold and a higher release threshold.
2. The decision depends on which state you are currently in, which is what `self.stopped` records.
3. Roughly: if `self.stopped` is set, clear it only when `d > self.d_s + self.resume_margin`; otherwise set it when `d < self.d_s`. Then force `v = 0.0` whenever the flag is set. This is a Schmitt trigger, the same idea as a thermostat's deadband.

---

## 7. Check your work

Run the simulation, then the node. Keep teleop **closed** — two publishers on `/cmd_vel` fight each other and you will not be able to tell whose command you are watching.

| Test | What to do | Pass criterion |
|---|---|---|
| Open room | Start with nothing ahead | Robot cruises at `cruise_speed`, does not freeze |
| Smooth approach | Let it drive at a wall | Speed falls steadily through the MIDDLE band, no jolt |
| Stop distance | Measure the gap when it halts | Gap $\approx d_s$ measured from the laser, and the bumper is still clear |
| Side obstacle | Place a box at $\pm 90^\circ$ | Robot ignores it entirely |
| Parameter change | `-p safety_radius:=0.5` | Stops further out, no code edit needed |
| Threshold noise | Park with the obstacle at exactly $d_s$ | Without blank 10, visible twitching; with it, steady |
| Clean shutdown | Ctrl-C while moving | Robot stops instead of coasting on |

### Common failure modes

| Symptom | Likely cause |
|---|---|
| Never moves in an open room | blank 4 missing — infinities skipped rather than replaced |
| `TypeError` from `range()` | blank 1 returned a float; wrap in `int()` |
| `IndexError` on a wide sector | blank 2 not clamping |
| Stops for walls it is driving alongside | sector too wide, or minimising over all 360 beams |
| Drives into a chair leg | you averaged the sector instead of taking the minimum |
| Slams from cruise to zero | fixed slow speed in MIDDLE instead of the ramp of section 4.2 |
| Speed goes negative | inverted fraction in blank 9b — check it at $d = d_s$ |
| Keeps creeping after "stopping" | `v` computed but `publish` not reached, or teleop still running |
| Twitches at the threshold | blank 10 |

---

## 8. Answer key

Read only after a real attempt.

```python
    # ---- PART A ----------------------------------------------------------
        i_lo = int(round((a_lo - msg.angle_min) / msg.angle_increment))     # 1a
        i_hi = int(round((a_hi - msg.angle_min) / msg.angle_increment))     # 1b

        def clamp(index):
            return max(0, min(n - 1, index))                                # 2

    # ---- PART B ----------------------------------------------------------
            if math.isnan(r):                                               # 3
                continue

            if math.isinf(r) or r > msg.range_max:                          # 4a
                r = msg.range_max                                           # 4b
            elif r < msg.range_min:                                         # 5
                continue

            if r < best:                                                    # 6a
                best = r                                                    # 6b

        if math.isinf(best):
            return None                                                     # 7

    # ---- PART C ----------------------------------------------------------
        if d < self.d_s:                                                    # 8a
            return 'CLOSE'
        elif d < self.d_c:                                                  # 8b
            return 'MIDDLE'
        else:
            return 'FAR'

    # ---- PART D ----------------------------------------------------------
        if zone == 'CLOSE':
            return 0.0                                                      # 9a
        if zone == 'MIDDLE':
            return self.v_max * (d - self.d_s) / (self.d_c - self.d_s)      # 9b
        return self.v_max                                                   # 9c

    # ---- BLANK 10 (stretch), replacing the naive latch in on_scan --------
        if self.stopped:
            if d > self.d_s + self.resume_margin:
                self.stopped = False
        elif d < self.d_s:
            self.stopped = True

        if self.stopped:
            v = 0.0
```

Delete the `TODO` helper once every blank is filled.

---

## 9. Questions to answer in your report

1. You chose $d_s$. Reproduce the inequality in section 4.1 with *your* `cruise_speed` and show that your value satisfies it. At what speed would it stop being adequate?
2. $d$ is measured from `laser_link`. Use `tf2_echo base_link laser_link` to find the offset to the front of the robot, and state the actual bumper-to-obstacle gap when the node halts.
3. Why is the minimum over the sector the right statistic, and what specific obstacle would an average miss?
4. Explain why `inf` is substituted in stage B here but rejected in task 2. What changed about the question being asked?
5. The fail-safe returns `None` on an empty sector. Describe the accident that would follow from returning `range_max` instead.
6. Without hysteresis, roughly how much sensor noise is needed to cause chattering, and how does `resume_margin` suppress it?

---

## 10. Towards task 4

Stages A and B already generalise. To turn instead of stopping:

1. Call `min_front_range` three times with different sector bounds — front-left, front, front-right — by making the half-angle limits arguments rather than reading one parameter.
2. In the CLOSE zone, set `cmd.angular.z` towards whichever side has the larger minimum, instead of returning zero linear speed only.
3. Keep the CLOSE-zone linear speed at or near zero while turning on the spot, so the robot never advances into a space it has not cleared.
4. Consider what happens in a symmetric dead end where left and right minima are equal, and what you would add to break the tie so the robot does not sit oscillating.
