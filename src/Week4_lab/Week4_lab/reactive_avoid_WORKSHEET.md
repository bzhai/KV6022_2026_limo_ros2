# Worksheet — Reactive obstacle avoidance (workshop task 4)

Companion to `reactive_avoid_CLOZE.py`. Read sections 1–5, fill the ten blanks, then use the hints in section 7 and the key in section 9.

---

## 1. What changes from task 3

Task 3 had one sector and one output. Task 4 has three sectors and two outputs, and that is the whole difficulty: you now have to decide *where to go*, not just *whether to move*.

```
                    ┌─ front-right  d_r ─┐
 /scan ──▶ slice ───┼─ front        d_f ─┼──▶ decide ──▶ (v, ω) ──▶ /cmd_vel
                    └─ front-left   d_l ─┘
```

The controller is **purely reactive**: the command depends only on the newest scan, with no map and no memory of where it has been. The single exception is the latched turn direction in blank 10, and section 5 explains why even that small amount of memory is needed.

`sector_bounds` and `min_in_sector` are given complete — you wrote them in task 3 and nothing about them changes. Your effort goes on the decision layer.

---

## 2. The sector geometry

### 2.1 Fixing the handout's indices first

The handout's snippet has three problems, and working out why is part of the task.

$$\theta_i = \theta_{\min} + i\,\Delta\theta, \qquad \theta_{\min} = -2.0,\ \ \Delta\theta = 0.011142061,\ \ N = 360$$

| Handout claim | Reality |
|---|---|
| `ranges[0]` is $-114.5^\circ$ | correct: $\theta_0 = -2.0$ rad $= -114.59^\circ$ |
| `ranges[180]` is $90^\circ$ | wrong. $\theta_{180} = -2.0 + 180(0.011142061) = +0.00557$ rad $= +0.32^\circ$, essentially straight ahead. The beam at $90^\circ$ is $i = \text{round}((1.5708+2.0)/0.011142061) = 320$ |
| `ranges[360]` is the leftmost beam | wrong, and it crashes. With $N = 360$ the last index is $359$: $\theta_{359} = -2.0 + 359(0.011142061) = +2.0$ rad $= +114.59^\circ$. `ranges[360]` raises `IndexError` |

The lesson is to never hard-code beam indices. Compute them from the message fields, which is exactly what the given `sector_bounds` does, and your code then survives a sensor with a different resolution or field of view.

### 2.2 The three windows

Two parameters define all six edges:

$$\text{FRONT-RIGHT} = [-\alpha_{\text{side}},\, -\alpha_{\text{front}}), \qquad \text{FRONT} = [-\alpha_{\text{front}},\, +\alpha_{\text{front}}], \qquad \text{FRONT-LEFT} = (+\alpha_{\text{front}},\, +\alpha_{\text{side}}]$$

With the defaults $\alpha_{\text{front}} = 25^\circ = 0.4363$ rad and $\alpha_{\text{side}} = 75^\circ = 1.3090$ rad, and $i(\theta) = \text{round}((\theta - \theta_{\min})/\Delta\theta)$:

| Sector | Bearings | Indices | Beams |
|---|---|---|---|
| front-right | $-75^\circ \dots -25^\circ$ | 62 … 140 | 79 |
| front | $-25^\circ \dots +25^\circ$ | 140 … 219 | 80 |
| front-left | $+25^\circ \dots +75^\circ$ | 219 … 297 | 79 |
| (ignored) | $|\theta| > 75^\circ$ | 0–61, 298–359 | 124 |

Verify one of these yourself: $i(-25^\circ) = \text{round}((-0.4363 + 2.0)/0.011142061) = \text{round}(140.34) = 140$.

Note the boundary beams 140 and 219 appear in two rows. A one-beam overlap is harmless, but decide deliberately whether you want it, because a beam counted in both FRONT and FRONT-LEFT contributes to two different decisions.

**Why ignore the outer beams?** A beam at $100^\circ$ points almost straight out of the robot's side. A wall there is not in the way, and including it would make the robot flinch away from every corridor wall it drives past. The sectors describe *where you might go*, not *what you can see*.

**Sign discipline.** A negative bearing is to the robot's right, $\omega > 0$ is anticlockwise which is a left turn, and $+y$ is to the left. All three conventions agree (REP 103). If you assign the negative-bearing window to `left`, the robot will swerve reliably *into* obstacles — a bug that is obvious in RViz and invisible in the code.

---

## 3. The decision layer

### 3.1 Blocked or not

$$\text{blocked} \iff d_f < d_s$$

$d_s$ should be a little larger here than in task 3. Stopping needs clearance ahead; turning needs clearance ahead *while the robot sweeps*, so the default is 0.35 m rather than 0.30 m.

### 3.2 Which way

The core of the task, in one expression:

$$s = \operatorname{sign}(d_l - d_r) = \begin{cases} +1 & d_l > d_r \quad (\text{turn left}) \\ -1 & d_l < d_r \quad (\text{turn right}) \\ ? & d_l = d_r \end{cases}$$

The third row is not a corner case you can ignore. In a symmetric dead end $d_l = d_r$ exactly in simulation, and within sensor noise on hardware, so whichever way the floating-point comparison happens to fall decides — and it can fall differently on consecutive scans. You must break the tie by policy. Either always prefer a fixed side, or keep the direction already committed to (`self.turn_sign`). Both are defensible; choose one and say why in your report.

### 3.3 How hard to turn

Bang-bang control, $\omega = s\,\omega_{\max}$, works but lurches: full lock for a wall 34 cm away and full lock for one 5 cm away.

Better is to scale the turn by *how* asymmetric the two sides are. Define the normalised asymmetry

$$A = \frac{d_l - d_r}{d_l + d_r} \in [-1, +1]$$

and command

$$\omega = \operatorname{sat}\big(\omega_{\max} A,\ \pm\omega_{\max}\big), \qquad \operatorname{sat}(x, \pm L) = \max(-L, \min(L, x))$$

Three things to notice.

$A$ is **dimensionless**, so it behaves the same in a 2 m corridor and a 20 m hall. The raw difference $d_l - d_r$ does not: the same 0.3 m asymmetry means something very different at 0.4 m and at 7 m.

$A$ already **carries the sign**. $d_l > d_r$ makes $A$ positive, which makes $\omega$ positive, which is a left turn towards the open side. So the explicit sign from §3.2 becomes redundant in the proportional version — except for the tie, where $A = 0$ gives no turn at all and you need the policy sign to break the deadlock.

The **denominator can vanish**. If both sides read zero (both `None`, mapped to 0.0 by the fail-safe in `on_scan`) you divide by zero. Guard it, or note that the dead-end branch catches that case first and make sure it really does.

### 3.4 How fast to drive

$$v = \begin{cases} 0 & \text{dead end} \\[2pt] v_{\text{creep}} & \text{blocked} \\[2pt] \operatorname{sat}\!\left(v_{\max}\dfrac{d_f - d_s}{d_c - d_s},\ [0, v_{\max}]\right) & \text{clear} \end{cases}$$

The third branch is task 3 unchanged. The first is the important new one: with no way out, rotating on the spot is the only safe move, because any forward component advances into space you have not cleared.

The middle branch is a judgement call. Zero is safest and the robot pivots then proceeds. A small creep is smoother, because a differential-drive robot traces a useful arc only when it has some headway — the instantaneous centre of rotation sits at $R = v/\omega$ from the robot, so $v = 0$ gives $R = 0$, a spin in place. Try both and describe the difference.

---

## 4. Worked numerical example

$d_s = 0.35$, $d_c = 0.70$, $d_{\text{side}} = 0.45$, $v_{\max} = 0.22$, $v_{\text{creep}} = 0.05$, $\omega_{\max} = 0.9$.

| $d_r$ | $d_f$ | $d_l$ | Blocked? | Dead end? | $A$ | $\omega$ | $v$ | Reading |
|---|---|---|---|---|---|---|---|---|
| 2.10 | 1.80 | 2.40 | no | no | — | 0.000 | 0.220 | open room, straight on at cruise |
| 1.50 | 0.50 | 1.90 | no | no | — | 0.000 | 0.094 | slowing on approach, not yet turning |
| 0.90 | 0.30 | 2.20 | yes | no | +0.419 | +0.377 | 0.050 | wall ahead, left much clearer, gentle left |
| 0.30 | 0.25 | 3.10 | yes | no | +0.824 | +0.741 | 0.050 | obstacle close on the right, hard left |
| 0.40 | 0.20 | 0.38 | yes | yes | — | $+0.9$ (policy) | 0.000 | dead end, spin on the spot |

Check row 2: $v = 0.22(0.50 - 0.35)/(0.70 - 0.35) = 0.22 \times 0.4286 = 0.094$ m/s.

Check row 3: $A = (2.20 - 0.90)/(2.20 + 0.90) = 1.30/3.10 = 0.419$, so $\omega = 0.9 \times 0.419 = 0.377$ rad/s.

---

## 5. Why commitment is needed

Drive the node at a flat wall head-on with blank 10 left naive and watch what happens.

Say the robot is a fraction of a degree off square, so $d_l$ is marginally larger and it turns left. Turning left changes what the three sectors see: the left sector now looks further along the wall and reads *shorter*, so on the next scan $d_r > d_l$ and the decision flips to right. Turning right flips it back. The robot wobbles straight into the wall at creep speed.

Formally, the head-on state $d_l = d_r$ is an equilibrium of the closed loop, and this controller makes it *attracting* in the position variable while the sign output oscillates. The standard fix is to add hysteresis in the state, not the measurement: latch the decision and refuse to revisit it until the situation has genuinely changed.

$$\text{blocked} = \begin{cases} d_f < d_s + \delta & \text{if already avoiding} \\ d_f < d_s & \text{otherwise} \end{cases}$$

with $\delta = $ `resume_margin`. While `self.avoiding` is set you reuse the stored `self.turn_sign` rather than recomputing it. This is a Schmitt trigger, the same deadband that stops a thermostat from clicking on and off at the set point.

**A reactive controller's hard limit.** Even with perfect commitment, this design cannot escape a U-shaped trap. It will drive in, spin, drive out, and drive back in — a limit cycle. No purely reactive rule can solve it, because escaping requires knowing where you have already been. That is what a map and a planner are for, and recognising the boundary is a better outcome for this task than pretending your controller is general.

---

## 6. The blanks at a glance

| Blank | What | Section |
|---|---|---|
| 1a–1c | three (low, high) bearing pairs | 2.2 |
| 2 | front-blocked test | 3.1 |
| 3a–3d | comparison picking the more open side | 3.2 |
| 4 | tie-break policy | 3.2 |
| 5 | dead-end test | 3.4 |
| 6 | raw turn rate | 3.3 |
| 7 | saturate to $\pm\omega_{\max}$ | 3.3 |
| 8a–8c | three forward speeds | 3.4 |
| 9a–9d | wire the decision together | 3.2–3.3 |
| 10 | commitment / hysteresis (stretch) | 5 |

---

## 7. Tiered hints

### Blank 1 — the three windows
1. Sketch the bearing axis from $-\alpha_{\text{side}}$ to $+\alpha_{\text{side}}$ and mark the four edges.
2. Each is a plain tuple of two floats built from `self.a_front` and `self.a_side`. Which sign belongs to the right?
3. `right = (-self.a_side, -self.a_front)`, `front = (-self.a_front, self.a_front)`, `left = (self.a_front, self.a_side)`. Print the resulting indices once and compare with the table in 2.2.

### Blank 2 — blocked?
1. One comparison, same shape as the CLOSE test in task 3.
2. Which parameter marks "too near to continue straight"?
3. `d_f < self.d_s`. Blank 10 later makes the threshold state-dependent.

### Blank 3 — which side is more open
1. Two distances, one comparison, and the answer is a direction not a distance.
2. Which sign of `angular.z` turns towards the larger distance? Check §2.2.
3. `d_l > d_r` returns `+1.0`; `d_r > d_l` returns `-1.0`.

### Blank 4 — the tie
1. What is left when neither side is strictly larger?
2. Returning `0.0` here means no turn at all, which in a dead end means sitting still forever. Is that what you want?
3. Either a fixed preference (`return 1.0`) or continuity (`return self.turn_sign if self.turn_sign != 0.0 else 1.0`). Say in your report which you chose.

### Blank 5 — dead end
1. Both sides below one threshold. Which parameter?
2. Two comparisons joined so that *both* must hold.
3. `d_l < self.d_side and d_r < self.d_side`. Contrast `and` with `or` here and be sure you know why `and` is right.

### Blank 6 — turn rate
1. Start with bang-bang: `sign * self.w_max`. Get the robot going round an obstacle, then improve it.
2. For the proportional version, write $A$ from §3.3 first as its own variable, then scale.
3. `self.w_max * (d_l - d_r) / (d_l + d_r)`, with a guard for a zero denominator. Ask yourself whether `sign` is still needed.

### Blank 7 — saturation
1. Two nested built-ins, same trick as clamping an index in task 3.
2. The bounds are $-\omega_{\max}$ and $+\omega_{\max}$.
3. `max(-self.w_max, min(self.w_max, w))`.

### Blank 8 — the three speeds
1. Two branches are a single literal or attribute.
2. The third is task 3's ramp; copy it and clip the result.
3. `0.0`; `self.v_creep`; `max(0.0, min(self.v_max, self.v_max * (d_f - self.d_s) / (self.d_c - self.d_s)))`.

### Blank 9 — wiring
1. Four small expressions. Two of them are just `0.0`.
2. When the front is clear there is nothing to avoid and no direction to remember.
3. Blocked: `self.choose_turn_sign(d_l, d_r)` then `self.angular_for(self.turn_sign, d_l, d_r)`. Clear: `0.0` and `0.0`.

### Blank 10 — commitment (stretch)
1. Two pieces: a wider threshold for leaving the avoiding state, and reuse of the stored sign while in it.
2. `self.avoiding` is the state; `self.resume_margin` is the deadband; `self.turn_sign` is what you must not recompute.
3. Set `self.avoiding = True` on becoming blocked, and clear it only when `d_f > self.d_s + self.resume_margin`. In `on_scan`, when `self.avoiding` is already set, keep `self.turn_sign` unchanged instead of calling `choose_turn_sign` again. See the key.

---

## 8. Check your work

Close teleop before running — two publishers on `/cmd_vel` will fight and you will not know whose command you are seeing. Add a Marker or LaserScan display in RViz with fixed frame `odom` so you can watch the sectors as well as the robot.

| Test | Setup | Pass criterion |
|---|---|---|
| Open room | nothing ahead | cruises straight, $\omega \approx 0$ |
| Single obstacle, open left | box ahead, wall on the right | turns **left**, passes it, resumes cruise |
| Mirror image | box ahead, wall on the left | turns **right**. If both tests turn the same way, blank 1 or blank 3 has a sign error |
| Corridor | parallel walls, gap ahead | drives down the middle without flinching at the walls |
| Head-on wall | square approach | without blank 10, visible wobble; with it, one committed turn |
| Dead end | three walls | stops translating and rotates on the spot |
| Proportional check | approach obliquely | $\omega$ grows as the asymmetry grows, no lurch |
| Parameter change | `-p turn_speed_max:=0.3` | turns visibly lazier, no code edit |
| Shutdown | Ctrl-C mid-turn | robot stops, does not coast |

### Common failure modes

| Symptom | Likely cause |
|---|---|
| Swerves **into** obstacles | left/right windows swapped in blank 1, or sign flipped in blank 3 |
| Flinches away from corridor walls | `a_side` too wide, or side beams leaking into the front window |
| Wobbles head-on into a wall | blank 10 not done, or you recompute the sign every scan |
| Spins forever in the open | `blocked` always true — check `d_s` against what `d_f` actually reads |
| `ZeroDivisionError` | both sides read 0.0 in blank 6; guard the denominator |
| $\omega$ exceeds what the robot can do | blank 7 missing |
| Turns hard for distant obstacles | you used the raw difference instead of the normalised $A$ |
| Creeps into the dead-end wall | dead-end branch returning `v_creep` rather than zero |
| `IndexError` | a hard-coded index survived somewhere; recompute from `msg` |

---

## 9. Answer key

Read only after a real attempt.

```python
    # ---- BLANK 1 ---------------------------------------------------------
        right = (-self.a_side, -self.a_front)
        front = (-self.a_front, self.a_front)
        left = (self.a_front, self.a_side)

    # ---- BLANK 2 ---------------------------------------------------------
        return d_f < self.d_s

    # ---- BLANKS 3 and 4 --------------------------------------------------
        if d_l > d_r:
            return 1.0
        if d_r > d_l:
            return -1.0
        # tie: stay committed if we already picked a side, else prefer left
        return self.turn_sign if self.turn_sign != 0.0 else 1.0

    # ---- BLANK 5 ---------------------------------------------------------
        return d_l < self.d_side and d_r < self.d_side

    # ---- BLANKS 6 and 7 --------------------------------------------------
        total = d_l + d_r
        if total < 1e-6:
            w = sign * self.w_max            # no usable asymmetry; use the policy sign
        else:
            w = self.w_max * (d_l - d_r) / total
            if abs(w) < 1e-3:                # near-symmetric: commit rather than dither
                w = sign * self.w_max
        return max(-self.w_max, min(self.w_max, w))

    # ---- BLANK 8 ---------------------------------------------------------
        if dead_end:
            return 0.0
        if blocked:
            return self.v_creep
        ramp = self.v_max * (d_f - self.d_s) / (self.d_c - self.d_s)
        return max(0.0, min(self.v_max, ramp))

    # ---- BLANK 9 ---------------------------------------------------------
        if blocked:
            self.turn_sign = self.choose_turn_sign(d_l, d_r)
            w = self.angular_for(self.turn_sign, d_l, d_r)
        else:
            self.turn_sign = 0.0
            w = 0.0

    # ---- BLANK 10 (stretch) ---------------------------------------------
    # Replace the `if blocked:` block above with this version, and add the
    # matching wider threshold to front_is_blocked:
    #
    #     def front_is_blocked(self, d_f):
    #         limit = self.d_s + self.resume_margin if self.avoiding else self.d_s
    #         return d_f < limit
    #
        if blocked:
            if not self.avoiding:
                self.avoiding = True
                self.turn_sign = self.choose_turn_sign(d_l, d_r)   # decide ONCE
            w = self.angular_for(self.turn_sign, d_l, d_r)
        else:
            self.avoiding = False
            self.turn_sign = 0.0
            w = 0.0
```

Note the ordering constraint blank 10 introduces: `front_is_blocked` now reads `self.avoiding`, so `self.avoiding` must be updated *after* the blocked test, not before. Getting that backwards gives a one-scan lag that mostly works and occasionally does not, which is the worst kind of bug to have in a demo.

Delete the `TODO` helper once every blank is filled.

---

## 10. Questions to answer in your report

1. Recompute the three sector index ranges for your own scan and show they match section 2.2. What would they become if you set $\alpha_{\text{front}} = 40^\circ$?
2. Explain why `ranges[360]` in the handout crashes, and give the correct index for the beam at $+90^\circ$.
3. Compare bang-bang and proportional turning. Which did you keep, and what did the robot's path look like under each?
4. Justify the normalised asymmetry $A$ over the raw difference $d_l - d_r$ with a concrete pair of distances where the two disagree about how hard to turn.
5. Which tie-break policy did you choose, and what does the robot do in a perfectly symmetric dead end under your choice?
6. Demonstrate the wobble without blank 10 and describe how the latch removes it. Roughly how large must `resume_margin` be to work on your robot?
7. Construct a situation your controller cannot escape and explain what class of algorithm would be needed instead.
