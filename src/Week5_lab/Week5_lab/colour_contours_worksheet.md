# Task 3 worksheet: the colour contour detector

**Files for this task**

| File | Purpose |
|---|---|
| `colour_contours_game.py` | The exercise. Ten blanks, with a built-in marker. |
| `colour_contours_solution.py` | The worked answers, for after you have attempted the levels. |
| `colour_contours_worksheet.md` | This sheet: theory, maths, hints, mark scheme. |

---

## 1. What the original node does, and what is wrong with it

`colour_contours_detector.py` in the module repository already works, in the sense that it runs. Read it and you will find four things that do not survive contact with a real scene.

1. **The filter is tuned for red.** Its HSV band is $H \in [155,179]$, $S \ge 25$, $V \ge 0$. Green objects will never appear.
2. **There is no noise cleanup.** The raw mask goes straight into `findContours`, so every stray pixel that happens to match the colour becomes a candidate object.
3. **It uses `RETR_TREE`.** The boundary of every *hole* is returned as well as the boundary of every object, so a ring shaped bush is reported twice.
4. **The only test is area.** A long thin smear of matching pixels passes just as easily as a compact object.

Your job is to rebuild the algorithm with those four problems fixed, keeping the published message format byte for byte identical so that anything already subscribing to `/object_polygon` keeps working.

```
/limo_camera/image                                  sensor_msgs/Image, bgr8
        |
        v  CvBridge
   BGR array  --> HSV                                Level 1
        |         |
        |         v  inRange, green band             Levels 0, 2
        |      raw mask
        |         |
        |         v  opening then closing            Level 3
        |     cleaned mask
        |         |
        |         v  findContours, outer only        Level 4
        |      contours
        |         |
        |         +--> area gate                     Level 5
        |         +--> extent gate                   Level 8
        |         |
        |         v  boundingRect                    Level 6
        |     (x, y, w, h) --> Polygon points        Level 7
        |         |
        +---------+--> sorted detection list         Level 9
                  |
                  v
        /object_polygon    geometry_msgs/PolygonStamped
```

### How to play

Ten levels, marked against fixed synthetic scenes that the script builds itself, each level locked until the one before it passes.

```bash
# offline, no ROS or Gazebo needed
python3 colour_contours_game.py --check

# live, once all ten levels pass
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py   # terminal 1
python3 colour_contours_game.py                         # terminal 2
ros2 topic echo /object_polygon                         # terminal 3
```

To run it as a node, place the file in `src/example_codes/example_codes/`, add to that package's `setup.py`

```python
'colour_contours_game = example_codes.colour_contours_game:main',
```

then `colcon build --symlink-install`, `source install/setup.bash`, `ros2 run example_codes colour_contours_game`.

---

## 2. The levels

### Level 0. Retune the filter from red to green (2 marks)

Change `HSV_LOWER` and `HSV_UPPER` so the band selects green.

**Theory.** HSV separates *which* colour a pixel is from *how bright* it is. With $R,G,B$ scaled to $[0,1]$, $C_{\max}=\max(R,G,B)$, $C_{\min}=\min(R,G,B)$, $\Delta = C_{\max}-C_{\min}$:

$$V = C_{\max}, \qquad S = \begin{cases} \Delta/C_{\max} & C_{\max}>0 \\ 0 & C_{\max}=0\end{cases}$$

$$H = 60^\circ \times \begin{cases} \left(\dfrac{G-B}{\Delta} \bmod 6\right) & C_{\max}=R \\[8pt] \dfrac{B-R}{\Delta}+2 & C_{\max}=G \\[8pt] \dfrac{R-G}{\Delta}+4 & C_{\max}=B \end{cases}$$

An 8 bit OpenCV image cannot hold $0$ to $359$ in a byte, so hue is halved:

$$H_{\text{cv}} = \left\lfloor H^\circ / 2 \right\rfloor \in [0,179], \qquad S, V \in [0,255].$$

Useful landmarks on that halved scale: red $0$, yellow $30$, green $60$, cyan $90$, blue $120$, magenta $150$.

Three separate decisions go into the band, and the marker tests all three.

* **The hue window.** Wide enough for yellow‑greens and blue‑greens, narrow enough to exclude yellow at $30$ and cyan at $90$.
* **The saturation floor.** As $\Delta \to 0$ the hue formula divides by something tiny, so grey, white and washed out pastels get an essentially random hue. A floor on $S$ discards them. This is why a pale green wall is in the reject list: it has the right hue and no colour.
* **The value floor.** In near darkness $V$ is tiny, $\Delta$ is tiny, and hue is pure sensor noise. A floor on $V$ discards it.

<details><summary>Hint 1</summary>

Leave the array types alone, they must stay `np.uint8` triples in $(H, S, V)$ order. Only the six numbers change.
</details>

<details><summary>Hint 2</summary>

Start from the landmarks: green is around $60$. A window of roughly $\pm 25$ around it clears yellow on one side and cyan on the other.
</details>

<details><summary>Hint 3</summary>

The second and third numbers of `HSV_LOWER` are what reject grey, white and the pale wall. Somewhere in the region of $40$ works for both, and still admits the shaded and dark greens, which have $S \approx 191$ and $V \approx 80$. Run `--check` and read which swatch names come back.
</details>

<details><summary>Tuning it on the real robot afterwards</summary>

Synthetic swatches are not Gazebo foliage. With the simulator running, hover over the object in `rqt_image_view` and read off the pixel, or add trackbars:

```python
cv2.createTrackbar('H low', 'colour mask', 35, 179, lambda v: None)
```

and read them each frame with `cv2.getTrackbarPos`. Widen the hue band until the object is solid, then raise the saturation floor until the background stops flickering.
</details>

---

### Level 1. BGR to HSV (1 mark)

**Theory.** OpenCV stores colour as B, G, R. That ordering is historical, from the byte layout of Windows bitmaps, and it is the single most common source of silent colour bugs, because the RGB version of the conversion runs happily and produces an image that looks plausible. On the test scene the correct call gives the green target a hue of $58$; the RGB flag gives $62$ there and shifts the grey background from $15$ to $105$.

<details><summary>Hint 1</summary>

The same function you use for greyscale conversion, with a different flag.
</details>

<details><summary>Hint 2</summary>

`cv2.cvtColor(image, cv2.COLOR_BGR2HSV)`.
</details>

---

### Level 2. The colour mask (1 mark)

**Theory.** Thresholding is an interval test applied to all three channels at once:

$$M(x,y) = \begin{cases} 255 & l_c \le I_c(x,y) \le u_c \;\; \forall c \in \{H,S,V\} \\ 0 & \text{otherwise.}\end{cases}$$

The output is $\{0,255\}$ rather than $\{0,1\}$ so that it can be displayed directly as an image and passed straight to the morphology and contour functions, which expect 8 bit input.

<details><summary>Hint 1</summary>

One call, three arguments: the image, the lower bound, the upper bound. The original node already shows you the function.
</details>

<details><summary>Hint 2</summary>

`cv2.inRange(hsv_image, HSV_LOWER, HSV_UPPER)`. Pass the module constants, not fresh `np.array((...))` literals, or level 0 stops controlling anything.
</details>

<details><summary>A note on red, for later</summary>

Red straddles the wrap point at $H_{\text{cv}} = 0$, so a single interval cannot capture it. You need two masks combined:

```python
mask = cv2.bitwise_or(cv2.inRange(hsv, (0, 100, 60), (10, 255, 255)),
                      cv2.inRange(hsv, (170, 100, 60), (179, 255, 255)))
```

The original node's band of $155$ to $179$ catches only one side of the wrap, which is part of why it detects red unreliably.
</details>

---

### Level 3. Morphological cleanup (2 marks)

Apply an opening with `OPEN_KERNEL`, then a closing with `CLOSE_KERNEL`.

**Theory.** Morphology treats the binary image as a set $A$ of foreground pixels and probes it with a structuring element $B$. The two primitives are

$$\text{erosion:} \quad A \ominus B = \{\, z \mid B_z \subseteq A \,\}$$

$$\text{dilation:} \quad A \oplus B = \{\, z \mid B_z \cap A \neq \emptyset \,\}$$

In words: a pixel survives erosion only if the whole kernel fits inside the foreground, and a pixel is added by dilation if the kernel touches the foreground anywhere. Composing them in the two possible orders gives

$$\text{opening:} \quad A \circ B = (A \ominus B) \oplus B$$

$$\text{closing:} \quad A \bullet B = (A \oplus B) \ominus B.$$

Opening deletes anything smaller than the kernel and restores what survives to its original size, so it removes specks without shrinking objects. Closing fills gaps narrower than the kernel and then pulls the outline back in, so it repairs holes without inflating objects. Both are idempotent, $(A \circ B) \circ B = A \circ B$, so applying them twice buys nothing.

**Order matters.** Opening first removes the specks while they are still single pixels. Closing first would bridge nearby specks into clumps large enough to survive the later opening. On the marker's test mask the outcome happens to be the same, but on a noisy real image it is not, and opening first is the habit to build.

The kernel size sets the scale you are willing to discard. A $3\times 3$ rectangle removes features of one or two pixels; the $5\times 5$ used for closing fills holes up to about four pixels across. The $3\times 3$ hole in the test mask is filled, and the six isolated pixels are removed, leaving exactly 4800 foreground pixels.

<details><summary>Hint 1</summary>

Two calls to the same function, `cv2.morphologyEx(src, op, kernel)`, with two different `op` flags. The second call takes the output of the first as its input.
</details>

<details><summary>Hint 2</summary>

The flags are `cv2.MORPH_OPEN` and `cv2.MORPH_CLOSE`. Use `OPEN_KERNEL` with the first and `CLOSE_KERNEL` with the second.
</details>

<details><summary>Hint 3</summary>

If the marker says the pepper survived, you skipped the opening or applied it second. If it says the hole is still open, you skipped the closing. The check reads one pixel inside the hole and one pixel on a speck, so it can tell you which.
</details>

---

### Level 4. Contour extraction (2 marks)

**Theory.** A contour is the ordered chain of boundary points of one connected component. Two arguments control what you get back.

**Retrieval mode** decides which boundaries are returned and how they are related.

| Mode | Returns |
|---|---|
| `RETR_EXTERNAL` | outermost boundaries only |
| `RETR_LIST` | all boundaries, no hierarchy |
| `RETR_CCOMP` | two levels: outer boundaries and holes |
| `RETR_TREE` | the full nesting tree |

A donut with a hole gives 1 contour under `RETR_EXTERNAL` and 2 under `RETR_TREE`. Since a hole in a bush is not a second bush, and the detector reports one bounding box per object, external retrieval is the correct choice here. This is a deliberate departure from the original node, and the marker's test mask includes a ring so it can tell which you used.

**Approximation** decides how the chain is stored. `CHAIN_APPROX_NONE` keeps every boundary pixel; `CHAIN_APPROX_SIMPLE` keeps only the end points of horizontal, vertical and diagonal runs. For the ring mask the difference is 196 points against 4 for the rectangle. Area, moments and bounding box are all identical either way, so the compressed form costs you nothing.

**The return signature.** OpenCV 3 returned three values, `(image, contours, hierarchy)`. OpenCV 4, which ROS 2 Humble uses, returns two. Code copied from older tutorials fails here, and the original node's commented out block shows the three value form.

<details><summary>Hint 1</summary>

Unpack both return values, even though you only need the first: `contours, hierarchy = cv2.findContours(...)`. Use `_hierarchy` if you want to signal that you are ignoring it.
</details>

<details><summary>Hint 2</summary>

`cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)`.
</details>

<details><summary>Hint 3</summary>

If the marker reports 4 contours where it wanted 3, you kept `RETR_TREE` and the hole was counted. If it reports 2, you forgot to unpack and returned the whole tuple.
</details>

---

### Level 5. The area gate (1 mark)

**Theory.** Contour area comes from the shoelace formula on the polygon vertices,

$$A = \frac{1}{2}\left|\sum_{i=0}^{n-1}\left(x_i y_{i+1} - x_{i+1} y_i\right)\right|, \qquad \text{indices modulo } n.$$

Note this measures the polygon through the pixel *centres*, so a solid block of $100 \times 100$ pixels returns $99 \times 99 = 9801$, not $10000$. That is a convention, not an error, and it is why the expected numbers in the marker look slightly small.

$\text{MIN\_AREA} = 100$ matches the original node. What it buys you is stability: a five pixel speck would otherwise produce a detection whose bounding box jumps around the frame from one frame to the next.

<details><summary>Hint 1</summary>

One comparison, returning a bool. Use `>=` so a contour exactly on the limit is kept.
</details>

<details><summary>Hint 2</summary>

`return cv2.contourArea(contour) >= MIN_AREA`.
</details>

---

### Level 6. The bounding box (2 marks)

**Theory.** The upright bounding box is

$$x = \min_i x_i, \quad y = \min_i y_i, \quad w = \max_i x_i - x + 1, \quad h = \max_i y_i - y + 1 .$$

$(x,y)$ is the **top left** corner, because image rows are numbered downwards from the top of the frame. `cv2.boundingRect` returns the four values as plain Python ints.

The alternative, `cv2.minAreaRect`, returns a *rotated* rectangle as a centre, a size and an angle. It fits tilted objects far more tightly, but it does not fit the four number message layout this node publishes, so it is the wrong tool here even though it is the better fit geometrically.

<details><summary>Hint 1</summary>

One call. It returns all four values as a single tuple, so you can return that tuple directly.
</details>

<details><summary>Hint 2</summary>

`cv2.boundingRect(contour)`.
</details>

---

### Level 7. Packing the box into the message (2 marks)

**Theory.** This level is about reading someone else's message convention rather than about vision. Look at what the original node builds:

```python
Polygon(points=[Point32(x=float(bbx), y=float(bby)),
                Point32(x=float(bbw), y=float(bbh))])
```

Two points, but the second is not a point at all: it is the width and the height packed into the `x` and `y` fields. So the published polygon is $\big((x,y),(w,h)\big)$, not the two opposite corners $\big((x,y),(x+w,y+h)\big)$, and a subscriber that assumes corners will draw boxes in the wrong place.

It is a poor design. Reproduce it anyway. Changing a published interface breaks every subscriber written against it, and a detector that quietly changes what its topic means is worse than one with an awkward convention. If you want to fix it, that is a new topic name and a deprecation note, not a silent edit.

**Why the floats.** `Point32` fields are `float32`. In `rclpy` the message setters are type checked, so an `int` from `boundingRect` or a `numpy.int32` raises an `AssertionError` when you build the message. `float()` on each value is what the original node does, for exactly this reason.

<details><summary>Hint 1</summary>

Two tuples of two floats each. No OpenCV, no NumPy, just `float()`.
</details>

<details><summary>Hint 2</summary>

`first_point = (float(x), float(y))` and the second is built the same way from `w` and `h`.
</details>

---

### Level 8. Extent, a shape gate (2 marks)

**Theory.** Extent is the fraction of the bounding box that the contour actually fills,

$$\text{extent} = \frac{A}{w\,h} \in (0, 1].$$

Reference values worth remembering:

| Shape | Extent |
|---|---|
| Axis aligned filled rectangle | $\approx 1$ |
| Disc | $\dfrac{\pi d^2/4}{d^2} = \dfrac{\pi}{4} \approx 0.785$ |
| Rectangle rotated by $45^\circ$ | $0.5$ |
| Thin diagonal streak | close to $0$ |

The area gate at level 5 asks *is there enough of it*. Extent asks *is it compact*. A row of separate leaves that the mask has merged, a long specular reflection along a table edge, or a scattering of grass all pass the area test with ease and all have low extent. `MIN_EXTENT = 0.25` is deliberately permissive so that real, ragged foliage still gets through; raise it towards $0.5$ if you only care about solid, box‑like objects.

Two related descriptors, if you want to go further:

$$\text{solidity} = \frac{A}{A_{\text{hull}}}, \qquad \text{aspect ratio} = \frac{w}{h},$$

where $A_{\text{hull}}$ is the area of the convex hull from `cv2.convexHull`. Solidity is better than extent at catching shapes with deep concavities, since it is unaffected by rotation.

<details><summary>Hint 1</summary>

The numerator you already have from level 5's function, the denominator from level 6's. Call your own functions rather than OpenCV.
</details>

<details><summary>Hint 2</summary>

`ratio = float(area) / float(w * h)`. Dividing by the perimeter, or by the image size, gives numbers far from 1 and the marker will say so.
</details>

<details><summary>Hint 3</summary>

The `w == 0 or h == 0` guard above the blank is already written for you. Do not remove it: a degenerate contour would otherwise divide by zero inside the callback and kill the node.
</details>

---

### Level 9. The whole detector (3 marks)

**Theory.** Nothing new, just composition, but three details carry the marks.

**Order of the gates.** Test the cheap condition first:

```python
if not is_big_enough(contour) or extent(contour) < MIN_EXTENT:
    continue
```

Python's `or` short circuits, so `extent` (which runs `contourArea` and `boundingRect`) never executes for the specks that the area gate has already rejected. On a noisy mask with hundreds of tiny contours, that is most of the work avoided.

**Sorting.** The node draws and publishes in list order, so the most prominent object should come first:

```python
sorted(objects, key=lambda entry: entry["area"], reverse=True)
```

**Empty, not `None`.** A frame with nothing green must give `[]`. A caller can iterate over an empty list without a special case; `None` forces a guard at every call site, and the one you forget becomes a `TypeError` inside a callback and takes the node down.

<details><summary>Hint 1</summary>

Four lines to build the cleaned contour list, and every one of them is a call to a function you have already written.
</details>

<details><summary>Hint 2</summary>

For the gate, the shape is `if <fails area> or <fails extent>: continue`. Watch the polarity: `is_big_enough` returns `True` for the contours you want to keep, so it needs a `not`.
</details>

<details><summary>Hint 3</summary>

`sorted(...)` returns a new list and leaves the original alone; `list.sort()` sorts in place and returns `None`. If the marker says nothing was returned, you used the second one in an assignment.
</details>

---

## 3. Checking it on the robot

Once the marker shows 18/18, run the simulator and the node together. Three windows open: the colour image with boxes drawn on it, the raw mask, and the cleaned mask. Compare the last two; the difference is what level 3 bought you.

```bash
ros2 topic echo /object_polygon
ros2 topic hz /object_polygon
ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/limo_camera/image
```

Each message looks like this, and remember the second point is the size:

```yaml
header:
  stamp: {sec: 1723, nanosec: 400000000}
  frame_id: camera_link
polygon:
  points:
  - {x: 214.0, y: 132.0, z: 0.0}    # top left corner
  - {x: 86.0,  y: 74.0,  z: 0.0}    # width, height
```

Things to try, in order:

1. Drive the robot towards a green object and watch the width and height grow. Roughly, for an object of real width $w_{\text{real}}$ at distance $Z$, the pinhole model gives $w_{\text{px}} = f_x w_{\text{real}} / Z$, so the box width should scale as $1/Z$.
2. Turn the robot slowly and watch $x$ sweep across the frame.
3. Put two green objects in view and confirm the larger one is published first.
4. Move an object into shadow. If it drops out, your saturation or value floor is too high.

---

## 4. When something goes wrong

| What you see | Most likely cause |
|---|---|
| Mask is completely black | Hue band still on the red numbers, or the saturation floor is too high for Gazebo's washed out materials. |
| Mask is completely white | No floors on $S$ and $V$, so the background is passing. |
| Boxes flicker on and off between frames | Object is near `MIN_AREA`. Lower the gate, or hold a detection for $n$ frames before publishing. |
| One object reported as two | Highlight splitting the blob. Increase the closing kernel. |
| Two objects reported as one | They touch in the mask. Decrease the closing kernel, or separate by hue. |
| `AssertionError` when publishing | An `int` or `numpy.int32` reached a `Point32` field. Level 7's `float()` calls. |
| `ValueError: too many values to unpack` | Three value `findContours` signature from an OpenCV 3 tutorial. |
| Node dies with `ZeroDivisionError` | A degenerate contour reached `extent` with the guard removed. |
| Windows freeze | `cv2.waitKey(1)` missing. It is what lets the GUI thread redraw. |

---

## 5. Mark scheme

| Level | Content | Marks |
|---|---|---|
| 0 | Green HSV band, with saturation and value floors | 2 |
| 1 | BGR to HSV conversion | 1 |
| 2 | Three channel threshold using the constants | 1 |
| 3 | Opening then closing, correct kernels | 2 |
| 4 | External retrieval, compressed chain, two value unpack | 2 |
| 5 | Area gate against `MIN_AREA` | 1 |
| 6 | Upright bounding box | 2 |
| 7 | Corner and size as floats, matching the published convention | 2 |
| 8 | Extent computed as area over box area | 2 |
| 9 | Full pipeline, both gates, sorted, empty list when nothing found | 3 |
| | **Total** | **18** |

---

## 6. Extensions

1. **Trackbars.** Six sliders for the HSV bounds, read each frame with `cv2.getTrackbarPos`, so the filter is tuned live instead of by editing and restarting.
2. **ROS parameters.** Declare the bounds with `self.declare_parameter('hue_low', 35)` and set them from the command line with `--ros-args -p hue_low:=30`, which is how a node like this should be configured in practice.
3. **Track across frames.** Match each detection to the previous frame's list by nearest centre, and assign a persistent ID. Then a flickering detection can be held alive for a few frames rather than disappearing.
4. **Publish something more useful.** A `vision_msgs/Detection2DArray` carries all detections in one message with a class and a score, instead of one `PolygonStamped` per object.
5. **Compare with Tasks 5 and 6.** Put the same object in front of the robot and record, for your colour detector, `find_object_2d` with ORB, and YOLO: does the detection survive a lighting change, a scale change, a rotation, and a same‑coloured distractor? What frame rate does each manage on the CPU? The colour detector will win on speed by a wide margin and lose on everything else, and being able to say precisely how much is the point of the exercise.
