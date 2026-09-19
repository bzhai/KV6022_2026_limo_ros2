# Task 2 worksheet: from `opencv_bridge.py` to a LIMO perception pipeline

**Files for this task**

| File | Purpose |
|---|---|
| `limo_vision_game.py` | The exercise. Nine blanks to fill, with a built-in marker. |
| `limo_vision_solution.py` | The worked answers, for after you have attempted the levels. |
| `limo_vision_worksheet.md` | This sheet: theory, maths, hints, mark scheme. |

---

## 1. What you are building

The template `opencv_bridge.py` stops at greyscale, blur and Canny on a topic called `/camera`, which does not exist on LIMO. You will extend it into a pipeline that finds a green object and reports where it is:

```
/limo_camera/image                      (sensor_msgs/Image, bgr8)
        |
        v  CvBridge                     Level 0
   NumPy array (H, W, 3) uint8
        |
        +--> greyscale --> mean brightness            Levels 1, 2
        |         |
        |         +--> 5x5 box blur                   Level 3
        |         +--> Canny edges                    Level 4
        |
        +--> HSV --> green mask                        Level 5
                        |
                        v  contours, largest blob      Level 6
                     centroid from moments             Level 7
                        |
                        v  bearing in degrees          Level 8
                   /limo/target_bearing  (std_msgs/Float32)
```

### How to play

The exercise is a **closed game**: nine levels, marked against a fixed synthetic frame that is generated inside the script, and each level stays locked until the one before it passes. Work top to bottom.

```bash
# offline, no ROS or Gazebo needed
python3 limo_vision_game.py --check

# live, once all nine levels pass
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py   # terminal 1
python3 limo_vision_game.py                             # terminal 2
```

To run it as a proper node instead, drop the file into `src/example_codes/example_codes/`, add an entry point to that package's `setup.py`,

```python
'limo_vision = example_codes.limo_vision_game:main',
```

then `colcon build --symlink-install`, `source install/setup.bash`, and `ros2 run example_codes limo_vision`.

---

## 2. Background theory

### 2.1 The image as a matrix

A colour frame is a function sampled on a grid. For an 8 bit BGR image of height $H$ and width $W$,

$$I : \{0,\dots,W-1\} \times \{0,\dots,H-1\} \rightarrow \{0,\dots,255\}^3 .$$

In NumPy this is an array of shape $(H, W, 3)$ and dtype `uint8`, indexed **row first**: `img[y, x]` gives the pixel at column $x$, row $y$. Channel order in OpenCV is B, G, R, not R, G, B. Almost every colour bug in this task comes from forgetting one of those two conventions.

### 2.2 Why CvBridge exists

A `sensor_msgs/Image` message carries the pixels as a flat byte array plus `height`, `width`, `step` and `encoding`. OpenCV wants a strided NumPy array. `CvBridge` performs that reinterpretation:

```python
cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
```

Asking for `bgr8` means "give me three 8 bit channels in B, G, R order, converting if the publisher used something else". If you point this at a depth topic such as `/limo_camera/depth/image_raw`, whose encoding is `32FC1`, the conversion will complain, because a single channel of 32 bit floats in metres is not a colour image.

### 2.3 The callback is the loop

There is no `while True` in a ROS 2 vision node. `rclpy.spin()` hands each arriving message to `camera_callback`, so the frame rate of your processing is set by the publisher. Check it with

```bash
ros2 topic hz /limo_camera/image
```

If your callback takes longer than one publishing period, messages queue up and then get dropped (the queue depth is the `10` in `create_subscription`), and your overlay lags behind reality. This is why the display windows are resized down by half, and why the heavy work happens on the greyscale image rather than on all three channels.

---

## 3. The levels

### Level 0. Find the camera topic (1 mark)

Set `CAMERA_TOPIC` to the LIMO colour image topic.

**Theory.** A topic name is a string used for name resolution at subscription time. Subscribing to a topic nobody publishes is not an error in ROS 2, the node simply waits forever, so a silent node with no windows is the symptom you are looking for.

<details><summary>Hint 1</summary>

With the simulation running, list everything and filter it:

```bash
ros2 topic list | grep image
```
</details>

<details><summary>Hint 2</summary>

You will see several candidates. Check the message type and the publisher of each before you commit:

```bash
ros2 topic info /limo_camera/image --verbose
```

You want `sensor_msgs/msg/Image` with a colour encoding, not the depth stream and not the `camera_info` topic.
</details>

<details><summary>Hint 3</summary>

Task 1 of this workshop already told you the answer when it asked you to run `rqt_image_view` with `-r image:=...`. Reuse exactly that topic.
</details>

---

### Level 1. Greyscale conversion (1 mark)

Return a 2D `uint8` array from a 3 channel `uint8` array.

**Theory.** Luminance is a weighted sum of the channels, with weights from the ITU‑R BT.601 standard that follow the eye's sensitivity, green most, blue least:

$$Y(x,y) = 0.299\,R(x,y) + 0.587\,G(x,y) + 0.114\,B(x,y).$$

Working on $Y$ costs one third of the memory bandwidth of the colour image, which matters for the per-frame budget in section 2.3.

<details><summary>Hint 1</summary>

One OpenCV call does the whole conversion. You are looking for the function that changes colour space, and a flag naming the source and destination.
</details>

<details><summary>Hint 2</summary>

The function is `cv2.cvtColor(src, code)`. The flags are named `cv2.COLOR_<from>2<to>`.
</details>

<details><summary>Hint 3</summary>

`cv2.COLOR_RGB2GRAY` will run without error and give an image that looks almost right, because it applies the $0.299$ weight to the blue channel and $0.114$ to red. The marker computes the mean and will catch it. Use the BGR flag.
</details>

---

### Level 2. Mean brightness (1 mark)

Return the average intensity as a Python `float`.

**Theory.**

$$\bar{I} = \frac{1}{HW}\sum_{y=0}^{H-1}\sum_{x=0}^{W-1} I(x,y).$$

The template file shows both ways of touching every pixel. The double `for` loop runs in the Python interpreter, roughly $HW$ bytecode iterations; the vectorised reduction runs in compiled C over contiguous memory. For a $640 \times 480$ frame that is around $3\times10^5$ pixels per frame, and at 30 Hz the loop version cannot keep up. Prefer the vectorised form everywhere in this task.

Note also that the accumulator matters: summing `uint8` values into a `uint8` overflows at 255. NumPy's `mean` accumulates in `float64`, so it is safe.

<details><summary>Hint 1</summary>

The original `opencv_bridge.py` already prints this quantity. Look at the line just after the greyscale conversion.
</details>

<details><summary>Hint 2</summary>

`np.mean(array)` returns a NumPy scalar. Wrap it in `float()` so that the type is a plain Python float, which is what you would put in a `Float32` message.
</details>

---

### Level 3. Box filter smoothing (1 mark)

Apply a $5 \times 5$ averaging filter using `BLUR_KERNEL`.

**Theory.** Smoothing is a discrete convolution with a kernel $K$:

$$I'(x,y) = \sum_{i=-a}^{a}\sum_{j=-a}^{a} K(i,j)\, I(x+i, y+j), \qquad a = \frac{k-1}{2}.$$

For the normalised box filter every weight is equal,

$$K_{\text{box}}(i,j) = \frac{1}{k^2}, \qquad \sum_{i,j} K(i,j) = 1,$$

so a flat region keeps its brightness. The Gaussian alternative weights by distance,

$$K_{\text{gauss}}(i,j) = \frac{1}{2\pi\sigma^2} \exp\!\left(-\frac{i^2+j^2}{2\sigma^2}\right),$$

and preserves edges slightly better for the same amount of noise suppression. Both reduce variance: for independent noise of variance $\sigma_n^2$ per pixel, the box filter output has variance $\sigma_n^2/k^2$, that is a factor of 25 here, at the price of blurring detail on a scale below $k$ pixels.

Also note what happens at the border, where the kernel hangs off the edge of the image. OpenCV's default is `BORDER_REFLECT_101`, mirroring the pixels without repeating the edge pixel itself. That is why the marker checks a pixel near the border as well as the total sum.

<details><summary>Hint 1</summary>

Two candidates exist in OpenCV for this. Only the plain averaging one matches the maths above with equal weights.
</details>

<details><summary>Hint 2</summary>

`cv2.blur(src, ksize)` where `ksize` is a `(width, height)` tuple. Pass `BLUR_KERNEL`, do not hard code `(5, 5)`, so that the constant stays in one place.
</details>

---

### Level 4. Canny edge detection (2 marks)

Return a binary edge map using `CANNY_LOW` and `CANNY_HIGH`.

**Theory.** Canny is four stages.

1. **Smooth** with a Gaussian, because differentiation amplifies noise.
2. **Gradient** by Sobel convolution. The horizontal kernel is

$$S_x = \begin{bmatrix} -1 & 0 & +1 \\ -2 & 0 & +2 \\ -1 & 0 & +1 \end{bmatrix}, \qquad S_y = S_x^{\mathsf T},$$

giving magnitude and orientation

$$\lVert \nabla I \rVert = \sqrt{G_x^2 + G_y^2}, \qquad \theta = \operatorname{atan2}(G_y, G_x).$$

3. **Non‑maximum suppression**: keep a pixel only if its magnitude is a local maximum along $\theta$, which thins wide ridges down to one pixel.
4. **Hysteresis** with two thresholds $T_{\text{low}} < T_{\text{high}}$:

$$\text{pixel} \in \begin{cases} \text{edge} & \lVert \nabla I \rVert \ge T_{\text{high}} \\ \text{edge, if connected to a strong edge} & T_{\text{low}} \le \lVert \nabla I \rVert < T_{\text{high}} \\ \text{discarded} & \lVert \nabla I \rVert < T_{\text{low}} \end{cases}$$

The two thresholds are what stop a single noisy pixel from starting an edge while still allowing a genuine but faint contour to be traced. A common starting ratio is $T_{\text{high}} : T_{\text{low}} \approx 3:1$, which is what $50$ and $150$ give you.

<details><summary>Hint 1</summary>

The template already calls this function, with different threshold values. Copy the call and change the numbers to the module constants.
</details>

<details><summary>Hint 2</summary>

`cv2.Canny(image, threshold1, threshold2)` takes the low threshold first. Output is `uint8` containing only 0 and 255, so you can count edge pixels with `np.count_nonzero`.
</details>

---

### Level 5. Green mask in HSV (2 marks)

Convert to HSV, then threshold with `HSV_GREEN_LOWER` and `HSV_GREEN_UPPER`.

**Theory.** In RGB or BGR, brightness is smeared across all three channels, so a leaf in sunlight and the same leaf in shadow are far apart in the cube. HSV separates *what colour it is* from *how bright it is*. With $R,G,B$ normalised to $[0,1]$, $C_{\max} = \max(R,G,B)$, $C_{\min} = \min(R,G,B)$ and $\Delta = C_{\max}-C_{\min}$:

$$V = C_{\max}, \qquad S = \begin{cases} \Delta / C_{\max} & C_{\max} > 0 \\ 0 & C_{\max} = 0 \end{cases}$$

$$H = 60^\circ \times \begin{cases} \left(\dfrac{G-B}{\Delta} \bmod 6\right) & C_{\max} = R \\[8pt] \dfrac{B-R}{\Delta} + 2 & C_{\max} = G \\[8pt] \dfrac{R-G}{\Delta} + 4 & C_{\max} = B \end{cases}$$

Hue is an angle, so it wraps at $360^\circ$. In an 8 bit OpenCV image the ranges are squeezed into a byte:

$$H \in [0, 179] \;\; (\text{i.e. } H_{\text{cv}} = H^\circ / 2), \qquad S, V \in [0, 255].$$

Green sits near $H^\circ \approx 120$, so near $H_{\text{cv}} \approx 60$, which is why the bounds here are $40$ to $85$. The lower bounds on $S$ and $V$ throw away washed out and nearly black pixels, where hue is numerically meaningless because $\Delta \to 0$.

Thresholding is then a per channel interval test:

$$M(x,y) = \begin{cases} 255 & l_c \le I_c(x,y) \le u_c \;\; \forall c \in \{H,S,V\} \\ 0 & \text{otherwise} \end{cases}$$

*Aside for later:* red straddles the wrap point at $H_{\text{cv}} = 0$, so a red target needs two masks, $[0, 10]$ and $[170, 179]$, combined with `cv2.bitwise_or`.

<details><summary>Hint 1</summary>

Two lines, two function calls, and you have already used one of them in level 1 with a different flag.
</details>

<details><summary>Hint 2</summary>

`cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)`, then `cv2.inRange(hsv, lower, upper)`. `inRange` checks all three channels at once and returns the 0/255 mask for you, so no loop and no `np.where`.
</details>

<details><summary>Hint 3</summary>

If you pass the BGR image straight to `inRange` it will still run and may even look plausible on a bright target, but it fails as soon as the lighting changes. The marker tests a shaded green patch for exactly this reason.
</details>

---

### Level 6. Largest blob (2 marks)

Extract contours, keep the biggest, reject anything below `MIN_BLOB_AREA`.

**Theory.** A contour is the ordered list of boundary points of a connected component of the mask. `cv2.findContours` needs two decisions.

* **Retrieval mode.** `RETR_EXTERNAL` returns only outermost boundaries. `RETR_LIST` or `RETR_TREE` would also return the boundaries of holes, so a leaf with a gap in it becomes two detections.
* **Approximation.** `CHAIN_APPROX_NONE` stores every boundary pixel. `CHAIN_APPROX_SIMPLE` stores only the end points of straight segments, which is smaller and faster and loses nothing you need for area or centroid.

Area comes from the shoelace formula on the polygon vertices,

$$A = \frac{1}{2}\left| \sum_{i=0}^{n-1} \left( x_i y_{i+1} - x_{i+1} y_i \right) \right|, \qquad \text{indices mod } n,$$

which `cv2.contourArea` implements. Note it measures the polygon through the pixel centres, so a solid $50 \times 60$ rectangle of pixels returns $49 \times 59 = 2891$, not $3000$. Both answers are defensible, this is a convention, not a bug, and the marker allows for it.

The minimum area test is a simple noise gate. Thresholding always leaves speckle; a few stray pixels of area 5 would otherwise become a "detection" whose centroid jumps randomly around the frame.

<details><summary>Hint 1</summary>

In OpenCV 4, `findContours` returns **two** values. Unpack both, even if you ignore the hierarchy, or you will be handed a tuple where you expected a list.
</details>

<details><summary>Hint 2</summary>

To choose the biggest, hand the built-in `max` a key function: `max(contours, key=cv2.contourArea)`. No sorting needed, and no loop.
</details>

<details><summary>Hint 3</summary>

Three separate `return None` cases must all work: an empty contour list, a best contour that is too small, and, in the caller, the case where this function returns `None`. The marker tests an empty mask and an 8 by 8 speck.
</details>

---

### Level 7. Centroid from image moments (2 marks)

Return `(cx, cy)` in pixels.

**Theory.** The raw geometric moments of order $p+q$ of an image region are

$$M_{pq} = \sum_{x}\sum_{y} x^{p} y^{q} I(x,y).$$

Reading off the low orders: $M_{00}$ is the area (the zeroth moment, the total mass), and $M_{10}$, $M_{01}$ are the first moments about the origin. The centre of mass follows:

$$\bar{x} = \frac{M_{10}}{M_{00}}, \qquad \bar{y} = \frac{M_{01}}{M_{00}}.$$

`cv2.moments(contour)` returns these in a dictionary keyed `'m00'`, `'m10'`, `'m01'`, and so on, computed from the polygon rather than pixel by pixel.

The guard on $M_{00} = 0$ is not decoration. A degenerate contour, for example a single point or a line, has zero area, and an unguarded division raises `ZeroDivisionError` inside the callback. In ROS 2 an exception thrown in a callback propagates out of `spin()` and your node dies mid demonstration.

Central moments $\mu_{pq} = \sum\sum (x-\bar{x})^p (y-\bar{y})^q I(x,y)$ are the next step if you want orientation, since the principal axis of the blob is

$$\theta = \frac{1}{2}\arctan\!\left(\frac{2\mu_{11}}{\mu_{20}-\mu_{02}}\right).$$

<details><summary>Hint 1</summary>

One call gives you the whole dictionary. Then it is two divisions.
</details>

<details><summary>Hint 2</summary>

`M = cv2.moments(contour)`, then `cx = M['m10'] / M['m00']`. Do not cast to `int` here; keep sub pixel precision and round only when you draw.
</details>

---

### Level 8. From pixels to a bearing (3 marks)

Convert a horizontal pixel position into an angle in degrees.

**Theory.** A pixel column on its own is useless to a controller, because it depends on the resolution. Normalise it instead. With image width $W$ and centroid column $c_x$, the signed error from the optical axis is

$$e = c_x - \frac{W}{2},$$

the resolution independent error is

$$\hat{e} = \frac{2e}{W} \in [-1, +1],$$

and scaling by the half field of view gives a bearing in degrees:

$$\beta = \hat{e}\cdot\frac{\text{HFOV}}{2}.$$

So $c_x = W/2$ gives $\beta = 0$, dead ahead, and the image edges give $\beta = \pm \text{HFOV}/2$.

This is the linear approximation of the exact pinhole relation

$$\beta_{\text{exact}} = \arctan\!\left(\frac{c_x - c_{x0}}{f}\right),$$

where $f$ is the focal length in pixels and $c_{x0}$ the principal point, both available from `/limo_camera/camera_info` in the matrix

$$K = \begin{bmatrix} f_x & 0 & c_{x0} \\ 0 & f_y & c_{y0} \\ 0 & 0 & 1 \end{bmatrix}.$$

The two agree to within a degree or so near the image centre, where $\arctan u \approx u$, and diverge towards the edges. The linear version needs no calibration, which is why it is a reasonable choice for a first steering loop.

A note on signs, because this is where marks are usually lost. Image $x$ increases to the right, so $\beta > 0$ means the target is to the robot's right. The ROS convention (REP 103) has positive yaw counter‑clockwise, that is to the *left*. A proportional controller therefore needs

$$\omega = -K_p \,\beta ,$$

with the minus sign, or the robot will turn away from the target.

<details><summary>Hint 1</summary>

Three lines, in the order given by the three equations above. No OpenCV, no NumPy.
</details>

<details><summary>Hint 2</summary>

Divide by `2.0` and not `2` if you want to be safe about integer division, and remember `image_width` arrives as an `int`.
</details>

<details><summary>Hint 3</summary>

Sanity check it by hand before running the marker: with $W = 640$ and HFOV $= 80^\circ$, $c_x = 320$ must give exactly $0$, and $c_x = 640$ must give exactly $+40$.
</details>

---

## 4. When something goes wrong

| What you see | Most likely cause |
|---|---|
| Node starts, no windows, no output | Wrong topic name, or the simulator is not running. Check `ros2 topic list`. |
| `cv_bridge` exception about encoding | Subscribed to the depth topic. `bgr8` needs the colour stream. |
| Windows open but stay grey or black | `cv2.waitKey(1)` missing, the GUI thread never gets to redraw. |
| Mask is empty on an obviously green object | Gazebo greens are often unsaturated. Widen the hue band or lower the $S$ and $V$ floors. |
| Detection flickers between objects | Two blobs of similar area. Raise `MIN_BLOB_AREA`, or track the blob nearest the previous centroid. |
| Overlay lags behind the simulation | Callback slower than the publishing rate. Process the small image, or drop frames deliberately. |
| `error: (-215:Assertion failed)` in `cvtColor` | Passing a single channel image to a BGR to HSV conversion, or the other way round. |

A useful debugging habit: `print` the `shape` and `dtype` of whatever you are passing. Nearly every OpenCV assertion failure is a shape or a dtype mismatch.

---

## 5. Mark scheme

| Level | Content | Marks |
|---|---|---|
| 0 | Camera topic identified | 1 |
| 1 | Greyscale conversion | 1 |
| 2 | Vectorised mean brightness | 1 |
| 3 | Box filter with the given kernel | 1 |
| 4 | Canny with correct hysteresis thresholds | 2 |
| 5 | HSV conversion and multi channel threshold | 2 |
| 6 | Contour extraction, largest blob, noise gate | 2 |
| 7 | Centroid from moments with zero area guard | 2 |
| 8 | Normalised bearing, correct at the centre and both edges | 3 |
| | **Total** | **15** |

---

## 6. Extensions once you have 15/15

1. **Close the loop.** Subscribe to `/limo/target_bearing` in a second node and publish `geometry_msgs/Twist` on `/cmd_vel` with $\omega = -K_p\beta$, plus a forward speed that decreases as the blob area grows. Start with $K_p = 0.02$ and watch the sign.
2. **Estimate range.** For an object of known real width $w$ at distance $Z$, the pinhole model gives pixel width $w_{px} = f_x w / Z$, so $Z = f_x w / w_{px}$. Take $f_x$ from `/limo_camera/camera_info` and compare against the depth image at the centroid.
3. **Morphology.** Insert `cv2.morphologyEx` with `MORPH_OPEN` then `MORPH_CLOSE` before the contour stage and measure how much speckle disappears from the mask.
4. **Trackbars.** Add `cv2.createTrackbar` for the six HSV bounds so you can tune the filter live rather than editing and restarting, which is what Task 3 asks you to do by hand.
5. **Compare with the detectors from Tasks 5 and 6.** Your colour blob detector, `find_object_2d` with ORB features, and YOLO all report a bounding box. Put the same object in front of the robot and record, for each: does it survive a lighting change, a scale change, a rotation, and a similarly coloured distractor? Note the frame rate of each as well. The result is a useful table for your report.
