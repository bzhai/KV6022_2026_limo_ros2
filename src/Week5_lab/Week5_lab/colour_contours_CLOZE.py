#!/usr/bin/env python3
# =====================================================================
#  KV6022 Robot Perception and Vision
#  Task 3 exercise: the colour contour detector
#
#  The starting point is colour_contours_detector.py from the module
#  repository. That node subscribes to /limo_camera/image, thresholds
#  the image in HSV, finds contours, and publishes the bounding box of
#  every large blob on /object_polygon as a PolygonStamped.
#
#  Its filter is tuned for RED objects, it has no noise cleanup, and it
#  counts the hole in a hollow shape as a second object. You are going
#  to rebuild the algorithm properly.
#
#  This file is a CLOSED GAME. Ten levels, each one a blank in the
#  detection pipeline. A level stays LOCKED until the level before it
#  passes, so exactly one thing is being marked at a time.
#
#    1) Offline grading, no ROS and no Gazebo required:
#           python3 colour_contours_game.py --check
#
#    2) Live on the robot, once all ten levels pass:
#           ros2 launch limo_gazebosim limo_gazebo_diff.launch.py
#           python3 colour_contours_game.py
#           ros2 topic echo /object_polygon        # in a third terminal
#
#  Only edit the blocks marked  >>> LEVEL n <<< .
# =====================================================================

import os
import sys

import cv2
import numpy as np

ROS_AVAILABLE = True
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Polygon, PolygonStamped, Point32
    from cv_bridge import CvBridge, CvBridgeError
except ImportError:  # pragma: no cover
    ROS_AVAILABLE = False
    Node = object

CAMERA_TOPIC = "/limo_camera/image"

# Structuring elements for level 3. Do not change them, the marker
# checks exact pixel counts.
OPEN_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

MIN_AREA = 100.0       # pixels, same gate as the original node
MIN_EXTENT = 0.25      # fraction of the bounding box that must be filled


# =====================================================================
#  >>> LEVEL 0 <<<   Retune the colour filter, red to green
# ---------------------------------------------------------------------
#  The two arrays below are copied straight from the original node.
#  They select RED:  hue 155 to 179, saturation above 25, any value.
#  Retune them so that they select GREEN instead.
#
#  Remember the 8 bit OpenCV convention:
#       H is in [0, 179]   (that is, degrees divided by two)
#       S is in [0, 255]
#       V is in [0, 255]
#
#  Your band has to pass all four of these greens
#       bright green, shaded green, dark green, grass green
#  and reject all eight of these
#       red, dark red, blue, yellow, cyan, grey, white, near black.
#
#  Grey and white are the interesting ones. They have no hue at all, so
#  the only thing that can exclude them is a floor on saturation. Near
#  black is excluded by a floor on value.
# =====================================================================

HSV_LOWER = np.array((155, 25, 0), np.uint8)      # <-- BLANK 0.1
HSV_UPPER = np.array((179, 255, 255), np.uint8)   # <-- BLANK 0.2


# =====================================================================
#  >>> LEVEL 1 <<<   BGR to HSV
# ---------------------------------------------------------------------
#  Input : bgr_image, shape (H, W, 3), dtype uint8, channels B, G, R
#  Output: the same shape and dtype, channels H, S, V
#
#  One call. Mind the channel order in the flag name: OpenCV stores
#  colour images as B, G, R, so the RGB flag will rotate your hues and
#  the marker will notice.
# =====================================================================

def to_hsv(bgr_image):
    hsv_image = None                 # <-- BLANK 1.1
    return hsv_image


# =====================================================================
#  >>> LEVEL 2 <<<   The colour mask
# ---------------------------------------------------------------------
#  Input : hsv_image from level 1
#  Output: binary mask, shape (H, W), dtype uint8, values 0 or 255
#
#  Maths, an interval test on all three channels at once:
#
#      M(x, y) = 255  if  l_c <= I_c(x, y) <= u_c  for every c in {H,S,V}
#      M(x, y) = 0    otherwise
#
#  Use HSV_LOWER and HSV_UPPER, not literal numbers, so that level 0
#  stays the single place where the filter is tuned.
# =====================================================================

def colour_mask(hsv_image):
    mask = None                      # <-- BLANK 2.1
    return mask


# =====================================================================
#  >>> LEVEL 3 <<<   Morphological cleanup
# ---------------------------------------------------------------------
#  Input : mask, the raw binary mask
#  Output: a cleaned mask, same shape and dtype
#
#  A raw threshold is always speckled: single stray pixels that match
#  the colour, and small holes inside objects where a highlight pushed
#  the pixel out of the band. Two operations fix this.
#
#      opening  = erosion then dilation   -> removes specks
#      closing  = dilation then erosion   -> fills small holes
#
#  Apply the OPENING first with OPEN_KERNEL, then the CLOSING with
#  CLOSE_KERNEL, to the result of the opening.
#
#  You need two calls to cv2.morphologyEx, with two different operation
#  flags. Do not call erode and dilate separately, the marker checks
#  the exact pixel count and the combined operations are easier to read.
# =====================================================================

def clean_mask(mask):
    opened = None                    # <-- BLANK 3.1
    closed = None                    # <-- BLANK 3.2
    return closed


# =====================================================================
#  >>> LEVEL 4 <<<   Contour extraction
# ---------------------------------------------------------------------
#  Input : a binary mask
#  Output: a list or tuple of contours
#
#  Two decisions to get right.
#
#  Retrieval mode. The original node uses RETR_TREE, which returns the
#  boundary of every hole as well, so a ring shaped bush comes back as
#  two detections. Use the mode that returns only outermost boundaries.
#
#  Approximation. CHAIN_APPROX_NONE stores every boundary pixel. Use
#  the mode that keeps only the end points of straight runs, so that a
#  rectangle is stored as four points instead of hundreds.
#
#  Careful: in OpenCV 4 this function returns TWO values.
# =====================================================================

def find_blob_contours(mask):
    contours = None                  # <-- BLANK 4.1
    return contours


# =====================================================================
#  >>> LEVEL 5 <<<   The area gate
# ---------------------------------------------------------------------
#  Input : one contour
#  Output: True if it is large enough to report, False otherwise
#
#  Maths, the area of the closed polygon with vertices (x_i, y_i) is
#  the shoelace formula
#
#      A = 0.5 * | sum_i ( x_i * y_{i+1} - x_{i+1} * y_i ) |
#
#  OpenCV implements this. Compare the result against MIN_AREA. Use
#  >= so that a contour exactly on the limit is kept.
# =====================================================================

def is_big_enough(contour):
    keep = None                      # <-- BLANK 5.1
    return keep


# =====================================================================
#  >>> LEVEL 6 <<<   The bounding box
# ---------------------------------------------------------------------
#  Input : one contour
#  Output: the tuple (x, y, w, h)
#
#  This is the upright box, defined by
#
#      x = min_i x_i ,   y = min_i y_i
#      w = max_i x_i - x + 1 ,   h = max_i y_i - y + 1
#
#  so (x, y) is the TOP LEFT corner, because image rows are numbered
#  downwards from the top. One OpenCV call gives all four integers.
# =====================================================================

def bounding_box(contour):
    box = None                       # <-- BLANK 6.1
    return box


# =====================================================================
#  >>> LEVEL 7 <<<   Packing the box into the message
# ---------------------------------------------------------------------
#  Input : box, the tuple (x, y, w, h) from level 6
#  Output: ((x, y), (w, h)) with all four values as Python floats
#
#  Look carefully at what the original node puts into the Polygon:
#
#      Point32(x=float(bbx), y=float(bby))     first point
#      Point32(x=float(bbw), y=float(bbh))     second point
#
#  The second point is NOT the opposite corner. It is the width and the
#  height, smuggled into a point field. That is a strange convention,
#  but any subscriber to /object_polygon has been written to expect it,
#  so reproduce it exactly rather than improving it.
#
#  The floats matter. Point32 fields are float32 and rospy style
#  implicit conversion does not happen in rclpy: passing the NumPy or
#  Python integers straight from boundingRect raises an AssertionError
#  when the message is built.
# =====================================================================

def box_to_polygon_points(box):
    x, y, w, h = box
    first_point = None               # <-- BLANK 7.1
    second_point = None              # <-- BLANK 7.2
    return (first_point, second_point)


# =====================================================================
#  >>> LEVEL 8 <<<   Extent, a shape gate
# ---------------------------------------------------------------------
#  Input : one contour
#  Output: a float in (0, 1]
#
#  Maths, extent is how much of the bounding box the shape actually
#  fills:
#
#      extent = A / (w * h)
#
#  where A is the contour area and w, h come from the bounding box.
#  A filled rectangle scores close to 1. A disc scores pi/4 = 0.785,
#  because a circle of diameter d has area pi*d^2/4 inside a box of
#  area d^2. A thin diagonal streak scores close to 0, because its box
#  is huge and almost empty.
#
#  That last case is why the gate exists. A row of separate leaves
#  merged by the mask, or a long thin reflection, will pass the area
#  test but is not a compact object.
#
#  Reuse the two functions you already wrote rather than calling
#  OpenCV again, and guard against a zero sized box.
# =====================================================================

def extent(contour):
    area = None                      # <-- BLANK 8.1
    x, y, w, h = bounding_box(contour)

    if w == 0 or h == 0:
        return 0.0

    ratio = None                     # <-- BLANK 8.2
    return ratio


# =====================================================================
#  >>> LEVEL 9 <<<   The whole detector
# ---------------------------------------------------------------------
#  Input : bgr_image straight from CvBridge
#  Output: a list of dictionaries, one per accepted object, each with
#
#      {"contour": <the contour>,
#       "box":     (x, y, w, h),
#       "area":    <float>,
#       "extent":  <float>}
#
#  sorted by area, largest first, and EMPTY (not None) when nothing is
#  detected.
#
#  Chain the levels you have already built:
#      to_hsv -> colour_mask -> clean_mask -> find_blob_contours
#  then for each contour, drop it unless it passes BOTH gates:
#      is_big_enough(contour)  and  extent(contour) >= MIN_EXTENT
#
#  Sorting matters because the node draws and publishes in order, so
#  the most prominent object arrives first on /object_polygon.
# =====================================================================

def detect_objects(bgr_image):
    hsv_image = None                 # <-- BLANK 9.1
    mask = None                      # <-- BLANK 9.2
    cleaned = None                   # <-- BLANK 9.3
    contours = None                  # <-- BLANK 9.4

    objects = []
    for contour in contours:
        if None:                     # <-- BLANK 9.5 (the two gates)
            continue
        objects.append({
            "contour": contour,
            "box": bounding_box(contour),
            "area": float(cv2.contourArea(contour)),
            "extent": float(extent(contour)),
        })

    objects = None                   # <-- BLANK 9.6 (sort, largest first)
    return objects


# =====================================================================
#  The node. Complete already, read it but do not edit it.
#  It is the same structure as colour_contours_detector.py: subscribe,
#  threshold, contour, publish a PolygonStamped per object.
# =====================================================================

class ColourContoursDetector(Node):

    def __init__(self):
        super().__init__("colour_contours_detector")

        self.object_pub = self.create_publisher(PolygonStamped, "/object_polygon", 10)
        self.create_subscription(Image, CAMERA_TOPIC, self.camera_callback, 10)

        self.contour_colour = (255, 255, 0)     # cyan, BGR
        self.contour_width = 1                   # pixels
        self.br = CvBridge()

        for window in ("colour image", "colour mask", "cleaned mask"):
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)

        self.get_logger().info("colour_contours_detector on %s" % CAMERA_TOPIC)

    def camera_callback(self, data):
        try:
            bgr_image = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as error:
            self.get_logger().error(str(error))
            return

        # The two mask stages are recomputed here purely so the debug
        # windows can show them. detect_objects() does its own work.
        hsv_image = to_hsv(bgr_image)
        mask = colour_mask(hsv_image)
        cleaned = clean_mask(mask)

        objects = detect_objects(bgr_image)

        for entry in objects:
            bbx, bby, bbw, bbh = entry["box"]

            cv2.drawContours(bgr_image, [entry["contour"]], -1, (0, 0, 255), 2)
            cv2.rectangle(bgr_image, (bbx, bby), (bbx + bbw, bby + bbh),
                          self.contour_colour, self.contour_width)
            cv2.putText(bgr_image, "%.0f px  ext %.2f" % (entry["area"], entry["extent"]),
                        (bbx, max(bby - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        self.contour_colour, 1)

            (px, py), (pw, ph) = box_to_polygon_points(entry["box"])
            polygon = Polygon(points=[Point32(x=px, y=py, z=0.0),
                                      Point32(x=pw, y=ph, z=0.0)])
            # The header is copied from the image so that the detection
            # keeps the timestamp and frame of the frame it came from.
            self.object_pub.publish(PolygonStamped(polygon=polygon, header=data.header))

        cv2.putText(bgr_image, "%d object(s)" % len(objects), (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("colour image", bgr_image)
        cv2.imshow("colour mask", mask)
        cv2.imshow("cleaned mask", cleaned)
        cv2.waitKey(1)

# =====================================================================
#  The marker. Fixed synthetic scenes, identical results on any machine.
# =====================================================================

def test_scene():
    """A 320x240 scene: two green targets, a red distractor, a speck,
    and five isolated green pixels of sensor noise."""
    img = np.zeros((240, 320, 3), np.uint8)
    for y in range(240):
        img[y, :] = (60 + y // 12, 62 + y // 15, 64)          # grey ramp
    cv2.rectangle(img, (40, 60), (139, 159), (60, 200, 70), -1)   # target A
    cv2.rectangle(img, (200, 40), (239, 99), (70, 160, 90), -1)   # target B
    cv2.rectangle(img, (250, 150), (299, 199), (60, 60, 200), -1)  # red
    cv2.rectangle(img, (10, 200), (17, 207), (60, 200, 70), -1)    # speck
    for (x, y) in [(170, 20), (175, 25), (300, 30), (60, 220), (250, 60)]:
        img[y, x] = (60, 200, 70)                                  # noise
    return img


def _swatch(bgr):
    patch = np.zeros((8, 8, 3), np.uint8)
    patch[:, :] = bgr
    return patch


def _ring_mask():
    m = np.zeros((160, 200), np.uint8)
    cv2.circle(m, (50, 80), 34, 255, -1)
    cv2.circle(m, (50, 80), 14, 0, -1)          # a hole, not a second object
    cv2.rectangle(m, (110, 20), (159, 69), 255, -1)
    cv2.rectangle(m, (120, 110), (139, 139), 255, -1)
    return m


def _noisy_mask():
    m = np.zeros((120, 160), np.uint8)
    cv2.rectangle(m, (30, 20), (109, 79), 255, -1)
    cv2.rectangle(m, (60, 40), (62, 42), 0, -1)     # 3x3 hole
    cv2.rectangle(m, (80, 60), (80, 62), 0, -1)     # 1x3 slit
    for (x, y) in [(5, 5), (10, 100), (150, 10), (140, 110), (20, 95), (155, 60)]:
        m[y, x] = 255                               # 6 pepper pixels
    return m


def _reference_mask(scene):
    """Built by the marker itself, so levels 3 to 8 do not depend on the
    exact HSV bounds you chose in level 0."""
    hsv = cv2.cvtColor(scene, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array((35, 40, 40), np.uint8),
                       np.array((85, 255, 255), np.uint8))


def _reference_clean(scene):
    mask = _reference_mask(scene)
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_KERNEL)
    return cv2.morphologyEx(opened, cv2.MORPH_CLOSE, CLOSE_KERNEL)


def _reference_contours(scene):
    contours, _h = cv2.findContours(_reference_clean(scene),
                                    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return sorted(contours, key=cv2.contourArea, reverse=True)


MUST_ACCEPT = [("bright green", (60, 200, 70)), ("shaded green", (30, 120, 40)),
               ("dark green", (20, 80, 25)), ("grass green", (70, 160, 90))]
MUST_REJECT = [("red", (60, 60, 200)), ("dark red", (30, 30, 110)),
               ("blue", (200, 70, 60)), ("yellow", (0, 220, 220)),
               ("cyan", (220, 220, 0)), ("grey", (128, 128, 128)),
               ("white", (240, 240, 240)), ("near black", (10, 10, 10)),
               ("pale green wall", (200, 215, 200)),
               ("green in darkness", (6, 8, 6))]


def _check_level_0(_scene):
    for name, arr in (("HSV_LOWER", HSV_LOWER), ("HSV_UPPER", HSV_UPPER)):
        if not isinstance(arr, np.ndarray) or arr.shape != (3,):
            return False, "%s must be a NumPy array of three values" % name
    if not np.all(HSV_LOWER <= HSV_UPPER):
        return False, "every lower bound must be at or below its upper bound"

    missed, leaked = [], []
    for name, bgr in MUST_ACCEPT:
        hsv = cv2.cvtColor(_swatch(bgr), cv2.COLOR_BGR2HSV)
        if np.count_nonzero(cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)) == 0:
            missed.append(name)
    for name, bgr in MUST_REJECT:
        hsv = cv2.cvtColor(_swatch(bgr), cv2.COLOR_BGR2HSV)
        if np.count_nonzero(cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)) > 0:
            leaked.append(name)

    if missed:
        return False, "these greens were missed: %s" % ", ".join(missed)
    if leaked:
        return False, "these non-greens got through: %s" % ", ".join(leaked)
    return True, "band %s to %s accepts 4 greens, rejects 10 others" % (
        tuple(int(v) for v in HSV_LOWER), tuple(int(v) for v in HSV_UPPER))


def _check_level_1(scene):
    hsv = to_hsv(scene)
    if hsv is None:
        return False, "nothing returned"
    if hsv.shape != scene.shape or hsv.dtype != np.uint8:
        return False, "expected the same shape and dtype as the input"
    if int(hsv[100, 100, 0]) != 58:
        return False, ("hue at (100,100) is %d, expected 58 for that green "
                       "(RGB flag instead of BGR?)" % hsv[100, 100, 0])
    if int(hsv[10, 10, 0]) != 15:
        return False, ("hue at (10,10) is %d, expected 15 (the channels look "
                       "swapped)" % hsv[10, 10, 0])
    if int(hsv[175, 275, 0]) != 0:
        return False, "the red patch should have hue 0, got %d" % hsv[175, 275, 0]
    return True, "hue of target A is %d, red patch is %d" % (hsv[100, 100, 0], hsv[175, 275, 0])


def _check_level_2(scene):
    mask = colour_mask(cv2.cvtColor(scene, cv2.COLOR_BGR2HSV))
    if mask is None:
        return False, "nothing returned"
    if mask.ndim != 2 or mask.dtype != np.uint8:
        return False, "expected a single channel uint8 mask"
    if set(np.unique(mask)) - {0, 255}:
        return False, "a mask holds only 0 and 255"
    count = int(np.count_nonzero(mask))
    if abs(count - 12469) > 60:
        return False, "%d pixels selected, expected about 12469" % count
    if mask[175, 275] != 0:
        return False, "the red distractor is in the mask"
    return True, "%d pixels pass the colour test" % count


def _check_level_3(_scene):
    noisy = _noisy_mask()
    out = clean_mask(noisy)
    if out is None:
        return False, "nothing returned"
    if out.shape != noisy.shape or out.dtype != np.uint8:
        return False, "shape or dtype changed"
    if out[5, 5] != 0:
        return False, "the isolated pepper pixels survived, the opening is missing"
    if out[41, 61] != 255:
        return False, "the 3x3 hole is still open, the closing is missing"
    count = int(np.count_nonzero(out))
    if count != 4800:
        return False, "%d pixels left, expected exactly 4800" % count
    return True, "pepper removed, hole filled, %d pixels" % count


def _check_level_4(_scene):
    ring = _ring_mask()
    contours = find_blob_contours(ring)
    if contours is None:
        return False, "nothing returned"
    try:
        n = len(contours)
    except TypeError:
        return False, "expected a list or tuple of contours, did you unpack both return values?"
    if n == 4:
        return False, "4 contours: the hole in the donut was counted as an object, use RETR_EXTERNAL"
    if n != 3:
        return False, "%d contours, expected 3" % n
    rect = [c for c in contours if len(c) == 4]
    if not rect:
        return False, "no contour was reduced to 4 points, use CHAIN_APPROX_SIMPLE"
    return True, "3 outer contours, rectangles stored as 4 points"


def _check_level_5(scene):
    big, speck = _reference_contours(scene)[0], _reference_contours(scene)[-1]
    if not is_big_enough(big):
        return False, "the 9801 px target was rejected"
    if is_big_enough(speck):
        return False, "the 49 px speck was accepted, it is below MIN_AREA"
    return True, "keeps 9801 px, drops 49 px"


def _check_level_6(scene):
    box = bounding_box(_reference_contours(scene)[0])
    if box is None:
        return False, "nothing returned"
    if len(box) != 4:
        return False, "expected four values, x, y, w, h"
    x, y, w, h = box
    if (int(x), int(y), int(w), int(h)) != (40, 60, 100, 100):
        return False, "got (%s, %s, %s, %s), expected (40, 60, 100, 100)" % (x, y, w, h)
    second = bounding_box(_reference_contours(scene)[1])
    if tuple(int(v) for v in second) != (200, 40, 40, 60):
        return False, "target B gave %s, expected (200, 40, 40, 60)" % (tuple(second),)
    return True, "target A at (40, 60), size 100 x 100"


def _check_level_7(_scene):
    pts = box_to_polygon_points((40, 60, 100, 100))
    if pts is None:
        return False, "nothing returned"
    if len(pts) != 2 or len(pts[0]) != 2 or len(pts[1]) != 2:
        return False, "expected two points, each an (x, y) pair"
    (ax, ay), (bx, by) = pts
    for value in (ax, ay, bx, by):
        if not isinstance(value, float):
            return False, "Point32 needs floats, got %s" % type(value).__name__
    if (ax, ay) != (40.0, 60.0):
        return False, "first point is (%.1f, %.1f), expected the top left corner" % (ax, ay)
    if (bx, by) == (140.0, 160.0):
        return False, "second point is the opposite corner, the node publishes width and height"
    if (bx, by) != (100.0, 100.0):
        return False, "second point is (%.1f, %.1f), expected (100.0, 100.0)" % (bx, by)
    return True, "corner then size, as floats"


def _check_level_8(scene):
    rect = extent(_reference_contours(scene)[0])
    if rect is None:
        return False, "nothing returned"
    if abs(float(rect) - 0.9801) > 0.02:
        return False, "a filled rectangle gave %.3f, expected about 0.98" % rect

    circ = np.zeros((120, 120), np.uint8)
    cv2.circle(circ, (60, 60), 40, 255, -1)
    cc, _h = cv2.findContours(circ, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if abs(float(extent(cc[0])) - 0.749) > 0.03:
        return False, "a disc gave %.3f, expected about 0.75" % extent(cc[0])

    line = np.zeros((120, 120), np.uint8)
    cv2.line(line, (10, 10), (100, 100), 255, 3)
    lc, _h = cv2.findContours(line, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if float(extent(lc[0])) > MIN_EXTENT:
        return False, "a thin diagonal streak scored above MIN_EXTENT, check the denominator"
    return True, "rectangle 0.98, disc 0.75, streak below the gate"


def _check_level_9(scene):
    objects = detect_objects(scene)
    if objects is None:
        return False, "nothing returned"
    if len(objects) != 2:
        return False, ("%d objects reported, expected 2 (the 49 px speck and the "
                       "isolated pixels must not survive)" % len(objects))
    for entry in objects:
        for key in ("box", "area", "extent"):
            if key not in entry:
                return False, "each object needs the keys box, area and extent"
    if objects[0]["area"] < objects[1]["area"]:
        return False, "the list is not sorted by area, largest first"
    if tuple(int(v) for v in objects[0]["box"]) != (40, 60, 100, 100):
        return False, "first box is %s, expected (40, 60, 100, 100)" % (tuple(objects[0]["box"]),)
    if tuple(int(v) for v in objects[1]["box"]) != (200, 40, 40, 60):
        return False, "second box is %s, expected (200, 40, 40, 60)" % (tuple(objects[1]["box"]),)
    if abs(objects[0]["area"] - 9801.0) > 40:
        return False, "first area is %.0f, expected about 9801" % objects[0]["area"]
    empty = detect_objects(np.zeros((60, 80, 3), np.uint8))
    if empty is None or len(empty) != 0:
        return False, "a frame with nothing green must give an empty list, not None"
    return True, "2 objects, areas %.0f and %.0f" % (objects[0]["area"], objects[1]["area"])


LEVELS = [
    ("Green HSV band", 2, _check_level_0),
    ("BGR to HSV", 1, _check_level_1),
    ("Colour mask", 1, _check_level_2),
    ("Morphological cleanup", 2, _check_level_3),
    ("Contour extraction", 2, _check_level_4),
    ("Area gate", 1, _check_level_5),
    ("Bounding box", 2, _check_level_6),
    ("Polygon message points", 2, _check_level_7),
    ("Extent, the shape gate", 2, _check_level_8),
    ("Full detector", 3, _check_level_9),
]


def run_marker():
    scene = test_scene()
    total = sum(points for _, points, _ in LEVELS)
    scored = 0
    stopped_at = None

    print("")
    print("=" * 66)
    print(" Colour contour detector, level check")
    print("=" * 66)

    for index, (name, points, check) in enumerate(LEVELS):
        if stopped_at is not None:
            print(" [ locked ] Level %d  %-24s  --/%d" % (index, name, points))
            continue
        try:
            passed, detail = check(scene)
        except NotImplementedError:
            passed, detail = False, "still a blank"
        except Exception as exc:
            passed, detail = False, "%s: %s" % (type(exc).__name__, exc)

        if passed:
            scored += points
            print(" [  pass  ] Level %d  %-24s  %d/%d   %s"
                  % (index, name, points, points, detail))
        else:
            stopped_at = index
            print(" [  FAIL  ] Level %d  %-24s   0/%d   %s"
                  % (index, name, points, detail))

    print("-" * 66)
    filled = int(round(20.0 * scored / total))
    print(" [%s%s]  %d / %d marks" % ("#" * filled, "." * (20 - filled), scored, total))

    if stopped_at is None:
        print(" All levels cleared. Start the simulator and run this file again")
        print(" without --check to publish on /object_polygon.")
    else:
        print(" Level %d is blocking the rest. Fix it and run the check again." % stopped_at)
        print(" The hints for that level are in the worksheet.")
    print("")
    return 0 if stopped_at is None else 1


def main(args=None):
    if not ROS_AVAILABLE:
        print("ROS 2 is not available here.")
        print("Run 'python3 %s --check' to work offline." % os.path.basename(sys.argv[0]))
        return

    rclpy.init(args=args)
    node = ColourContoursDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    if "--check" in sys.argv:
        sys.exit(run_marker())
    main()
