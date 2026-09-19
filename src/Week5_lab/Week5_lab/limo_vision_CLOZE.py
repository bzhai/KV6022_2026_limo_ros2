#!/usr/bin/env python3
# =====================================================================
#  KV6022 Robot Perception and Vision
#  Task 2 exercise: from opencv_bridge.py to a LIMO camera pipeline
#
#  This file is a CLOSED GAME. Nine levels, each one a small blank in
#  the perception pipeline. A level stays LOCKED until the level before
#  it passes, so you always know exactly which line is being marked.
#
#  Two ways to run this file:
#
#    1) Offline grading (no ROS, no Gazebo needed):
#           python3 limo_vision_game.py --check
#       Every level is scored against a fixed synthetic test frame that
#       is generated inside this file, so the marks are reproducible on
#       any machine.
#
#    2) Live on the robot (all nine levels must pass first):
#           ros2 launch limo_gazebosim limo_gazebo_diff.launch.py
#           python3 limo_vision_game.py
#
#  Only edit the blocks marked  >>> LEVEL n <<< .
#  Everything outside those blocks is already written for you.
# =====================================================================

import sys

import cv2
import numpy as np

# ROS is imported lazily so that --check works on a laptop without ROS.
ROS_AVAILABLE = True
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover
    ROS_AVAILABLE = False
    Node = object


# =====================================================================
#  >>> LEVEL 0 <<<   The camera topic
# ---------------------------------------------------------------------
#  The template you started from (opencv_bridge.py) subscribes to the
#  topic '/camera'. That topic does not exist on LIMO. Find the real one
#  with:
#
#      ros2 topic list | grep image
#      ros2 topic info /<the topic you found> --verbose
#      ros2 topic hz  /<the topic you found>
#
#  Replace the string below with the colour image topic of the LIMO
#  simulation. Keep the leading slash.
# =====================================================================

CAMERA_TOPIC = "FILL_ME_IN"          # <-- BLANK 0.1

# Fixed parameters used later. Do not change these, the marker relies
# on them.
BLUR_KERNEL = (5, 5)
CANNY_LOW = 50
CANNY_HIGH = 150
HSV_GREEN_LOWER = np.array([40, 60, 60], dtype=np.uint8)
HSV_GREEN_UPPER = np.array([85, 255, 255], dtype=np.uint8)
MIN_BLOB_AREA = 200.0
CAMERA_HFOV_DEG = 80.0


# =====================================================================
#  >>> LEVEL 1 <<<   Colour to greyscale
# ---------------------------------------------------------------------
#  Input : bgr_image, a NumPy array of shape (H, W, 3), dtype uint8
#  Output: a NumPy array of shape (H, W), dtype uint8
#
#  Maths:  Y = 0.299 R + 0.587 G + 0.114 B
#
#  Do NOT write the weighted sum by hand. Use the single OpenCV call
#  that performs a colour space conversion, with the BGR-to-GRAY flag.
#  Note the channel order: OpenCV stores images as B, G, R.
# =====================================================================

def to_grayscale(bgr_image):
    gray = None                      # <-- BLANK 1.1
    return gray


# =====================================================================
#  >>> LEVEL 2 <<<   Mean brightness
# ---------------------------------------------------------------------
#  Input : gray_image, shape (H, W), dtype uint8
#  Output: a single Python float
#
#  Maths:  Ibar = (1 / (H*W)) * sum over y sum over x of I(x, y)
#
#  Use the NumPy reduction, not a double for loop. Wrap the result in
#  float() so the return type is a plain Python float and not a NumPy
#  scalar.
# =====================================================================

def mean_brightness(gray_image):
    mean_value = None                # <-- BLANK 2.1
    return mean_value


# =====================================================================
#  >>> LEVEL 3 <<<   Smoothing
# ---------------------------------------------------------------------
#  Input : gray_image, shape (H, W), dtype uint8
#  Output: blurred image, same shape, same dtype
#
#  Maths (normalised box filter of size k x k, here k = 5):
#
#      I'(x, y) = (1 / k^2) * sum_{i=-a}^{a} sum_{j=-a}^{a} I(x+i, y+j)
#      with a = (k - 1) / 2
#
#  Use the plain averaging filter from OpenCV and pass BLUR_KERNEL.
#  Do not use the Gaussian version here, the marker checks exact pixel
#  values and the two filters give different numbers.
# =====================================================================

def smooth(gray_image):
    blurred = None                   # <-- BLANK 3.1
    return blurred


# =====================================================================
#  >>> LEVEL 4 <<<   Canny edge detection
# ---------------------------------------------------------------------
#  Input : gray_image, shape (H, W), dtype uint8
#  Output: binary edge map, shape (H, W), values 0 or 255
#
#  Maths: Sobel gradients Gx and Gy, then
#
#      |grad I| = sqrt(Gx^2 + Gy^2)
#      theta    = atan2(Gy, Gx)
#
#  followed by non-maximum suppression along theta and hysteresis with
#  two thresholds Tlow and Thigh:
#
#      |grad I| >= Thigh              -> strong edge, kept
#      Tlow <= |grad I| < Thigh       -> kept only if connected to a
#                                        strong edge
#      |grad I| < Tlow                -> discarded
#
#  Pass CANNY_LOW as Tlow and CANNY_HIGH as Thigh, in that order.
# =====================================================================

def find_edges(gray_image):
    edges = None                     # <-- BLANK 4.1
    return edges


# =====================================================================
#  >>> LEVEL 5 <<<   Green mask in HSV
# ---------------------------------------------------------------------
#  Input : bgr_image, shape (H, W, 3), dtype uint8
#  Output: binary mask, shape (H, W), values 0 or 255, dtype uint8
#
#  Two steps.
#
#  Step A, convert BGR to HSV. With V = max(R,G,B) and Cmin = min(R,G,B):
#
#      V = Cmax
#      S = (Cmax - Cmin) / Cmax     if Cmax > 0, else 0
#      H = 60 * ((G - B) / (Cmax - Cmin) mod 6)    if Cmax = R
#      H = 60 * ((B - R) / (Cmax - Cmin) + 2)      if Cmax = G
#      H = 60 * ((R - G) / (Cmax - Cmin) + 4)      if Cmax = B
#
#  In an 8 bit OpenCV image H is halved so that it fits in a byte:
#  H is in [0, 179], while S and V are in [0, 255].
#
#  Step B, threshold every channel at once:
#
#      M(x, y) = 255  if  l_c <= I_c(x, y) <= u_c  for all c in {H,S,V}
#      M(x, y) = 0    otherwise
#
#  Use HSV_GREEN_LOWER and HSV_GREEN_UPPER as the two bounds.
# =====================================================================

def green_mask(bgr_image):
    hsv = None                       # <-- BLANK 5.1
    mask = None                      # <-- BLANK 5.2
    return mask


# =====================================================================
#  >>> LEVEL 6 <<<   Largest blob
# ---------------------------------------------------------------------
#  Input : mask, binary image from level 5
#  Output: the single largest contour, or None if there is nothing
#          worth reporting
#
#  Three steps.
#
#  Step A, extract contours. Ask only for the outermost ones, because a
#  leaf with a hole in it should still count as one object, and use the
#  compression mode that stores only the end points of straight runs.
#
#  Step B, pick the contour with the greatest area. The area of a
#  polygon with vertices (x_i, y_i) is the shoelace formula
#
#      A = 0.5 * | sum_{i=0}^{n-1} ( x_i * y_{i+1} - x_{i+1} * y_i ) |
#
#  OpenCV already implements this, so call it instead of writing it.
#
#  Step C, reject noise. Return None if no contour was found at all, or
#  if the best area is smaller than MIN_BLOB_AREA.
# =====================================================================

def largest_blob(mask):
    contours = None                  # <-- BLANK 6.1 (contour extraction)

    if contours is None or len(contours) == 0:
        return None

    best = None                      # <-- BLANK 6.2 (pick the biggest)

    if None:                         # <-- BLANK 6.3 (area rejection test)
        return None

    return best


# =====================================================================
#  >>> LEVEL 7 <<<   Centroid from image moments
# ---------------------------------------------------------------------
#  Input : contour, as returned by level 6
#  Output: the tuple (cx, cy) as two floats, in pixels
#
#  Maths, the raw moments of order p + q are
#
#      M_pq = sum over x sum over y of ( x^p * y^q * I(x, y) )
#
#  so that M_00 is the area, and the centroid is
#
#      cx = M_10 / M_00 ,      cy = M_01 / M_00
#
#  Guard against M_00 = 0, a division by zero will crash the callback
#  and kill your node in the middle of a demonstration.
# =====================================================================

def blob_centroid(contour):
    moments = None                   # <-- BLANK 7.1

    if moments is None or moments["m00"] == 0:
        return None

    cx = None                        # <-- BLANK 7.2
    cy = None                        # <-- BLANK 7.3
    return (cx, cy)


# =====================================================================
#  >>> LEVEL 8 <<<   From pixels to a bearing
# ---------------------------------------------------------------------
#  Inputs : cx          horizontal centroid in pixels
#           image_width W, the image width in pixels
#           hfov_deg    the horizontal field of view in degrees
#  Output : the bearing to the object in degrees, as a float.
#           Negative means the object sits to the left of the optical
#           axis, positive means to the right, zero means dead ahead.
#
#  Maths, first the pixel error with respect to the image centre,
#
#      e = cx - W/2
#
#  then normalise it so that it does not depend on resolution,
#
#      ehat = 2e / W ,     ehat in [-1, +1]
#
#  and finally scale by the half field of view,
#
#      beta = ehat * (hfov / 2)
#
#  This is the small angle approximation of the exact pinhole result
#  beta = atan( (cx - cx0) / f ). It is good enough for steering a
#  differential drive robot and it needs no calibration matrix.
# =====================================================================

def bearing_deg(cx, image_width, hfov_deg):
    error_px = None                  # <-- BLANK 8.1
    normalised = None                # <-- BLANK 8.2
    bearing = None                   # <-- BLANK 8.3
    return bearing


# =====================================================================
#  Everything below this line is complete. Read it, do not edit it.
# =====================================================================

class LimoVisionNode(Node):
    """Subscribes to the LIMO camera and runs the pipeline you built."""

    def __init__(self):
        super().__init__("limo_vision")

        self.bridge = CvBridge()
        self.create_subscription(Image, CAMERA_TOPIC, self.camera_callback, 10)
        self.bearing_pub = self.create_publisher(Float32, "/limo/target_bearing", 10)

        for window in ("camera", "blur", "canny", "green mask", "detection"):
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)

        self.get_logger().info("limo_vision subscribing to %s" % CAMERA_TOPIC)

    def camera_callback(self, msg):
        # sensor_msgs/Image -> NumPy array in BGR order
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        gray = to_grayscale(frame)
        blurred = smooth(gray)
        edges = find_edges(gray)
        mask = green_mask(frame)

        overlay = frame.copy()
        contour = largest_blob(mask)

        if contour is not None:
            centre = blob_centroid(contour)
            if centre is not None:
                cx, cy = centre
                beta = bearing_deg(cx, frame.shape[1], CAMERA_HFOV_DEG)

                cv2.drawContours(overlay, [contour], -1, (0, 0, 255), 2)
                cv2.circle(overlay, (int(cx), int(cy)), 5, (255, 0, 255), -1)
                cv2.putText(
                    overlay,
                    "area %.0f px  bearing %+.1f deg" % (cv2.contourArea(contour), beta),
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )

                out = Float32()
                out.data = float(beta)
                self.bearing_pub.publish(out)

        cv2.putText(
            overlay,
            "mean brightness %.1f" % mean_brightness(gray),
            (10, overlay.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        small = lambda im: cv2.resize(im, (0, 0), fx=0.5, fy=0.5)
        cv2.imshow("camera", small(frame))
        cv2.imshow("blur", small(blurred))
        cv2.imshow("canny", small(edges))
        cv2.imshow("green mask", small(mask))
        cv2.imshow("detection", small(overlay))
        cv2.waitKey(1)


# ---------------------------------------------------------------------
#  The marker
# ---------------------------------------------------------------------

def reference_frame():
    """A fixed synthetic frame. Same pixels on every machine."""
    img = np.zeros((120, 160, 3), np.uint8)
    for x in range(160):
        img[:, x] = (40 + x // 8, 45 + x // 10, 50)     # background ramp
    cv2.rectangle(img, (20, 30), (69, 89), (60, 200, 70), -1)   # green target
    cv2.circle(img, (120, 60), 18, (200, 70, 60), -1)           # blue decoy
    return img


def _check_level_0():
    if not isinstance(CAMERA_TOPIC, str) or not CAMERA_TOPIC.startswith("/"):
        return False, "CAMERA_TOPIC must be a string starting with '/'"
    if CAMERA_TOPIC != "/limo_camera/image":
        return False, "that topic is not the LIMO colour camera"
    return True, "subscribing to %s" % CAMERA_TOPIC


def _check_level_1(frame):
    gray = to_grayscale(frame)
    if gray is None:
        return False, "nothing returned"
    if gray.ndim != 2 or gray.dtype != np.uint8:
        return False, "expected a 2D uint8 array, got %s %s" % (gray.shape, gray.dtype)
    if abs(float(gray.mean()) - 67.88) > 0.6:
        return False, "mean is %.2f, expected about 67.88 (wrong conversion flag?)" % gray.mean()
    if abs(int(gray[60, 45]) - 145) > 2:
        return False, "pixel (45,60) is %d, expected about 145" % gray[60, 45]
    return True, "greyscale mean %.2f" % gray.mean()


def _check_level_2(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    value = mean_brightness(gray)
    if value is None:
        return False, "nothing returned"
    if not isinstance(value, float):
        return False, "expected a Python float, got %s" % type(value).__name__
    if abs(value - 67.883385) > 0.01:
        return False, "got %.4f, expected 67.8834" % value
    return True, "mean brightness %.4f" % value


def _check_level_3(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = smooth(gray)
    if blurred is None:
        return False, "nothing returned"
    if blurred.shape != gray.shape or blurred.dtype != np.uint8:
        return False, "shape or dtype changed"
    if int(blurred.sum()) != 1303379:
        return False, "pixel sum is %d, expected 1303379 (wrong kernel or wrong filter)" % blurred.sum()
    if int(blurred[31, 21]) != 110:
        return False, "pixel (21,31) is %d, expected 110" % blurred[31, 21]
    return True, "5x5 box filter applied"


def _check_level_4(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = find_edges(gray)
    if edges is None:
        return False, "nothing returned"
    if edges.ndim != 2 or edges.dtype != np.uint8:
        return False, "expected a 2D uint8 array"
    if set(np.unique(edges)) - {0, 255}:
        return False, "edge map must contain only 0 and 255"
    count = int(np.count_nonzero(edges))
    if abs(count - 336) > 20:
        return False, "%d edge pixels, expected about 336 (did you pass CANNY_LOW and CANNY_HIGH?)" % count
    return True, "%d edge pixels" % count


def _check_level_5(frame):
    mask = green_mask(frame)
    if mask is None:
        return False, "nothing returned"
    if mask.ndim != 2 or mask.dtype != np.uint8:
        return False, "expected a 2D uint8 mask"
    if set(np.unique(mask)) - {0, 255}:
        return False, "mask must contain only 0 and 255"
    count = int(np.count_nonzero(mask))
    if abs(count - 3000) > 60:
        return False, "%d pixels selected, expected about 3000" % count
    if mask[60, 120] != 0:
        return False, "the blue decoy leaked into the mask"
    shaded = np.zeros((20, 20, 3), np.uint8)
    shaded[:, :] = (30, 120, 40)          # a green patch sitting in shadow
    if np.count_nonzero(green_mask(shaded)) < 390:
        return False, "a shaded green patch was missed, threshold in HSV and not in BGR"
    return True, "%d green pixels" % count


def _check_level_6(frame):
    mask = green_mask(frame)
    blob = largest_blob(mask)
    if blob is None:
        return False, "nothing returned for a frame that clearly has a green target"
    area = cv2.contourArea(blob)
    if abs(area - 2891.0) > 60:
        return False, "largest area is %.0f, expected about 2891" % area
    empty = largest_blob(np.zeros((120, 160), np.uint8))
    if empty is not None:
        return False, "an empty mask must return None"
    speck = np.zeros((120, 160), np.uint8)
    cv2.rectangle(speck, (10, 10), (17, 17), 255, -1)
    if largest_blob(speck) is not None:
        return False, "a 64 pixel speck is below MIN_BLOB_AREA and must return None"
    return True, "largest blob area %.0f px" % area


def _check_level_7(frame):
    blob = largest_blob(green_mask(frame))
    centre = blob_centroid(blob)
    if centre is None:
        return False, "nothing returned"
    cx, cy = centre
    if abs(cx - 44.5) > 1.5 or abs(cy - 59.5) > 1.5:
        return False, "centroid (%.1f, %.1f), expected about (44.5, 59.5)" % (cx, cy)
    return True, "centroid (%.2f, %.2f)" % (cx, cy)


def _check_level_8(_frame):
    cases = [((320.0, 640, 80.0), 0.0),
             ((640.0, 640, 80.0), 40.0),
             ((0.0, 640, 80.0), -40.0),
             ((480.0, 640, 80.0), 20.0),
             ((160.0, 320, 60.0), 0.0),
             ((240.0, 320, 60.0), 15.0)]
    for args, expected in cases:
        got = bearing_deg(*args)
        if got is None:
            return False, "nothing returned"
        if abs(float(got) - expected) > 1e-6:
            return False, "bearing_deg%s gave %.3f, expected %.3f" % (args, got, expected)
    return True, "bearing maps the image width onto +/- hfov/2"


LEVELS = [
    ("Camera topic", 1, lambda f: _check_level_0()),
    ("Greyscale conversion", 1, _check_level_1),
    ("Mean brightness", 1, _check_level_2),
    ("Box filter smoothing", 1, _check_level_3),
    ("Canny edges", 2, _check_level_4),
    ("HSV green mask", 2, _check_level_5),
    ("Largest blob", 2, _check_level_6),
    ("Centroid from moments", 2, _check_level_7),
    ("Pixel to bearing", 3, _check_level_8),
]


def run_marker():
    frame = reference_frame()
    total = sum(points for _, points, _ in LEVELS)
    scored = 0
    stopped_at = None

    print("")
    print("=" * 62)
    print(" LIMO vision pipeline, level check")
    print("=" * 62)

    for index, (name, points, check) in enumerate(LEVELS):
        if stopped_at is not None:
            print(" [ locked ] Level %d  %-26s  --/%d" % (index, name, points))
            continue
        try:
            passed, detail = check(frame)
        except NotImplementedError:
            passed, detail = False, "still a blank"
        except Exception as exc:
            passed, detail = False, "%s: %s" % (type(exc).__name__, exc)

        if passed:
            scored += points
            print(" [  pass  ] Level %d  %-26s  %d/%d   %s"
                  % (index, name, points, points, detail))
        else:
            stopped_at = index
            print(" [  FAIL  ] Level %d  %-26s   0/%d   %s"
                  % (index, name, points, detail))

    print("-" * 62)
    filled = int(round(20.0 * scored / total))
    print(" [%s%s]  %d / %d marks" % ("#" * filled, "." * (20 - filled), scored, total))

    if stopped_at is None:
        print(" All levels cleared. Start the simulator and run this file")
        print(" again without --check to see the pipeline on the robot.")
    else:
        print(" Level %d is blocking the rest. Fix it, then run the check again." % stopped_at)
        print(" Hints for that level are in the worksheet.")
    print("")
    return 0 if stopped_at is None else 1


def main(args=None):
    if not ROS_AVAILABLE:
        print("ROS 2 is not available in this environment.")
        print("Run 'python3 limo_vision_game.py --check' to work offline.")
        return

    rclpy.init(args=args)
    node = LimoVisionNode()
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
