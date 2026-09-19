#!/usr/bin/env python3
# =====================================================================
#  KV6022 Robot Perception and Vision
#  Task 2 exercise, WORKED SOLUTION
#
#  Every blank from limo_vision_game.py is filled in below, with a
#  short note on why that particular call is the right one. The marker
#  is kept in this file too, so you can confirm it reports 15/15:
#
#      python3 limo_vision_solution.py --check
#
#  Live run:
#      ros2 launch limo_gazebosim limo_gazebo_diff.launch.py
#      python3 limo_vision_solution.py
# =====================================================================

import sys

import cv2
import numpy as np

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


# --- Level 0 ---------------------------------------------------------
# 'ros2 topic list' on the running simulation shows /limo_camera/image
# for the colour stream. /limo_camera/depth/image_raw is the depth
# stream and would fail the bgr8 conversion, and /camera from the
# original template does not exist at all.
CAMERA_TOPIC = "/limo_camera/image"

BLUR_KERNEL = (5, 5)
CANNY_LOW = 50
CANNY_HIGH = 150
HSV_GREEN_LOWER = np.array([40, 60, 60], dtype=np.uint8)
HSV_GREEN_UPPER = np.array([85, 255, 255], dtype=np.uint8)
MIN_BLOB_AREA = 200.0
CAMERA_HFOV_DEG = 80.0


# --- Level 1 ---------------------------------------------------------
def to_grayscale(bgr_image):
    # cvtColor applies Y = 0.299R + 0.587G + 0.114B in one pass.
    # COLOR_RGB2GRAY would swap the red and blue weights and give a
    # visibly different mean.
    gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    return gray


# --- Level 2 ---------------------------------------------------------
def mean_brightness(gray_image):
    # np.mean sums in float64 internally, so there is no uint8
    # overflow. float() strips the NumPy scalar wrapper.
    mean_value = float(np.mean(gray_image))
    return mean_value


# --- Level 3 ---------------------------------------------------------
def smooth(gray_image):
    # cv2.blur is the normalised box filter. cv2.GaussianBlur would
    # also smooth the image but weights the neighbours unequally, so
    # the pixel values would not match the expected ones.
    blurred = cv2.blur(gray_image, BLUR_KERNEL)
    return blurred


# --- Level 4 ---------------------------------------------------------
def find_edges(gray_image):
    # Argument order is (image, threshold1=low, threshold2=high).
    # Swapping them widens the hysteresis band and returns far more
    # weak edges.
    edges = cv2.Canny(gray_image, CANNY_LOW, CANNY_HIGH)
    return edges


# --- Level 5 ---------------------------------------------------------
def green_mask(bgr_image):
    # Hue separates colour from illumination, which raw BGR does not,
    # so a shaded leaf and a lit leaf still share a hue band.
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    # inRange tests all three channels at once and returns 0 or 255.
    mask = cv2.inRange(hsv, HSV_GREEN_LOWER, HSV_GREEN_UPPER)
    return mask


# --- Level 6 ---------------------------------------------------------
def largest_blob(mask):
    # RETR_EXTERNAL keeps only outer boundaries, so a hole inside a
    # leaf is not reported as a second object.
    # CHAIN_APPROX_SIMPLE stores end points instead of every pixel.
    # In OpenCV 4 findContours returns (contours, hierarchy).
    contours, _hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if contours is None or len(contours) == 0:
        return None

    best = max(contours, key=cv2.contourArea)

    if cv2.contourArea(best) < MIN_BLOB_AREA:
        return None

    return best


# --- Level 7 ---------------------------------------------------------
def blob_centroid(contour):
    moments = cv2.moments(contour)

    if moments is None or moments["m00"] == 0:
        return None

    cx = moments["m10"] / moments["m00"]
    cy = moments["m01"] / moments["m00"]
    return (cx, cy)


# --- Level 8 ---------------------------------------------------------
def bearing_deg(cx, image_width, hfov_deg):
    error_px = cx - image_width / 2.0
    normalised = 2.0 * error_px / image_width
    bearing = normalised * (hfov_deg / 2.0)
    return bearing


# =====================================================================
#  Live node
# =====================================================================

class LimoVisionNode(Node):
    """Subscribes to the LIMO camera and runs the pipeline."""

    def __init__(self):
        super().__init__("limo_vision")

        self.bridge = CvBridge()
        self.create_subscription(Image, CAMERA_TOPIC, self.camera_callback, 10)
        self.bearing_pub = self.create_publisher(Float32, "/limo/target_bearing", 10)

        for window in ("camera", "blur", "canny", "green mask", "detection"):
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)

        self.get_logger().info("limo_vision subscribing to %s" % CAMERA_TOPIC)

    def camera_callback(self, msg):
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


# =====================================================================
#  Marker, identical to the one in limo_vision_game.py
# =====================================================================

def reference_frame():
    img = np.zeros((120, 160, 3), np.uint8)
    for x in range(160):
        img[:, x] = (40 + x // 8, 45 + x // 10, 50)
    cv2.rectangle(img, (20, 30), (69, 89), (60, 200, 70), -1)
    cv2.circle(img, (120, 60), 18, (200, 70, 60), -1)
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
        print("Run 'python3 limo_vision_solution.py --check' to work offline.")
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
