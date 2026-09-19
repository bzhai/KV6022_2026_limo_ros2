#!/usr/bin/env python3
# =====================================================================
#  KV6022 Robot Perception and Vision
#  Task 3, WORKED SOLUTION for the colour contour detector
#
#  Every blank from colour_contours_game.py is filled in, with a note
#  on why that call and not the nearby alternative. Confirm 18/18 with
#
#      python3 colour_contours_solution.py --check
#
#  Live:
#      ros2 launch limo_gazebosim limo_gazebo_diff.launch.py
#      python3 colour_contours_solution.py
#      ros2 topic echo /object_polygon
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

OPEN_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

MIN_AREA = 100.0
MIN_EXTENT = 0.25


# --- Level 0 ---------------------------------------------------------
# Green sits at roughly 100 to 140 degrees of hue, which is 50 to 70 in
# OpenCV's halved scale. The band 35 to 85 leaves room for yellowish
# and bluish greens while staying clear of yellow at 30 and cyan at 90.
# The saturation floor of 40 throws out grey and white, which have no
# meaningful hue at all, and the value floor of 40 throws out anything
# almost black, where the hue is pure numerical noise.
HSV_LOWER = np.array((35, 40, 40), np.uint8)
HSV_UPPER = np.array((85, 255, 255), np.uint8)


# --- Level 1 ---------------------------------------------------------
def to_hsv(bgr_image):
    # COLOR_RGB2HSV would treat the blue channel as red and shift every
    # hue by roughly 120 degrees, so greens would land where blues are.
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    return hsv_image


# --- Level 2 ---------------------------------------------------------
def colour_mask(hsv_image):
    # inRange tests all three channels and returns 0 or 255 directly,
    # so no comparison chain and no np.where is needed.
    mask = cv2.inRange(hsv_image, HSV_LOWER, HSV_UPPER)
    return mask


# --- Level 3 ---------------------------------------------------------
def clean_mask(mask):
    # Opening first: erosion deletes the isolated pixels, dilation then
    # restores the surviving objects to their original size.
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_KERNEL)
    # Closing second: dilation bridges the small holes, erosion pulls
    # the outer boundary back in. Doing it the other way round would
    # first inflate the specks and make them harder to remove.
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, CLOSE_KERNEL)
    return closed


# --- Level 4 ---------------------------------------------------------
def find_blob_contours(mask):
    # RETR_EXTERNAL returns outer boundaries only. RETR_TREE, as used in
    # the original node, also returns the boundary of each hole, which
    # is why a hollow shape appeared there as two objects.
    # CHAIN_APPROX_SIMPLE keeps only segment end points: a rectangle
    # becomes 4 points instead of the full perimeter.
    contours, _hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    return contours


# --- Level 5 ---------------------------------------------------------
def is_big_enough(contour):
    keep = cv2.contourArea(contour) >= MIN_AREA
    return keep


# --- Level 6 ---------------------------------------------------------
def bounding_box(contour):
    # Returns four ints: x, y of the top left corner, then width and
    # height. cv2.minAreaRect would give a rotated box instead, which
    # does not fit the message layout this node publishes.
    box = cv2.boundingRect(contour)
    return box


# --- Level 7 ---------------------------------------------------------
def box_to_polygon_points(box):
    x, y, w, h = box
    # Corner, then size. This mirrors the original node exactly, so an
    # existing subscriber keeps working.
    first_point = (float(x), float(y))
    second_point = (float(w), float(h))
    return (first_point, second_point)


# --- Level 8 ---------------------------------------------------------
def extent(contour):
    area = cv2.contourArea(contour)
    x, y, w, h = bounding_box(contour)

    if w == 0 or h == 0:
        return 0.0

    ratio = float(area) / float(w * h)
    return ratio


# --- Level 9 ---------------------------------------------------------
def detect_objects(bgr_image):
    hsv_image = to_hsv(bgr_image)
    mask = colour_mask(hsv_image)
    cleaned = clean_mask(mask)
    contours = find_blob_contours(cleaned)

    objects = []
    for contour in contours:
        # Cheap test first: the area gate rejects most speckle before
        # the extent calculation has to run boundingRect on it.
        if not is_big_enough(contour) or extent(contour) < MIN_EXTENT:
            continue
        objects.append({
            "contour": contour,
            "box": bounding_box(contour),
            "area": float(cv2.contourArea(contour)),
            "extent": float(extent(contour)),
        })

    objects = sorted(objects, key=lambda entry: entry["area"], reverse=True)
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
