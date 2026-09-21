"""
mycobot_color_sort.py
----------------------
myCobot 280 Pi (2023) + Adaptive Gripper + external USB camera.

Detects colored blocks on the workspace, and on a physical button
press, picks each block up and sorts it into a color-specific bin.

REQUIRES:
    - Run mycobot_calibrate.py first (produces homography.npy in the
      same folder as this script).
    - pymycobot, opencv-python, numpy, RPi.GPIO installed.
    - A push-button wired between BUTTON_PIN and GND (see "WIRING"
      section below).

    pip install pymycobot opencv-python numpy RPi.GPIO --break-system-packages

WIRING (physical button)
    - One leg of the button -> a free GPIO pin (default: GPIO17 / physical pin 11)
    - Other leg of the button -> GND (any ground pin, e.g. physical pin 9)
    - We use the Pi's internal pull-up resistor, so no external
      resistor is required. Pressing the button pulls the pin LOW.

USAGE
    python3 mycobot_color_sort.py

    The robot moves to HOME and waits. Press the button to run one
    full sort cycle: it looks at the camera, finds every colored
    block it recognizes, and picks/places them one at a time into
    their matching bin, then returns to HOME.
"""

import time
import threading

import cv2
import numpy as np

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False
    print("WARNING: RPi.GPIO not available. Button trigger disabled; "
          "press ENTER in the terminal instead to run a sort cycle.")

from pymycobot.mycobot280 import MyCobot280
from pymycobot import PI_PORT, PI_BAUD


# ============================== CONFIG ==============================

CAMERA_INDEX = 0            # external USB camera device index
HOMOGRAPHY_FILE = "homography.npy"

BUTTON_PIN = 17             # BCM numbering; physical pin 11
BUTTON_DEBOUNCE_MS = 300

MOVE_SPEED = 40
GRIPPER_SPEED = 50

HOVER_Z = 230               # safe travel height (mm), above all blocks
PICK_Z = 90                 # height to descend to for grasping (mm)
GRASP_ORIENTATION = [180, 0, 0]   # rx, ry, rz for a straight-down grip

MIN_BLOCK_AREA = 400        # pixel-area filter for noise rejection

# Home position, expressed as joint angles (degrees). Safe to change
# freely -- this is just wherever you want the arm to rest and start
# every cycle from. Jog the real arm to your preferred spot and read
# mc.get_angles() to get accurate values for your setup.
HOME_ANGLES = [0, 0, 0, 0, 0, 0]

# HSV color ranges to detect. Add/remove/tune entries as needed.
# Red wraps around hue 0/180 in OpenCV's HSV, so it gets two ranges.
COLOR_RANGES = {
    "red": [
        (np.array([0, 120, 70]),   np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255])),
    ],
    "green": [
        (np.array([40, 70, 70]), np.array([80, 255, 255])),
    ],
    "blue": [
        (np.array([100, 150, 0]), np.array([140, 255, 255])),
    ],
    "yellow": [
        (np.array([20, 100, 100]), np.array([35, 255, 255])),
    ],
}

# Drop-off coordinates for each color's bin: [x, y, z, rx, ry, rz] (mm / deg).
# These are placeholders -- jog the arm to just above each real bin,
# read mc.get_coords(), and replace these values.
DROP_ZONES = {
    "red":    [200, -150, HOVER_Z, 180, 0, 0],
    "green":  [200,  -50, HOVER_Z, 180, 0, 0],
    "blue":   [200,   50, HOVER_Z, 180, 0, 0],
    "yellow": [200,  150, HOVER_Z, 180, 0, 0],
}

# =====================================================================


class ColorSorter:
    def __init__(self):
        print("Connecting to robot...")
        self.mc = MyCobot280(PI_PORT, PI_BAUD)
        time.sleep(1)

        print("Loading calibration...")
        try:
            self.H = np.load(HOMOGRAPHY_FILE)
        except FileNotFoundError:
            raise RuntimeError(
                f"{HOMOGRAPHY_FILE} not found. Run mycobot_calibrate.py first."
            )

        print(f"Opening camera at index {CAMERA_INDEX}...")
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"Could not open camera at index {CAMERA_INDEX}. "
                "Check the USB connection or try a different index."
            )

        self._button_event = threading.Event()
        self._setup_button()

    # ---------------------------------------------------------------- #
    # Button handling
    # ---------------------------------------------------------------- #
    def _setup_button(self):
        if not GPIO_AVAILABLE:
            return
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(
            BUTTON_PIN, GPIO.FALLING,
            callback=self._on_button_press,
            bouncetime=BUTTON_DEBOUNCE_MS,
        )

    def _on_button_press(self, channel):
        print("\n[button] Pressed -- queuing a sort cycle.")
        self._button_event.set()

    def wait_for_trigger(self):
        """Blocks until the button is pressed (or ENTER, if no GPIO)."""
        if GPIO_AVAILABLE:
            self._button_event.wait()
            self._button_event.clear()
        else:
            input("Press ENTER to run a sort cycle...")

    # ---------------------------------------------------------------- #
    # Robot motion
    # ---------------------------------------------------------------- #
    def go_home(self):
        self.mc.send_angles(HOME_ANGLES, MOVE_SPEED)
        time.sleep(2)

    def open_gripper(self):
        self.mc.set_gripper_state(0, GRIPPER_SPEED)
        time.sleep(0.6)

    def close_gripper(self):
        self.mc.set_gripper_state(1, GRIPPER_SPEED)
        time.sleep(0.8)

    def pixel_to_robot(self, u, v):
        pt = np.array([u, v, 1.0])
        mapped = self.H @ pt
        mapped /= mapped[2]
        return float(mapped[0]), float(mapped[1])

    def pick_and_place(self, x, y, color):
        print(f"  -> picking {color} block at robot XY ({x:.1f}, {y:.1f})")

        # Move above the block, open gripper
        self.mc.send_coords([x, y, HOVER_Z] + GRASP_ORIENTATION, MOVE_SPEED, 1)
        time.sleep(1.5)
        self.open_gripper()

        # Descend and grasp
        self.mc.send_coords([x, y, PICK_Z] + GRASP_ORIENTATION, MOVE_SPEED, 1)
        time.sleep(1.5)
        self.close_gripper()

        # Lift back up
        self.mc.send_coords([x, y, HOVER_Z] + GRASP_ORIENTATION, MOVE_SPEED, 1)
        time.sleep(1.2)

        # Move to the matching bin and release
        drop = DROP_ZONES.get(color)
        if drop is None:
            print(f"  ! no drop zone configured for '{color}', skipping placement")
            return

        self.mc.send_coords(drop, MOVE_SPEED, 1)
        time.sleep(1.5)
        self.open_gripper()

        # Retreat upward before the next pick
        self.mc.send_coords(
            [drop[0], drop[1], HOVER_Z] + drop[3:], MOVE_SPEED, 1
        )
        time.sleep(1)

    # ---------------------------------------------------------------- #
    # Vision
    # ---------------------------------------------------------------- #
    def detect_blocks(self, frame):
        """Returns a list of (color_name, pixel_x, pixel_y, area)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        found = []

        for color_name, ranges in COLOR_RANGES.items():
            mask = None
            for lower, upper in ranges:
                m = cv2.inRange(hsv, lower, upper)
                mask = m if mask is None else cv2.bitwise_or(mask, m)

            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for c in contours:
                area = cv2.contourArea(c)
                if area < MIN_BLOCK_AREA:
                    continue
                M = cv2.moments(c)
                if M["m00"] == 0:
                    continue
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                found.append((color_name, cx, cy, area))

        return found

    def capture_frame(self):
        # Flush a couple of stale frames (common with USB cams / buffering)
        for _ in range(3):
            self.cap.read()
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame from camera.")
        return frame

    # ---------------------------------------------------------------- #
    # Main sort cycle
    # ---------------------------------------------------------------- #
    def run_sort_cycle(self):
        print("\n=== Sort cycle starting ===")
        self.go_home()  # make sure the arm is out of the camera's view

        frame = self.capture_frame()
        blocks = self.detect_blocks(frame)

        if not blocks:
            print("No blocks detected.")
            return

        print(f"Detected {len(blocks)} block(s):")
        for color_name, cx, cy, area in blocks:
            print(f"  - {color_name} at pixel ({cx}, {cy}), area={area:.0f}")

        for color_name, cx, cy, area in blocks:
            x, y = self.pixel_to_robot(cx, cy)
            self.pick_and_place(x, y, color_name)
            self.go_home()

        print("=== Sort cycle complete ===\n")

    def shutdown(self):
        self.cap.release()
        if GPIO_AVAILABLE:
            GPIO.cleanup()


def main():
    sorter = ColorSorter()
    try:
        print("Moving to HOME position...")
        sorter.go_home()
        print("Ready. Waiting for button press to start a sort cycle.")
        print("(Ctrl+C to quit)\n")

        while True:
            sorter.wait_for_trigger()
            sorter.run_sort_cycle()

    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        sorter.shutdown()


if __name__ == "__main__":
    main()