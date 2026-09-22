"""
mycobot_object_pick_place.py
------------------------------
myCobot 280 Pi (2023) + Adaptive Gripper + external USB camera.

Detects ANY object placed on the workspace (regardless of color) via
background subtraction, and on a physical button press, picks it up
and places it into a single bin.

REQUIRES:
    - Run mycobot_calibrate.py first (produces homography.npy).
    - Run this script's background capture step FIRST with an EMPTY
      workspace (see step 1 below) so it knows what "nothing there"
      looks like.
    - pymycobot, opencv-python, numpy, RPi.GPIO installed.
    - A push-button wired between BUTTON_PIN and GND.

    pip install pymycobot opencv-python numpy RPi.GPIO --break-system-packages

WIRING (physical button)
    - One leg -> GPIO17 (physical pin 11)
    - Other leg -> GND (physical pin 9)
    - Internal pull-up used, no external resistor needed.

USAGE
    python3 mycobot_object_pick_place.py

    On first run with an empty workspace, it captures a background
    reference frame automatically before the main loop starts. Then:
    the robot moves to HOME and waits. Place an object anywhere in
    the grid, press the button, and it finds it, picks it up, and
    drops it in the bin.
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
          "press ENTER in the terminal instead to run a pick cycle.")

from pymycobot.mycobot280 import MyCobot280
from pymycobot import PI_PORT, PI_BAUD


# ============================== CONFIG ==============================

CAMERA_INDEX = 0
HOMOGRAPHY_FILE = "homography.npy"

BUTTON_PIN = 17
BUTTON_DEBOUNCE_MS = 300

MOVE_SPEED = 40
GRIPPER_SPEED = 50

HOVER_Z = 230
PICK_Z = 90
GRASP_ORIENTATION = [180, 0, 0]

MIN_BLOCK_AREA = 400          # ignore blobs smaller than this (noise)
DIFF_THRESHOLD = 30           # sensitivity of background subtraction

# Home position (joint angles, degrees). Jog the arm to your preferred
# resting spot and read mc.get_angles() to fill this in accurately.
HOME_ANGLES = [0, 0, 0, 0, 0, 0]

# Single bin location: [x, y, z, rx, ry, rz] (mm / deg). Jog the arm
# above your bin and read mc.get_coords() to fill this in.
BIN_ZONE = [200, 150, HOVER_Z, 180, 0, 0]

# =====================================================================


class ObjectPicker:
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
                f"Could not open camera at index {CAMERA_INDEX}."
            )

        self.background = None
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
        print("\n[button] Pressed -- queuing a pick cycle.")
        self._button_event.set()

    def wait_for_trigger(self):
        if GPIO_AVAILABLE:
            self._button_event.wait()
            self._button_event.clear()
        else:
            input("Press ENTER to run a pick cycle...")

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

    def pick_and_place(self, x, y):
        print(f"  -> picking object at robot XY ({x:.1f}, {y:.1f})")

        self.mc.send_coords([x, y, HOVER_Z] + GRASP_ORIENTATION, MOVE_SPEED, 1)
        time.sleep(1.5)
        self.open_gripper()

        self.mc.send_coords([x, y, PICK_Z] + GRASP_ORIENTATION, MOVE_SPEED, 1)
        time.sleep(1.5)
        self.close_gripper()

        self.mc.send_coords([x, y, HOVER_Z] + GRASP_ORIENTATION, MOVE_SPEED, 1)
        time.sleep(1.2)

        self.mc.send_coords(BIN_ZONE, MOVE_SPEED, 1)
        time.sleep(1.5)
        self.open_gripper()

        self.mc.send_coords(
            [BIN_ZONE[0], BIN_ZONE[1], HOVER_Z] + BIN_ZONE[3:], MOVE_SPEED, 1
        )
        time.sleep(1)

    # ---------------------------------------------------------------- #
    # Vision
    # ---------------------------------------------------------------- #
    def capture_frame(self):
        for _ in range(3):
            self.cap.read()
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame from camera.")
        return frame

    def capture_background(self):
        """Call this once with an empty workspace before starting."""
        print("Capturing background reference (make sure the workspace is empty)...")
        frame = self.capture_frame()
        self.background = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.background = cv2.GaussianBlur(self.background, (21, 21), 0)
        print("Background captured.")

    def detect_object(self, frame):
        """Returns (pixel_x, pixel_y) of the largest new object, or None."""
        if self.background is None:
            raise RuntimeError("Background not captured yet. Call capture_background() first.")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        diff = cv2.absdiff(self.background, gray)
        _, mask = cv2.threshold(diff, DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < MIN_BLOCK_AREA:
            return None

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy

    # ---------------------------------------------------------------- #
    # Main cycle
    # ---------------------------------------------------------------- #
    def run_pick_cycle(self):
        print("\n=== Pick cycle starting ===")
        self.go_home()  # keep the arm out of the camera's view

        frame = self.capture_frame()
        result = self.detect_object(frame)

        if result is None:
            print("No object detected.")
            return

        cx, cy = result
        print(f"Object found at pixel ({cx}, {cy})")

        x, y = self.pixel_to_robot(cx, cy)
        self.pick_and_place(x, y)
        self.go_home()

        print("=== Pick cycle complete ===\n")

    def shutdown(self):
        self.cap.release()
        if GPIO_AVAILABLE:
            GPIO.cleanup()


def main():
    picker = ObjectPicker()
    try:
        print("Moving to HOME position...")
        picker.go_home()

        input("Make sure the workspace is EMPTY, then press ENTER to capture background...")
        picker.capture_background()

        print("\nReady. Place an object anywhere in the grid, then press the button.")
        print("(Ctrl+C to quit)\n")

        while True:
            picker.wait_for_trigger()
            picker.run_pick_cycle()

    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        picker.shutdown()


if __name__ == "__main__":
    main()