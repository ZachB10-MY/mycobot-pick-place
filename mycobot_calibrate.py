"""
mycobot_calibrate.py
---------------------
Run this ONCE before using mycobot_color_sort.py.

It builds a homography (pixel -> robot XY) mapping so the sorter knows
where a detected block actually is in robot coordinates.

HOW IT WORKS
1. This script opens your external USB camera.
2. You click 4+ points on the live image (spread out, covering the
   whole area where blocks will appear).
3. For each point you clicked, you physically jog the arm's gripper
   tip (using free-move / drag mode, or myStudio, or small scripted
   moves) until it is directly above that exact spot on the table,
   then read the coordinates off the robot and type them in when
   prompted.
4. The script fits a homography matrix and saves it to
   homography.npy in this folder. mycobot_color_sort.py loads that
   file automatically.

TIP: Use a piece of tape or a printed cross as a physical reference
point so you can precisely align the gripper tip with the same spot
you clicked in the image.

You do NOT need the robot connected to run the pixel-clicking part,
but you do need it connected to read out coordinates during the
prompt (or you can read them off myStudio / the app separately and
type them in here).
"""

import cv2
import numpy as np

CAMERA_INDEX = 0          # change if your USB camera isn't index 0
OUTPUT_FILE = "homography.npy"
MIN_POINTS = 4

clicked_points = []


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_points.append((x, y))
        print(f"Captured pixel point #{len(clicked_points)}: ({x}, {y})")


def main():
    global clicked_points
    clicked_points = []
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(
            f"Could not open camera at index {CAMERA_INDEX}. "
            "Try a different CAMERA_INDEX value."
        )

    window_name = "Calibration - click points, press 'q' when done"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    print("Click on 4 or more distinct points in the camera view.")
    print("Spread them across the corners/edges of your workspace.")
    print("Press 'q' once you have clicked all the points you want.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        display = frame.copy()
        for i, (px, py) in enumerate(clicked_points):
            cv2.circle(display, (px, py), 6, (0, 255, 0), -1)
            cv2.putText(
                display, str(i + 1), (px + 8, py - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
            )

        cv2.imshow(window_name, display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(clicked_points) < MIN_POINTS:
        raise RuntimeError(
            f"Need at least {MIN_POINTS} points, only got {len(clicked_points)}. "
            "Run the script again."
        )

    print(f"\nCollected {len(clicked_points)} pixel points.")
    print("Now enter the corresponding ROBOT coordinates for each point.")
    print("Jog/move the gripper tip directly above each spot on the table")
    print("(same physical order you clicked them), and read the X, Y")
    print("values off the robot (mc.get_coords() or myStudio's live readout).\n")

    robot_points = []
    for i, (px, py) in enumerate(clicked_points):
        print(f"--- Pixel point #{i + 1}: ({px}, {py}) ---")
        while True:
            try:
                x = float(input("  Robot X (mm): ").strip())
                y = float(input("  Robot Y (mm): ").strip())
                robot_points.append((x, y))
                break
            except ValueError:
                print("Invalid input. Please enter numeric values for X and Y.")
                continue

    pixel_pts = np.array(clicked_points, dtype=np.float32)
    robot_pts = np.array(robot_points, dtype=np.float32)

    H, status = cv2.findHomography(pixel_pts, robot_pts)
    if H is None:
        raise RuntimeError("Homography computation failed. Try again with more spread-out points.")

    np.save(OUTPUT_FILE, H)
    print(f"\nSaved homography matrix to {OUTPUT_FILE}")
    print("You can now run mycobot_color_sort.py")


if __name__ == "__main__":
    main()