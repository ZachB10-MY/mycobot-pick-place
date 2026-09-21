# myCobot 280 Pi — Color Block Sorter

Plain Python (no ROS) color-sorting demo for the myCobot 280 Pi 2023 with
the adaptive gripper, an external USB camera, a fixed HOME position, and
a physical push-button trigger.

## Files
- `mycobot_calibrate.py` — run once to map camera pixels to robot coordinates.
- `mycobot_color_sort.py` — the main detect → pick → sort loop.

## Hardware setup
1. **Camera**: mount it overhead, looking straight down at the workspace.
   Fix its position — if it moves, you'll need to recalibrate.
2. **Workspace surface**: a plain, matte, contrasting-color mat helps a
   lot with color thresholding. Avoid glare and harsh shadows.
3. **Button**: wire one leg to a free GPIO pin (default `GPIO17`,
   physical pin 11) and the other leg to any ground pin (e.g. physical
   pin 9). The script uses the Pi's internal pull-up resistor, so no
   external resistor is needed — pressing the button pulls the pin low.
4. **Bins**: place 4 bins/zones within reach for red / green / blue /
   yellow.

## Install dependencies
```bash
pip install pymycobot opencv-python numpy RPi.GPIO --break-system-packages
```

## Step 1 — Calibrate
```bash
python3 mycobot_calibrate.py
```
- Click 4+ points spread across the camera's view of the workspace.
- For each clicked point, physically move the gripper tip directly
  above that same spot (drag-teach mode, myStudio, or small scripted
  moves), then read off the robot's real X/Y (`mc.get_coords()`) and
  type it in when prompted.
- This saves `homography.npy`, which the main script needs.

## Step 2 — Tune constants in `mycobot_color_sort.py`
Before running for real, adjust:
- `HOME_ANGLES` — jog the arm to wherever you want it to rest/start,
  then read `mc.get_angles()` and paste the values in.
- `DROP_ZONES` — jog the arm to just above each bin, read
  `mc.get_coords()`, and paste those in per color.
- `HOVER_Z` / `PICK_Z` — safe travel height and how far down to
  descend to actually grasp a block (depends on block height and your
  mat).
- `COLOR_RANGES` — the built-in HSV ranges are a reasonable starting
  point, but lighting varies a lot. If detection is unreliable, tune
  these using a simple trackbar/HSV-picker script.
- `BUTTON_PIN` — change if you wired the button to a different GPIO.

## Step 3 — Run
```bash
python3 mycobot_color_sort.py
```
- The arm moves to HOME and waits.
- Press the button: it captures a frame, finds every recognized
  colored block, and picks/places them one at a time into the
  matching bin, then returns to HOME.
- If `RPi.GPIO` isn't available (e.g. testing off the Pi), the script
  falls back to pressing ENTER in the terminal instead.

## Notes / gotchas
- The arm returns to HOME before capturing each frame and between
  picks, so it doesn't occlude the camera or get mistaken for an
  object.
- All blocks are assumed to be the same height (fixed `PICK_Z`). If
  your blocks vary in height, you'll need per-block depth handling.
- Grasp orientation is fixed straight-down (`GRASP_ORIENTATION`). Fine
  for small blocks; for angle-aware grasping you'd want ArUco markers
  instead of plain color blobs.
- If two blocks of the same color overlap or touch, they may be
  detected as one blob — spread blocks out for reliable results.