"""
get_position.py
-----------------
Small helper to read the robot's CURRENT position and print it in a
copy-paste-ready format for the CONFIG section of mycobot_calibrate.py
/ mycobot_object_pick_place.py / mycobot_color_sort.py.

HOW TO USE
    1. Run this script:
           python3 get_position.py
       It automatically releases the servos on startup, so the arm
       goes limp and can be moved by hand right away.
    2. Physically move the arm to the position you want to record --
       e.g. your HOME position, or just above a bin.
    3. Press ENTER in the terminal to read and print that position.
       It prints the current joint angles AND coordinates, each
       already formatted as a Python list you can paste directly into
       HOME_ANGLES, BIN_ZONE, or DROP_ZONES.
    4. Keep the terminal open and repeat step 2-3 for each position
       you need (HOME, above each bin, etc). Ctrl+C to quit.

NOTE: reading position with get_angles()/get_coords() does NOT
re-lock the servos -- only sending a motion command does. So the arm
stays movable by hand for as many readings as you need in one run.
"""

import time

from pymycobot.mycobot280 import MyCobot280
from pymycobot import PI_PORT, PI_BAUD


def read_and_print(mc, label):
    angles = mc.get_angles()
    coords = mc.get_coords()

    print(f"\n--- {label} ---")

    if angles:
        print("Joint angles (for HOME_ANGLES):")
        print(f"  {angles}")
    else:
        print("Could not read angles (got empty response). Try again.")

    if coords:
        print("Coordinates (for BIN_ZONE / DROP_ZONES / anywhere needing x,y,z,rx,ry,rz):")
        print(f"  {coords}")
    else:
        print("Could not read coords (got empty response). Try again.")


def main():
    print("Connecting to robot...")
    mc = MyCobot280(PI_PORT, PI_BAUD)
    time.sleep(1)
    print("Connected.")

    print("Releasing servos so the arm can be moved by hand...")
    mc.release_all_servos()
    time.sleep(0.5)
    print("Servos released -- the arm should now move freely by hand.\n")

    print("Move the arm by hand to the position you want to record,")
    print("then press ENTER here to read it.")
    print("You can do this repeatedly for HOME, each bin, etc.")
    print("Ctrl+C to quit.\n")

    reading_number = 1
    try:
        while True:
            input(f"Position the arm, then press ENTER to read position #{reading_number}...")
            label = input("  Label this reading (e.g. 'HOME', 'bin - red', 'bin'): ").strip()
            if not label:
                label = f"reading #{reading_number}"
            read_and_print(mc, label)
            reading_number += 1
            print()
    except KeyboardInterrupt:
        print("\nDone.")


if __name__ == "__main__":
    main()