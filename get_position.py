"""
get_position.py
-----------------
Small helper to read the robot's CURRENT position and print it in a
copy-paste-ready format for the CONFIG section of mycobot_calibrate.py
/ mycobot_object_pick_place.py / mycobot_color_sort.py.

HOW TO USE
    1. Physically move the arm (drag-teach / free-move mode, or jog it
       with myStudio, or run small scripted moves) to the position you
       want to record -- e.g. your HOME position, or just above a bin.
    2. Run this script:
           python3 get_position.py
    3. It prints the current joint angles AND coordinates, each
       already formatted as a Python list you can paste directly into
       HOME_ANGLES, BIN_ZONE, or DROP_ZONES.
    4. Keep the terminal open and press ENTER to take another reading
       (e.g. move the arm above a different bin) without restarting
       the script. Ctrl+C to quit.

NOTE: if the arm is in a "locked"/powered state, it may resist being
moved by hand. Many myCobot setups let you release the servos briefly
for manual positioning -- check your specific release/free-move method
if the arm feels stiff.
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
    print("Connected.\n")

    print("Move the arm (by hand, myStudio, or however you jog it) to the")
    print("position you want to record, then press ENTER here to read it.")
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