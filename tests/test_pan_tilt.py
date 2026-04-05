#!/home/mie_g28/venv/bin/python3
"""
test_pan_tilt.py
────────────────
Standalone pan-tilt motor test — NO ROS 2 required.

Tests Motor A (theta, Z rotation) and Motor B (phi, arc position)
independently. Motors are mechanically independent at 1:1 ratio.

Usage:
  python3 tests/test_pan_tilt.py

Pin assignments must match pan_tilt_controller.py.

Troubleshooting:
  Motor doesn't move      → Check EN pin is LOW, VMOT connected,
                             SLEEP+RESET tied together on TB6600.
  Wrong direction         → Swap one coil pair (A+↔A− or B+↔B− on TB6600).
  Skips steps / stalls    → Increase STEP_DELAY, or adjust TB6600 current
                             limit DIP switches.
"""

import RPi.GPIO as GPIO
import time
import threading

# ── Pin assignments — must match pan_tilt_controller.py ──────────────────────
PA_STEP = 16;  PA_DIR = 20;  PA_EN = 21    # Motor A — theta (Z rotation)
PB_STEP = 12;  PB_DIR = 7;   PB_EN = 8    # Motor B — phi   (arc position)

STEP_DELAY = 0.002      # slower than production — safer for first test
TEST_STEPS = 400
PAUSE_S    = 1.0


def setup():
    GPIO.setmode(GPIO.BCM)
    for pin in [PA_STEP, PA_DIR, PA_EN, PB_STEP, PB_DIR, PB_EN]:
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(PA_EN, GPIO.LOW)
    GPIO.output(PB_EN, GPIO.LOW)
    print("GPIO initialised. Both pan-tilt drivers enabled.")


def move(step_pin, dir_pin, steps, label=""):
    direction = "forward" if steps > 0 else "backward"
    print(f"  {label}: {abs(steps)} steps {direction}...")
    GPIO.output(dir_pin, GPIO.HIGH if steps > 0 else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(STEP_DELAY)
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(STEP_DELAY)
    print(f"  {label}: done.")


def move_both(steps_a, steps_b):
    t_a = threading.Thread(target=move, args=(PA_STEP, PA_DIR, steps_a, "Motor A (theta)"))
    t_b = threading.Thread(target=move, args=(PB_STEP, PB_DIR, steps_b, "Motor B (phi)"))
    t_a.start(); t_b.start()
    t_a.join();  t_b.join()


def main():
    print("═" * 60)
    print("  ScanToCAD — Pan-Tilt Motor Test")
    print(f"  Steps: {TEST_STEPS}  |  Delay: {STEP_DELAY*1000:.1f} ms/edge")
    print("═" * 60)

    try:
        setup()
        time.sleep(0.5)

        print("\n── Test 1: Motor A (theta — Z rotation) ─────────────────────")
        print("  Expected: pan-tilt head rotates CCW, pauses, returns CW.")
        move(PA_STEP, PA_DIR,  TEST_STEPS, "Motor A (theta)")
        time.sleep(PAUSE_S)
        move(PA_STEP, PA_DIR, -TEST_STEPS, "Motor A (theta)")
        time.sleep(PAUSE_S)

        print("\n── Test 2: Motor B (phi — arc position) ──────────────────────")
        print("  Expected: pan-tilt head moves along arc toward phi=90, pauses, returns.")
        move(PB_STEP, PB_DIR,  TEST_STEPS, "Motor B (phi)")
        time.sleep(PAUSE_S)
        move(PB_STEP, PB_DIR, -TEST_STEPS, "Motor B (phi)")
        time.sleep(PAUSE_S)

        print("\n── Test 3: Both motors simultaneously ────────────────────────")
        print("  Expected: both axes move forward together, then both return.")
        move_both( TEST_STEPS,  TEST_STEPS)
        time.sleep(PAUSE_S)
        move_both(-TEST_STEPS, -TEST_STEPS)

        print("\n✓ All pan-tilt tests complete.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        GPIO.output(PA_EN, GPIO.HIGH)
        GPIO.output(PB_EN, GPIO.HIGH)
        GPIO.cleanup()
        print("GPIO cleaned up.")


if __name__ == '__main__':
    main()
