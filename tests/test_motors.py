#!/home/mie_g28/venv/bin/python3
"""
test_motors.py
──────────────
Standalone gantry motor test — NO ROS 2 required.

Run directly on the Pi to verify Motor 1 (servo, theta) and
Motor 2 (stepper, phi) are wired and working before launching
any ROS 2 nodes.

Usage:
  python3 tests/test_motors.py

What it does:
  1. Motor 1 (theta) — servo sweeps 0° → 150° → 300° → 0°.
  2. Motor 2 (phi)   — stepper 600 steps forward, pause, 600 steps back.
  3. Both simultaneously — servo sweeps while stepper moves.

Troubleshooting:
  Servo doesn't move      → Check 24V power, PWM signal on BCM 13 (Pin 33).
  Servo jitters           → Check PWM signal quality, ensure good ground connection.
  Stepper doesn't move    → Check EN pin is LOW, VMOT connected,
                            check TB6600 DIP switches.
  Stepper wrong direction → Swap one coil pair (A+↔A− or B+↔B− on TB6600).
"""

import RPi.GPIO as GPIO
import time
import threading

# ── Pin assignments — must match motor_controller.py ──────────────────────────
SERVO_PIN = 13                          # Motor 1 — theta (Z rotation)
M2_STEP = 23;  M2_DIR = 24;  M2_EN = 25  # Motor 2 — phi (arc position)

# ── Servo constants ───────────────────────────────────────────────────────────
SERVO_FREQ      = 50
SERVO_MIN_PULSE = 0.5
SERVO_MAX_PULSE = 2.5
SERVO_MAX_ANGLE = 300.0

# ── Stepper constants ─────────────────────────────────────────────────────────
STEP_DELAY  = 0.002
M2_STEPS    = 600
PAUSE_S     = 1.0


def angle_to_duty(angle):
    angle = max(0.0, min(SERVO_MAX_ANGLE, angle))
    pulse_ms = SERVO_MIN_PULSE + (angle / SERVO_MAX_ANGLE) * \
               (SERVO_MAX_PULSE - SERVO_MIN_PULSE)
    return (pulse_ms / (1000.0 / SERVO_FREQ)) * 100.0


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    for pin in [M2_STEP, M2_DIR, M2_EN]:
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(M2_EN, GPIO.LOW)
    print("GPIO initialised. Servo and stepper driver enabled.")


def servo_move(angle, label=""):
    print(f"  {label}: moving to {angle}°...")
    pwm.ChangeDutyCycle(angle_to_duty(angle))
    time.sleep(0.5)
    print(f"  {label}: done.")


def stepper_move(steps, label=""):
    direction = "forward" if steps > 0 else "backward"
    print(f"  {label}: {abs(steps)} steps {direction}...")
    GPIO.output(M2_DIR, GPIO.HIGH if steps > 0 else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(M2_STEP, GPIO.HIGH)
        time.sleep(STEP_DELAY)
        GPIO.output(M2_STEP, GPIO.LOW)
        time.sleep(STEP_DELAY)
    print(f"  {label}: done.")


def main():
    global pwm
    print("═" * 60)
    print("  ScanToCAD — Gantry Motor Test")
    print("═" * 60)

    try:
        setup()
        pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
        pwm.start(0)
        time.sleep(0.5)

        print("\n── Test 1: Motor 1 (theta — servo Z rotation) ───────────────")
        print("  Expected: servo moves 0° → 90° → 0°.")
        servo_move(0,  "Motor 1 (theta)")
        time.sleep(PAUSE_S)
        servo_move(90, "Motor 1 (theta)")
        time.sleep(PAUSE_S)
        servo_move(0,  "Motor 1 (theta)")
        time.sleep(PAUSE_S)

        print("\n── Test 2: Motor 2 (phi — stepper arc position) ─────────────")
        print("  Expected: head moves along arc, pauses, returns.")
        stepper_move( M2_STEPS, "Motor 2 (phi)")
        time.sleep(PAUSE_S)
        stepper_move(-M2_STEPS, "Motor 2 (phi)")
        time.sleep(PAUSE_S)

        print("\n── Test 3: Both motors simultaneously ────────────────────────")
        print("  Expected: servo moves to 90° while stepper moves forward.")
        t1 = threading.Thread(target=servo_move,   args=(90, "Motor 1 (theta)"))
        t2 = threading.Thread(target=stepper_move, args=(M2_STEPS, "Motor 2 (phi)"))
        t1.start(); t2.start()
        t1.join();  t2.join()
        time.sleep(PAUSE_S)
        servo_move(0,            "Motor 1 (theta)")
        stepper_move(-M2_STEPS,  "Motor 2 (phi)")

        print("\n✓ All motor tests complete.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        pwm.stop()
        GPIO.output(M2_EN, GPIO.HIGH)
        GPIO.cleanup()
        print("GPIO cleaned up.")


if __name__ == '__main__':
    main()
