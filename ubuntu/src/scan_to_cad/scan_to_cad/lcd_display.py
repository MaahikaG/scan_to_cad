"""
lcd_display.py
──────────────
Drives the 16x2 LCD in 4-bit PARALLEL mode (no I2C backpack).

WIRING:
  LCD Pin 1  VSS        → Pi GND
  LCD Pin 2  VDD        → Pi 5V (Pin 2)
  LCD Pin 3  V0         → middle wiper of 10kΩ contrast potentiometer
                          (pot outer legs between 5V and GND)
  LCD Pin 4  RS         → Pi BCM 26 (Physical Pin 37)
  LCD Pin 5  RW         → Pi GND  (always write mode)
  LCD Pin 6  E          → Pi BCM 18 (Physical Pin 12)
  LCD Pin 7  D0         → not connected
  LCD Pin 8  D1         → not connected
  LCD Pin 9  D2         → not connected
  LCD Pin 10 D3         → not connected
  LCD Pin 11 D4         → Pi BCM 4  (Physical Pin 7)
  LCD Pin 12 D5         → Pi BCM 9  (Physical Pin 21)
  LCD Pin 13 D6         → Pi BCM 10 (Physical Pin 19)
  LCD Pin 14 D7         → Pi BCM 11 (Physical Pin 23)
  LCD Pin 15 A (bklt+)  → Pi 3.3V via 220Ω resistor
  LCD Pin 16 K (bklt-)  → Pi GND

CONTRAST:
  If the screen is blank or shows solid black squares, adjust the
  potentiometer on pin 3 slowly until characters appear.

DISPLAY LAYOUT (16x2):
  Row 0:  T:000.0 P:000.0    (gantry theta and phi in degrees from /odom)
  Row 1:  a:000.0 b:000.0    (pan alpha and tilt beta in degrees from /pan_tilt/angles)

Topics subscribed:
  /odom              (nav_msgs/Odometry)       — twist.linear.x=θ, twist.linear.y=φ
  /pan_tilt/angles   (geometry_msgs/Vector3)   — x=α (pan), y=β (tilt)

Install library:
  pip install RPLCD
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from RPLCD.gpio import CharLCD
try:
    import RPi.GPIO as GPIO
except ImportError:
    from unittest.mock import MagicMock
    GPIO = MagicMock()

# ── GPIO pin assignments (BCM numbering) ──────────────────────────────────────
LCD_RS = 26
LCD_E  = 18
LCD_D4 = 4
LCD_D5 = 9
LCD_D6 = 10
LCD_D7 = 11

LCD_COLS = 16
LCD_ROWS = 2


class LCDDisplay(Node):
    def __init__(self):
        super().__init__('lcd_display')

        # ── Initialise LCD in 4-bit parallel mode ─────────────────────────────
        self.lcd = CharLCD(
            numbering_mode=GPIO.BCM,
            cols=LCD_COLS,
            rows=LCD_ROWS,
            pin_rs=LCD_RS,
            pin_e=LCD_E,
            pins_data=[LCD_D4, LCD_D5, LCD_D6, LCD_D7],
            compat_mode=True,
        )
        self.lcd.clear()

        # ── Cached display values ─────────────────────────────────────────────
        self.theta_deg = 0.0   # gantry θ from /odom
        self.phi_deg   = 0.0   # gantry φ from /odom
        self.alpha_deg = 0.0   # pan α from /pan_tilt/angles
        self.beta_deg  = 0.0   # tilt β from /pan_tilt/angles

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom',            self.odom_cb,      10)
        self.create_subscription(Vector3,  '/pan_tilt/angles', self.pan_tilt_cb,  10)

        # Refresh at 2 Hz — LCD writes are slow, going faster causes glitches
        self.create_timer(0.5, self.update_display)

        self.get_logger().info('LCD display ready (parallel 4-bit mode)')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        self.theta_deg = msg.twist.twist.linear.x
        self.phi_deg   = msg.twist.twist.linear.y

    def pan_tilt_cb(self, msg: Vector3):
        self.alpha_deg = msg.x
        self.beta_deg  = msg.y

    # ── Display update ────────────────────────────────────────────────────────

    def update_display(self):
        # Row 0: gantry θ and φ
        line1 = f'T:{self.theta_deg:6.1f} P:{self.phi_deg:6.1f}'
        # Row 1: pan α and tilt β
        line2 = f'a:{self.alpha_deg:6.1f} b:{self.beta_deg:6.1f}'

        self.lcd.cursor_pos = (0, 0)
        self.lcd.write_string(line1[:LCD_COLS].ljust(LCD_COLS))
        self.lcd.cursor_pos = (1, 0)
        self.lcd.write_string(line2[:LCD_COLS].ljust(LCD_COLS))

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.lcd.clear()
        self.lcd.close(clear=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LCDDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()