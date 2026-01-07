#!/usr/bin/env python3
import os, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import gpiod

# Robot parameters
WHEEL_DIAMETER = 0.10   # meters
WHEEL_RADIUS   = WHEEL_DIAMETER / 2
WHEELBASE      = 0.33   # meters (13 inch)

MAX_LINEAR = 1.0   # m/s (tune)
PERIOD_NS  = 50_000  # 20 kHz

# GPIO pins
M1_EN, M2_EN = 22, 23
M1_DIR, M2_DIR = 24, 25

PWMCHIP = "pwmchip0"
PWM0 = f"/sys/class/pwm/{PWMCHIP}/pwm0"  # Motor1
PWM1 = f"/sys/class/pwm/{PWMCHIP}/pwm1"  # Motor2

def w(path, val):
    with open(path, "w") as f:
        f.write(str(val) + "\n")

def set_pwm(path, duty_frac):
    if duty_frac < 0: duty_frac = 0
    if duty_frac > 1: duty_frac = 1
    duty = int(PERIOD_NS * duty_frac)
    w(f"{path}/period", PERIOD_NS)
    w(f"{path}/duty_cycle", duty)
    w(f"{path}/enable", 1)

def stop_pwm(path):
    w(f"{path}/enable", 0)

class DiffDrivePWM(Node):
    def __init__(self):
        super().__init__('diffdrive_pwm')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        # Setup GPIO lines
        self.chip = gpiod.Chip("gpiochip4")
        self.lines = self.chip.get_lines([M1_EN, M2_EN, M1_DIR, M2_DIR])
        self.lines.request(consumer="ros2_diffdrive", type=gpiod.LINE_REQ_DIR_OUT)
        self.lines.set_values([1,1,0,0])  # enable drivers

        # Init PWM
        for pwm in [PWM0, PWM1]:
            if not os.path.isdir(pwm):
                with open(f"/sys/class/pwm/{PWMCHIP}/export","w") as f:
                    f.write(pwm[-1])  # export channel

        self.get_logger().info("DiffDrivePWM node ready.")

    def cmd_callback(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z

        v_left  = v - (omega * WHEELBASE / 2)
        v_right = v + (omega * WHEELBASE / 2)

        # Scale to duty [-1,1]
        duty_left  = max(min(v_left / MAX_LINEAR, 1.0), -1.0)
        duty_right = max(min(v_right / MAX_LINEAR, 1.0), -1.0)

        # Set directions
        dir_left  = 0 if duty_left >= 0 else 1
        dir_right = 0 if duty_right >= 0 else 1
        self.lines.set_values([1,1,dir_left,dir_right])

        # Set PWM
        set_pwm(PWM0, abs(duty_left))
        set_pwm(PWM1, abs(duty_right))

        self.get_logger().info(f"cmd_vel: v={v:.2f}, ω={omega:.2f} -> "
                               f"L={duty_left:.2f}, R={duty_right:.2f}")

    def destroy_node(self):
        stop_pwm(PWM0)
        stop_pwm(PWM1)
        self.lines.set_values([0,0,0,0])
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePWM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
