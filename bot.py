#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as gpio

class WheelInterface:
    def __init__(self):
        # Subscribe to the "cmd_vel" topic to receive Twist messages
        self.joy_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        
        # Initialize GPIO pins and print initial message
        print("\n")
        print("The default speed & direction of motor is LOW & Forward.....")
        print("\n")
        self.pins = {
            'en1': 11,
            'en2': 12,
            'motor1in1': 13,
            'motor1in2': 15,
            'motor2in1': 16,
            'motor2in2': 18
        }
        gpio.setmode(gpio.BCM)
        for pin in self.pins.values():
            gpio.setup(pin, gpio.OUT)
        self.motor1 = gpio.PWM(self.pins['en1'], 100)
        self.motor1.start(25)
        self.motor2 = gpio.PWM(self.pins['en2'], 100)
        self.motor2.start(25)

    def set_motor_outputs(self, left_pwm, right_pwm, left_in1, left_in2, right_in1, right_in2):
        # Set motor PWM and direction outputs
        self.motor1.ChangeDutyCycle(left_pwm)
        self.motor2.ChangeDutyCycle(right_pwm)
        gpio.output(self.pins['motor1in1'], left_in1)
        gpio.output(self.pins['motor1in2'], left_in2)
        gpio.output(self.pins['motor2in1'], right_in1)
        gpio.output(self.pins['motor2in2'], right_in2)

    def cmd_vel_callback(self, msg):
        length = 0.15
        pre_left = msg.linear.x - (msg.angular.z * (length / 2))
        pre_right = msg.linear.x + (msg.angular.z * (length / 2))
        main_left = int(abs(136.4 * pre_left))
        main_right = int(abs(136.4 * pre_right))

        # Determine motor actions based on pre-calculated values
        # Inside the cmd_vel_callback function

        if pre_left > 0 and pre_right > 0:
            print("forward")
            self.set_motor_outputs(main_left, main_right, True, False, True, False)
        elif pre_left == 0 and pre_right > 0:
            print("left")
            self.set_motor_outputs(main_left, main_right, True, False, False, False)
        elif pre_left > 0 and pre_right == 0:
            print("right")
            self.set_motor_outputs(main_left, main_right, False, False, True, False)
        elif pre_left == 0 and pre_right == 0:
            print("brake")
            self.set_motor_outputs(0, 0, False, False, False, False)
        elif pre_left > 0 and pre_right < 0:
            print("fast right")
            self.set_motor_outputs(main_left, main_right, True, False, False, True)
        elif pre_left < 0 and pre_right > 0:
            print("fast left")
            self.set_motor_outputs(main_left, main_right, False, True, True, False)
        elif pre_left < 0 and pre_right < 0:
            print("backwards")
            self.set_motor_outputs(main_left, main_right, False, True, False, True)
        elif pre_left < 0 and pre_right == 0:
            print("reverse right")
            self.set_motor_outputs(main_left, main_right, False, True, False, False)
        else:
            print("reverse left")
            self.set_motor_outputs(main_left, main_right, True, False, False, False)


if __name__ == '__main__':
    # Initialize ROS node and WheelInterface instance
    rospy.init_node('velocity_to_wheel')
    wheel_vel = WheelInterface()
    # Keep the program running and wait for incoming messages
    rospy.spin()
