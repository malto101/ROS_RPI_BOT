#!/usr/bin/env python3
import roslib
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as gpio
from time import sleep


class wheel_interface:

    def __init__(self):

        self.joy_sub = rospy.Subscriber(
            "cmd_vel", Twist, self.cmd_vel_callback)
        print("\n")
        print("The default speed & direction of motor is LOW & Forward.....")
        print("\n")
        self.en1 = 11
        self.en2 = 12
        self.motor1in1 = 13
        self.motor1in2 = 15
        self.motor2in1 = 16
        self.motor2in2 = 18
        gpio.setmode(gpio.BCM)
        gpio.setup(self.motor1in1, gpio.OUT)
        gpio.setup(self.motor1in2, gpio.OUT)
        gpio.setup(self.motor2in1, gpio.OUT)
        gpio.setup(self.motor2in2, gpio.OUT)
        gpio.setup(self.en1, gpio.OUT)
        gpio.setup(self.en2, gpio.OUT)
        self.motor1 = gpio.PWM(self.en1, 100)
        self.motor1.start(25)
        self.motor2 = gpio.PWM(self.en2, 100)
        self.motor2.start(25)

    def forward(self, left, right):

        self.motor1.ChangeDutyCycle(left)
        sleep(0.1)
        self.motor2.ChangeDutyCycle(right)
        sleep(0.1)
        gpio.output(self.motor1in1, True)
        gpio.output(self.motor1in2, False)
        gpio.output(self.motor2in1, True)
        gpio.output(self.motor2in2, False)

    def reverse(self, left, right):

        self.motor1.ChangeDutyCycle(left)
        sleep(0.1)
        self.motor2.ChangeDutyCycle(right)
        sleep(0.1)
        gpio.output(self.motor1in1, False)
        gpio.output(self.motor1in2, True)
        gpio.output(self.motor2in1, False)
        gpio.output(self.motor2in2, True)

    def left_turn(self, left, right):

        self.motor1.ChangeDutyCycle(left)
        sleep(0.1)
        self.motor2.ChangeDutyCycle(right)
        sleep(0.1)
        gpio.output(self.motor1in1, True)
        gpio.output(self.motor1in2, False)
        gpio.output(self.motor2in1, False)
        gpio.output(self.motor2in2, False)

    def right_turn(self, left, right):

        self.motor1.ChangeDutyCycle(left)
        sleep(0.1)
        self.motor2.ChangeDutyCycle(right)
        sleep(0.1)
        gpio.output(self.motor1in1, False)
        gpio.output(self.motor1in2, False)
        gpio.output(self.motor2in1, True)
        gpio.output(self.motor2in2, False)

    def fast_left_turn(self, left, right):

        self.motor1.ChangeDutyCycle(left)
        sleep(0.1)
        self.motor2.ChangeDutyCycle(right)
        sleep(0.1)
        gpio.output(self.motor1in1, True)
        gpio.output(self.motor1in2, False)
        gpio.output(self.motor2in1, False)
        gpio.output(self.motor2in2, True)

    def fast_right_turn(self, left, right):

        self.motor1.ChangeDutyCycle(left)
        sleep(0.1)
        self.motor2.ChangeDutyCycle(right)
        sleep(0.1)
        gpio.output(self.motor1in1, False)
        gpio.output(self.motor1in2, True)
        gpio.output(self.motor2in1, True)
        gpio.output(self.motor2in2, False)

    def reverse_left(self, left, right):

        self.motor1.ChangeDutyCycle(left)
        sleep(0.1)
        self.motor2.ChangeDutyCycle(right)
        sleep(0.1)
        gpio.output(self.motor1in1, False)
        gpio.output(self.motor1in2, False)
        gpio.output(self.motor2in1, False)
        gpio.output(self.motor2in2, True)

    def reverse_right(self, left, right):

        self.motor1.ChangeDutyCycle(left)
        sleep(0.1)
        self.motor2.ChangeDutyCycle(right)
        sleep(0.1)
        gpio.output(self.motor1in1, False)
        gpio.output(self.motor1in2, True)
        gpio.output(self.motor2in1, False)
        gpio.output(self.motor2in2, False)

    def cmd_vel_callback(self, msg):

        self.length = 0.15
        pre_left = msg.linear.x-(msg.angular.z*(self.length/2))
        pre_right = msg.linear.x+(msg.angular.z*(self.length/2))
        main_left = int(abs(136.4*pre_left))
        main_right = int(abs(136.4*pre_right))

        if (pre_left > 0 and pre_right > 0):
            print("forward")
            self.forward(main_left, main_right)
        elif (pre_left == 0 and pre_right > 0):
            print("left")
            self.left_turn(main_left, main_right)
        elif (pre_left > 0 and pre_right == 0):
            print("right")
            self.right_turn(main_left, main_right)
        elif (pre_left == 0 and pre_right == 0):
            print("brake")
        elif (pre_left > 0 and pre_right < 0):
            print("fast right")
            self.fast_right_turn(main_left, main_right)
        elif (pre_left < 0 and pre_right > 0):
            print("fast left")
            self.fast_left_turn(main_left, main_right)
        elif (pre_left < 0 and pre_right < 0):
            print("backwards")
            self.reverse(main_left, main_right)
        elif (pre_left < 0 and pre_right == 0):
            print("reverse right")
            self.reverse_right(main_left, main_right)
        else:
            print("reverse left")
            self.reverse_left(main_left, main_right)


if __name__ == '__main__':
    rospy.init_node('velocity_to_wheel')
    wheel_vel = wheel_interface()
    rospy.spin()
