#!/usr/bin/env python3


import rospy
from std_msgs.msg import Float32, UInt32
from time import sleep
import serial
from random import randint
import threading
import math as m
from low_pass_filter import LowPassFilter
import numpy as np

exitThread = False


class Control:
    def __init__(self, port_num):
        self.loadData = 0.0
        self.servoData = 120

        self.p_gain = 1.
        self.i_gain = 1.
        self.d_gain = 0.

        self.p_err = 0.0
        self.i_err = 0.0
        self.d_err = 0.0

        rospy.Subscriber("p_gain", Float32, callback=self.p_callback)
        rospy.Subscriber("i_gain", Float32, callback=self.i_callback)
        rospy.Subscriber("d_gain", Float32, callback=self.d_callback)

        # Threading
        thread = threading.Thread(
            target=self.receive_data)
        thread.daemon = True
        thread.start()

    def p_callback(self, msg):
        self.p_gain = msg.data

    def i_callback(self, msg):
        self.i_gain = msg.data

    def d_callback(self, msg):
        self.d_gain = msg.data

    def receive_data(self):
        # Predict load data from degree + noise
        r = rospy.Rate(10)
        while True:
            degree = self.servoData

            predictedLoad = degreeToLoad(degree=degree)

            noise = randint(-500, 500) / 50.

            self.loadData = predictedLoad + noise

            r.sleep()

    def PIControl(self, currentLoad: int, desiredLoad: int, dt: float):
        err = desiredLoad - currentLoad
        self.d_err = (err - self.p_err) / dt
        self.p_err = err
        self.i_err += self.p_err * dt

        gain = (self.p_gain * self.p_err) + (self.i_gain *
                                             self.i_err) + (self.d_gain * self.d_err)

        return gain


def inputFilter(value):
    if value < 60:
        return 60

    elif value > 100:
        return 100

    else:
        return value


def loadToDegree(load: float, a=0.9447612500347906, b=-6.5668370833003396):
    return a * load + b


def degreeToLoad(degree: int, a=1.0462126454570502, b=7.931256436573569):
    return a * degree + b


if __name__ == "__main__":
    rospy.init_node("PID")

    inputDegree = 0
    hz = 10

    # Objects
    control = Control(port_num="/dev/ttyACM0")
    lpf = LowPassFilter(cutoff_freq=1., ts=(1 / hz))

    servo_pub = rospy.Publisher("servo", UInt32, queue_size=1)
    load_pub = rospy.Publisher("load", Float32, queue_size=1)

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():
        val = control.loadData
        val = lpf.filter(val)

        # Test Area

        gain = (control.PIControl(
            currentLoad=val, desiredLoad=80, dt=(1 / hz)))
        inputLoad = val + gain

        inputDegree = int(inputFilter(loadToDegree(inputLoad)))

        control.servoData = inputDegree

        # control.write_data(inputDegree)

        servo_data = UInt32()
        servo_data.data = inputDegree

        load_data = Float32()
        load_data.data = val

        servo_pub.publish(servo_data)
        load_pub.publish(load_data)

        rospy.loginfo(
            "Input DEG: %d, Load: %f, gain: %f"
            % (inputDegree, val, gain)
        )

        r.sleep()
