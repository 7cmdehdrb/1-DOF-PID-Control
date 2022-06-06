#!/usr/bin/env python3


import rospy
import math as m
import numpy as np
import serial
import threading
from time import sleep
from std_msgs.msg import Float32, UInt32
from low_pass_filter import LowPassFilter

exitThread = False


class Control:
    def __init__(self, port_num):

        self.ser = serial.Serial(
            port=port_num,
            baudrate=9600,
        )

        self.loadData = 0.0

        self.p_gain = 1.2
        self.i_gain = 2.0
        self.d_gain = 0.01

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

        self.initialize()

    def p_callback(self, msg):
        self.p_gain = msg.data

    def i_callback(self, msg):
        self.i_gain = msg.data

    def d_callback(self, msg):
        self.d_gain = msg.data

    def receive_data(self):
        # Read Serial
        while not exitThread:
            try:
                data = self.ser.readline()
                loadData = float(data)
                self.loadData = loadData

            except Exception as ex:
                rospy.loginfo(ex)
                rospy.loginfo(data)

    def initialize(self):

        waitTime = 10

        rospy.loginfo("RESET SERVO...")
        for i in range(20):
            self.write_data(120)
            sleep(0.1)

        for i in range(waitTime):
            rospy.loginfo("Start after %d sec..." % (waitTime - i))
            self.write_data(120)
            sleep(1)

    def write_data(self, data: int):
        """
        Code

            990: Start DC Motor
            991: Stop DC Motor
            992: Reset Loadcell
            993: Reset Servo Motor

        """
        bytesData = bytes((str(data) + "\n").encode("utf-8"))
        self.ser.write(bytesData)

    def PIDControl(self, currentLoad: int, desiredLoad: int, dt: float):
        err = desiredLoad - currentLoad
        self.d_err = (err - self.p_err) / dt
        self.p_err = err
        self.i_err += self.p_err * dt

        gain = (self.p_gain * self.p_err) + (self.i_gain *
                                             self.i_err) + (self.d_gain * self.d_err)

        return gain


def inputFilter(value):
    if value < 60:
        return 50

    elif value > 120:
        return 120

    else:
        return int(value)


def loadToDegree(load: float, a=0.9447612500347906, b=-6.5668370833003396):
    return a * load + b


def degreeToLoad(degree: int, a=1.0462126454570502, b=7.931256436573569):
    return a * degree + b


if __name__ == "__main__":
    rospy.init_node("PID")

    inputDegree = 120
    hz = 10

    # Objects
    control = Control(port_num="/dev/ttyACM0")
    lpf = LowPassFilter(cutoff_freq=1., ts=(1 / hz))

    servo_pub = rospy.Publisher("servo", UInt32, queue_size=1)
    load_pub = rospy.Publisher("load", Float32, queue_size=1)
    load_raw_pub = rospy.Publisher("load_raw", Float32, queue_size=1)

    r = rospy.Rate(hz)

    while not rospy.is_shutdown():
        val = control.loadData
        val = lpf.filter(val)

        gain = (control.PIDControl(
            currentLoad=val, desiredLoad=80, dt=(1 / hz)))
        inputLoad = val + gain

        inputDegree = inputFilter(loadToDegree(inputLoad))

        control.write_data(inputDegree)

        servo_data = UInt32()
        servo_data.data = inputDegree

        load_data = Float32()
        load_data.data = val

        load_data_raw = Float32()
        load_data_raw.data = control.loadData

        servo_pub.publish(servo_data)
        load_pub.publish(load_data)
        load_raw_pub.publish(load_data_raw)

        rospy.loginfo(
            "Input DEG: %d\nInput Load: %f\nOutput Load: %f\ngain: %f\n"
            % (inputDegree, inputLoad, val, gain)
        )

        r.sleep()
