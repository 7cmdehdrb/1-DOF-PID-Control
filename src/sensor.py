#!/usr/bin/env python3


import rospy
from std_msgs.msg import Float32, UInt32
from model import *
from low_pass_filter import LowPassFilter


class PID(object):
    def __init__(self):

        self.load = 0.

        self.lpf = LowPassFilter(cutoff_freq=1., ts=(1 / 10))

        self.p_gain = 1.2
        self.i_gain = 2.0
        self.d_gain = 0.01

        self.p_err = 0.0
        self.i_err = 0.0
        self.d_err = 0.0

        rospy.Subscriber("load", Float32, self.load_callback)
        rospy.Subscriber("p_gain", Float32, callback=self.p_callback)
        rospy.Subscriber("i_gain", Float32, callback=self.i_callback)
        rospy.Subscriber("d_gain", Float32, callback=self.d_callback)

    def load_callback(self, msg):
        data = self.lpf(msg.data)
        self.load = data

    def p_callback(self, msg):
        self.p_gain = msg.data

    def i_callback(self, msg):
        self.i_gain = msg.data

    def d_callback(self, msg):
        self.d_gain = msg.data

    def PIDControl(self, currentLoad: int, desiredLoad: int, dt: float):
        err = desiredLoad - currentLoad
        self.d_err = (err - self.p_err) / dt
        self.p_err = err
        self.i_err += self.p_err * dt

        gain = (self.p_gain * self.p_err) + (self.i_gain *
                                             self.i_err) + (self.d_gain * self.d_err)

        return gain


if __name__ == "__main__":
    rospy.init_node("sensor_node")

    pid = PID()

    servo_pub = rospy.Publisher("servo", UInt32, queue_size=1)

    r = rospy.Rate(hz=10)
    while not rospy.is_shutdown():
        gain = pid.PIDControl(
            currentLoad=pid.load, desiredLoad=80, dt=1/10
        )

        inputLoad = pid.load + gain
        inputDegree = inputFilter(loadToDegree(inputLoad))

        servo_pub.publish(inputDegree)

        r.sleep()
