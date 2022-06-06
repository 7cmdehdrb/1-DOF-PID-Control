#!/usr/bin/env python3


import rospy
import math as m
import numpy as np
import serial
import threading
from time import sleep
from std_msgs.msg import Float32, UInt32

exitThread = False


class Control:
    def __init__(self, port_num):

        self.ser = serial.Serial(
            port=port_num,
            baudrate=9600,
        )

        self.load_pub = rospy.Publisher("load", Float32, queue_size=1)
        self.servo_sub = rospy.Subscriber(
            "servo", UInt32, callback=self.servo_callback)

        # Threading
        thread = threading.Thread(
            target=self.receive_data)
        thread.daemon = True
        thread.start()

        self.initialize()

    def receive_data(self):
        # Read Serial
        while not exitThread:
            try:
                data = self.ser.readline()
                self.load_pub.publish(float(data))

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

    def servo_callback(self, msg):
        data = msg.data
        self.write_data(data)


if __name__ == "__main__":
    rospy.init_node("control_node")

    # Objects
    control = Control(port_num="/dev/ttyACM0")

    rospy.spin()
