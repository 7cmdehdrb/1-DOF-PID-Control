#!/usr/bin/env python3


import rospy
from std_msgs.msg import Float32, UInt32
from time import sleep
import serial
import threading
import math as m
from low_pass_filter import LowPassFilter


exitThread = False


class Control:
    def __init__(self, port_num):

        self.ser = serial.Serial(
            port=port_num,
            baudrate=9600,
        )

        self.humid = 0.
        self.temp = 0.

        # Threading
        thread = threading.Thread(target=self.receive_data)
        thread.daemon = True
        thread.start()

        # self.initialize()

    def receive_data(self):
        # Read Serial
        while not exitThread:
            try:
                data = str(self.ser.readline())

                if len(data) < 5:
                    continue

                humid, temp = data.split(",")

                self.humid = float(humid.split("'")[1])
                self.temp = float(temp.split("\\r")[0])

                rospy.loginfo(str(self.humid) + " / " + str(self.temp))

            except Exception as ex:
                rospy.logwarn(ex)
                rospy.logwarn(data)


if __name__ == "__main__":
    rospy.init_node("PID")

    inputDegree = 120
    hz = 10

    # Objects
    control = Control(port_num="/dev/ttyACM0")
    Hlpf = LowPassFilter(cutoff_freq=1., ts=(1 / hz))
    Tlpf = LowPassFilter(cutoff_freq=1., ts=(1 / hz))

    humid = rospy.Publisher("humidity", Float32, queue_size=1)
    temp = rospy.Publisher("temperture", Float32, queue_size=1)

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():

        h = Hlpf.filter(control.humid)
        t = Tlpf.filter(control.temp)

        humid.publish(h)
        temp.publish(t)

        r.sleep()
