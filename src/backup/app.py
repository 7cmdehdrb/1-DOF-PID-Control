#!/usr/bin/env python3


import rospy
from std_msgs.msg import Float32
from random import randint
from time import sleep
import serial
import threading
import math as m


exitThread = False


class Control:
    def __init__(self, port_num):

        self.ser = serial.Serial(
            port=port_num,
            baudrate=9600,
        )

        self.loadData = 0.0

        # Threading

        thread = threading.Thread(target=self.receive_data)
        thread.daemon = True
        thread.start()

    def receive_data(self):
        # Read Serial
        while not exitThread:
            try:
                # print(self.ser.readline())
                loadData = float(self.ser.readline())
                self.loadData = loadData
                # print(loadData)

            except Exception as ex:
                print(ex)

    def write_data(self, data: int):
        bytesData = bytes((str(data) + "\n").encode("utf-8"))
        self.ser.write(bytesData)
        # self.ser.write(randint(0, 180))


def PControl(inputVal: int, desiredOutput: int):
    return desiredOutput - inputVal


def inputFilter(value: int):
    if value < 0:
        return 0

    elif value > 180:
        return 180

    else:
        return value


if __name__ == "__main__":
    rospy.init_node("PID")

    # Objects
    control = Control(port_num="/dev/ttyACM0")
    pub = rospy.Publisher("loadcell", Float32, queue_size=1)

    inputVal = 120
    gain = 0.1

    r = rospy.Rate(4)
    while not rospy.is_shutdown():
        val = int(control.loadData)

        temp = PControl(inputVal=val, desiredOutput=100)
        inputVal += int(gain * temp)

        inputVal = int(inputFilter(inputVal))

        control.write_data(inputVal)

        print("Input DEG: %d, Load: %f" % (inputVal, control.loadData))

        data = Float32()
        data.data = val
        pub.publish(data)
        r.sleep()
