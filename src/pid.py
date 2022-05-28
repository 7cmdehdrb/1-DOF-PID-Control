#!/usr/bin/env python3


import rospy
from std_msgs.msg import Float32, UInt32
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

        self.p_gain = 0.17
        self.i_gain = 0.02

        self.p_err = 0.0
        self.i_err = 0.0

        # Threading
        thread = threading.Thread(target=self.receive_data)
        thread.daemon = True
        thread.start()

        self.initialize()

    def receive_data(self):
        # Read Serial
        while not exitThread:
            try:
                data = self.ser.readline()
                loadData = float(data)
                self.loadData = loadData

            except Exception as ex:
                print(ex)
                print(data)

    def initialize(self):
        # print("STOP DC MOTOR...")
        # self.write_data(991)
        # sleep(5)

        waitTime = 10

        print("RESET SERVO...")
        for i in range(20):
            self.write_data(120)
            sleep(0.1)

        for i in range(waitTime):
            print("Start after %d sec..." % (waitTime - i))
            self.write_data(120)
            sleep(1)

        # print("Start DC Motor")
        # self.write_data(990)

    def write_data(self, data: int):
        """
        Code

            990: Start DC Motor
            991: Stop DC Motor
            992: Reset Loadcell
            993: Reset Servo Motor

        """
        # print("Write %d" % data)
        bytesData = bytes((str(data) + "\n").encode("utf-8"))
        self.ser.write(bytesData)

    def PControl(self, currentLoad: int, desiredLoad: int, dt: float):
        self.p_err = desiredLoad - currentLoad
        self.i_err += self.p_err * self.i_gain

        requiredLoad = (self.p_gain * self.p_err) + (self.i_err * dt)

        return requiredLoad


def inputFilter(value: int):
    if value >= 990:
        return value

    if value < 50:
        return 50

    elif value > 180:
        return 180

    else:
        return value


if __name__ == "__main__":
    rospy.init_node("PID")

    # Objects
    control = Control(port_num="/dev/ttyACM0")

    servo_pub = rospy.Publisher("servo", UInt32, queue_size=1)
    load_pub = rospy.Publisher("load", Float32, queue_size=1)

    inputVal = 120
    hz = 10

    r = rospy.Rate(hz)
    while not rospy.is_shutdown():
        val = int(control.loadData)

        requiredLoad = control.PControl(
            currentLoad=val, desiredLoad=70, dt=(1 / hz))
        inputVal += requiredLoad

        inputVal = int(inputFilter(inputVal))

        control.write_data(inputVal)

        servo_data = UInt32()
        servo_data.data = inputVal

        load_data = Float32()
        load_data.data = val

        if servo_pub.get_num_connections() > 0:
            servo_pub.publish(servo_data)

        if load_pub.get_num_connections() > 0:
            load_pub.publish(load_data)

        rospy.loginfo(
            "Input DEG: %d, Load: %f, Gain: %f"
            % (inputVal, control.loadData, requiredLoad)
        )

        r.sleep()
