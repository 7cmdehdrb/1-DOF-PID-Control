#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, UInt32


servo = 0
load = 0.


def servoCallback(msg):
    global servo
    servo = msg.data


def loadCallback(msg):
    global load
    load = msg.data


if __name__ == "__main__":
    rospy.init_node("CSV")

    rospy.Subscriber("servo", UInt32, callback=servoCallback)
    rospy.Subscriber("load", Float32, callback=loadCallback)

    r = rospy.Rate(10)
    with open("datasheet.csv", 'w') as csvfile:
        while not rospy.is_shutdown():

            if servo == 0:
                continue

            content = str(servo) + "," + str(load) + "\n"
            csvfile.write(content)

            rospy.loginfo(content)

            r.sleep()
