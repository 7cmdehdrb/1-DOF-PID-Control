#!/usr/bin/env python


from random import randint
import rospy
from time import sleep
from std_msgs.msg import UInt32
import tf
from tf.transformations import quaternion_from_euler
import math as m


servo_msg = UInt32()


def servoCallback(msg):
    global servo_msg
    servo_msg = msg


if __name__ == "__main__":
    rospy.init_node("servo_joint")

    tf_pub = tf.TransformBroadcaster()

    rospy.Subscriber("servo", UInt32, callback=servoCallback)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        quat = quaternion_from_euler(0., m.radians(servo_msg.data - 40), 0.)

        tf_pub.sendTransform(
            translation=[0., 3., 5.],
            rotation=quat,
            time=rospy.Time.now(),
            child="servo_arm",
            parent="servo_support"
        )

        r.sleep()
