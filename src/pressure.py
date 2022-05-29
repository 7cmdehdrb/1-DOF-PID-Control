#!/usr/bin/env python


import rospy
from std_msgs.msg import Float32, ColorRGBA
from geometry_msgs.msg import Pose, Vector3
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
import math as m


class Pressure(object):
    def __init__(self):
        self.value = 0.

        self.pub = rospy.Publisher(
            "pressure", Marker, queue_size=1
        )

        self.sub = rospy.Subscriber(
            "load", Float32, callback=self.pressureCallback)

    def pressureCallback(self, msg):
        self.value = msg.data

    def visualize(self):

        marker = Marker()

        marker.header.frame_id = "loadcell"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "pressure"
        marker.id = 0

        marker.type = 0
        marker.action = 0

        pose = Pose()

        pose.position.y = 2.
        pose.position.z = 0.

        quat = quaternion_from_euler(0., m.pi / 2, 0.)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        marker.pose = pose

        scale = Vector3()

        scale.x = (self.value / 10.)
        scale.y = 1.
        scale.z = 1.
        marker.scale = scale

        color = ColorRGBA()

        color.b = 1.
        color.a = 1.
        marker.color = color

        self.pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("pressure_visualizer")

    pressure = Pressure()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        pressure.visualize()
        r.sleep()
