
import rospy
from sensor_msgs.msg import FluidPressure, Imu
from random import randint


if __name__ == "__main__":
    rospy.init_node("test")

    pub = rospy.Publisher("/test", FluidPressure, queue_size=1)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():

        msg = FluidPressure()
        msg.header.frame_id = "base_link"
        msg.header.stamp = rospy.Time.now()

        msg.fluid_pressure = 3000
        msg.variance = 0.

        pub.publish(msg)
        r.sleep()
