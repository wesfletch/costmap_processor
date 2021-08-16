#!/usr/bin/env python

''' sends dummy Contour messages filled with random info, so that I don't have to spin up the whole autonomy stack to test the ROSBridge Java code '''

# system imports
import random

# ros imports
import rospy
from occupancy_grid_python import OccupancyGridManager
from rospy_message_converter import json_message_converter

# ros msg imports
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32
from costmap_processor.msg import Contour as ContourMsg

if __name__ == '__main__':

    rospy.init_node('dummy_contours', anonymous=True, log_level=rospy.DEBUG)

    contour_pub = rospy.Publisher("/dummy_contours", ContourMsg, queue_size=10)

    rate = rospy.Rate(1)   # Hz

    while not rospy.is_shutdown():

        # create message to be published
        msg = ContourMsg()

        # fill header fields
        msg.contour.header.stamp = rospy.Time.now()
        msg.contour.header.frame_id = "/map"

        # add points in the contour to message contour.polygon.points
        for idx in range(0, random.randint(0,10)):

            # Point32 message
            point = Point32()
            point.x = random.uniform(-10.0, 10.0)
            point.y = random.uniform(-10.0, 10.0)
            point.z = 0.0

            msg.contour.polygon.points.append(point)

        # fill the pose
        msg.pose.position.x = random.uniform(-10.0, 10.0)
        msg.pose.position.y = random.uniform(-10.0, 10.0)
        msg.pose.position.z = 0 # no contours in the air, man

        # don't really care about this
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1

        # label the contour
        msg.label.data = str(random.randint(0, 100.0))

        # msg.metadata = ""

        contour_pub.publish(msg)

        rate.sleep()

    