#!/usr/bin/env python

''' subscribes to a nav_msgs/OccupancyGrid topic and displays received costmaps as images
    uses the Costmap object from costmap_generator.py
    used for diagnostic purposes, mostly '''

import sys
import numpy as np
from PIL import Image as im
from matplotlib import pyplot as plt

import rospy
from nav_msgs.msg import OccupancyGrid

from costmap_processor import Costmap, CostmapProcessor


def ocg_callback(msg):

    # create a Costmap object from the metadata of the message
    costy = Costmap.create_from_metadata(msg.info)
    # set the data field to the received data
    costy.data = msg.data

    print(np.unique(costy.data))

    prossy = CostmapProcessor(None, None, None, True)
    prossy.get_grid_contours(costy.data)

    # plt.subplot(111),plt.imshow(costy.data,cmap = 'gray')
    # plt.title('Original'), plt.xticks([]), plt.yticks([])
    # plt.show()

if __name__ == '__main__':

    rospy.init_node("capture_costmap")

    # args = sys.argv[1:]

    ocg_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, ocg_callback)

    rospy.spin()
