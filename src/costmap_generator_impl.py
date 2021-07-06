#!/usr/bin/env python

# system imports
import signal
import sys
import threading

# ros imports
import rospy
from occupancy_grid_python import OccupancyGridManager
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point32, Pose, PoseWithCovarianceStamped
from costmap_processor.msg import Contour as ContourMsg

# image processing imports
import numpy as np
import cv2 as cv
from PIL import Image as im
from matplotlib import pyplot as plt

from costmap_processor_impl import *

class Costmap(object):

    def __init__(self, width, height, resolution, origin=None):

        self._width = width
        self._height = height
        self._resolution = resolution

        # if not given an origin, assume we're at zero
        if origin is None:
            og = Pose()
            og.position.x = 0
            og.position.y = 0
            og.position.z = 0

            og.orientation.x = 0
            og.orientation.y = 0
            og.orientation.z = 0
            og.orientation.w = 1

            self._origin = og
        elif origin is not None:
            self._origin = origin

        # costmap beings as 2darray of zeros
        self._data = np.zeros((height, width)).astype('uint8')

        # default to "map"
        self._frame = "map"

    @property
    def width(self):
        return self._width

    @property
    def height(self):
        return self._height

    @property
    def resolution(self):
        return self._resolution

    @property
    def origin(self):
        return self._origin

    @origin.setter
    def origin(self, new_pose):
        self._origin = new_pose

    @property
    def data(self):
        return self._data

    @property
    def frame(self):
        return self._frame

    @property
    def ready(self):
        return (self._origin is not None)

    def mark(self, points, value):

        ''' given list of world-frame points, marks costmap cell equiv. points as given value '''

        for point in points:

            # TODO: if point is in map
            x, y = self.world_point_to_costmap_point(point.x, point.y)
            x = max(x, 1)
            y = max(y, 1)

            # if the point is in the map, mark the point
            # if not, expand the map to include it (plus some extra?), then mark the point

            # if point is outside [0, height), expand the y-dimension
            if not -1 < y < self._height:
                # create array of zeros of size (delta_height, width)
                delta_height = max(self._height, y) - self._height
                delta_arr = np.zeros((delta_height, self._width))

                # combine delta_arr and our data array, enlarging the map
                self._data = np.concatenate((self._data, delta_arr), axis=0)
                self._height += delta_height    

            # if point is outside [0, width), expand the x-dimension
            if not -1 < x < self._width:
                # create array of zeros of size (height, delta_width)
                # delta_width = max(self._width, x) - self._width
                delta_width = max(self._width, abs(x)) - self._width
                delta_arr = np.zeros((self._height, delta_width))

                # combine delta_arr and our data array, enlarging the map
                self._data = np.concatenate((self._data, delta_arr), axis=1)
                self._width += delta_width
            
            # (y, x) because np + OccupancyGrid are row-major
            self._data[y-1][x-1] = value

        # TODO: filling contour to make obstacles from it, rather than just the shape of an obstacle
        # cv.fillPoly(self._data, pts=[points], color=(255,255,255))

    def clear(self):

        self._data.fill(0)

    def world_point_to_costmap_point(self, x, y):

        '''converts world-frame points to costmap-frame points based on costmap origin and costmap resolution'''    

        costmap_x = int(round((x - self._origin.position.x) / self._resolution))
        costmap_y = int(round((y - self._origin.position.y) / self._resolution))

        return costmap_x, costmap_y

class CostmapGenerator(object):

    def __init__(self, publish=True, rolling_window=False):
        
        self.cnt_subscriber = rospy.Subscriber("/costmap_processor/slope_grid", ContourMsg, self.cnt_callback)
        
        # if rolling_window is configured, subscribe to pose (so we can set costmap center-point later)
        if rolling_window:
            self.pose_subscriber = rospy.Subscriber("/pose", PoseWithCovarianceStamped, self.pose_callback)

        self._costmap_pub = rospy.Publisher('costmap_generator', OccupancyGrid, queue_size=10)
        self._publish = publish

        # TODO: need to get these dynamically (from a MapMetaData message?)
        # does the generated costmap need to have the same properties as the contour source?
        height = 101    # y
        width = 100     # x
        resolution = 0.05   # meters per pixel/cell

        self._costmap = Costmap(width, height, resolution)

        self._ready = False
    
    @property
    def publish(self):
        return self._publish
    
    @publish.setter
    def publish(self, new_publish):
        self._publish = new_publish

    @property
    def costmap(self):
        return self._costmap

    @property
    def ready(self):
        return self._ready

    def run(self):
        
        ''' main loop of node '''

        # TODO: needs to be param-ed
        rate = rospy.Rate(1)    # Hz

        while not rospy.is_shutdown():

            if self.publish:

                if self._costmap.ready and self._ready:    
                    # create costmap message from data in Costmap object
                    msg = self.costmap_to_occ_grid_msg()
                    self._costmap_pub.publish(msg)
                
            rate.sleep()

    def cnt_callback(self, contour):

        ''' receives Contour messages, passes points to Costmap object for marking '''

        # print("cnt callback!")

        self._ready = False

        # TODO: should I just create a VALUE field in the contour message?
        contour_label = contour.label.data

        # self._costmap.clear()

        self._costmap.mark(points=contour.contour.polygon.points, 
                            value=contour.label.data) 

        self._ready = True

    def pose_callback(self, msg):

        self._costmap.origin = msg.pose.pose

    def costmap_to_occ_grid_msg(self):

        ''' translates the Costmap object into an OccupancyGrid message for publishing '''

        msg = OccupancyGrid()

        # fill header fields
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._costmap.frame

        # fill the info field with a MapMetaData message
        meta = MapMetaData()
        meta.map_load_time = rospy.Time.now()   # not sure about this
        meta.resolution = self._costmap.resolution
        meta.width = self._costmap.width
        meta.height = self._costmap.height
        meta.origin = self._costmap.origin

        msg.info = meta

        # fill the data field with the "raveled" 1D array
        #  ravel() turns [[1,2,3], [4,5,6]] -> [1,2,3,4,5,6]
        msg.data = np.ravel(self._costmap.data)

        return msg


def ctrl_c_handler(sig, frame):
    sys.exit(0)

if __name__ == '__main__':

    rospy.init_node('costmap_generator', anonymous=True, log_level=rospy.INFO)

    signal.signal(signal.SIGINT, ctrl_c_handler)

    costy = CostmapGenerator(publish=True)
    costy.run()