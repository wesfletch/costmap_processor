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

        self._center_pixel = ((self._width/2), (self._height/2))    # (X, Y)

        # if not given an origin, assume we're at (0,0)
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

        # costmap begins as 2darray of zeros; row (height) major
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

    def expand(self, delta_width=0, delta_height=0):

        ''' expands map dimension(s) by given delta value, keeps current data '''

        # (potentially) expand the width dimension
        if delta_width != 0:

            # resizing due to positive values ( > self._width)
            if delta_width > 0:
                # don't need to update origin since we're adding points to the end of grid data
                delta_arr = np.zeros((self._height, abs(delta_width)))
                self._data = np.concatenate((self._data, delta_arr), axis=1)

            # resizing due to negative values (< origin.position.x) requires some special handling
            # new, empty cells must be placed IN FRONT OF the current grid data
            # this requires shifting the origin to the left so that already-marked points remain valid
            elif delta_width < 0:
                # create array of zeros, use concatenate to place new arr IN FRONT OF grid_data 
                # requires origin to be updated 
                delta_arr = np.zeros((self._height, abs(delta_width)))
                self._data = np.concatenate((delta_arr, self._data), axis=1)
                self._origin.position.x += delta_width * self._resolution

            self._width += abs(delta_width)             # update width
            self._center_pixel += (delta_width / 2, 0)  # update center pixel
            rospy.logdebug('Map width expanded from {} to {}'.format(self._width - abs(delta_width), self._width))

        # (potentially) expand the height dimension
        if delta_height != 0:

            # handle "negative" resizes
            if delta_height < 0:
                # create array of zeros, use concatenate to place new arr IN FRONT OF grid_data 
                # requires origin to be updated 
                delta_arr = np.zeros((abs(delta_height), self._width))
                self._data = np.concatenate((delta_arr, self._data), axis=0)
                self._origin.position.y += delta_height * self._resolution
            elif delta_height > 0:
                # don't need to update origin since we're adding points to the end of grid data
                delta_arr = np.zeros((abs(delta_height), self._width))
                self._data = np.concatenate((self._data, delta_arr), axis=0)

            self._height += abs(delta_height)           # update height
            self._center_pixel += (0, delta_height / 2) # update center pixel
            rospy.logdebug('Map height expanded from {} to {}'.format(self._height - abs(delta_height), self._height))

        # recompute center pixel since we changed dimensions
        # if delta_width != 0 or delta_height != 0:
        #     # self._center_pixel = ((self._width / 2), (self._height / 2))
        #     self.update_origin()
            
    def mark(self, points, value):

        for point in points:

            # TODO: should turn this into a public-private interface type deal instead of this type-checking garbage
            if isinstance(point, tuple):
                x, y = self.world_point_to_costmap_point(point[0], point[1])
            elif isinstance(point, Point32): 
                x, y = self.world_point_to_costmap_point(point.x, point.y)
            else:
                rospy.logerr('incorrect point type {}... skipping'.format(type(point)))
                continue
            
            # rospy.loginfo("x: {}, y: {}  (pixels)".format(x, y))

            delta_x = 0
            delta_y = 0

            if not -1 < x < self._width:
                delta_x = x
            if not -1 < y < self._height:
                delta_y = y

            if delta_x != 0 or delta_y != 0:
                # expand the map if needed
                self.expand(delta_width=delta_x, delta_height=delta_y)
                # update x, y now that we've moved the center (and updated the origin)
                # x, y = self.world_point_to_costmap_point(point[0], point[1])
                if isinstance(point, tuple):
                    x, y = self.world_point_to_costmap_point(point[0], point[1])
                elif isinstance(point, Point32): 
                    x, y = self.world_point_to_costmap_point(point.x, point.y)

            # (y, x) because np + OccupancyGrid are row-major
            self._data[y][x] = value

    def update_origin(self):

        ''' updates origin (assuming rolling_window==True) '''

        self._origin.position.x = - ((self._width * self._resolution) / 2.0)
        self._origin.position.y = - ((self._height * self._resolution) / 2.0)

        rospy.loginfo('new origin point ({},{})'
            .format(self._origin.position.x, self._origin.position.y))

    def clear(self):

        self._data.fill(0)

    def world_point_to_costmap_point(self, x, y):

        ''' converts world-frame points to costmap-frame points based on costmap origin and costmap resolution
            does NOT verify that the point returned is actually a cell contained in the current map '''    

        costmap_x = int(round((x - self._origin.position.x) / self._resolution))
        costmap_y = int(round((y - self._origin.position.y) / self._resolution))

        return costmap_x, costmap_y

    def costmap_point_to_world_point(self, x, y):

        world_x = self._origin.position.x + (x * self._resolution)
        world_y = self._origin.position.y + (y * self._resolution)

        # rospy.loginfo("got world point ({},{})".format(world_x, world_y))

        return world_x, world_y

    def get_occupied(self):

        ''' returns list of non-zero world-frame point tuples '''

        # doing it the stupid way, for the moment

        points = []

        for y in range(0, self._height -1):
            for x in range(0, self._width - 1):
                if self._data[y][x] != 0:
                    points.append(self.costmap_point_to_world_point(x, y))

        return points

    def get_cell_cost(self, x, y):

        return self._data[y][x]

    def costmap_to_occ_grid_msg(self):

        ''' translates the Costmap object's data into an OccupancyGrid message for publishing '''

        msg = OccupancyGrid()

        # fill header fields
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._frame

        # fill the info field with a MapMetaData message
        meta = MapMetaData()
        meta.map_load_time = rospy.Time.now()   # not sure about this
        meta.resolution = self._resolution
        meta.width = self._width
        meta.height = self._height
        meta.origin = self._origin

        msg.info = meta

        # fill the data field with the "raveled" 1D array
        # ravel() turns [[1,2,3], [4,5,6]] -> [1,2,3,4,5,6]
        msg.data = np.ravel(self._data)

        return msg

class CostmapGenerator(object):

    def __init__(self, publish=True, rolling_window=False):
        
        self.cnt_subscriber = rospy.Subscriber("/costmap_processor/slope_grid", ContourMsg, self.cnt_callback)
        self.other_subscriber = rospy.Subscriber("/costmap_processor/costmap", ContourMsg, self.cnt_callback)
        
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

                # TODO: costmap.ready is never really used or set up
                if self._costmap.ready and self._ready:    
                    # create costmap message from data in Costmap object
                    msg = self._costmap.costmap_to_occ_grid_msg()
                    self._costmap_pub.publish(msg)
                
            rate.sleep()

    def cnt_callback(self, contour):

        ''' receives Contour messages, passes points to Costmap object for marking '''

        self._ready = False

        # TODO: should I just create a VALUE field in the contour message?
        contour_label = contour.label.data

        self._costmap.mark(points=contour.contour.polygon.points, 
                            value=contour.label.data) 

        self._ready = True

    def pose_callback(self, msg):

        self._costmap.origin = msg.pose.pose

def ctrl_c_handler(sig, frame):
    sys.exit(0)

if __name__ == '__main__':

    rospy.init_node('costmap_generator', anonymous=True, log_level=rospy.INFO)

    signal.signal(signal.SIGINT, ctrl_c_handler)

    costy = CostmapGenerator(publish=True)
    costy.run()