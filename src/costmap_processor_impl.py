#!/usr/bin/env python

# ros imports
import rospy
from occupancy_grid_python import OccupancyGridManager
from nav_msgs.msg import OccupancyGrid

# image processing imports
import numpy as np
import cv2 as cv
from PIL import Image as im
from matplotlib import pyplot as plt

# system imports
import signal, sys
import threading

class Obstacle:

    pass

class Point2D:

    def __init__(self, x, y, value=0):
        self._x = x
        self._y = y
        self._value = value
    
    def __str__(self):
        return "Point: ({}, {}) \nValue: {}".format(self.x, self.y, self.value)

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, new_value):
        self._value = new_value

class Contour:

    def __init__(self, label, center, contour, pose_rel_to_base=(0,0)):
        self._label = label
        self._center = center
        self._contour = contour
        self._pose_rel_to_base = pose_rel_to_base

    def __str__(self):
        return "label: {}\ncenter: {}\npose: {}".format(self.label, self.center, self.pose_rel_to_base)

    @property
    def label(self):
        return self._label

    @property
    def center(self):
        return self._center

    @property
    def pose_rel_to_base(self):
        return self._pose_rel_to_base

    @pose_rel_to_base.setter
    def pose_rel_to_base(self, new_pose):
        self._pose_rel_to_base = new_pose

    def to_ros_message_function(self):
        pass


def ctrl_c_handler(sig, frame):
    # plt.close()
    sys.exit(0)

class CostmapProcessor(threading.Thread):

    def __init__(self, type, ogm, stop):
      threading.Thread.__init__(self)
      self._type = type
      self._ogm = ogm
      self._stop = stop 
      self._contours = []

    @property
    def stop(self):
        # stop is a lambda function, accessed via stop()
        return self._stop()

    @property
    def type(self):
        return self._type

    @property
    def ogm(self):
        return self._ogm

    @property
    def contours(self):
        return self._contours

    @contours.setter
    def contours(self, new_contours):
        self._contours = new_contours

    # runs at thread start
    def run(self):

        rate = rospy.Rate(.1)    # Hz

        while not self.stop:

            if self.type == 'heightmap':
                self.contours = self.get_height_contours() # could probably just do this in get_height_contours

                print(self.contours)

            rate.sleep()

    def print_ogm_info(self):

        print('costmap topic: {0}'.format(self.ogm.topic))

        # Now you can do basic operations
        print('Resolution: {0}'.format(self.ogm.resolution))
        # Note that OccupancyGrid data starts on lower left corner (if seen as an image)
        # width / X is from left to right
        # height / Y is from bottom to top
        print('width: {0}'.format(self.ogm.width))
        print('height: {0}'.format(self.ogm.height))
        print('origin: \n{0}'.format(self.ogm.origin))  # geometry_msgs/Pose
        print('reference_frame: {0}'.format(self.ogm.reference_frame))  # frame_id of this OccupancyGrid

        print('----------------------')

    def get_height_contours(self):

        # convert negative values (-1 == UNKNOWN) to zero
        pos_grid = np.where(self.ogm._grid_data == -1, 0, self.ogm._grid_data)

        # generate greyscale grid image by scaling from (0,100) to (0,255)
        grid_img = np.array(pos_grid * 2.55).astype('uint8')

        # height "buckets" to quantize the image
        heights = [0, 25, 50, 100, 125, 150, 200, 250]

        # quantize grid_img based on height "buckets"
        # TODO: find/evaulate alternatives to this (expensive, butt-ugly) technique
        for idx in range(0, len(heights)-1):
            grid_img = np.where(np.logical_and(grid_img >= heights[idx], grid_img < heights[idx+1]),
                heights[idx], grid_img)
        
        contours = []

        # get (sorted) list of unique values, will contain a value for each height "region" in the image
        regions = np.unique(grid_img)
        
        # create separate image for each height bucket
        # use (now) binary image to determine contours in each 
        for region in regions:

            # get only the area from the image where the height equals our region height
            # zero out everything else
            area = np.where(grid_img == region, 255, 0).astype('uint8')

            # Gaussian Blur to de-noise
            blur = cv.GaussianBlur(area, (5,5), 0)
        
            # statistical approach to canny parameters from: 
            # https://www.pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/
            sigma = .50
            v = np.median(area)
            minVal = int(max(0, (1.0 - sigma) * v))
            maxVal = int(min(255, (1.0 + sigma) * v))

            # Canny edge detection
            edges = cv.Canny(blur, minVal, maxVal)

            # get contours from region image
            img, cnts, hierarchy = cv.findContours(edges.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            color = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
            img2 = cv.drawContours(color, cnts, -1, (0,255,0), 1)

            # find centers of each contour, get physical location of the centers
            for cnt in cnts:

                # use moments to get center of contour
                M = cv.moments(cnt)
                if M["m00"] != 0:       # avoid DivideByZero
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    continue

                # draw the center of the contour
                cv.circle(img2, (cX, cY), 1, (0, 0, 255), -1)

                rect = cv.minAreaRect(cnt)
                box = cv.boxPoints(rect)
                box = np.int0(box)  # ???
                cv.drawContours(img2,[box],0,(0,0,255),2)

                contour = Contour(label=region, center=Point2D(cX, cY, region), contour=cnt)
                contour.label = region
                contour.pose_rel_to_base = self.ogm.get_world_x_y(contour.center.x, contour.center.y)
                contours.append(contour)


            # plt.subplot(131),plt.imshow(grid_img,cmap = 'gray')
            # plt.title('Original'), plt.xticks([]), plt.yticks([])
            # plt.subplot(132),plt.imshow(area,cmap = 'gray')
            # plt.title('Thresholded'), plt.xticks([]), plt.yticks([])
            # plt.subplot(133),plt.imshow(img2,cmap = 'gray')
            # plt.title('Region: ' + str(region)), plt.xticks([]), plt.yticks([])

            # plt.show()

            return contours

    # returns actual elevations of the points in a contour,
    # rather than the estimated values in get_height_regions
    def get_real_values(self, contour, grid_img):
        pass

def configure_params():
    ''' configure OccupancyGridManagers from loaded YAML config files
        returns list of configured OGMs and list of types
    '''

    print(rospy.get_namespace())

    sources = rospy.get_param('~costmap_processor/sources').split()

    managers = []
    types = []

    for source in sources:
        
        prefix = str('~costmap_processor/' + source)
        
        _topic = rospy.get_param(prefix + '/topic')

        # get 'type' of this source
        if not rospy.has_param(prefix + '/type'):
            rospy.loginfo('parameter \'type\' not defined for {0}; defaulting to heightmap'.format(source))
            _type = '2d'
        else:
            _type = rospy.get_param(prefix + '/type')

        types.append(_type)

        # get 'subscribe_to_updates' of this source
        if not rospy.has_param(prefix + '/subscribe_to_updates'):
            rospy.loginfo('parameter \'subscrive_to_updates\' not defined for {0}; defaulting to TRUE'.format(prefix))
            _subscribe_to_updates = True
        else:
            _subscribe_to_updates = rospy.get_param(prefix + '/subscribe_to_updates')

        _ogm = OccupancyGridManager(_topic, _subscribe_to_updates)
        managers.append(_ogm)
 
    return managers, types

if __name__ == '__main__':

    rospy.init_node('costmap_processor', anonymous=True, log_level=rospy.WARN)

    signal.signal(signal.SIGINT, ctrl_c_handler)

    managers, types = configure_params()

    threads = []
    stop_threads = False

    # zip() to loop through two lists in parallel
    for ogm, type in zip(managers, types):
        # use lambda function to stop threads later
        thread = CostmapProcessor(type, ogm, lambda: stop_threads)
        thread.daemon = True    # so the threads will die when the process does
        threads.append(thread)
        thread.start()

    rospy.spin()

    stop_threads = True

    for thread in threads:
        thread.join()

    # topic1 = '/elevation_map_visualization/slope_grid'
    # topic2 = '/elevation_map_visualization/elevation_grid'

    # # Subscribe to the nav_msgs/OccupancyGrid topic
    # # ogm_1 = OccupancyGridManager(topic1, subscribe_to_updates=True)  # default False
    # ogm_2 = OccupancyGridManager(topic2, subscribe_to_updates=True)  # default False

    # # # get_height_regions(ogm_1._grid_data)
    # print_info(ogm_2)
    # get_height_regions(ogm_2._grid_data, ogm_2)
