#!/usr/bin/env python

# system imports
import signal
import sys
import threading

# ros imports
import rospy
from occupancy_grid_python import OccupancyGridManager
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32
from costmap_processor.msg import Contour as ContourMsg

# image processing imports
import numpy as np
import cv2 as cv
from PIL import Image as im
from matplotlib import pyplot as plt


class Contour(object):

    def __init__(self, label, center, points, pose_rel_to_base=(0,0)):
        self._label = str(label)
        self._center = center
        self._points = points
        self._pose_rel_to_base = pose_rel_to_base

    def __str__(self):
        return "label: {}\ncenter: {}\npose: {}\ndata: {}".format(self._label, self._center, self._pose_rel_to_base, self._points)

    @property
    def label(self):
        return self._label

    @label.setter
    def label(self, new_label):
        self._label = str(new_label)

    @property
    def center(self):
        return self._center

    @property
    def points(self):
        return self._points

    @points.setter
    def points(self, new_points):
        self._points = new_points

    @property
    def pose_rel_to_base(self):
        return self._pose_rel_to_base

    @pose_rel_to_base.setter
    def pose_rel_to_base(self, new_pose):
        self._pose_rel_to_base = new_pose

    def translate_to_msg(self):
        pass

    def get_dimensions(self):
        # https://arachnoid.com/area_irregular_polygon/index.html
        # https://en.wikipedia.org/wiki/Shoelace_formula
        pass


def ctrl_c_handler(sig, frame):
    sys.exit(0)


class CostmapProcessor(threading.Thread):

    def __init__(self, source_type, ogm, stop, publish=True):
      threading.Thread.__init__(self)
      self._source_type = source_type
      self._ogm = ogm
      self._stop = stop 
      self._contours = []

      self._publish = publish

    @property
    def stop(self):
        # stop is a lambda function, accessed via stop()
        return self._stop()

    @property
    def source_type(self):
        return self._source_type

    @property
    def ogm(self):
        return self._ogm

    @property
    def contours(self):
        return self._contours

    @contours.setter
    def contours(self, new_contours):
        self._contours = new_contours

    @property
    def publish(self):
        return self._publish

    # runs at thread start
    def run(self):

        ''' main loop of thread, checks for costmap updates, then processes them and publishes Contour messages '''

        # TODO: should be param-ed
        rate = rospy.Rate(1)    # loop frequency (Hz)

        if self._publish:

            # name output topic with last part of input topic
            prefix = '/costmap_processor/'
            topic = self._ogm.topic.split("/")[-1]  # last string in /../../...
            pub_name = prefix + topic

            contour_pub = rospy.Publisher(pub_name, ContourMsg, queue_size=10 )

        while not self.stop:

            # if the OGM received either a full or partial costmap update
            if self._ogm.update:

                # reset OGM update flag
                self._ogm.update = False

                # get new contours of things in our FOV
                # if self.source_type == 'heightmap':
                new_contours = self.get_grid_contours()

                # convert points in new_contours to "real" X,Y coords in meters
                self._contours = self.get_frame_coords(new_contours)

                if self._publish:

                    for contour in self._contours:
                        msg = self.contour_to_contour_msg(contour)
                        contour_pub.publish(msg)

            rate.sleep()

    def print_ogm_info(self):

        ''' prints basic info about the costmap messages being recieved by the OGM '''

        print('costmap topic: {0}'.format(self._ogm.topic))

        # Now you can do basic operations
        print('Resolution: {0}'.format(self._ogm.resolution))
        # Note that OccupancyGrid data starts on lower left corner (if seen as an image)
        # width / X is from left to right
        # height / Y is from bottom to top
        print('width: {0}'.format(self._ogm.width))
        print('height: {0}'.format(self._ogm.height))
        print('origin: \n{0}'.format(self._ogm.origin))  # geometry_msgs/Pose
        print('reference_frame: {0}'.format(self._ogm.reference_frame))  # frame_id of this OccupancyGrid

        print('----------------------')

    def get_grid_contours(self):
        
        ''' extracts image contours from grid_data in OGM, then convert to list of Contour objects '''

        # convert negative values (-1 == UNKNOWN) to zero
        pos_grid = np.where(self._ogm.grid_data == -1, 0, self._ogm.grid_data)

        # generate greyscale grid image by scaling from (0,100) to (0,255)
        grid_img = np.array(pos_grid * 2.55).astype('uint8')

        # height "buckets" to quantize the image
        heights = [0, 25, 50, 100, 125, 150, 200, 250]

        # quantize grid_img based on height "buckets"
        # TODO: find/evaulate alternatives to this (expensive, butt-ugly) technique
        for idx in range(0, len(heights)-1):
            grid_img = np.where(np.logical_and(grid_img >= heights[idx], grid_img < heights[idx+1]),
                heights[idx], grid_img)
        
        contours = []       # returned
        region_grids = {}

        # get (sorted) list of unique values, will contain a value for each height "region" in the image
        regions = np.unique(grid_img)
        
        # create separate image for each height bucket
        # use (now) binary image to determine contours in each 
        for region in regions:

            # get only the area from the image where the height equals our region height
            # zero out everything else
            region_grid = np.where(grid_img == region, 255, 0).astype('uint8')
            region_grids[region] = region_grid  # add region_grid to dict for later intersection check

            # Gaussian Blur to de-noise
            blur = cv.GaussianBlur(region_grid, (5,5), 0)
        
            # statistical approach to canny parameters from: 
            # https://www.pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/
            sigma = .50
            v = np.median(region_grid)
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

                # # draw the center of the contour
                # cv.circle(img2, (cX, cY), 1, (0, 0, 255), -1)

                # rect = cv.minAreaRect(cnt)
                # box = cv.boxPoints(rect)
                # box = np.int0(box)  # ???
                # cv.drawContours(img2,[box],0,(0,0,255),2)

                # convert to list of tuple points (x,y)
                cnt_points = self.contour_to_points(cnt)

                # create Contour object and add it to list of contours (to be returned)
                contour = Contour(label=region, center=(cX, cY), points=cnt_points)
                contour.label = region
                contour.pose_rel_to_base = self._ogm.get_world_x_y(contour.center[0], contour.center[1])
                contours.append(contour)

            # plt.subplot(131),plt.imshow(grid_img,cmap = 'gray')
            # plt.title('Original'), plt.xticks([]), plt.yticks([])
            # plt.subplot(132),plt.imshow(area,cmap = 'gray')
            # plt.title('Thresholded'), plt.xticks([]), plt.yticks([])
            # plt.subplot(133),plt.imshow(img2,cmap = 'gray')
            # plt.title('Region: ' + str(region)), plt.xticks([]), plt.yticks([])

            # plt.show()

            # TODO: actually implement this function
            self.remove_duplicates(contours, region_grids)

            return contours

    def contour_to_contour_msg(self, contour):

        ''' takes a Contour object and creates a Contour ROS message for publishing '''

        # create message to be published
        msg = ContourMsg()

        # fill header fields
        msg.contour.header.stamp = rospy.Time.now()
        msg.contour.header.frame_id = self._ogm.reference_frame

        # add points in the contour to message contour.polygon.points
        for idx in range(0, len(contour.points)-1):

            # Point32 message
            point = Point32()
            point.x = contour.points[idx][0]
            point.y = contour.points[idx][1]
            point.z = 0.0

            msg.contour.polygon.points.append(point)

        # fill the pose
        msg.pose.position.x = contour.pose_rel_to_base[0]
        msg.pose.position.y = contour.pose_rel_to_base[1]
        msg.pose.position.z = 0 # no contours in the air, man

        # don't really care about this
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1

        # label the contour
        msg.label.data = contour.label

        msg.metadata = self._ogm.metadata

        return msg

    def contour_to_points(self, contour):

        ''' taks np.ndarray created by np.findContours() and returns list of tuple points '''
        
        # reshape to 2D array of points
        contour = np.reshape(contour, (contour.shape[0], 2))

        # convert to float (from int)
        contour = contour.astype('float32')

        # list() -> numpy scalars
        # toList() -> closest compatible python type, e.g. float
        return contour.tolist()

    # TODO: implement this, dork
    def remove_duplicates(self, contours, region_grids):

        ''' checks for and removes overlapping contours (which represent the same region of space) '''
        
        return contours # modified

    def get_frame_coords(self, contours):

        ''' converts costmap pixel coordinates to "real-world" coordinates in the reference frame '''

        # loop through contour list
        for contour in contours:
            # convert each point in the contour to a point in the world/reference frame
            for point in contour.points:

                world_point = self._ogm.get_world_x_y(point[0], point[1])
                point[0], point[1] = world_point

        return contours

def configure_params():

    ''' configure OccupancyGridManagers from loaded YAML config file
        returns list of configured OGMs and list of types (if YAML exists)
        else, returns None
    '''

    # TODO: add checking for params before trying to actually set them

    managers = []
    source_types = []

    # check for required 'sources' param
    if rospy.has_param('~costmap_processor/sources'):

        sources = rospy.get_param('~costmap_processor/sources').split()

        for source in sources:
        
            prefix = str('~costmap_processor/' + source)
            
            _topic = rospy.get_param(prefix + '/topic')

            # get 'source_type' of this source
            if not rospy.has_param(prefix + '/source_type'):
                rospy.logdebug('parameter \'source_type\' not defined for {0}; defaulting to heightmap'.format(source))
                _source_type = '2d'
            else:
                _source_type = rospy.get_param(prefix + '/source_type')

            source_types.append(_source_type)

            # get 'subscribe_to_updates' of this source
            if not rospy.has_param(prefix + '/subscribe_to_updates'):
                rospy.logdebug('parameter \'subscrive_to_updates\' not defined for {0}; defaulting to TRUE'.format(prefix))
                _subscribe_to_updates = True
            else:
                _subscribe_to_updates = rospy.get_param(prefix + '/subscribe_to_updates')

            _ogm = OccupancyGridManager(_topic, _subscribe_to_updates)
            managers.append(_ogm)

        return managers, source_types
    else:
        rospy.logwarn('parameter \'sources\' not configured in YAML, OccupancyGridManager must be configured by hand')
        return None
    
if __name__ == '__main__':

    rospy.init_node('costmap_processor', anonymous=True, log_level=rospy.WARN)

    signal.signal(signal.SIGINT, ctrl_c_handler)

    managers, source_types = configure_params()

    threads = []
    stop_threads = False

    # zip() to loop through two lists in parallel
    for ogm, source_type in zip(managers, source_types):
        # use lambda function to stop threads later
        thread = CostmapProcessor(source_type, ogm, lambda: stop_threads, publish=True)
        thread.daemon = True    # so the threads will die when the process does
        threads.append(thread)
        thread.start()

    rospy.spin()

    stop_threads = True

    for thread in threads:
        thread.join()