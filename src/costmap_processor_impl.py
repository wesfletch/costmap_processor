#!/usr/bin/env python

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

# system imports
import signal, sys
import threading

class Point2D(object):

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

class Contour(object):

    def __init__(self, label, center, points, pose_rel_to_base=(0,0)):
        self._label = str(label)
        self._center = center
        # TODO: change _points to _points
        self._points = points     
        self._pose_rel_to_base = pose_rel_to_base

    def __str__(self):
        return "label: {}\ncenter: {}\npose: {}\ndata: {}".format(self.label, self.center, self.pose_rel_to_base, self._points)

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

    def __init__(self, source_type, ogm, stop, publish=False):
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

        # TODO: should be param-ed
        rate = rospy.Rate(1)    # loop frequency (Hz)

        if self.publish:

            prefix = '/costmap_processor/'
            topic = self.ogm.topic.split("/")[-1]  # last string in /../../...
            pub_name = prefix + topic

            contour_pub = rospy.Publisher(pub_name, ContourMsg, queue_size=10 )

        while not self.stop:

            # if we received either a full or partial costmap update
            if self.ogm.update:

                # reset OGM update flag
                self.ogm.update = False

                # get new contours of things in our FOV
                # if self.source_type == 'heightmap':
                new_contours = self.get_height_contours()

                # convert points in new_contours to "real" X,Y coords in meters
                self.contours = self.get_frame_coords(new_contours)

                if self.publish:

                    for contour in self.contours:
                        msg = self.contour_to_contour_msg(contour)
                        contour_pub.publish(msg)

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

                # draw the center of the contour
                cv.circle(img2, (cX, cY), 1, (0, 0, 255), -1)

                rect = cv.minAreaRect(cnt)
                box = cv.boxPoints(rect)
                box = np.int0(box)  # ???
                cv.drawContours(img2,[box],0,(0,0,255),2)

                # convert to list of Point2D 
                
                cnt_points = self.contour_to_points(cnt)

                contour = Contour(label=region, center=Point2D(cX, cY, region), points=cnt_points)
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

            # TODO: actually implement this function
            self.remove_duplicates(contours, region_grids)

            return contours

    def contour_to_contour_msg(self, contour):

        msg = ContourMsg()

        msg.contour.header.stamp = rospy.Time.now()
        msg.contour.header.frame_id = self.ogm.reference_frame

        # print(contour.points)

        for idx in range(0, len(contour.points)-1):

            point = Point32()
            point.x = contour.points[idx][0]
            point.y = contour.points[idx][1]
            point.z = 0.0

            # msg.contour.polygon.points[idx].x = contour.points[idx][0]
            # msg.contour.polygon.points[idx].y = contour.points[idx][1]
            # msg.contour.polygon.points[idx].z = 0.0

            msg.contour.polygon.points.append(point)

        # fill the pose
        msg.pose.position.x = contour.pose_rel_to_base[0]
        msg.pose.position.y = contour.pose_rel_to_base[1] 
        msg.pose.position.z = 0 # no contours in the air

        # don't really care about this
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1

        msg.label.data = contour.label

        return msg


    # taks np.ndarray created by np.findContours() and returns list of tuple points
    # TODO: doing this with a loop is hella stupid
    def contour_to_points(self, contour):

        points = []

        # convert to 2D array of points
        contour = np.reshape(contour, (contour.shape[0], 2))

        contour = contour.astype('float32')

        # list() -> numpy scalars
        # toList() -> closest compatible python type, e.g. float
        points = contour.tolist()

        # for idx in range(0, contour.shape[0]):
        #     points.append((contour[idx][0][0], contour[idx][0][1]))

        return points

    def remove_duplicates(self, contours, region_grids):
        
        return contours # modified

    # converts pixel coords into "real-world" coords
    # based on reference frame of costmap
    def get_frame_coords(self, contours):

        # TODO: can I just reshape the entire contours array rather than loop through for performance?
        # thought: reshape may not be necessary, or desirable for either loop
        # for contour in contours:

        #     # change data type to from int to float
        #     # contour.points = contour.points.astype('float32')

        #     # convert each point in the contour to real-world equiv.
        #     for idx in range(0, contour.points.shape[0]):
                
        #         costmap_point = (contour.points[idx][0][0], contour.points[idx][0][1])
        #         world_point = self.ogm.get_world_x_y(costmap_point[0], costmap_point[1])

        #         contour.points[idx][0][0] = world_point[0]
        #         contour.points[idx][0][1] = world_point[1]

        for contour in contours:

            for point in contour.points:

                world_point = self.ogm.get_world_x_y(point[0], point[1])
                point[0], point[1] = world_point


        return contours        

def configure_params():
    ''' configure OccupancyGridManagers from loaded YAML config files
        returns list of configured OGMs and list of types (if YAML exists)
        else, returns None
    '''

    # TODO: add checking for params before trying to actually set them

    managers = []
    source_types = []

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