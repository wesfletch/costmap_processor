#!/usr/bin/env python

# system imports
import unittest

# ROS imports
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from costmap_processor import Costmap

import numpy as np


class CostmapTest(unittest.TestCase):

    def setUp(self):

        rospy.init_node("test_costmap", anonymous=True, log_level=rospy.DEBUG)
        
        self.height = 100
        self.width = 100
        self.resolution = 0.05
        self.costy = Costmap(self.width, self.height, self.resolution, origin=None)

    def test_costmap_init(self):

        ''' Test the initialization of the Costmap object '''

        self.assertEqual(self.costy.height, self.height)
        self.assertEqual(self.costy.width, self.width)
        self.assertEqual(self.costy.resolution, self.resolution)

        # make sure our origin is a Pose
        self.assertIsInstance(self.costy.origin, Pose)

    def test_world_point_to_costmap_point(self):

        x = self.costy.origin.position.x
        y = self.costy.origin.position.y

        costmap_x, costmap_y = self.costy.world_point_to_costmap_point(x, y)

        self.assertEqual((costmap_x, costmap_y), (0,0))

        # check that it calculates edges properly
        x_edge = self.costy.resolution * (self.costy.width / 2)  # meters
        y_edge = self.costy.resolution * (self.costy.height / 2)  # meters

        costmap_x, costmap_y = self.costy.world_point_to_costmap_point(x_edge, y_edge)

        self.assertEqual((costmap_x, costmap_y), (self.costy.width, self.costy.height))

        # check negative edges
        costmap_x, costmap_y = self.costy.world_point_to_costmap_point(-1 * x_edge, -1 * y_edge)

        self.assertEqual((costmap_x, costmap_y), (0, 0))


    def test_costmap_point_to_world_point(self):

        # check the origin
        world_x = self.costy.origin.position.x
        world_y = self.costy.origin.position.y

        costmap_x, costmap_y = self.costy.costmap_point_to_world_point(0,0)

        self.assertEqual((world_x, world_y), (costmap_x, costmap_y))

        # check the edges of the map
        world_x = self.costy.origin.position.x + (self.costy.width * self.costy.resolution)
        world_y = self.costy.origin.position.y + (self.costy.height * self.costy.resolution)

        costmap_x, costmap_y = self.costy.costmap_point_to_world_point(self.costy.width, self.costy.height)

        self.assertEqual((world_x, world_y), (costmap_x, costmap_y))

        # check negative values

    def test_mark(self):

        points = [(0.0, 0.0), (1.0, 0.0), (0.0,1.0), (1.0,1.0)]
        
        # marks corners of 1m square with bottom-left at the origin
        self.costy.mark(points, 100)

        costmap_points = [] 
        for point in points:      
            costmap_points.append(self.costy.world_point_to_costmap_point(point[0], point[1]))

        # checking without using Costmap.get_occupied()
        for point in costmap_points:
            x = point[0]
            y = point[1]
            self.assertEqual(self.costy.data[y][x], 100)

    def test_get_occupied(self):

        ''' Tests that the get_occupied() method actually returns all occupied cells '''

        # simple 1m square with the four corners occupied
        points = [(0.0,0.0), (1.0, 0.0), (0.0,1.0), (1.0,1.0)]

        self.costy.mark(points, 100)

        occupied = self.costy.get_occupied()

        self.assertListEqual(points, occupied)

    def test_costmap_to_occ_grid_msg(self):

        msg = self.costy.costmap_to_occ_grid_msg()

        # make sure it's an occupancy grid message
        self.assertIsInstance(msg, OccupancyGrid)

        # make sure message data is correct
        # since we've not marked any points, data should be  array of zeros
        # with length (height * width)
        zeros = np.zeros(self.costy.height * self.costy.width, dtype="uint8")

        # test for array equality
        self.assertIsNone(np.testing.assert_array_equal(msg.data, zeros))

if __name__ == '__main__':

    unittest.main()

    # rospy.init_node('geometry_test', anonymous=True)
    # ocg_pub = rospy.Publisher('ocg', OccupancyGrid, queue_size=10)

    # height = 100        # y-axis
    # width = 100         # x-axis
    # resolution = 0.05

    # costy = Costmap(width, height, resolution)

    # points = [(0.0,0.0), (1.0, 0.0), (0.0,1.0), (1.0,1.0)]

    # # marks corners of 1m square with bottom-left at the origin
    # for point in points:
    #     costy.mark(point, 100)

    # occupied = costy.get_occupied()

    # assert occupied == points, ""

    # rospy.loginfo(costy.get_occupied())

    # # edges of the map
    # x_edge = width * resolution     # meters
    # y_edge = height * resolution    # meters
    # costy.mark([(x_edge, 0)], 100)
    # costy.mark([(0, y_edge)], 100)

    # print(costy.get_occupied())

    # # costy.clear()

    # # # just past the edges of the map (force positive resize)
    # # costy.mark([(x_edge + .05, 0)], 100)
    # # costy.mark([(0, y_edge + .05)], 100)

    # # costy.clear()

    # # # negative points
    # costy.mark([(-1, 0)], 100)
    # costy.mark([(0, -1)], 100)

    # costy.mark([(0.5, -1)], 50)
    # costy.mark([(-5, -5)], 50)

    # print(costy.get_occupied())

    # # og = Pose()
    # # og.position.x = - (costy.width / 2) * resolution
    # # og.position.y = - (costy.height / 2) * resolution
    # # og.position.z = 0

    # # og.orientation.x = 0
    # # og.orientation.y = 0
    # # og.orientation.z = 0
    # # og.orientation.w = 1

    # # costy.origin = og

    # # # costy.clear()

    # # # costy.mark([(-1, -1)], 100)
    # # # costy.mark([(0,0)], 100)

    # # print(costy.get_occupied())

    # msg = costy.costmap_to_occ_grid_msg()

    # # rate = rospy.Rate(1)    # Hz

    # # while not rospy.is_shutdown():

    # #     ocg_pub.publish(msg)
        
    # #     rate.sleep()

