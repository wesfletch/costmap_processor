#!/usr/bin/env python

''' test suite for the Costmap object and it's functions '''

# system imports
import unittest
import random

# ROS imports
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from costmap_processor import Costmap

# general imports
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

        # test the very edges of the Costmap
        point = self.costy.costmap_point_to_world_point(self.width-1, self.height-1)
        self.costy.mark([point], 100)

        occupied = self.costy.get_occupied()
        points.append(point)

        self.assertListEqual(points, occupied)

    def test_get_cell_cost(self):

        points = [(0.0,0.0), (1.0, 0.0), (0.0,1.0), (1.0,1.0)]

        self.costy.mark(points, 100)

        occupied = self.costy.get_occupied()

        for point in occupied:
            costmap_point = self.costy.world_point_to_costmap_point(point[0], point[1])
            self.assertEqual(100, self.costy.get_cell_cost(costmap_point[0], costmap_point[1]))

    def test_costmap_to_occ_grid_msg(self):

        msg = self.costy.costmap_to_occ_grid_msg()

        # make sure it's an occupancy grid message
        self.assertIsInstance(msg, OccupancyGrid)

        # make sure message data is correct
        # since we've not marked any points, data should be  array of zeros
        # with length (height * width)
        zeros = np.zeros(self.costy.height * self.costy.width, dtype="uint8")

        # test for array equality using np's testing module
        self.assertIsNone(np.testing.assert_array_equal(msg.data, zeros))

    def test_expand(self):

        ''' Tests the expand functionality '''

        # ensure that no expansion doesn't change the dimensions
        self.costy.expand(delta_width=0,delta_height=0)
        self.assertTupleEqual((self.costy.width, self.costy.height),(self.width, self.height))
        origin_x = self.costy.origin.position.x

        # test positive (right of zero) width expansion
        self.costy.expand(delta_width=50, delta_height=0)
        # width dimension should have increased by 50
        self.assertTupleEqual((self.costy.width, self.costy.height), (self.width+50, self.height))
        # origin shouldn't have moved since we expanded to the right
        self.assertEqual(origin_x, self.costy.origin.position.x)
        origin_x = self.costy.origin.position.x

        # test negative (left of zero) width expansion
        self.costy.expand(delta_width=-50, delta_height=0)
        # test dimensions - costmap has dimension w=200, h=100 (np.shape(100,200))
        self.assertTupleEqual((self.costy.width, self.costy.height), (self.width+100, self.height))
        # test origin update - origin should have moved 50 pixels to the left
        neg_origin = origin_x + (-50 * self.resolution)
        self.assertEqual(neg_origin, self.costy.origin.position.x)

        # origin_x, origin_y = (self.costy.origin.position.x, self.costy.origin.position.y)

        # reset the Costmap
        self.costy = Costmap(self.width, self.height, self.resolution, origin=None)

        # test whether costmap expansion maintains point positions
        # get costmap (0,0) and mark it
        zero_x, zero_y = self.costy.costmap_point_to_world_point(self.width/2, self.height/2)
        self.costy.mark({(zero_x,zero_y)}, 100) # (0.0, 0.0) meters
        # expand the costmap
        self.costy.expand(delta_width=-10, delta_height=0)
        # recalculate the position of the point
        zero_x, zero_y = self.costy.world_point_to_costmap_point(0.0, 0.0)
        # the point should be in the same place (in meters) as before
        # when accessed directly as in data[y][x], costmap data is [height][width]
        # however, using get_cell_cost() returns [width][height]
        self.assertEqual(100, self.costy.data[zero_y][zero_x])  

    def test_create_from_metadata(self):

        meta = MapMetaData()
        meta.resolution = self.resolution
        meta.height = self.height
        meta.width = self.width
        meta.origin = None

        new_costy = Costmap.create_from_metadata(meta)

        self.assertEquals(self.costy.resolution, new_costy.resolution)
        self.assertEquals(self.costy.height, new_costy.height)
        self.assertEquals(self.costy.width, new_costy.width)
        self.assertEquals(self.costy.origin, new_costy.origin)

    def test_data_setter(self):

        new_data = np.zeros((150, 150))
        # self.costy.width = 150
        # self.costy.height = 150
        
        # ensure that arrays with the incorrect number of elements cannot replace data
        # i.e. if dimensions of Costmap are (100, 100), setting data with array dimensions (150, 150) should fail
        # this SHOULD throw a rospy error, don't let the red text scare you
        self.costy.data = new_data
        self.assertFalse(np.array_equal(self.costy.data, new_data))

        # after adjusting the dimensions of Costmap to match new_data, setting the data should work
        self.costy.width = 150
        self.costy.height = 150
        self.costy.data = new_data
        self.assertIsNone(np.testing.assert_array_equal(self.costy.data, new_data))

        # make sure that it's actually keeping all of the data we set using a bunch of random points
        points = []
        # values = []

        for i in range(100):
            # select random point in new_data...
            x = random.randint(0, new_data.shape[1]-1)
            y = random.randint(0, new_data.shape[0]-1)

            # then set it to random value
            value = random.randint(1, 100)
            
            # and mark it in new_data
            new_data[y][x] = value

            # save it to check later
            points.append((x, y))
            # values.append(value)
        
        # set the costmap data to include our random points
        self.costy.data = new_data
        occupied = self.costy.get_occupied()    # <-- should only contain random points from above

        world_points = []
        # get world-frame values of the points we set
        for i in range(100):
            x, y = self.costy.costmap_point_to_world_point(points[i][0], points[i][1])
            world_points.append((x,y))

        # sort the lists by x value
        world_points.sort(key=lambda tup: tup[0])
        occupied.sort(key=lambda tup: tup[0])

        # make sure all x values are the same
        for i in range(100):
            self.assertEqual(world_points[i][0], occupied[i][0])

        # sort by y value
        world_points.sort(key=lambda tup: tup[1])
        occupied.sort(key=lambda tup: tup[1])

        # make sure all y values are the same
        for i in range(100):
            self.assertEqual(world_points[i][1], occupied[i][1])


if __name__ == '__main__':

    unittest.main()