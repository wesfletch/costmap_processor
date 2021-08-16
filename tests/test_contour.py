#!/usr/bin/env python

''' set of tests for the Contour class (obviously not comprehensive) '''

# system imports
import unittest

# ROS imports
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

from costmap_processor import Contour

class ContourTest(unittest.TestCase):

    def setUp(self):

        self.label = ""
        self.center_x = 0
        self.center_y = 0
        
        self.points = []

        self.conty = Contour(
            label=self.label, 
            center=(self.center_x, self.center_y), 
            points=self.points, 
            pose_rel_to_base=(0,0))

    def test_contour_init(self):

        ''' Tests the Contour() constructor '''

        self.assertEqual(self.conty.label, self.label)
        self.assertEqual(self.conty.center, (self.center_x, self.center_y))
        self.assertListEqual(self.conty.points, self.points)
        self.assertEqual(self.conty.pose_rel_to_base, (0,0))


if __name__ == '__main__':

    unittest.main()
