#!/usr/bin/env python

import rospy
import random as r
import math
import numpy as np
from copy import deepcopy
from std_msgs.msg import Bool
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from read_config import read_config
from helper_functions import get_pose

class Particle():
	def __init__(self):
		self.x = None
		self.y = None
		self.theta = None
		self.pose = None
		self.weight = 1/800


class Robot():
	def __init__(self):
		rospy.init_node("robot")
		self.config = read_config()

		self.map_data_sub = rospy.Subscriber(
			"/map", 
			OccupancyGrid, 
			self.handle_map_data
		)

		self.particle_pose_pub = rospy.Publisher(
			"/particlecloud",
			PoseArray,
			queue_size = 1
		)

		self.create_particles()		
		self.handle_map()
		rospy.spin()	


	def create_particles(self):
		self.particle_array = []

		self.pose_array = PoseArray()		
		self.pose_array.header.stamp = rospy.Time.now()
		self.pose_array.header.frame_id = 'map'
		self.pose_array.poses = []
		for i in range (800):
			x = r.uniform (range(self.width))
			y = r.uniform (range(self.height))
			theta = r.uniform (range(1))
			pose = get_pose (x,y,theta)	
			particle = Particle()
			particle.x = x
			particle.y = y
			particle.theta = theta
			particle.pose = pose
			self.particle_array.append(particle)
			self.pose_array.poses.append(pose)
			print pose
			

	def handle_map_data(self, data):
		self.tmp_map = data
		self.width = data.info.width
		self.height = data.info.height


	def handle_map(self):
		self.my_map = Map(self.tmp_map)
		array = self.my_map.grid	
		for i in range (grid.length):
			for j in range (grid[0].length):
				print "hi"
			
	
if __name__ == '__main__':
   r = Robot()

