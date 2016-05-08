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
from map_utils import Map
from sklearn.neighbors import KDTree

class Particle():
	def __init__(self):
		self.x = None
		self.y = None
		self.theta = None
		self.pose = None
		self.weight = np.float32(1/800)

class Robot():
	def __init__(self):
		rospy.init_node("robot")
		self.config = read_config()
		self.num_particles = self.config['num_particles']
		self.laser_sigma_hit = self.config['laser_sigma_hit']
		self.map_data_sub = rospy.Subscriber(
			"/map", 
			OccupancyGrid, 
			self.handle_map_data
		)

		self.particle_pose_pub = rospy.Publisher(
			"/particlecloud",
			PoseArray,
			queue_size = 1,
			latch = True
		)
		
		self.likelihood_pub = rospy.Publisher(
			"/likelihood_field",
			OccupancyGrid,
			queue_size = 1,
			latch = True
		)

		rospy.spin()	


	def create_particles(self):
		self.particle_array = []

		self.pose_array = PoseArray()		
		self.pose_array.header.stamp = rospy.Time.now()
		self.pose_array.header.frame_id = 'map'
		self.pose_array.poses = []
		for i in range (self.num_particles):
			x = r.random() * self.width
			y = r.random() * self.height
			theta = r.random() * 2 * math.pi
			pose = get_pose (x,y,theta)	
			particle = Particle()
			particle.x = x
			particle.y = y
			particle.theta = theta
			particle.pose = pose
			self.particle_array.append(particle)
			self.pose_array.poses.append(pose)
			
		self.particle_pose_pub.publish(self.pose_array)
		self.construct_field()


	def handle_map_data(self, data):
		self.tmp_map = data
		self.width = data.info.width
		self.height = data.info.height
		self.create_particles()		


	def construct_field(self):
		self.my_map = Map(self.tmp_map)
		self.my_map_width = self.my_map.width
		self.my_map_height = self.my_map.height
		array = self.my_map.grid	
		self.kdtree_array = []
		for i in range (self.my_map_width):
			for j in range (self.my_map_height):
				self.coordinate = self.my_map.cell_position(i,j)
			        value = self.my_map.get_cell(self.coordinate[0], self.coordinate[1])	
				if( value == 1.0 ):
					self.kdtree_array.append([j,i])	

		self.kdtree = KDTree (self.kdtree_array)
		result = self.kdtree.query(self.kdtree_array, k=1, return_distance=True)

		self.dist_array = result[0]
		self.indices_array = result[1]
		
		self.update_field()


	def update_field(self):
		for i in range (len(self.dist_array)):
			value = self.calculate (self.dist_array[i])
			self.my_map.set_cell(self.coordinate[0], self.coordinate[1], value)


	def calculate (self, distance):
		power = np.float32(-1 * ((distance*distance)/(2*self.laser_sigma_hit*self.laser_sigma_hit)))
		value = math.pow( math.e, power)
		return value	

	
if __name__ == '__main__':
   r = Robot()

