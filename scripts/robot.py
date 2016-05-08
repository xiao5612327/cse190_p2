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
		for i in range (self.my_map_height):
			for j in range (self.my_map_width):
				coordinate = self.my_map.cell_position(i,j)
			        value = self.my_map.get_cell(coordinate[0], coordinate[1])	
				if( value == 1.0 ):
					self.kdtree_array.append([i,j])	

		self.kdtree = KDTree (self.kdtree_array)
		self.update_field()


	def update_field(self):
		for i in range (self.my_map_height):
			for j in range (self.my_map_width):
				value = self.kdtree.query([[i,j]], k=1)
				new_value = self.calculate (value[0][0])
				coordinate = self.my_map.cell_position(i,j)
				self.my_map.set_cell(coordinate[0], coordinate[1], new_value)


	def calculate (self, distance):
		constant = 2 * math.pi
		constant = np.float32(math.sqrt(constant)) * self.laser_sigma_hit
		constant = np.float32(1.0/constant)
		power = np.float32(-1 * 
			((distance*distance)/(2*self.laser_sigma_hit*self.laser_sigma_hit)))
		value = math.pow( math.e, power)
		value = constant * value
		return value	

	
if __name__ == '__main__':
   r = Robot()

