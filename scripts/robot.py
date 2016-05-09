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
from helper_functions import *
from map_utils import Map
from sensor_msgs.msg import LaserScan
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
		#get move list and set move list length
		self.move_list = self.config['move_list']
		self.move_list_length = len(self.move_list)
		#get first move segima x, y, and theta
		self.first_move_sigma_x = self.config['first_move_sigma_x']
		self.first_move_sigma_y = self.config['first_move_sigma_y']
		self.first_move_sigma_angle = self.config['first_move_sigma_angle']
		self.laser_z_hit = self.config['laser_z_hit']
		self.laser_z_rand = self.config['laser_z_rand']
		self.move_made = 0
		self.map_data_sub = rospy.Subscriber(
			"/map", 
			OccupancyGrid, 
			self.handle_map_data
		)
		
		self.base_scan_data = rospy.Subscriber(
			"/base_scan",
			LaserScan,
			self.handle_base_scan_data
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

	def handle_base_scan_data (self, data):
		self.scan_data = data

	def process_scan_data (self):
		total = 0
		for i in range (self.num_particles):
			pz_array = []
			for j in range (100):
				angle = self.particle_array[i].theta + self.scan_data.angle_min + self.scan_data.angle_increment * j
				x = self.particle_array[i].x + self.scan_data.ranges[j] * cos(angle)
				y = self.particle_array[i].x + self.scan_data.ranges[j] * sin(angle)
				lp = self.my_map.get_cell( x, y )
				pz = (self.laser_z_hit * lp) + self.laser_z_rand
				pz_array.append(pz)
			
			p_tot = 0
			for x in range (len(pz_array)):
				value_cubed = math.pow(pz_array[x],3)
				p_tot = p_tot + value_cubed

			self.particle_array[i].weight = self.particle_array[i].weight * p_tot
			total = total + self.particle_array[i].weight

		# normalize weights
		for j in range (self.num_particles):
			self.particle_array[j].weight = np.float32(self.particle_array[j].weight/total)
			
				
				
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

		# publish OccupancyMap
		grid_mssg = self.my_map.to_message()
		self.likelihood_pub.publish(grid_mssg)


	def handle_map_data(self, data):
		self.tmp_map = data
		self.width = data.info.width
		self.height = data.info.height
		self.create_particles()
		self.make_move()


	def make_move(self):
		i = self.move_made
		#add noise to x, y, theta when first move
		move_function(self.move_list[i][0], 0)
		for j in range(len(self.particle_array)):
			self.particle_array[j].theta += self.move_list[i][0]
		    
		for a in range(self.move_list[i][2]):
			move_function(0, self.move_list[i][1])
			#update x, y, theta and pose
			self.particle_update(i, a)
			if(i == 0):
		   		for a in range (self.num_particles):
			 
			    		self.particle_array[a].x += self.add_first_move_noise(self.particle_array[a].x, self.first_move_sigma_x)
			    		self.particle_array[a].y += self.add_first_move_noise(self.particle_array[a].y, self.first_move_sigma_y)
			    		self.particle_array[a].theta += self.add_first_move_noise(self.particle_array[a].theta, self.first_move_sigma_angle)
			    		self.particle_array[a].pose = get_pose(self.particle_array[a].x, self.particle_array[a].y, self.particle_array[a].theta)
			
			self.process_base_scan_data()	


	def particle_update (self, i, a):
		update_x = self.particle_array[a].x + self.move_list[i][1] * cos(self.move_[i][0] + self.particle_array[a].theta)
		update_y = self.particle_array[a].y + self.move_list[i][1] * sin(self.move_[i][0] + self.particle_array[a].theta)
		self.particle_array[a].x = update_x
		self.particle_array[a].y = update_y
		self.particle_array[a].pose = get_pose(update_x, update_y, self.particle_array[a].theta)

	
	def add_first_move_noise(self, coordinate, sd):
		noise = math.ceil(r.gauss(0, sd)*100.)/100.
		added_noise = coordinate + noise
		return added_noise


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
		power = np.float32(-1 * ((distance*distance)/(2*self.laser_sigma_hit*self.laser_sigma_hit)))
		value = math.pow( math.e, power)
		value = constant * value
		return value	

	
if __name__ == '__main__':
   r = Robot()

