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
		self.weight = (1.0/800.0)

class Robot():
	def __init__(self):
		rospy.init_node("robot")
		self.config = read_config()
		self.num_particles = self.config['num_particles']
		self.laser_sigma_hit = self.config['laser_sigma_hit']
		self.move_list = self.config['move_list']
		self.move_list_length = len(self.move_list)
		self.first_move_sigma_x = self.config['first_move_sigma_x']
		self.first_move_sigma_y = self.config['first_move_sigma_y']
		self.first_move_sigma_angle = self.config['first_move_sigma_angle']
		self.laser_z_hit = self.config['laser_z_hit']
		self.laser_z_rand = self.config['laser_z_rand']
		self.resample_sigma_x = self.config['resample_sigma_x']
		self.resample_sigma_y = self.config['resample_sigma_y']
		self.resample_sigma_theta = self.config['resample_sigma_angle']

		self.handle_map_first_called = 1
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
			queue_size = 10
		)
		
		self.likelihood_pub = rospy.Publisher(
			"/likelihood_field",
			OccupancyGrid,
			queue_size = 1,
			latch = True
		)

		self.result_update_pub = rospy.Publisher(
			"/result_update",
			Bool,
			queue_size = 10
		)

		self.sim_complete_pub = rospy.Publisher(
			"/sim_complete",
			Bool,
			queue_size = 10
		)

		self.scan_data_avai = 0
		#rospy.spin()	
		self.sensor_loop()



	def sensor_loop(self):
		while not rospy.is_shutdown():
			if ( self.handle_map_first_called == 0):
				print "went in while(true) loop"
				self.create_particles()
				self.construct_field()
				print "finished construct field"
				# publish OccupancyMap
				grid_mssg = self.my_map.to_message()
				rospy.sleep(1)
				self.likelihood_pub.publish(grid_mssg)

				self.make_all_moves()
				break
		
		rospy.sleep(1)
		self.sim_complete_pub.publish(True)
		rospy.sleep(1)
		rospy.signal_shutdown(Robot)


	def handle_base_scan_data (self, data):
		self.scan_data = data
		self.scan_data_avai = 1


	def process_scan_data (self):
		while( self.scan_data_avai == 0 ):
			rospy.sleep(1)
		print "process data"
		self.scan_data_avai = 0
		for i in range (self.num_particles):
			pz_array = []
			for j in range (100):
				angle = self.particle_array[i].theta + self.scan_data.angle_min + (self.scan_data.angle_increment * j)
				x = self.particle_array[i].x + self.scan_data.ranges[j] * np.cos(angle)
				y = self.particle_array[i].y + self.scan_data.ranges[j] * np.sin(angle)
				
				### double check with TA!!!!!
				if( np.isnan(x) or np.isnan(y) or np.isinf(x) or np.isinf(y) ):
					lp = 0
				else:
					lp = self.my_map.get_cell( x, y )

				pz = (self.laser_z_hit * lp) + self.laser_z_rand
				pz_array.append(pz)
			
			p_tot = 0
			for a in range (len(pz_array)):
				value_cubed = math.pow(pz_array[a],3)
				if(np.isnan(value_cubed)):
					continue
				else:	
					p_tot = p_tot + value_cubed

			#self.particle_array[i].weight = self.particle_array[i].weight * p_tot

			# set weight to itself if particle goes outside of map
			if( np.isnan(self.particle_array[i].x) or np.isnan(self.particle_array[i].y) or np.isinf(self.particle_array[i].x) or np.isinf(self.particle_array[i].y) ):
				self.particle_array[i].weight = self.particle_array[i].weight
			elif (np.isnan(self.my_map.get_cell(self.particle_array[i].x, self.particle_array[i].y))):
				self.particle_array[i].weight = 0
			else:
				self.particle_array[i].weight = self.particle_array[i].weight * p_tot

			total = total + self.particle_array[i].weight

		self.normalize_weight()	
				

	def create_particles(self):
		self.particle_array = []
		self.pose_array = PoseArray()		
		self.pose_array.header.stamp = rospy.Time.now()
		self.pose_array.header.frame_id = 'map'
		self.pose_array.poses = []
		for i in range (self.num_particles):
			# x and y are coordinates
			x = r.random() * self.my_map_width
			y = r.random() * self.my_map_height
			x, y = self.my_map.cell_position (x,y)
			theta = math.radians(r.random() * 360)
			pose = get_pose (x,y,theta)	
			particle = Particle()
			particle.x = x
			particle.y = y
			particle.theta = theta
			particle.pose = pose
			self.particle_array.append(particle)
			self.pose_array.poses.append(pose)
		
		rospy.sleep(1)	
		self.particle_pose_pub.publish(self.pose_array)


	def handle_map_data(self, data):
		if( self.handle_map_first_called == 1 ):
			self.my_map = Map(data)
			self.my_map_width = self.my_map.width
			self.my_map_height = self.my_map.height
			self.handle_map_first_called = 0
			print "handle map is called"	
		
		"""self.create_particles()
		self.num_moves = len(self.move_list)	
		for i in range (self.num_moves):
			print "make_move"
			self.make_move()
			rospy.sleep(1)
			self.result_update_pub.publish(True)
			self.move_made = self.move_made + 1
		"""
	def make_all_moves(self):
		self.num_moves = len(self.move_list)	
		for i in range (self.num_moves):
			print "make_move"
			self.make_move()
			rospy.sleep(1)
			self.result_update_pub.publish(True)
			self.move_made = self.move_made + 1


	def make_move(self):
		i = self.move_made
		#add noise to x, y, theta when first move
		move_function(self.move_list[i][0], 0)
		for j in range(len(self.particle_array)):
			self.particle_array[j].theta += math.radians(self.move_list[i][0])
		    
		for a in range(self.move_list[i][2]):
			move_function(0, self.move_list[i][1])
			#update x, y, theta and pose
			self.particle_update(i)
			if(i == 0):
				self.particle_add_noise_first_move()

			self.process_scan_data()	
			self.resampling_particle()

	
	def normalize_weight(self):
		total = 0
		total1 = 0
		for x in range (self.num_particles):
			total = total + self.particle_array[x].weight
		for j in range (self.num_particles):
			self.particle_array[j].weight = np.float32(self.particle_array[j].weight/total)
		for k in range (self.num_particles):
			total1 = total1 + self.particle_array[j].weight

		print "sum should be 1: ",total1
			

	def resampling_particle(self):
		"""
		totalWeight = 0	
		new_array = []
		counter = 0
		while (counter < self.num_particles):
			random = r.random()
			for i in range (self.num_particles):
				weight = self.particle_array[i].weight
				totalWeight = totalWeight + weight
				if (random > totalWeight):
					continue
				else:
					new_particle = self.particle_array[i]
					new_x = self.add_resample_noise(new_particle.x, self.resample_sigma_x)
					new_y = self.add_resample_noise(new_particle.y, self.resample_sigma_y)
					new_theta = self.add_resample_noise(new_particle.theta, self.resample_sigma_theta) % math.radians(360)
					new_pose = get_pose(new_x, new_y, new_theta)
					new_particle.x = new_x
					new_particle.y = new_y
					new_particle.theta = new_theta
					new_particle.pose = new_pose
					new_array.append(new_particle)
					totalWeight = 0
					break
			
			counter = counter + 1
		"""
		print "resampling"
		self.weight_array = []
		new_array = []
		self.normalize_weight()	
		for i in range (800):
			self.weight_array.append(self.particle_array[i].weight)

		for j in range (800):
			new_particle = np.random.choice(self.particle_array, None, True, self.weight_array)
			new_x = self.add_resample_noise(new_particle.x, self.resample_sigma_x)
			new_y = self.add_resample_noise(new_particle.y, self.resample_sigma_y)
			new_theta = self.add_resample_noise(new_particle.theta, self.resample_sigma_theta) % math.radians(360)
			new_pose = get_pose(new_x, new_y, new_theta)
			new_particle.x = new_x
			new_particle.y = new_y
			new_particle.theta = new_theta
			new_particle.pose = new_pose
			new_array.append(new_particle)
		
		for m in range (self.num_particles):
			self.particle_array[m] = new_array[m]
			self.pose_array.poses[m] = new_array[m].pose
		
		self.normalize_weight()	
		# publish the pose array
		rospy.sleep(1)
		self.particle_pose_pub.publish(self.pose_array)

		"""
		self.the_list = []
		new_array = []
		ddd = 0
		for i in range (self.num_particles):
			count = 0
			w = self.particle_array[i].weight * self.num_particles
			while (count < w):
				self.the_list.append(i)
				count = count + 1
				ddd = ddd +1

		for k in range (self.num_particles):
			random = r.randint(0, len(self.the_list)-1)
		
			list_index = self.the_list[random]
		
			new_particle = self.particle_array[list_index]
					
			# set weight to 0 if coordinate is out of map
				
			new_x = self.add_resample_noise(new_particle.x, self.resample_sigma_x)
			new_y = self.add_resample_noise(new_particle.y, self.resample_sigma_y)
			new_theta = self.add_resample_noise(new_particle.theta, self.resample_sigma_theta)
			new_pose = get_pose(new_x, new_y, new_theta)
			new_particle.x = new_x
			new_particle.y = new_y
			new_particle.theta = new_theta
			new_particle.pose = new_pose
			new_array.append(new_particle)
			
		for m in range (self.num_particles):
			self.particle_array[m] = new_array[m]
			self.pose_array.poses[m] = new_array[m].pose

		self.the_list = []
		new_array = []
		# publish the pose array
		rospy.sleep(1)
		self.particle_pose_pub.publish(self.pose_array)
		"""


	def particle_update (self, i):
		for a in range (self.num_particles):
			update_x = self.particle_array[a].x + self.move_list[i][1] * np.cos(self.particle_array[a].theta)
			update_y = self.particle_array[a].y + self.move_list[i][1] * np.sin(self.particle_array[a].theta)
			self.particle_array[a].x = update_x
			self.particle_array[a].y = update_y
			self.particle_array[a].pose = get_pose(update_x, update_y, self.particle_array[a].theta)


	def particle_add_noise_first_move(self):
		for a in range (self.num_particles):
			self.particle_array[a].x = self.add_first_move_noise(self.particle_array[a].x, self.first_move_sigma_x)
			self.particle_array[a].y = self.add_first_move_noise(self.particle_array[a].y, self.first_move_sigma_y)
			self.particle_array[a].theta = self.add_first_move_noise(self.particle_array[a].theta, self.first_move_sigma_angle)
			self.particle_array[a].pose = get_pose(self.particle_array[a].x, self.particle_array[a].y, self.particle_array[a].theta % (math.radians(360)))
			

	
	def add_first_move_noise(self, coordinate, sd):
		noise = math.ceil(r.gauss(0, sd)*100.)/100.
		added_noise = coordinate + noise
		return added_noise

	def add_resample_noise(self, coordinate, sd):
		noise = math.ceil(r.gauss(0, sd) * 100.) /100.
		added_noise = coordinate + noise
		return added_noise
	
	def construct_field(self):
		self.kdtree_array = []
		#self.query_array = []
		#self.grid_array= []
		for i in range (self.my_map_height):
			for j in range (self.my_map_width):
				coordinate = self.my_map.cell_position(i,j)
				#self.query_array.append(coordinate)
			        value = self.my_map.get_cell(coordinate[0], coordinate[1])	
				#self.grid_array.append(i,j)
				if( value == 1.0 ):
					self.kdtree_array.append([coordinate[0], coordinate[1]])	

		self.kdtree = KDTree (self.kdtree_array)
		self.update_field()


	def update_field(self):
		"""value = self.kdtree.query(self.query_array, k=1)
		for i in range (len(self.dist_array)):
			new_value = self.calculate (value[0][i])
			r, c = self.grid_array[i]
			coordinate = self.my_map.cell_position(r,c)
			if( self.my_map.get_cell(coordinate[0], coordinate[1]) == 0.0):
				self.my_map.set_cell(coordinate[0], coordinate[1], new_value)
		"""	
		for i in range (self.my_map_height):
			for j in range (self.my_map_width):
				coordinate = self.my_map.cell_position(i,j)
				value = self.kdtree.query([[coordinate[0],coordinate[1]]], k=1)
				new_value = self.calculate (value[0][0])
				coordinate = self.my_map.cell_position(i,j)
				if( self.my_map.get_cell(coordinate[0], coordinate[1]) == 0.0):
					self.my_map.set_cell(coordinate[0], coordinate[1], new_value)


	def calculate (self, distance):
		"""constant = 2 * math.pi
		constant = np.float32(math.sqrt(constant)) * self.laser_sigma_hit
		constant = np.float32(1.0/constant)
		"""
		power = np.float32(-1 * ((distance*distance)/(2*self.laser_sigma_hit*self.laser_sigma_hit)))
		value = math.pow( math.e, power)
		#value = constant * value
		return value	

	
if __name__ == '__main__':
   r = Robot()

