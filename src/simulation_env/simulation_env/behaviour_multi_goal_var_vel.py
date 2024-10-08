from math import radians
import random
import numpy as np
from time import time
import matplotlib.pyplot as plt

from matplotlib.patches import Ellipse, Rectangle
from numpy import dtype
from shapely.geometry import Point
from shapely.affinity import scale, rotate

import yaml
from yaml.loader import SafeLoader	

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from acado_msgs.srv import GetVelocityCmd
from acado_msgs.srv import GetControlsMulti
from acado_msgs.msg import OdomArray

class Behaviour(Node):
	def __init__(self):
		super().__init__('behaviour_tests')
		self.cli = self.create_client(GetControlsMulti, "/get_vel")
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
			
		self.lane_ids = []
		self.radius = 116.0
		self.agent_pose = [0.0, -2.0, 0.0]
		self.agent_vel = [15.0, 0.0]
		self.dt = 0.1

		self.lane_y = [-2.0, 2.0]
		self.radius = 30.0
		self.dist_goal = 80.0
		self.goal = np.array([100.0, random.choice(self.lane_y) ])
		with open('src/simulation_env/config.yaml') as f:
			data = yaml.load(f, Loader=SafeLoader)
			setting = str(data["behaviour"])

		self.obs_vel = np.array([10.0, 11.0, 9.0, 10.5, 9.5, 10.2, 10.8, 9.2, 9.8, 10.0])
		self.nearest_obstacles = []
		self.all_cruise_speed = []
		self.all_obs_res = []
		self.all_ang_speed = []
		self.all_min_cost = []
		self.all_goal_costs = []
		self.wait = False

		# Use this for overtake and combined with pedestrian
		# self.obs = np.array([[-10.0, -2.0, 0.0, 10.0, 0.0],
		# 					[25.0, 2.0, 0.0, 11.0, 0.0],
		# 					[30.0, -2.0, 0.0, 9.0, 0.0],
		# 					[65.0, 2.0, 0.0, 10.5, 0.0],
		# 					[65.0, -2.0, 0.0, 9.5, 0.0],
		# 					[95.0, 2.0, 0.0, 10.2, 0.0],
		# 					[100.0, -2.0, 0.0, 10.8, 0.0],
		# 					[140.0, 2.0, 0.0, 9.2, 0.0],
		# 					[135.0, -2.0, 0.0, 9.8, 0.0],
		# 					[-30.0, 2.0, 0.0, 10.0, 0.0]])
		self.obs = np.array([[-10.0, -2.0, 0.0, -10.0, 0.0],
							[25.0, 2.0, 0.0, 11.0, 0.0],
							[30.0, -2.0, 0.0, -9.0, 0.0],
							[65.0, 2.0, 0.0, 10.5, 0.0],
							[65.0, -2.0, 0.0, -9.5, 0.0],
							[95.0, 2.0, 0.0, 10.2, 0.0],
							[100.0, -2.0, 0.0, -10.8, 0.0],
							[140.0, 2.0, 0.0, 9.2, 0.0],
							[135.0, -2.0, 0.0, -9.8, 0.0],
							[-30.0, 2.0, 0.0, 10.0, 0.0]])

		# Use this for follow vehicle only behaviour
		"""self.obs = np.array([[10.0*1000, -2.0, 0.0, 10.0, 0.0],
							[25.0*1000, 2.0, 0.0, 11.0, 0.0],
							[40.0*1000, -2.0, 0.0, 9.0, 0.0],
							[85.0*1000, 2.0, 0.0, 10.5, 0.0],
							[85.0*1000, -2.0, 0.0, 9.5, 0.0],
							[135.0*1000, 2.0, 0.0, 10.2, 0.0],
							[140.0*1000, -2.0, 0.0, 10.8, 0.0],
							[180.0*1000, 2.0, 0.0, 9.2, 0.0],
							[185.0*1000, -2.0, 0.0, 9.8, 0.0],
							[50.0*1000, 2.0, 0.0, 10.0, 0.0]])"""
		self.obs_pedestrian = np.array([[1200000000.0, -6.0, 1.5707963, 0.0, 0.0]])
		self.sign = -1

		self.behaviour_event = 1

		self.time_arr = []
		self.time_arr = np.linspace(0, 5, 51)
		self.vel = []
		self.loop = 0
		self.move = True

		self.xlim = 0.0

		self.flag = 1
		# self.fig = plt.figure(0)
		# self.ax1 = self.fig.add_subplot(211, aspect='equal')
		# self.ax2 = self.fig.add_subplot(212, aspect='equal')
		# mng = plt.get_current_fig_manager()
		# # mng.full_screen_toggle()
		# self.fig.set_size_inches(20, 10)
		self.req = GetControlsMulti.Request()
		print("STARTING SIMULATION")

	def send_request(self):
		self.req.start.pose.pose.position.x = self.agent_pose[0]
		self.req.start.pose.pose.position.y = self.agent_pose[1]
		self.req.start.pose.pose.orientation.z = self.agent_pose[2]
		self.req.start.twist.twist.linear.x = self.agent_vel[0]
		self.req.start.twist.twist.angular.z = self.agent_vel[1]

		obstacles = OdomArray()
		self.nearest_obstacles = []
		"""odom = Odometry()
		odom.pose.pose.position.x = self.obs[0][0]
		odom.pose.pose.position.y = self.obs[0][1]
		odom.pose.pose.orientation.z = self.obs[0][2]
		odom.twist.twist.linear.x = self.obs[0][3]
		odom.twist.twist.angular.z = self.obs[0][4]
		#obs = plt.Circle((sorted_obs[i][0], sorted_obs[i][1]), 1.2, color='g')
		#self.ax1.add_patch(obs)
		obstacles.odom.append(odom)
		for i in range(9):
			odom = Odometry()
			odom.pose.pose.position.x = 10000.0#self.obs[i][0]
			odom.pose.pose.position.y = 10000.0#self.obs[i][1]
			odom.pose.pose.orientation.z = 10000.0#self.obs[i][2]
			odom.twist.twist.linear.x = 10000.0#self.obs[i][3]
			odom.twist.twist.angular.z = 10000.0#self.obs[i][4]
			#obs = plt.Circle((sorted_obs[i][0], sorted_obs[i][1]), 1.2, color='g')
			#self.ax1.add_patch(obs)
			obstacles.odom.append(odom)"""
		for i in range(10):
			odom = Odometry()
			odom.pose.pose.position.x = self.obs[i][0]
			odom.pose.pose.position.y = self.obs[i][1]
			odom.pose.pose.orientation.z = self.obs[i][2]
			odom.twist.twist.linear.x = self.obs[i][3]
			odom.twist.twist.angular.z = self.obs[i][4]
			#obs = plt.Circle((sorted_obs[i][0], sorted_obs[i][1]), 1.2, color='g')
			#self.ax1.add_patch(obs)
			obstacles.odom.append(odom)
			obs_path_x = self.obs[i][0] + self.obs[i][3]*self.time_arr
			obs_path_y = self.obs[i][1] + self.obs[i][4]*self.time_arr
			self.nearest_obstacles.append(np.concatenate((obs_path_x, obs_path_y)))
		self.req.obstacles = obstacles
		self.nearest_obstacles = np.array(self.nearest_obstacles)
		goal = PoseArray()
		lane_cons = PoseArray()

		if self.sign>0:
			if abs(self.agent_pose[0]-self.obs_pedestrian[0][0])<15 and (self.obs_pedestrian[0][1] - 0)>-2.0:
				self.obs_pedestrian[0][3] = 2.0
				self.move = False
			else:
				self.move = True
		else:
			if abs(self.agent_pose[0]-self.obs_pedestrian[0][0])<15 and (self.obs_pedestrian[0][1] - 0)<2.0:
				self.obs_pedestrian[0][3] = 2.0
				self.move = False
				lane_cons, goal = self.get_lane_cons_stop()
			else:
				self.move = True
				lane_cons, goal = self.get_lane_cons_overtake()
		if self.move:
			lane_cons, goal = self.get_lane_cons_overtake()
		else:
			lane_cons, goal = self.get_lane_cons_stop()
		if self.agent_pose[0]-self.obs_pedestrian[0][0]>20:
			signs = [-1, 1]
			self.sign = random.choice(signs)
			self.obs_pedestrian[0][0] = self.agent_pose[0]+random.choice([80, 120, 160, 200])*2
			self.obs_pedestrian[0][1] = self.sign*6
			self.obs_pedestrian[0][2] = -self.sign*np.pi/2#self.obs_pedestrian[0][2]
			self.obs_pedestrian[0][3] = 0.0
		
		# for i in range(len(self.obs)):
		# 	if self.obs[i][3]<self.obs_vel[i] and self.move:
		# 		self.obs[i][3] = self.obs[i][3] + 4.0*self.dt
		# 		# print("++++++++++++++++++++++++++", self.obs[i][3], self.obs_vel[i])
		
		#lane_cons, goal = self.br_cons(0.0)
		self.req.goal = goal
		self.req.lane_cons = lane_cons
		#print("Goal = ", goal, self.behaviour_event)
		#print(self.obs[:,0])
		#print("Lane Cons == ", self.req.lane_cons)
		self.goal_p = goal
		self.future = self.cli.call_async(self.req)

	def get_lane_cons_overtake(self):
		self.wait = False
		# print("Overtake")
		lane_info = PoseArray()
		goals = PoseArray()
		lanes = [-2.0, 2.0, -2.0, 2.0]
		self.lane_y = lanes
		#lanes = [-2.0]
		g_dist = 50.0
		xgoal = [self.dist_goal, self.dist_goal, self.dist_goal*3/4, self.dist_goal*3/4]
		goal_pose = Pose()
		goal_pose.position.x = self.agent_pose[0] + self.dist_goal*3/4 + 10.0
		dists = np.abs(self.agent_pose[1] - np.array(self.lane_y))
		gy = np.argmin(dists)
		goal_pose.position.y = np.clip(self.agent_pose[1], -2, 2)#self.lane_y[gy]	#lanes[0]#random.choice(lanes)
		goal_pose.orientation.z = 0.0
		goals.poses.append(goal_pose)
		for i in range(len(lanes)):
			goal_pose = Pose()
			goal_pose.position.x = self.agent_pose[0] + xgoal[i]
			goal_pose.position.y = lanes[i]#lanes[0]#random.choice(lanes)
			goal_pose.orientation.z = 0.0
			goals.poses.append(goal_pose)
		info = Pose()
		info.position.x = 0.0
		info.position.y = 0.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 1.0  #goal_pose.position.y+1e12
		info.position.z = -goal_pose.position.y  #1e12
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 0.0 #26.0 + 1e12
		info.position.z = 0.0 #1e12
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = 3.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 0.0 #self.agent_pose[0]
		info.position.y =  0.0 #-4.0 + 1e12
		info.position.z = 0.0
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = -3.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = -10.0
		info.position.y = 10.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 5*1e4
		info.position.y = 5*1e4
		lane_info.poses.append(info)
		
		info = Pose()   #min rad cons
		info.position.x = 7.0*1e3
		lane_info.poses.append(info)

		info = Pose()   #behaviour id
		info.position.x = 1.0
		lane_info.poses.append(info)
		return lane_info, goals
		# info = Pose()   #min max acc constraints
		# info.position.x = -2.0
		# info.position.y = 2.0
		# lane_info.poses.append(info)

		# info = Pose()   #terminal position weights
		# info.position.x = 2.5*1e3
		# info.position.y = 2.5*1e3
		# lane_info.poses.append(info)
		
		# info = Pose()   #linear aceleration weights
		# info.position.x = 1.0*1e3
		# lane_info.poses.append(info)

		# info = Pose()   #behaviour id
		# info.position.x = 1.0
		# lane_info.poses.append(info)

		# return lane_info, goals
	
	def get_lane_cons_stop(self):
		self.wait = True
		# print("Stop ", self.obs_pedestrian[0])
		#if self.agent_vel[0] < 0.1:
		#	self.obs[0][3] = 2.0
		for i in range(len(self.obs)):
			if self.obs[i][3]>0.0:
				self.obs[i][3] = np.maximum(self.obs[i][3] - 8.0*self.dt, 0.0)
				#print("------------------",self.obs[i][3], self.obs_vel[i])

		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [-2.0, 2.0]
		lanes = [-2.0]
		g_dist = 50.0
		"""move = False
		if (self.sign>0 and (self.obs_pedestrian[0][1] - 0)<-2.0) or (self.sign<0 and (self.obs_pedestrian[0][1] - 0)>2.0):
			move = True
			print("MOVE")

		if move:
			goal_pose.position.x = self.agent_pose[0]+80.0
			self.behaviour_event = 1
			for i in range(len(self.obs)):
				if self.obs[i][3]<self.obs_vel[i] and move:
					self.obs[i][3] = self.obs[i][3] + 4.0*self.dt
					#print("***********************", self.obs[i][3], self.obs_vel[i])
		else:"""
		goal_pose.position.x = self.obs_pedestrian[0][0] - 5.0
		#goal_pose.position.x = self.obs[0][0] - 5.0
		dists = np.abs(self.agent_pose[1] - np.array(self.lane_y))
		gy = np.argmin(dists)
		goal_pose.position.y = np.clip(self.agent_pose[1], -2, 2)#self.lane_y[gy]	#random.choice(lanes)
		goal_pose.orientation.z = 0.0
		goals = PoseArray()
		goals.poses.append(goal_pose)
		lanes = [-2.0, 2.0, -2.0, 2.0]
		self.lane_y = lanes
		xgoal = [self.dist_goal, self.dist_goal, self.dist_goal*1/2, self.dist_goal*1/2]
		for i in range(len(lanes)):
			goal_pose = Pose()
			goal_pose.position.x = self.agent_pose[0] + xgoal[i]
			goal_pose.position.y = lanes[i]#lanes[0]#random.choice(lanes)
			goal_pose.orientation.z = 0.0
			goals.poses.append(goal_pose)
		info = Pose()
		info.position.x = 0.0
		info.position.y = 0.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 1.0  #goal_pose.position.y+1e12
		info.position.z = -goal_pose.position.y  #1e12
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 0.0 #26.0 + 1e12
		info.position.z = 0.0 #1e12
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = 3.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 0.0 #self.agent_pose[0]
		info.position.y =  0.0 #-4.0 + 1e12
		info.position.z = 0.0
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = -3.0
		lane_info.poses.append(info)

		info = Pose()   #min max acc constraints
		info.position.x = -100.0
		info.position.y = 10.0
		lane_info.poses.append(info)

		info = Pose()   #terminal position weights
		info.position.x = 5.0*1e5
		info.position.y = 5.0*1e5
		lane_info.poses.append(info)

		info = Pose()   #linear aceleration weights
		info.position.x = 5.0*1e2
		lane_info.poses.append(info)

		return lane_info, goals
	
	def update_agent(self, twist):
		self.agent_vel[0] = twist.linear.x
		self.agent_vel[1] = twist.angular.z
		self.agent_pose[2] = self.agent_pose[2] + self.agent_vel[1]*self.dt
		self.agent_pose[0] = self.agent_pose[0] + self.agent_vel[0]*np.cos(self.agent_pose[2])*self.dt
		self.agent_pose[1] = self.agent_pose[1] + self.agent_vel[0]*np.sin(self.agent_pose[2])*self.dt
		#self.ax2.plot(self.agent_vel[0])
	
	def update_obstacles(self):
		for i in range(10):
			self.obs[i][2] = self.obs[i][2] + self.obs[i][4]*self.dt
			self.obs[i][1] = self.obs[i][1] + self.obs[i][3]*np.sin(self.obs[i][2])*self.dt
			self.obs[i][0] = self.obs[i][0] + self.obs[i][3]*np.cos(self.obs[i][2])*self.dt
			if self.obs[i][3]<0:
				if self.obs[i][0]-self.agent_pose[0]<-30.0:
					self.obs[i][0] = self.obs[i][0] + 250.0
			if self.obs[i][3]>0:
				if self.obs[i][0]-self.agent_pose[0]<-70.0:
					self.obs[i][0] = self.obs[i][0] + 150.0
		self.obs_pedestrian[0][2] = self.obs_pedestrian[0][2] + self.obs_pedestrian[0][4]*self.dt
		self.obs_pedestrian[0][1] = self.obs_pedestrian[0][1] + self.obs_pedestrian[0][3]*np.sin(self.obs_pedestrian[0][2])*self.dt
		self.obs_pedestrian[0][0] = self.obs_pedestrian[0][0] + self.obs_pedestrian[0][3]*np.cos(self.obs_pedestrian[0][2])*self.dt
	
	def plot_lanes(self):
		plt.plot([-20000, 20000], [4, 4], color='black', linewidth=2.0)
		plt.plot([-20000, 20000], [0, 0], color='black', linewidth=1.0)
		plt.plot([-20000, 20000], [-4, -4], color='black', linewidth=2.0)
	
	def plot_obstacles(self):
		for i in range(10):
			obs = plt.Circle((self.obs[i][0], self.obs[i][1]), 1.0, color='r')
			plt.gca().add_patch(obs)
		obs = plt.Circle((self.obs_pedestrian[0][0], self.obs_pedestrian[0][1]), 1.0, color='r')
		plt.gca().add_patch(obs)
		agent = plt.Circle((self.agent_pose[0], self.agent_pose[1]), 1.0, color='g')
		plt.text(self.xlim, 23, 'Vel = %s'%(round(self.agent_vel[0],2)), fontsize=10)
		# plt.text(self.agent_pose[0]-20, 23, 'Vel = %s'%(round(self.agent_vel[0],2)), fontsize=10)
		"""if int(self.behaviour_event) == 2:
			plt.text(self.xlim+50, 33, 'Behaviour = Follow', fontsize=10)
		elif int(self.behaviour_event) == 1:
			plt.text(self.xlim+50, 33, 'Behaviour = Overtake', fontsize=10)
		elif int(self.behaviour_event) == 3:
			plt.text(self.xlim+50, 33, 'Behaviour = Wait for Vehicle to Cross', fontsize=10)
		elif int(self.behaviour_event) == 8:
			plt.text(self.xlim+50, 33, 'Behaviour = Increase Speed', fontsize=10)
		elif int(self.behaviour_event) == 9:
			plt.text(self.xlim+50, 33, 'Behaviour = Decrease Speed', fontsize=10)"""

		plt.gca().add_patch(agent)
	
	def on_press(self, event):
		self.behaviour_event = event.key

	
	def plot(self, twist, path, kkt):
		plt.ion()
		#plt.show()
		plt.clf()
		#self.ax1 = self.fig.add_subplot(211, aspect='equal')
		#self.ax2 = self.fig.add_subplot(212, aspect='equal')
		self.path_x = []
		self.path_y = []
		self.path_theta = []
		self.path_v = []
		self.path_w = []
		#print(len(path.poses))
		for i in path.poses:
			self.path_x.append(i.position.x)
			self.path_y.append(i.position.y)
			self.path_theta.append(i.position.z)
			self.path_v.append(i.orientation.x)
			self.path_w.append(i.orientation.y)
		self.path_x = np.array(self.path_x)
		self.path_y = np.array(self.path_y)
		self.path_theta = np.array(self.path_theta)
		self.path_v = np.array(self.path_v)
		self.path_w = np.array(self.path_w)

		## Get Ranks
		y_dist = []
		kkt_cost = []
		cruise_speed = []
		angular_vel = []
		obs_cost = []
		total_obs_in_rad = 1
		target_cruise_speed = 30.0
		for i in range(len(self.obs)):
			dist = np.sqrt((self.agent_pose[0] - self.obs[i][0])**2 + (self.agent_pose[1] - self.obs[i][1])**2)
			if dist<self.radius:
				target_cruise_speed = target_cruise_speed + abs(self.obs[i][3])
				total_obs_in_rad = total_obs_in_rad + 1
		target_cruise_speed = target_cruise_speed/total_obs_in_rad
		print(target_cruise_speed)
		plt.text(self.xlim+30, 23, 'Target Cruise Velocity = %s'%(round(target_cruise_speed,2)), fontsize=10)
		# plt.text(self.agent_pose[0]+30, 23, 'Target Cruise Velocity = %s'%(round(target_cruise_speed,2)), fontsize=10)


		for i in range(len(self.goal_p.poses)):
			dist_x = 0.0
			dist_y = 0.0
			dists = []
			for j in range(10):
				dist_x = np.abs(self.path_x[i*51:i*51 + 51] - self.nearest_obstacles[j, :51])
				dist_y = np.abs(self.path_y[i*51:i*51 + 51] - self.nearest_obstacles[j, 51:])
				dists.append(np.min((dist_x**2 + dist_y**2)**0.5)) 
			# dists = (dist_x**2 + dist_y**2)**0.5
			res_obs = np.min(dists)
			print(res_obs)
			obs_cost.append(1/res_obs)
			# res_obs_x = np.linalg.norm(self.path_x[i*51:i*51 + 51] - self.nearest_obstacles[:, :51], axis=1)
			# res_obs_y = np.linalg.norm(self.path_y[i*51:i*51 + 51] - self.nearest_obstacles[:, 51:], axis=1)
			# res_obs = np.vstack((res_obs_x, res_obs_y))
			# res_obs = np.linalg.norm(res_obs, axis=0)
			# obs_cost.append(1/np.min(res_obs))
			y_dist.append(np.linalg.norm(self.path_y[i*51:i*51 + 51] - 14.0))
			kkt_cost.append(kkt[i])
			cruise_speed.append(np.linalg.norm(self.path_v[i*51:i*51 + 51] - target_cruise_speed))
			angular_vel.append(np.linalg.norm(self.path_w[i*51:i*51 + 51] - 0.0))
		# quit()
		obs_idx = np.array(np.array(obs_cost)).argsort().argsort()
		y_idx = np.array(np.array(y_dist)).argsort().argsort()
		kkt_idx = np.array(np.array(kkt_cost)).argsort().argsort()
		cruise_idx = np.array(np.array(cruise_speed)).argsort().argsort()
		ang_idx = np.array(np.array(angular_vel)).argsort().argsort()
		index = np.inf
		min_cost = np.inf
		costs = []
		for i in range(len(self.goal_p.poses)):
			cost = 0*y_idx[i] + 1*obs_idx[i] + 2*cruise_idx[i] + 0*ang_idx[i]
			# cost = 0*y_idx[i] + 2*obs_idx[i] + 0*cruise_idx[i] + 0*ang_idx[i]
			costs.append(cost)
			# cost = 0*y_idx[i] + 5*obs_idx[i] + 2*cruise_idx[i] + 2*ang_idx[i]
			if cost<min_cost:
				min_cost = cost
				index = i
		
		if self.wait:
			index = 0
		self.index = index
		goals = []
		for i in range(len(self.goal_p.poses)):
			if i == index:
				plt.plot(self.path_x[i*51:i*51 + 51], self.path_y[i*51:i*51 + 51], 'g')
				twist.linear.x = self.path_v[i*51+1]
				twist.angular.z = self.path_w[i*51+1]
				goals.append([self.path_x[i*51 + 50], self.path_y[i*51 + 50]])
			else:
				plt.plot(self.path_x[i*51:i*51 + 51], self.path_y[i*51:i*51 + 51], 'pink')
		plt.plot(self.path_x[index*51:index*51 + 51], self.path_y[index*51:index*51 + 51], 'g')
		if self.agent_pose[0]<10000:
			self.update_agent(twist)
		self.update_obstacles()
		self.plot_lanes()
		self.plot_obstacles()
		#obs = plt.Circle((self.goal_p.position.x, self.goal_p.position.y), 1.0, color='b')
		#plt.add_patch(obs)
		#print(self.goal_p.poses[0].position.y, self.goal_p.poses[1].position.y, len(self.goal_p.poses))
		for i in range(len(self.goal_p.poses)):
			#if i == index:
			plt.plot(self.goal_p.poses[i].position.x, self.goal_p.poses[i].position.y, 'xb')
		#plt.plot(self.goal_p.poses[1].position.x, self.goal_p.poses[1].position.y, 'xb')
		#plt.ylim(self.agent_pose[1]-50, self.agent_pose[1]+50)
		#plt.xlim(self.agent_pose[0]-50, self.agent_pose[0]+150)
		plt.ylim(-20, 20)
		#self.time_arr.append(self.agent_pose[0])
		#self.vel.append(self.agent_vel[0])
		#plt.xlim(-30+self.agent_pose[0], 100+self.agent_pose[0])
		if (self.agent_pose[0]-self.xlim)>70.0:
			self.xlim = self.xlim+100.0
		plt.xlim(-30+self.xlim, 100+self.xlim)
		# plt.xlim(-30+self.agent_pose[0], 100+self.agent_pose[0])
		self.all_cruise_speed.append(cruise_idx)
		
		self.all_obs_res.append(obs_idx)
		
		self.all_ang_speed.append(ang_idx)
		
		self.all_min_cost.append(costs)
		
		self.all_goal_costs.append([self.path_x[index*51 + 50], self.path_y[index*51 + 50]])
		# np.savetxt("txt/cruise_speed.txt", np.array(self.all_cruise_speed))
		# np.savetxt("txt/obs_res.txt", np.array(self.all_obs_res))
		# np.savetxt("txt/ang_speed.txt", np.array(self.all_ang_speed))
		# np.savetxt("txt/min_costs.txt", np.array(self.all_min_cost))
		# np.savetxt("txt/min_goals.txt", np.array(self.all_goal_costs))
		#self.ax2.plot(self.time_arr, self.vel)
		#self.ax2.set_xlim(-30+self.xlim, 100+self.xlim)
		#self.ax2.set_ylim(0,22)
		#plt.ylim(-50, 250)
		#plt.xlim(-350, 350)
		#plt.draw()
		if self.agent_pose[0]<1000:
			self.loop+=1
		#plt.draw()
		#plt.show()
		plt.savefig("milestone-3/behaviour/"+str(self.loop)+".png")
		plt.pause(0.0001)
		#if self.agent_pose[0]>20.0:
			#quit()
		#if self.agent_pose[1]>100.0:
		#	quit()


def main(args=None):
	rclpy.init(args=args)

	minimal_subscriber = Behaviour()

	#rclpy.spin(minimal_subscriber)


	while rclpy.ok():
		minimal_subscriber.send_request()
		while rclpy.ok():
			rclpy.spin_once(minimal_subscriber)
			if minimal_subscriber.future.done():
				try:
					response = minimal_subscriber.future.result()
				except Exception as e:
					minimal_subscriber.get_logger().info(
						'Service call failed %r' % (e,))
				else:
					minimal_subscriber.get_logger().info(
						'Got res')
					minimal_subscriber.plot(response.twist, response.path, response.kkt)
				break
		#if np.linalg.norm(minimal_subscriber.agent_p - minimal_subscriber.goal) <= 1.0:
			#break

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()