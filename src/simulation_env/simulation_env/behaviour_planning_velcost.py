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
from acado_msgs.srv import GetControls
from acado_msgs.msg import OdomArray

class Behaviour(Node):
	def __init__(self):
		super().__init__('behaviour_tests')
		self.cli = self.create_client(GetControls, "/get_vel")
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
			
		self.lane_ids = []
		self.radius = 116.0
		self.agent_pose = [0.0, -2.0, 0.0]
		self.agent_vel = [15.0, 0.0]
		self.dt = 0.1

		self.lane_y = [-2.0, 2.0]
		self.dist_goal = 80.0
		self.goal = np.array([100.0, random.choice(self.lane_y) ])
		with open('src/simulation_env/config.yaml') as f:
			data = yaml.load(f, Loader=SafeLoader)
			setting = str(data["behaviour"])
		
		if setting == "overtake":
			self.behaviour_setting = 1
		elif setting == "wait for pedestrian to cross":
			self.behaviour_setting = 3
		elif setting == "increase/decrease speed":
			self.behaviour_setting = 5
		elif setting == "follow":
			self.behaviour_setting = 2

		self.obs_vel = np.array([10.0, 11.0, 9.0, 10.5, 9.5, 10.2, 10.8, 9.2, 9.8, 10.0])

		# Use this for overtake and combined with pedestrian
		self.obs = np.array([[-10.0, -2.0, 0.0, 10.0, 0.0],
							[25.0, 2.0, 0.0, 11.0, 0.0],
							[30.0, -2.0, 0.0, 9.0, 0.0],
							[65.0, 2.0, 0.0, 10.5, 0.0],
							[65.0, -2.0, 0.0, 9.5, 0.0],
							[95.0, 2.0, 0.0, 10.2, 0.0],
							[100.0, -2.0, 0.0, 10.8, 0.0],
							[140.0, 2.0, 0.0, 9.2, 0.0],
							[135.0, -2.0, 0.0, 9.8, 0.0],
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
		self.obs_pedestrian = np.array([[100.0, -6.0, 1.5707963, 0.0, 0.0]])

		self.behaviour_event = 1

		self.time_arr = []
		self.vel = []
		self.loop = 0

		self.xlim = 0.0

		self.flag = 1
		self.fig = plt.figure(0)
		self.ax1 = self.fig.add_subplot(211, aspect='equal')
		self.ax2 = self.fig.add_subplot(212, aspect='equal')
		mng = plt.get_current_fig_manager()
		# mng.full_screen_toggle()
		self.fig.set_size_inches(20, 10)
		self.req = GetControls.Request()
		print("STARTING SIMULATION")

	def send_request(self):
		self.req.start.pose.pose.position.x = self.agent_pose[0]
		self.req.start.pose.pose.position.y = self.agent_pose[1]
		self.req.start.pose.pose.orientation.z = self.agent_pose[2]
		self.req.start.twist.twist.linear.x = self.agent_vel[0]
		self.req.start.twist.twist.angular.z = self.agent_vel[1]

		obstacles = OdomArray()
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
		self.req.obstacles = obstacles

		goal = Pose()
		lane_cons = PoseArray()
		if self.behaviour_setting == 2 and int(self.behaviour_event) == 2:
			lane_cons, goal = self.get_lane_cons_follow()
		elif  int(self.behaviour_event) == 1 or int(self.behaviour_event) == 8 or int(self.behaviour_event) == 9:
			lane_cons, goal = self.get_lane_cons_overtake()
		#elif int(self.behaviour_event) == 3:
			#lane_cons, goal = self.get_lane_cons_stop()
		elif self.behaviour_setting == 5 and (int(self.behaviour_event) == 4 or int(self.behaviour_event) == 5):
			lane_cons, goal = self.get_lane_cons_sf(int(self.behaviour_event))
		#if self.obs[0][1]<2.0 and self.obs[0][1]>-4.0:
		#lane_cons, goal = self.get_lane_cons_stop()
		# default = 30
		if self.behaviour_setting == 3:
			if abs(self.agent_pose[0]-self.obs_pedestrian[0][0])<15:
				self.obs_pedestrian[0][3] = 2.0
				lane_cons, goal = self.get_lane_cons_stop()
			#if abs(self.obs_pedestrian[0][0]-self.agent_pose[0])>30:
			#	self.behaviour_event = 1
			#	return self.get_lane_cons_overtake()
			if self.agent_pose[0]-self.obs_pedestrian[0][0]>20:
				self.obs_pedestrian[0][0] = self.agent_pose[0]+random.choice([80, 120, 160, 200])
				self.obs_pedestrian[0][1] = -6
				self.obs_pedestrian[0][3] = 0.0
			for i in range(len(self.obs)):
				if self.obs[i][3]<self.obs_vel[i] and self.obs_pedestrian[0][1]>2.0:
					self.obs[i][3] = self.obs[i][3] + 4.0*self.dt
					print("++++++++++++++++++++++++++", self.obs[i][3], self.obs_vel[i])
		
		#lane_cons, goal = self.br_cons(0.0)
		self.req.goal = goal
		self.req.lane_cons = lane_cons
		print("Goal = ", goal, self.behaviour_event, self.behaviour_setting)
		#print(self.obs[:,0])
		#print("Lane Cons == ", self.req.lane_cons)
		self.goal_p = goal
		self.future = self.cli.call_async(self.req)

	def get_lane_cons_overtake(self):
		print("Overtake")
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [-2.0, 2.0]
		lanes = [-2.0]
		g_dist = 50.0
		goal_pose.position.x = self.agent_pose[0] + self.dist_goal
		goal_pose.position.y = random.choice(lanes)
		goal_pose.orientation.z = 0.0
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
		info.position.x = -2.0
		info.position.y = 2.0
		lane_info.poses.append(info)

		info = Pose()   #terminal position weights
		info.position.x = 2.5*1e3
		info.position.y = 2.5*1e3
		lane_info.poses.append(info)
		
		info = Pose()   #linear aceleration weights
		info.position.x = 1.0*1e3
		lane_info.poses.append(info)

		info = Pose()   #behaviour id
		info.position.x = 1.0
		info = Pose()   #behaviour id
		info.position.y = 15.0
		info.position.z = 0.0
		lane_info.poses.append(info)
		lane_info.poses.append(info)

		return lane_info, goal_pose
	
	def get_lane_cons_follow(self):
		print("Follow")
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [14.0, 10.0, 6.0]
		lanes = [-2.0]
		g_dist = 50.0
		y_dist = np.abs(self.obs[:,1] - self.agent_pose[1])
		self.obs = self.obs[y_dist.argsort()]
		#sorted_obs = sorted_y_obs[:5,:]
		sorted_obs = self.obs[np.abs(self.obs[:5,0] - self.agent_pose[0]).argsort()]
		self.obs[:5,:] = sorted_obs
		print(self.obs.shape)
		
		if self.behaviour_event == 'x':
			self.obs[0][3] = self.obs[0][3] - 3*self.dt
			self.behaviour_event = 2
		if self.behaviour_event == 'z':
			self.obs[0][3] = self.obs[0][3] + 3*self.dt
			self.behaviour_event = 2
		#if self.obs[0][3]>5.0:
		goal_pose.position.x = self.obs[0][0] + self.obs[0][3]*10
		print(self.obs[0][3])
		#else:
		#	goal_pose.position.x = self.obs[0][0] - 5.0
		#goal_pose.position.x = self.obs[0][0] + 30.0
		goal_pose.position.y = self.obs[0][1]
		goal_pose.orientation.z = 0.0
		info = Pose()
		info.position.x = 0.0
		info.position.y = 0.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 1.0  #goal_pose.position.y+1e12
		info.position.z = -goal_pose.position.y  #1e12
		lane_info.poses.append(info)

		
		if self.agent_pose[1]>0:
			info = Pose()   #max rad cons
			info.position.x = 0.0  #self.agent_pose[0]
			info.position.y = 0.0 #26.0 + 1e12
			info.position.z = 0.0 #1e12
			info.orientation.x = 0.0
			info.orientation.y = 1.0
			info.orientation.z = 0.0
			info.orientation.w = -1.0
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
			info.position.x = -20.0
			info.position.y = 2.0
			lane_info.poses.append(info)

			info = Pose()   #terminal position weights
			info.position.x = 2.5*1e3
			info.position.y = 2.5*1e3
			lane_info.poses.append(info)
		else:
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
			info.orientation.w = 1.0
			lane_info.poses.append(info)

			info = Pose()   #min max acc constraints
			info.position.x = -20.0
			info.position.y = 2.0
			lane_info.poses.append(info)

			info = Pose()   #terminal position weights
			info.position.x = 2.5*1e3
			info.position.y = 2.5*1e3
			lane_info.poses.append(info)
		
		info = Pose()   #linear aceleration weights
		info.position.x = 1.0*1e3
		lane_info.poses.append(info)

		info = Pose()   #behaviour id
		info.position.x = 2.0
		lane_info.poses.append(info)

		return lane_info, goal_pose
	
	def get_lane_cons_stop(self):
		print("Stop")
		#if self.agent_vel[0] < 0.1:
		#	self.obs[0][3] = 2.0
		for i in range(len(self.obs)):
			if (self.obs_pedestrian[0][0]-self.obs[i][0])<100 and self.obs[i][3]>0.0:
				self.obs[i][3] = self.obs[i][3] - 4.0*self.dt
				print("------------------",self.obs[i][3], self.obs_vel[i])

		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [-2.0, 2.0]
		lanes = [-2.0]
		g_dist = 50.0
		if (self.obs_pedestrian[0][1] - 0)>2.0:
			goal_pose.position.x = self.agent_pose[0]+80.0
			self.behaviour_event = 1
			for i in range(len(self.obs)):
				if self.obs[i][3]<self.obs_vel[i] and self.obs_pedestrian[0][1]>2.0:
					self.obs[i][3] = self.obs[i][3] + 4.0*self.dt
					print("***********************", self.obs[i][3], self.obs_vel[i])
		else:
			goal_pose.position.x = self.obs_pedestrian[0][0] - 5.0
		#goal_pose.position.x = self.obs[0][0] - 5.0
		goal_pose.position.y = random.choice(lanes)
		goal_pose.orientation.z = 0.0
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
		info.position.y = 2.0
		lane_info.poses.append(info)

		info = Pose()   #terminal position weights
		info.position.x = 5.0*1e5
		info.position.y = 5.0*1e5
		lane_info.poses.append(info)

		info = Pose()   #linear aceleration weights
		info.position.x = 5.0*1e2
		lane_info.poses.append(info)

		return lane_info, goal_pose

	def get_lane_cons_sf(self, num):
		print("SF ", num)
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [-2.0, 2.0]
		lanes = [-2.0]
		if num==4:
			#self.dist_goal+=5.0
			goal_pose.position.x = self.agent_pose[0] + self.dist_goal
			self.behaviour_event = 8
		else:
			#self.dist_goal-=5.0
			goal_pose.position.x = self.agent_pose[0] + self.dist_goal
			self.behaviour_event = 9
		goal_pose.position.y = random.choice(lanes)
		goal_pose.orientation.z = 0.0
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
		info.position.x = -2.0
		info.position.y = 2.0
		lane_info.poses.append(info)

		info = Pose()   #terminal position weights
		info.position.x = 2.5*1e3
		info.position.y = 2.5*1e3
		lane_info.poses.append(info)

		info = Pose()   #linear aceleration weights
		info.position.x = 1.0*1e3
		lane_info.poses.append(info)

		if num==4:
			info = Pose()   #behaviour id
			info.position.x = 4.0
			info.position.y = 25.0
			info.position.z = 1*1e3
			lane_info.poses.append(info)
		elif num==5:
			info = Pose()   #behaviour id
			info.position.x = 5.0
			info.position.y = 0.0
			info.position.z = 1*1e3
			lane_info.poses.append(info)

		return lane_info, goal_pose
	
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
			if self.obs[i][0]-self.agent_pose[0]<-70.0:
				self.obs[i][0] = self.obs[i][0] + 130.0
		self.obs_pedestrian[0][2] = self.obs_pedestrian[0][2] + self.obs_pedestrian[0][4]*self.dt
		self.obs_pedestrian[0][1] = self.obs_pedestrian[0][1] + self.obs_pedestrian[0][3]*np.sin(self.obs_pedestrian[0][2])*self.dt
		self.obs_pedestrian[0][0] = self.obs_pedestrian[0][0] + self.obs_pedestrian[0][3]*np.cos(self.obs_pedestrian[0][2])*self.dt
	
	def plot_lanes(self):
		self.ax1.plot([-20000, 20000], [4, 4], color='black', linewidth=2.0)
		self.ax1.plot([-20000, 20000], [0, 0], color='black', linewidth=1.0)
		self.ax1.plot([-20000, 20000], [-4, -4], color='black', linewidth=2.0)
	
	def plot_obstacles(self):
		for i in range(10):
			obs = plt.Circle((self.obs[i][0], self.obs[i][1]), 1.0, color='r')
			self.ax1.add_patch(obs)
		obs = plt.Circle((self.obs_pedestrian[0][0], self.obs_pedestrian[0][1]), 1.0, color='r')
		self.ax1.add_patch(obs)
		agent = plt.Circle((self.agent_pose[0], self.agent_pose[1]), 1.0, color='g')
		self.ax1.text(self.xlim, 33, 'Vel = %s'%(round(self.agent_vel[0],2)), fontsize=10)
		if int(self.behaviour_event) == 2:
			self.ax1.text(self.xlim+50, 33, 'Behaviour = Follow', fontsize=10)
		elif int(self.behaviour_event) == 1:
			self.ax1.text(self.xlim+50, 33, 'Behaviour = Overtake', fontsize=10)
		elif int(self.behaviour_event) == 3:
			self.ax1.text(self.xlim+50, 33, 'Behaviour = Wait for Vehicle to Cross', fontsize=10)
		elif int(self.behaviour_event) == 8:
			self.ax1.text(self.xlim+50, 33, 'Behaviour = Increase Speed', fontsize=10)
		elif int(self.behaviour_event) == 9:
			self.ax1.text(self.xlim+50, 33, 'Behaviour = Decrease Speed', fontsize=10)

		self.ax1.add_patch(agent)
	
	def on_press(self, event):
		self.behaviour_event = event.key

	
	def plot(self, twist, path):
		plt.ion()
		plt.show()
		plt.clf()
		self.ax1 = self.fig.add_subplot(211, aspect='equal')
		self.ax2 = self.fig.add_subplot(212, aspect='equal')
		self.path_x = []
		self.path_y = []
		for i in path.poses:
			self.path_x.append(i.position.x)
			self.path_y.append(i.position.y)
		print(len(self.path_x))
		self.fig.canvas.mpl_connect('key_press_event', self.on_press)
		self.ax1.plot(self.path_x, self.path_y, 'y')
		self.update_agent(twist)
		self.update_obstacles()
		self.plot_lanes()
		self.plot_obstacles()
		#obs = plt.Circle((self.goal_p.position.x, self.goal_p.position.y), 1.0, color='b')
		#self.ax1.add_patch(obs)
		self.ax1.plot(self.goal_p.position.x, self.goal_p.position.y, 'xb')
		#self.ax1.ylim(self.agent_pose[1]-50, self.agent_pose[1]+50)
		#self.ax1.xlim(self.agent_pose[0]-50, self.agent_pose[0]+150)
		self.ax1.set_ylim(-30, 30)
		self.time_arr.append(self.agent_pose[0])
		self.vel.append(self.agent_vel[0])
		#self.ax1.xlim(-30+self.agent_pose[0], 100+self.agent_pose[0])
		if (self.agent_pose[0]-self.xlim)>70.0:
			self.xlim = self.xlim+100.0
		self.ax1.set_xlim(-30+self.xlim, 100+self.xlim)
		self.ax2.plot(self.time_arr, self.vel)
		self.ax2.set_xlim(-30+self.xlim, 100+self.xlim)
		self.ax2.set_ylim(0,22)
		#self.ax1.ylim(-50, 250)
		#self.ax1.xlim(-350, 350)
		#self.ax1.draw()
		self.loop+=1
		plt.draw()
		plt.pause(0.0001)
		if self.agent_pose[1]>100.0:
			quit()


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
					minimal_subscriber.plot(response.twist, response.path)
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