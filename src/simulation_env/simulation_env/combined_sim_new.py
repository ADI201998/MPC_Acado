import numpy as np
import random
import matplotlib.pylab as plt
import enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from acado_msgs.srv import GetControls
from acado_msgs.srv import GetControlsMulti
from acado_msgs.msg import OdomArray

class CombinedEnv(Node):
	def __init__(self) -> None:
		super().__init__('combined_plot')
		self.cli = self.create_client(GetControlsMulti, "/get_vel")
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')

		self.timesteps = 30
		self.time_secs = 3.0

		# self.num_goals = 6
		self.num_goals = 1

		self.horiz_lane_lims = np.array([[12, 12], [8, 8], [4, 4], [0, 0], [-4, -4], [-8, -8], [-12, -12]])
		self.horiz_lane_lims_inter = np.array([[212, 212], [216, 216], [220, 220], [224, 224], [228, 228], [232, 232], [236, 236]])

		self.vert_lane_lims = np.array([[224, 224], [220, 220], [216, 216], [212, 212], [208, 208], [204, 204], [200, 200]])
			
		self.radius = 112.0
		self.intersection_radios = 10.0
		self.agent_pose = [0.0, 10.0, 0.0]
		self.agent_pose_frenet = [0.0, 10.0, 0.0]
		self.agent_vel = [15.0, 0.0]
		self.agent_vel_frenet = [15.0, 0.0]
		self.dt = 0.1
		self.nearest_obs_dist = 0.0

		self.time_arr = np.linspace(0, self.time_secs, (self.timesteps + 1))

		# self.dist_goal = 80
		self.dist_goal = 48

		self.lane_radii = [100.0, 104.0, 108.0, 112.0, 116.0, 120.0, 124.0]
		self.lanes = [12.0, 8.0, 4.0, 0.0, -4.0, -8.0, -12.0]
		self.circular_lane_centers = [244, 112]
		angles = np.arange(0, 360.1, 0.1)
		angles_left = angles[900:2701]
		angles_right = np.concatenate([angles[2700:3600], angles[0:901]])
		self.right_circle_x = []
		self.right_circle_y = []
		self.left_circle_x = []
		self.left_circle_y = []
		for i in self.lane_radii:
			self.right_circle_x.append([i*np.cos(ang*2*np.pi/360)+self.circular_lane_centers[0] for ang in angles_right])
			self.right_circle_y.append([i*np.sin(ang*2*np.pi/360)+self.circular_lane_centers[1] for ang in angles_right])
			self.left_circle_x.append([i*np.cos(ang*2*np.pi/360)-self.circular_lane_centers[0] for ang in angles_left])
			self.left_circle_y.append([i*np.sin(ang*2*np.pi/360)+self.circular_lane_centers[1] for ang in angles_left])
		
		self.obstacles_intersection_north = np.array([[202, -22.0, 90*2*np.pi/360, 6.8, 0.0, 7.0, 0],
												[202.0, 28.0, 90*2*np.pi/360, 7.3, 0.0, 7.0, 0],
												[202.0, 68.0, 90*2*np.pi/360, 6.9, 0.0, 7.0, 0],
												#######################
												[206.0, -37.0, 90*2*np.pi/360, 8.8, 0.0, 8.0, 0],
												[206.0, 18.0, 90*2*np.pi/360, 7.9, 0.0, 8.0, 0],
												[206.0, 53.0, 90*2*np.pi/360, 8.3, 0.0, 8.0, 0],
												#######################
												[210.0, -13.0, 90*2*np.pi/360, 9.3, 0.0, 9.0, 0],
												[210.0, 38.0, 90*2*np.pi/360, 8.8, 0.0, 9.0, 0],
												[210.0, 70.0, 90*2*np.pi/360, 8.9, 0.0, 9.0, 0],
												])

		self.obstacles_intersection_south = np.array([[222.0, 0.0, 270*2*np.pi/360, 9.3, 0.0, 9.0, 0],
												[222.0, 40.0, 270*2*np.pi/360, 8.8, 0.0, 9.0, 0],
												[222.0, 80.0, 270*2*np.pi/360, 8.9, 0.0, 9.0, 0],
												#######################
												[218.0, -10.0, 270*2*np.pi/360, 8.8, 0.0, 8.0, 0],
												[218.0, 20.0, 270*2*np.pi/360, 8.9, 0.0, 8.0, 0],
												[218.0, 60.0, 270*2*np.pi/360, 9.3, 0.0, 8.0, 0],
												######################
												[214.0, -40.0, 270*2*np.pi/360, 6.8, 0.0, 7.0, 0],
												[214.0, 0.0, 270*2*np.pi/360, 7.3, 0.0, 7.0, 0],
												[214.0, 30.0, 270*2*np.pi/360, 6.9, 0.0, 7.0, 0],
												])
						
		self.obstacles_right = np.array([[-20.0, 10.0, 0.0*2*np.pi/360, 7.8, 0.0, 8.0, 102.0],
										[35.0, 10.0, 0.0*2*np.pi/360, 8.3, 0.0, 8.0, 102.0],
										[80.0, 10.0, 0.0*2*np.pi/360, 7.9, 0.0, 8.0, 102.0],
										#######################
										[-5.0, 6.0, 0.0*2*np.pi/360, 9.8, 0.0, 10.0, 106.0],
										[45.0, 6.0, 0.0*2*np.pi/360, 9.9, 0.0, 10.0, 106.0],
										[80.0, 6.0, 0.0*2*np.pi/360, 10.3, 0.0, 10.0, 106.0],
										#######################
										[-20.0, 2.0, 0.0*2*np.pi/360, 12.3, 0.0, 12.0, 110.0],
										[40.0, 2.0, 0.0*2*np.pi/360, 11.8, 0.0, 12.0, 110.0],
										[100.0, 2.0, 0.0*2*np.pi/360, 11.9, 0.0, 12.0, 110.0],
										])

		self.obstacles_left = np.array([[-12.0, -2.0, 180*2*np.pi/360, 7.8, 0.0, 8.0, 114.0],
										[25.0, -2.0, 180*2*np.pi/360, 8.3, 0.0, 8.0, 114.0],
										[60.0, -2.0, 180*2*np.pi/360, 7.9, 0.0, 8.0, 114.0],
										#######################
										[15.0, -6.0, 180*2*np.pi/360, 9.8, 0.0, 10.0, 118.0],
										[45.0, -6.0, 180*2*np.pi/360, 9.9, 0.0, 10.0, 118.0],
										[80.0, -6.0, 180*2*np.pi/360, 10.3, 0.0, 10.0, 118.0],
										#######################
										[-10.0, -10.0, 180*2*np.pi/360, 12.3, 0.0, 12.0, 122.0],
										[30.0, -10.0, 180*2*np.pi/360, 11.8, 0.0, 12.0, 122.0],
										[70.0, -10.0, 180*2*np.pi/360, 11.9, 0.0, 12.0, 122.0],
										])	
		
		self.nearest_obstacles = []
		self.loop_length = 244 + self.radius*np.pi + 488 + self.radius*np.pi + 244
		self.loop = 0

		self.ego_poses_straight = []
		self.ego_poses_intersection = []
		self.ego_poses_curved = []

		self.fig = plt.figure(0)
		#self.ax1 = self.fig.add_subplot(111, aspect='equal')
		#self.ax2 = self.fig.add_subplot(212, aspect='equal')
		#mng = plt.get_current_fig_manager()
		# mng.full_screen_toggle()
		self.fig.set_size_inches(20, 10)

		self.req = GetControlsMulti.Request()
		print("STARTING SIMULATION")

	def send_request(self):
		self.nearest_obstacles = []

		self.req.start.pose.pose.position.x = self.agent_pose[0]
		self.req.start.pose.pose.position.y = self.agent_pose[1]
		self.req.start.pose.pose.orientation.z = self.agent_pose[2]
		self.req.start.twist.twist.linear.x = self.agent_vel[0]
		self.req.start.twist.twist.angular.z = self.agent_vel[1]

		obs_pos = np.concatenate((self.obstacles_right, #self.obstacles_left, 
								self.obstacles_intersection_south, self.obstacles_intersection_north))
		dist = np.sqrt((obs_pos[:,0] - self.agent_pose[0])**2 + (obs_pos[:,1] - self.agent_pose[1])**2)
		sorted_obs = obs_pos[dist.argsort()]
		dist = ((self.agent_pose[0] - sorted_obs[:,0])**2 + (self.agent_pose[1] - sorted_obs[:,1])**2)**0.5
		self.nearest_obs_dist = dist[0]
		obstacles = OdomArray()
		for i in range(10):
			odom = Odometry()
			odom.pose.pose.position.x = sorted_obs[i][0]# - self.agent_pose[0]
			odom.pose.pose.position.y = sorted_obs[i][1]
			odom.pose.pose.orientation.z = sorted_obs[i][2]
			odom.twist.twist.linear.x = sorted_obs[i][3]
			odom.twist.twist.angular.z = sorted_obs[i][4]
			obs_theta = sorted_obs[i][2] + sorted_obs[i][4]*self.time_arr
			obs_x = sorted_obs[i][0] + sorted_obs[i][3]*np.cos(obs_theta)
			obs_y = sorted_obs[i][1] + sorted_obs[i][3]*np.sin(obs_theta)
			self.nearest_obstacles.append(np.concatenate((obs_x, obs_y)))
			obstacles.odom.append(odom)
		
		self.nearest_obstacles = np.array(self.nearest_obstacles)
		self.req.obstacles = obstacles
		goal = PoseArray()
		lane_cons = PoseArray()
		lane_cons, goal = self.get_lane_cons_overtake()
		self.goal_p = goal
		self.req.goal = goal
		self.req.lane_cons = lane_cons
		self.future = self.cli.call_async(self.req)
	
	def get_lane_cons_overtake(self):
		lane_info = PoseArray()
		goals = PoseArray()
		# lanes = [10.0, 6.0, 2.0, -2.0, -6.0, -10.0]
		# lanes = [10.0, 6.0, 2.0, 2.0, 6.0, 10.0]
		lanes = [6.0]
		cur_lane = lanes[np.argmin(np.abs(np.array(lanes) - self.agent_pose[1]))]
		#lanes = [-2.0]
		# if cur_lane == 10.0:
		# 	lanes = [10.0, 6.0, 2.0, 10.0, 6.0, 2.0]
		# elif cur_lane == -10.0:
		# 	lanes = [-2.0, -6.0, -10.0, -2.0, -6.0, -10.0]
		# else:
		# 	lanes = [cur_lane - 4.0, cur_lane, cur_lane + 4.0]*2
		xgoal = [self.dist_goal, self.dist_goal, self.dist_goal, self.dist_goal*3/4, self.dist_goal*3/4, self.dist_goal*3/4] 
		self.lane_y = lanes
		g_dist = 50.0
		for i in range(len(lanes)):
			goal_pose = Pose()
			goal_pose.position.x = float(xgoal[i]) + self.agent_pose[0]
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
		info.orientation.w = -1.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 0.0 #self.agent_pose[0]
		info.position.y =  0.0 #-4.0 + 1e12
		info.position.z = 0.0
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = -11.0
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

	def plot_intersection_curves(self):
		angles = np.arange(0, 360.1, 0.1)
		r1 = []
		r2 = []
		r3 = []
		r4 = []
		
		for i in range(len(angles)):
			if i<900:
				r1.append([5*np.cos(angles[i]*2*np.pi/360), 5*np.sin(angles[i]*2*np.pi/360)])
			elif i>=900 and i<1800:
				r2.append([5*np.cos(angles[i]*2*np.pi/360), 5*np.sin(angles[i]*2*np.pi/360)])
			elif i>=1800 and i<2700:
				r3.append([5*np.cos(angles[i]*2*np.pi/360), 5*np.sin(angles[i]*2*np.pi/360)])
			else:
				r4.append([5*np.cos(angles[i]*2*np.pi/360), 5*np.sin(angles[i]*2*np.pi/360)])

		r1 = np.array(r1)
		r2 = np.array(r2)
		r3 = np.array(r3)
		r4 = np.array(r4)

		plt.plot(r1[:, 0] + 195, r1[:, 1] - 17, color='black', linewidth=2.0)
		plt.plot(r2[:, 0] + 229, r2[:, 1] - 17, color='black', linewidth=2.0)
		plt.plot(r3[:, 0] + 229, r3[:, 1] + 17, color='black', linewidth=2.0)
		plt.plot(r4[:, 0] + 195, r4[:, 1] + 17, color='black', linewidth=2.0)

		plt.plot(r1[:, 0] + 195, r1[:, 1] + 207, color='black', linewidth=2.0)
		plt.plot(r2[:, 0] + 229, r2[:, 1] + 207, color='black', linewidth=2.0)
		plt.plot(r3[:, 0] + 229, r3[:, 1] + 241, color='black', linewidth=2.0)
		plt.plot(r4[:, 0] + 195, r4[:, 1] + 241, color='black', linewidth=2.0)

		plt.plot(r1[:, 0] - 229, r1[:, 1] - 17, color='black', linewidth=2.0)
		plt.plot(r2[:, 0] - 195, r2[:, 1] - 17, color='black', linewidth=2.0)
		plt.plot(r3[:, 0] - 195, r3[:, 1] + 17, color='black', linewidth=2.0)
		plt.plot(r4[:, 0] - 229, r4[:, 1] + 17, color='black', linewidth=2.0)

		plt.plot(r1[:, 0] - 229, r1[:, 1] + 207, color='black', linewidth=2.0)
		plt.plot(r2[:, 0] - 195, r2[:, 1] + 207, color='black', linewidth=2.0)
		plt.plot(r3[:, 0] - 195, r3[:, 1] + 241, color='black', linewidth=2.0)
		plt.plot(r4[:, 0] - 229, r4[:, 1] + 241, color='black', linewidth=2.0)

	def plot_straight_lanes(self):
		for i in range(len(self.lanes)):
			lw = 1.0
			if i == 0 or i == 6:
				lw = 2.0

			#   Plot intersetion lanes
			plt.plot(-1*self.vert_lane_lims[i], [17, 207], color='black', linewidth=lw)
			plt.plot(self.vert_lane_lims[i], [17, 207], color='black', linewidth=lw)
			plt.plot(-1*self.vert_lane_lims[i], [241, 300], color='black', linewidth=lw)
			plt.plot(-1*self.vert_lane_lims[i], [-17, -60], color='black', linewidth=lw)
			plt.plot(self.vert_lane_lims[i], [241, 300], color='black', linewidth=lw)
			plt.plot(self.vert_lane_lims[i], [-17, -60], color='black', linewidth=lw)

			########################################################################################

			#   Plot normal lane
			plt.plot([-195, 195], self.horiz_lane_lims[i], color='black', linewidth=lw)
			plt.plot([229, 244], self.horiz_lane_lims[i], color='black', linewidth=lw)
			plt.plot([-195, 195], self.horiz_lane_lims_inter[i], color='black', linewidth=lw)
			plt.plot([-229, -244], self.horiz_lane_lims_inter[i], color='black', linewidth=lw)
			plt.plot([229, 244], self.horiz_lane_lims_inter[i], color='black', linewidth=lw)
			plt.plot([-229, -244], self.horiz_lane_lims[i], color='black', linewidth=lw)

		self.plot_intersection_curves()
	
	def plot_circular_lanes(self):
		for i in range(len(self.lane_radii)):
			lw = 1.0
			if i==0 or i==6:
				lw = 2.0
			plt.plot(self.right_circle_x[i], self.right_circle_y[i], color='black', linewidth=lw)
			plt.plot(self.left_circle_x[i], self.left_circle_y[i], color='black', linewidth=lw)

	def plot_lanes(self):
		self.plot_straight_lanes()
		self.plot_circular_lanes()
	
	def update_obstacles_intersection(self):

		##################################
		
		if self.agent_pose_frenet[0]>0 and self.obstacles_intersection_north[0][0]<0.0:
			self.obstacles_intersection_north[:,0] = self.obstacles_intersection_north[:,0] + 424
		elif self.agent_pose_frenet[0]<0 and self.obstacles_intersection_north[0][0]>0.0:
			self.obstacles_intersection_north[:,0] = self.obstacles_intersection_north[:,0] - 424
		for i in range(self.obstacles_intersection_north.shape[0]):
			if self.agent_pose_frenet[1]>116 and self.obstacles_intersection_north[i][1]<116:
				self.obstacles_intersection_north[i][1] = self.obstacles_intersection_north[i][1] + 200.0
			if self.agent_pose_frenet[1]<116 and self.obstacles_intersection_north[i][1]>116:
				self.obstacles_intersection_north[i][1] = self.obstacles_intersection_north[i][1] - 200.0				
			if self.agent_pose_frenet[1]<116 and self.obstacles_intersection_north[i][1]<116:
				if self.obstacles_intersection_north[i][1]>60:
					self.obstacles_intersection_north[i][1] = self.obstacles_intersection_north[i][1] - 120.0
			if self.agent_pose_frenet[1]>116 and self.obstacles_intersection_north[i][1]>116:
				if self.obstacles_intersection_north[i][1]>244+60:
					self.obstacles_intersection_north[i][1] = self.obstacles_intersection_north[i][1] - 120.0
			so = 1
			T = 1

			l = 1.5

			a = 4
			b = 3

			nearest = -1
			x = self.obstacles_intersection_north[i][0]
			y = self.obstacles_intersection_north[i][1]
			inLane = self.obstacles_intersection_north[:,0] - x * np.ones(len(self.obstacles_intersection_north))
			index = np.where( inLane == 0)

			min = 10000
			for k in range(len(index[0])):
				if index[0][k] != i:
					if y < self.obstacles_intersection_north[index[0][k]][1]:
						dist = self.obstacles_intersection_north[index[0][k]][1] - y
						if dist < min:
							min = dist
							nearest = index[0][k]
			
			v_0 = self.obstacles_intersection_north[nearest][3]
			x_0 = self.obstacles_intersection_north[nearest][1]
			if abs(x - self.agent_pose[0]) <= 1.5 and y < self.agent_pose[1]:
				if nearest != -1:
					if self.agent_pose[1] < self.obstacles_intersection_north[nearest][1]: 
						nearest = 1
						v_0 = self.agent_vel[0]
						x_0 = self.agent_pose[1]
				else:
					v_0 = self.agent_vel[0]
					x_0 = self.agent_pose[1]
			v_1 = self.obstacles_intersection_north[i][3]
			x_1 = y

			v_r = self.obstacles_intersection_north[i][5]
			delta_v = v_1 - v_0
			s_alpha = x_0 - x_1 - l

			if nearest != -1:
				s_star = so + v_1 * T + (v_1 * delta_v)/(2 * np.sqrt(a*b))
				decc = a * (1 - (v_1/v_r)**4 - (s_star/s_alpha)**2)
				if decc < -a:
					decc = -a
				self.obstacles_intersection_north[i][3] += decc * self.dt
			else:
				decc = a * (1 - (v_1/v_r)**4)
				if decc > a:
					decc = a
				self.obstacles_intersection_north[i][3] += decc * self.dt
			self.obstacles_intersection_north[i][2] = self.obstacles_intersection_north[i][2] + self.obstacles_intersection_north[i][4]*self.dt
			self.obstacles_intersection_north[i][0] = self.obstacles_intersection_north[i][0] + \
											self.obstacles_intersection_north[i][3]*np.cos(self.obstacles_intersection_north[i][2])*self.dt
			self.obstacles_intersection_north[i][1] = self.obstacles_intersection_north[i][1] + \
											self.obstacles_intersection_north[i][3]*np.sin(self.obstacles_intersection_north[i][2])*self.dt

		##################################

		if self.agent_pose_frenet[0]>0 and self.obstacles_intersection_south[0][0]<0.0:
			self.obstacles_intersection_south[:,0] = self.obstacles_intersection_south[:,0] + 424
		elif self.agent_pose_frenet[0]<0 and self.obstacles_intersection_south[0][0]>0.0:
			self.obstacles_intersection_south[:,0] = self.obstacles_intersection_south[:,0] - 424
		for i in range(self.obstacles_intersection_south.shape[0]):
			if self.agent_pose_frenet[1]>116 and self.obstacles_intersection_south[i][1]<116:
				self.obstacles_intersection_south[i][1] = self.obstacles_intersection_south[i][1] + 200.0
			if self.agent_pose_frenet[1]<116 and self.obstacles_intersection_south[i][1]>116:
				self.obstacles_intersection_south[i][1] = self.obstacles_intersection_south[i][1] - 200.0				
			if self.agent_pose_frenet[1]<116 and self.obstacles_intersection_south[i][1]<116:
				if self.obstacles_intersection_south[i][1]<-60:
					self.obstacles_intersection_south[i][1] = self.obstacles_intersection_south[i][1] + 120.0
			if self.agent_pose_frenet[1]>116 and self.obstacles_intersection_south[i][1]>116:
				if self.obstacles_intersection_south[i][1]<244-60:
					self.obstacles_intersection_south[i][1] = self.obstacles_intersection_south[i][1] + 120.0
			so = 1
			T = 1

			l = 1.5

			a = 4
			b = 3
			
			nearest = -1
			x = self.obstacles_intersection_south[i][0]
			y = self.obstacles_intersection_south[i][1]
			inLane = self.obstacles_intersection_south[:,0] - x * np.ones(len(self.obstacles_intersection_south))
			index = np.where( inLane == 0)

			min = 10000
			for k in range(len(index[0])):
				if index[0][k] != i:
					if y > self.obstacles_intersection_south[index[0][k]][1]:
						dist = self.obstacles_intersection_south[index[0][k]][1] - y
						if dist < min:
							min = dist
							nearest = index[0][k]
			
			v_0 = self.obstacles_intersection_south[nearest][3]
			x_0 = self.obstacles_intersection_south[nearest][1]
			if abs(x - self.agent_pose[0]) <= 1.5 and y > self.agent_pose[1]:
				if nearest != -1:
					if self.agent_pose[1] > self.obstacles_intersection_south[nearest][1]: 
						nearest = 1
						v_0 = self.agent_vel[0]
						x_0 = self.agent_pose[1]
				else:
					v_0 = self.agent_vel[0]
					x_0 = self.agent_pose[1]
			v_1 = self.obstacles_intersection_south[i][3]
			x_1 = y

			v_r = self.obstacles_intersection_south[i][5]
			delta_v = v_1 - v_0
			s_alpha = x_0 - x_1 - l

			if nearest != -1:
				s_star = so + v_1 * T + (v_1 * delta_v)/(2 * np.sqrt(a*b))
				decc = a * (1 - (v_1/v_r)**4 - (s_star/s_alpha)**2)
				if decc < -a:
					decc = -a
				self.obstacles_intersection_south[i][3] += decc * self.dt
			else:
				decc = a * (1 - (v_1/v_r)**4)
				if decc > a:
					decc = a
				self.obstacles_intersection_south[i][3] += decc * self.dt
			self.obstacles_intersection_south[i][2] = self.obstacles_intersection_south[i][2] + self.obstacles_intersection_south[i][4]*self.dt
			self.obstacles_intersection_south[i][0] = self.obstacles_intersection_south[i][0] + \
											self.obstacles_intersection_south[i][3]*np.cos(self.obstacles_intersection_south[i][2])*self.dt
			self.obstacles_intersection_south[i][1] = self.obstacles_intersection_south[i][1] + \
											self.obstacles_intersection_south[i][3]*np.sin(self.obstacles_intersection_south[i][2])*self.dt

	def update_obstacles_normal(self):

		##################################

		for i in range(self.obstacles_right.shape[0]):
			so = 1
			T = 1

			l = 1.5

			a = 4
			b = 3

			nearest = -1
			x = self.obstacles_right[i][0]
			y = self.obstacles_right[i][1]
			inLane = self.obstacles_right[:,0] - y * np.ones(len(self.obstacles_right))
			index = np.where( inLane == 0)

			min = 10000
			for k in range(len(index[0])):
				if index[0][k] != i:
					if x < self.obstacles_right[index[0][k]][0]:
						dist = self.obstacles_right[index[0][k]][0] - x
						if dist < min:
							min = dist
							nearest = index[0][k]
			
			v_0 = self.obstacles_right[nearest][3]
			x_0 = self.obstacles_right[nearest][0]
			if abs(y - self.agent_pose[1]) <= 1.5 and x < self.agent_pose[0]:
				if nearest != -1:
					if self.agent_pose[0] < self.obstacles_right[nearest][0]: 
						nearest = 1
						v_0 = self.agent_vel[0]
						x_0 = self.agent_pose[0]
				else:
					v_0 = self.agent_vel[0]
					x_0 = self.agent_pose[0]
			v_1 = self.obstacles_right[i][3]
			x_1 = x

			v_r = self.obstacles_right[i][5]
			delta_v = v_1 - v_0
			s_alpha = x_0 - x_1 - l

			if nearest != -1:
				s_star = so + v_1 * T + (v_1 * delta_v)/(2 * np.sqrt(a*b))
				decc = a * (1 - (v_1/v_r)**4 - (s_star/s_alpha)**2)
				if decc < -a:
					decc = -a
				self.obstacles_right[i][3] += decc * self.dt
			else:
				decc = a * (1 - (v_1/v_r)**4)
				if decc > a:
					decc = a
				self.obstacles_right[i][3] += decc * self.dt

			self.obstacles_right[i][2] = self.obstacles_right[i][2] + self.obstacles_right[i][4]*self.dt
			self.obstacles_right[i][0] = self.obstacles_right[i][0] + \
											self.obstacles_right[i][3]*np.cos(self.obstacles_right[i][2])*self.dt
			self.obstacles_right[i][1] = self.obstacles_right[i][1] + \
											self.obstacles_right[i][3]*np.sin(self.obstacles_right[i][2])*self.dt
			

			if self.obstacles_right[i][0] - self.agent_pose[0] < -50.0:
				self.obstacles_right[i][0] = self.obstacles_right[i][0] + 120.0
		
		##################################

		for i in range(self.obstacles_left.shape[0]):
			so = 1
			T = 1

			l = 1.5

			a = 4
			b = 3

			nearest = -1
			x = self.obstacles_left[i][0]
			y = self.obstacles_left[i][1]
			inLane = self.obstacles_left[:,0] - y * np.ones(len(self.obstacles_left))
			index = np.where( inLane == 0)

			min = 10000
			for k in range(len(index[0])):
				if index[0][k] != i:
					if x > self.obstacles_left[index[0][k]][0]:
						dist = self.obstacles_left[index[0][k]][0] - y
						if dist < min:
							min = dist
							nearest = index[0][k]
			
			v_0 = self.obstacles_left[nearest][3]
			x_0 = self.obstacles_left[nearest][0]
			if abs(y - self.agent_pose[1]) <= 1.5 and x > self.agent_pose[0]:
				if nearest != -1:
					if self.agent_pose[0] > self.obstacles_left[nearest][0]: 
						nearest = 1
						v_0 = self.agent_vel[0]
						x_0 = self.agent_pose[0]
				else:
					v_0 = self.agent_vel[0]
					x_0 = self.agent_pose[0]
			v_1 = self.obstacles_left[i][3]
			x_1 = x

			v_r = self.obstacles_left[i][5]
			delta_v = v_1 - v_0
			s_alpha = x_0 - x_1 - l

			if nearest != -1:
				s_star = so + v_1 * T + (v_1 * delta_v)/(2 * np.sqrt(a*b))
				decc = a * (1 - (v_1/v_r)**4 - (s_star/s_alpha)**2)
				if decc < -a:
					decc = -a
				self.obstacles_left[i][3] += decc * self.dt
			else:
				decc = a * (1 - (v_1/v_r)**4)
				if decc > a:
					decc = a
				self.obstacles_left[i][3] += decc * self.dt

			self.obstacles_left[i][2] = self.obstacles_left[i][2] + self.obstacles_left[i][4]*self.dt
			self.obstacles_left[i][0] = self.obstacles_left[i][0] + \
											self.obstacles_left[i][3]*np.cos(self.obstacles_left[i][2])*self.dt
			self.obstacles_left[i][1] = self.obstacles_left[i][1] + \
											self.obstacles_left[i][3]*np.sin(self.obstacles_left[i][2])*self.dt
			if self.obstacles_left[i][0] - self.agent_pose[0] < -10.0:
				self.obstacles_left[i][0] = self.obstacles_left[i][0] + 100.0

	def update_obstacles(self):
		self.update_obstacles_intersection()
		self.update_obstacles_normal()
	
	def update_agent(self, twist):
		self.agent_vel[0] = twist.linear.x
		self.agent_vel[1] = twist.angular.z
		self.agent_pose[2] = self.agent_pose[2] + self.agent_vel[1]*self.dt
		self.agent_pose[0] = self.agent_pose[0] + self.agent_vel[0]*np.cos(self.agent_pose[2])*self.dt
		self.agent_pose[1] = self.agent_pose[1] + self.agent_vel[0]*np.sin(self.agent_pose[2])*self.dt
	
	def centerline_to_env(self, obs):
		x = env_x = obs[0]
		y = env_y = obs[1]
		env_theta = obs[2]
		if x - self.loop*self.loop_length > self.loop_length:
			self.loop = self.loop + 1
			x = x - self.loop*self.loop_length
		if x>244:
			x = x-244
			env_x = (self.lane_radii[3]-y)*np.cos(x/self.radius + 3*np.pi/2) + self.circular_lane_centers[0]
			env_y = (self.lane_radii[3]-y)*np.sin(x/self.radius + 3*np.pi/2) + self.circular_lane_centers[1]
			env_theta = env_theta + x/self.radius #+ 3*np.pi/2 + np.pi/2
			if x > self.radius*np.pi:
				x = x - self.radius*np.pi
				env_x = 244 - x
				env_y = 224 - y
				env_theta = np.pi + env_theta
				if x>488:
					x = x - 488
					env_x = (self.lane_radii[3]-y)*np.cos(x/self.radius + np.pi/2) - self.circular_lane_centers[0]
					env_y = (self.lane_radii[3]-y)*np.sin(x/self.radius + np.pi/2) + self.circular_lane_centers[1]
					if x > self.radius*np.pi:
						x = x - self.radius*np.pi
						env_x = -244 + x
						env_y = y
						env_theta = x/self.radius + np.pi/2 + np.pi/2
		return env_x, env_y, env_theta

	def plot_agents(self):

		##################################

		for i in self.obstacles_left:
			env_x, env_y, env_theta = self.centerline_to_env(i)
			obs = plt.Circle((env_x, env_y), 1.0, color='r')
			plt.gca().add_patch(obs)

		##################################

		for i in self.obstacles_right:
			env_x, env_y, env_theta = self.centerline_to_env(i)
			obs = plt.Circle((env_x, env_y), 1.0, color='r')
			plt.gca().add_patch(obs)

		##################################

		for i in self.obstacles_intersection_south:
			obs = plt.Circle((i[0], i[1]), 1.0, color='r')
			plt.gca().add_patch(obs)

		##################################

		for i in self.obstacles_intersection_north:
			obs = plt.Circle((i[0], i[1]), 1.0, color='r')
			plt.gca().add_patch(obs)
		
		##################################

		agent = [self.agent_pose[0], self.agent_pose[1], self.agent_pose[2]]
		env_x, env_y, env_theta = self.centerline_to_env(agent)
		self.agent_pose_frenet = [env_x, env_y, env_theta]
		agent = plt.Circle((env_x, env_y), 1.0, color='g')
		plt.text(env_x, env_y+30, 'Vel = %s'%(round(self.agent_vel[0],2)), fontsize=10)
		plt.gca().add_patch(agent)
	
	def plot(self, twist, path, kkt):
		plt.clf()

		path_x = []
		path_y = []
		path_theta = []
		path_v = []
		path_w = []
		for i in path.poses:
			# path_x.append(i.position.x + self.agent_pose[0])
			path_x.append(i.position.x)
			path_y.append(i.position.y)
			path_theta.append(i.position.z)
			path_v.append(i.orientation.x)
			path_w.append(i.orientation.y)
		path_x = np.array(path_x)
		path_y = np.array(path_y)
		path_theta = np.array(path_theta)
		path_v = np.array(path_v)
		path_w = np.array(path_w)

		## Get Ranks
		y_dist = []
		kkt_cost = []
		cruise_speed = []
		angular_vel = []
		obs_cost = []
		for i in range(self.num_goals):
			res_obs_x = np.linalg.norm(path_x[i*(self.timesteps + 1):i*(self.timesteps + 1) + (self.timesteps + 1)] - self.nearest_obstacles[:, :(self.timesteps + 1)], axis=1)
			res_obs_y = np.linalg.norm(path_y[i*(self.timesteps + 1):i*(self.timesteps + 1) + (self.timesteps + 1)] - self.nearest_obstacles[:, (self.timesteps + 1):], axis=1)
			res_obs = np.vstack((res_obs_x, res_obs_y))
			res_obs = np.linalg.norm(res_obs, axis=0)
			obs_cost.append(1/np.min(res_obs))
			y_dist.append(np.linalg.norm(path_y[i*(self.timesteps + 1):i*(self.timesteps + 1) + (self.timesteps + 1)] - 14.0))
			kkt_cost.append(kkt[i])
			cruise_speed.append(np.linalg.norm(path_v[i*(self.timesteps + 1):i*(self.timesteps + 1) + (self.timesteps + 1)] - 15.0))
			angular_vel.append(np.linalg.norm(path_w[i*(self.timesteps + 1):i*(self.timesteps + 1) + (self.timesteps + 1)] - 0.0))
		obs_idx = np.array(np.array(obs_cost)).argsort().argsort()
		y_idx = np.array(np.array(y_dist)).argsort().argsort()
		kkt_idx = np.array(np.array(kkt_cost)).argsort().argsort()
		cruise_idx = np.array(np.array(cruise_speed)).argsort().argsort()
		ang_idx = np.array(np.array(angular_vel)).argsort().argsort()
		index = np.inf
		min_cost = np.inf

		for i in range(self.num_goals):
			cost = 0*y_idx[i] + 2*obs_idx[i] + 1*cruise_idx[i] + 5*ang_idx[i]
			if cost<min_cost:
				min_cost = cost
				index = i
		fv = 0.0
		fw = 0.0
		for i in range(self.num_goals):
			env_x = []
			env_y = []
			env_theta = []
			if i == index:
				twist.linear.x = path_v[i*(self.timesteps + 1)+1]
				twist.angular.z = path_w[i*(self.timesteps + 1)+1]
				for p in range((self.timesteps + 1)):
					agent = [path_x[i*(self.timesteps + 1) + p], path_y[i*(self.timesteps + 1) + p], path_theta[i*(self.timesteps + 1) + p], self.lane_radii[0]+2]
					x_env, y_env, theta_env = self.centerline_to_env(agent)
					env_x.append(x_env)
					env_y.append(y_env)
				env_theta = np.arctan2(np.diff(np.array(env_y)), np.diff(np.array(env_x)))
				plt.plot(np.array(env_x), np.array(env_y), 'y')
				fv = path_v[i*(self.timesteps + 1)+1]
				fw = np.diff(np.array(env_theta))[1]/self.dt
				
			else:
				for p in range((self.timesteps + 1)):
					agent = [path_x[i*(self.timesteps + 1) + p], path_y[i*(self.timesteps + 1) + p], path_theta[i*(self.timesteps + 1) + p], self.lane_radii[0]+2]
					x_env, y_env, theta_env = self.centerline_to_env(agent)
					env_x.append(x_env)
					env_y.append(y_env)
				plt.plot(np.array(env_x), np.array(env_y), 'pink')

		self.plot_lanes()
		self.update_agent(twist)
		self.update_obstacles()
		self.plot_agents()

		for i in range(len(self.lane_y)):
			# goal = [self.goal_p.poses[i].position.x + self.agent_pose[0], self.goal_p.poses[i].position.y, 0.0]
			goal = [self.goal_p.poses[i].position.x, self.goal_p.poses[i].position.y, 0.0]
			env_x, env_y, env_theta = self.centerline_to_env(goal)
			plt.plot(env_x, env_y, 'xb')
		plt.ylim(-30+self.agent_pose_frenet[1], 30+self.agent_pose_frenet[1])
		plt.xlim(-30+self.agent_pose_frenet[0], 100+self.agent_pose_frenet[0])
		plt.draw()
		if self.agent_pose[0]<1000:
			self.loop+=1
		print([self.agent_pose_frenet[0], self.agent_pose_frenet[1], self.agent_pose_frenet[2], fv, fw])
		if self.agent_pose[0]<190:
			self.ego_poses_straight.append([self.agent_pose_frenet[0], self.agent_pose_frenet[1], self.agent_pose_frenet[2], fv, fw, self.nearest_obs_dist])
			np.savez("results/single_goal_straight/combined_env_straight.npz", np.array(self.ego_poses_straight))
			plt.savefig("results/single_goal_straight/images/"+str(self.loop)+".png")
		elif self.agent_pose[0]>=190 and self.agent_pose[0]<244:
			self.ego_poses_intersection.append([self.agent_pose_frenet[0], self.agent_pose_frenet[1], self.agent_pose_frenet[2], fv, fw, self.nearest_obs_dist])
			np.savez("results/single_goal_intersection/combined_env_intersection.npz", np.array(self.ego_poses_intersection))
			plt.savefig("results/single_goal_intersection/images/"+str(self.loop)+".png")
		elif self.agent_pose[0]>=244 and self.agent_pose[0]<244 + self.radius*np.pi:
			self.ego_poses_curved.append([self.agent_pose_frenet[0], self.agent_pose_frenet[1], self.agent_pose_frenet[2], fv, fw, self.nearest_obs_dist])
			np.savez("results/single_goal_curved/combined_env_curved.npz", np.array(self.ego_poses_curved))
			plt.savefig("results/single_goal_curved/images/"+str(self.loop)+".png")
		
		plt.pause(0.01)

		if self.agent_pose[0]>245 + self.radius*np.pi:
			quit()

	
def main(args=None):
	rclpy.init(args=args)

	minimal_subscriber = CombinedEnv()

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

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_subscriber.destroy_node()
	rclpy.shutdown()
	

if __name__ == "__main__":
	main()
