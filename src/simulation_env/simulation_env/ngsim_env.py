from math import radians
import random
import numpy as np
from numpy import genfromtxt
from time import time
import matplotlib.pyplot as plt

from matplotlib.patches import Ellipse, Rectangle
from numpy import dtype
from shapely.geometry import Point
from shapely.affinity import scale, rotate

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from acado_msgs.srv import GetVelocityCmd
from acado_msgs.srv import GetControls
from acado_msgs.srv import GetControlsMulti
from acado_msgs.msg import OdomArray

class NGSIMTest(Node):
	def __init__(self):
		super().__init__("ngsim_env")
		self.cli = self.create_client(GetControlsMulti, "/get_vel")
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		
		self.ngsim_data = genfromtxt('src/simulation_env/i-80-traj-new-400-415.csv', delimiter=',')
		self.ngsim_data[:, 8] = np.round(self.ngsim_data[:, 8], 1)
		self.time_arr = np.round(np.arange(0.0, 874.0, 0.1), 1)
		self.loop = 0
		#	400:415 = 100
		#	500:515 = 98
		self.time_shift = 140#97#200.0
		self.num_obs = 10
		self.time_arr = np.linspace(0, 5, 51)

		self.xlim = 0.0

		self.obs_list = []
		self.nearest_obstacles = []

		#self.agent_pose = [0.0, 2.0, 0.0]
		self.agent_pose = [0.0, 14.0, 0.0]
		#self.agent_pose = [0.0, 18.0, 0.0]
		self.agent_vel = [15.0, 0.0]
		self.dt = 0.1

		self.lane_y = [2.0, 6.0, 10.0, 14.0, 18.0, 22.0]
		self.dist_goal = 80.0

		#self.fig = plt.figure(0)
		#self.ax1 = self.fig.add_subplot(111, aspect='equal')
		#self.ax2 = self.fig.add_subplot(212, aspect='equal')
		#mng = plt.get_current_fig_manager()
		# mng.full_screen_toggle()
		#self.fig.set_size_inches(20, 10)
		self.req = GetControlsMulti.Request()
		print("STARTING SIMULATION")

	def send_request(self):
		self.obs_list = []
		self.nearest_obstacles = []
		cur_time = np.round(self.time_shift+ self.loop*self.dt, 2)
		idxs = np.where((self.ngsim_data[:, 8] == (cur_time)))[0]
		print(len(idxs), cur_time)
		vehicles = self.ngsim_data[idxs]
		y = vehicles[:, 1]
		#y = vehicles[:, 1] - 4.0
		x = vehicles[:, 2]
		#print(x)
		self.obs_list.append(x)
		self.obs_list.append(y)	

		psi = np.pi/2 - vehicles[:, 3]
		vy = vehicles[:, 6]
		vx = vehicles[:, 7]
		vobs = (vx**2 + vy**2)**0.5
		wobs = (psi - np.arccos(vx/vobs))/self.dt
		self.num_obs = min(len(idxs), 10)

		obs_list = np.vstack((np.array(x), np.array(y), np.array(psi), np.array(vx), np.array(vy))).T
		#print(obs_list.shape)
		dist = np.sqrt((obs_list[:,0] - self.agent_pose[0])**2 + (obs_list[:,1] - self.agent_pose[1])**2)
		sorted_obs = obs_list[dist.argsort()]
		#print(sorted_obs.shape)

		self.req.start.pose.pose.position.x = self.agent_pose[0]
		self.req.start.pose.pose.position.y = self.agent_pose[1]
		self.req.start.pose.pose.orientation.z = self.agent_pose[2]
		self.req.start.twist.twist.linear.x = self.agent_vel[0]
		self.req.start.twist.twist.angular.z = self.agent_vel[1]

		obstacles = OdomArray()
		for i in range(self.num_obs):
			odom = Odometry()
			odom.pose.pose.position.x = sorted_obs[i][0]
			odom.pose.pose.position.y = sorted_obs[i][1]
			odom.pose.pose.orientation.z = sorted_obs[i][2]
			odom.twist.twist.linear.x = sorted_obs[i][3]
			odom.twist.twist.linear.y = sorted_obs[i][4]
			#obs = plt.Circle((sorted_obs[i][0], sorted_obs[i][1]), 1.2, color='g')
			#plt.gca().add_patch(obs)
			obstacles.odom.append(odom)
			obs_path_x = sorted_obs[i][0] + sorted_obs[i][3]*self.time_arr
			obs_path_y = sorted_obs[i][1] + sorted_obs[i][4]*self.time_arr
			self.nearest_obstacles.append(np.concatenate((obs_path_x, obs_path_y)))
		if self.num_obs<10:
			for i in range(10-self.num_obs):
				odom = Odometry()
				odom.pose.pose.position.x = sorted_obs[i][0]
				odom.pose.pose.position.y = sorted_obs[i][1]
				odom.pose.pose.orientation.z = sorted_obs[i][2]
				odom.twist.twist.linear.x = sorted_obs[i][3]
				odom.twist.twist.linear.y = sorted_obs[i][4]
				obs_path_x = sorted_obs[i][0] + sorted_obs[i][3]*self.time_arr
				obs_path_y = sorted_obs[i][1] + sorted_obs[i][4]*self.time_arr
				self.nearest_obstacles.append(np.concatenate((obs_path_x, obs_path_y)))
				#obs = plt.Circle((sorted_obs[i][0], sorted_obs[i][1]), 1.2, color='g')
				#plt.gca().add_patch(obs)
				obstacles.odom.append(odom)
		self.req.obstacles = obstacles
		self.nearest_obstacles = np.array(self.nearest_obstacles)
		print(self.nearest_obstacles.shape)
		goal = PoseArray()
		lane_cons = PoseArray()
		lane_cons, goal = self.get_lane_cons_overtake()
		self.req.goal = goal
		self.req.lane_cons = lane_cons
		#print("Goal = ", goal)
		#print(self.obs[:,0])
		#print("Lane Cons == ", self.req.lane_cons)
		self.goal_p = goal
		self.future = self.cli.call_async(self.req)

	def get_lane_cons_overtake(self):
		lane_info = PoseArray()
		goals = PoseArray()
		lanes = [2.0, 6.0, 10.0, 14.0, 18.0, 22.0]
		cur_lane = lanes[np.argmin(np.abs(np.array(lanes) - self.agent_pose[1]))]
		#lanes = [-2.0]
		if cur_lane == 2.0:
			lanes = [2.0, 6.0, 10.0, 2.0, 6.0, 10.0]
		elif cur_lane == 22.0:
			lanes = [14.0, 18.0, 22.0, 14.0, 18.0, 22.0]
		else:
			lanes = [cur_lane - 4.0, cur_lane, cur_lane + 4.0]*2
		##lanes = [2.0, 6.0, 10.0, 14.0, 18.0, 22.0]
		xgoal = [self.dist_goal, self.dist_goal, self.dist_goal, self.dist_goal*3/4, self.dist_goal*3/4, self.dist_goal*3/4] 
		self.lane_y = lanes
		print(lanes)
		g_dist = 50.0
		for i in range(len(lanes)):
			goal_pose = Pose()
			goal_pose.position.x = self.agent_pose[0] + xgoal[i]
			goal_pose.position.y = lanes[i]#lanes[0]#random.choice(lanes)
			goal_pose.orientation.z = 0.0
			goals.poses.append(goal_pose)
		"""#print(goals)
		goal_pose = Pose()
		goal_pose.position.x = self.agent_pose[0] + self.dist_goal
		goal_pose.position.y = lanes[1]#random.choice(lanes)
		goal_pose.orientation.z = 0.0
		goals.poses.append(goal_pose)
		#print(goals)"""

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
		info.orientation.w = -23.0
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

	
	def update_agent(self, twist):
		self.agent_vel[0] = twist.linear.x
		self.agent_vel[1] = twist.angular.z
		self.agent_pose[2] = self.agent_pose[2] + self.agent_vel[1]*self.dt
		self.agent_pose[0] = self.agent_pose[0] + self.agent_vel[0]*np.cos(self.agent_pose[2])*self.dt
		self.agent_pose[1] = self.agent_pose[1] + self.agent_vel[0]*np.sin(self.agent_pose[2])*self.dt
		#self.ax2.plot(self.agent_vel[0])
	
	
	def plot_lanes(self):
		plt.plot([-20000, 20000], [0, 0], color='black', linewidth=2.0)
		plt.plot([-20000, 20000], [4, 4], color='black', linewidth=1.0)
		plt.plot([-20000, 20000], [8, 8], color='black', linewidth=1.0)
		plt.plot([-20000, 20000], [12, 12], color='black', linewidth=1.0)
		plt.plot([-20000, 20000], [16, 16], color='black', linewidth=1.0)
		plt.plot([-20000, 20000], [20, 20], color='black', linewidth=1.0)
		plt.plot([-20000, 20000], [24, 24], color='black', linewidth=2.0)
	
	def plot_obstacles(self):
		for i in range(len(self.obs_list[0])):
			obs = plt.Circle((self.obs_list[0][i], self.obs_list[1][i]), 1.0, color='r')
			plt.gca().add_patch(obs)
		agent = plt.Circle((self.agent_pose[0], self.agent_pose[1]), 1.0, color='g')
		plt.text(self.xlim, 33, 'Vel = %s'%(round(self.agent_vel[0],2)), fontsize=10)
		plt.text(self.xlim+40, 33, 'Index = %s'%(self.index), fontsize=10)

		plt.gca().add_patch(agent)
	
	def on_press(self, event):
		self.behaviour_event = event.key
	

	def plot(self, twist, path, kkt):
		plt.ion()
		plt.show()
		plt.clf()
		#self.ax1 = self.fig.add_subplot(211, aspect='equal')
		#self.ax2 = self.fig.add_subplot(212, aspect='equal')
		self.path_x = []
		self.path_y = []
		self.path_theta = []
		self.path_v = []
		self.path_w = []
		print(len(path.poses))
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
		for i in range(len(self.lane_y)):
			res_obs_x = np.linalg.norm(self.path_x[i*51:i*51 + 51] - self.nearest_obstacles[:, :51], axis=1)
			res_obs_y = np.linalg.norm(self.path_y[i*51:i*51 + 51] - self.nearest_obstacles[:, 51:], axis=1)
			res_obs = np.vstack((res_obs_x, res_obs_y))
			res_obs = np.linalg.norm(res_obs, axis=0)
			obs_cost.append(1/np.min(res_obs))
			y_dist.append(np.linalg.norm(self.path_y[i*51:i*51 + 51] - 14.0))
			kkt_cost.append(kkt[i])
			cruise_speed.append(np.linalg.norm(self.path_v[i*51:i*51 + 51] - 15.0))
			angular_vel.append(np.linalg.norm(self.path_w[i*51:i*51 + 51] - 0.0))
		obs_idx = np.array(np.array(obs_cost)).argsort().argsort()
		y_idx = np.array(np.array(y_dist)).argsort().argsort()
		kkt_idx = np.array(np.array(kkt_cost)).argsort().argsort()
		cruise_idx = np.array(np.array(cruise_speed)).argsort().argsort()
		ang_idx = np.array(np.array(angular_vel)).argsort().argsort()
		index = np.inf
		min_cost = np.inf
		for i in range(len(self.lane_y)):
			cost = 20*y_idx[i] + 40*obs_idx[i] + 1*cruise_idx[i] + 10*ang_idx[i]
			if cost<min_cost:
				min_cost = cost
				index = i
		self.index = index
		#self.fig.canvas.mpl_connect('key_press_event', self.on_press)
		#print(len(self.path_x))
		#print(twist)
		for i in range(len(self.lane_y)):
			if i == index:
				plt.plot(self.path_x[i*51:i*51 + 51], self.path_y[i*51:i*51 + 51], 'y')
				twist.linear.x = self.path_v[i*51+1]
				twist.angular.z = self.path_w[i*51+1]
			else:
				plt.plot(self.path_x[i*51:i*51 + 51], self.path_y[i*51:i*51 + 51], 'pink')
		print(self.index)
		if self.agent_pose[0]<10000:
			self.update_agent(twist)
		#self.update_obstacles()
		self.plot_lanes()
		self.plot_obstacles()
		#obs = plt.Circle((self.goal_p.position.x, self.goal_p.position.y), 1.0, color='b')
		#plt.add_patch(obs)
		#print(self.goal_p.poses[0].position.y, self.goal_p.poses[1].position.y, len(self.goal_p.poses))
		for i in range(len(self.lane_y)):
			#if i == index:
			plt.plot(self.goal_p.poses[i].position.x, self.goal_p.poses[i].position.y, 'xb')
		#plt.plot(self.goal_p.poses[1].position.x, self.goal_p.poses[1].position.y, 'xb')
		#plt.ylim(self.agent_pose[1]-50, self.agent_pose[1]+50)
		#plt.xlim(self.agent_pose[0]-50, self.agent_pose[0]+150)
		plt.ylim(-10, 30)
		#self.time_arr.append(self.agent_pose[0])
		#self.vel.append(self.agent_vel[0])
		#plt.xlim(-30+self.agent_pose[0], 100+self.agent_pose[0])
		if (self.agent_pose[0]-self.xlim)>70.0:
			self.xlim = self.xlim+100.0
		plt.xlim(-30+self.xlim, 100+self.xlim)
		#self.ax2.plot(self.time_arr, self.vel)
		#self.ax2.set_xlim(-30+self.xlim, 100+self.xlim)
		#self.ax2.set_ylim(0,22)
		#plt.ylim(-50, 250)
		#plt.xlim(-350, 350)
		#plt.draw()
		if self.agent_pose[0]<1000:
			self.loop+=1
		plt.draw()
		#plt.pause(0.0001)
		#plt.savefig("3_"+str(self.loop)+".png")
		plt.pause(0.0001)
		#if self.agent_pose[0]>20.0:
			#quit()
		#if self.agent_pose[1]>100.0:
		#	quit()
	


def main(args=None):
	rclpy.init(args=args)

	minimal_subscriber = NGSIMTest()

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