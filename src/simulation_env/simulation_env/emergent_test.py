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

class EmergentEnv(Node):
	def __init__(self) -> None:
		super().__init__('emergent_plot')
		self.cli = self.create_client(GetControlsMulti, "/get_vel")
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')

		self.timesteps = 30
		self.time_secs = 3.0

		self.num_goals = 6
		# self.num_goals = 1

		self.vert_lane_lims = np.array([[-4.5, -4.5], [-1.5, -1.5], [1.5, 1.5], [4.5, 4.5]])

		self.agent_pose = [3.0, 30.0, np.pi/2]
		self.agent_vel = [10.0, 0.0]
		self.dt = 0.1

		self.dist_goal = 30.0
		self.time_arr = np.linspace(0, self.time_secs, (self.timesteps + 1))


		self.obstacles = np.array([[3, 43.0, 90*2*np.pi/360, 0.0, 0.0, 0.0, 0],
									[0.0, 35.0, 90*2*np.pi/360, 0.0, 0.0, 0.0, 0],
									[0.0, 40.0, 90*2*np.pi/360, 0.0, 0.0, 0.0, 0],
									[0.0, 4100.0, 90*2*np.pi/360, 0.0, 0.0, 0.0, 0],
									[0.0, 4200.0, 90*2*np.pi/360, 0.0, 0.0, 0.0, 0],
									[0.0, 4300.0, 90*2*np.pi/360, 0.0, 0.0, 0.0, 0],
									[0.0, 4400.0, 90*2*np.pi/360, 0.0, 0.0, 0.0, 0],
									[0.0, 4500.0, 90*2*np.pi/360, 0.0, 0.0, 0.0, 0],
									[0.0, 4600.0, 90*2*np.pi/360, 0.0, 0.0, 0.0, 0],
									[0.0, 4700.0, 90*2*np.pi/360, 0.0, 0.0, 0.0, 0]])

		# self.fig = plt.figure(0)
		# #self.ax1 = self.fig.add_subplot(111, aspect='equal')
		# #self.ax2 = self.fig.add_subplot(212, aspect='equal')
		# #mng = plt.get_current_fig_manager()
		# # mng.full_screen_toggle()
		# self.fig.set_size_inches(20, 10)

		self.req = GetControlsMulti.Request()
		print("STARTING SIMULATION")

	def send_request(self):
		self.nearest_obstacles = []

		self.req.start.pose.pose.position.x = self.agent_pose[0]
		self.req.start.pose.pose.position.y = self.agent_pose[1]
		self.req.start.pose.pose.orientation.z = self.agent_pose[2]
		self.req.start.twist.twist.linear.x = self.agent_vel[0]
		self.req.start.twist.twist.angular.z = self.agent_vel[1]

		obs_pos = self.obstacles
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
		lanes = [-3.0, 0.0, 3.0, -3.0, 0.0, 3.0]
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
			goal_pose.position.y = float(xgoal[i]) + self.agent_pose[1]
			goal_pose.position.x = lanes[i]#lanes[0]#random.choice(lanes)
			goal_pose.orientation.z = np.pi/2
			goals.poses.append(goal_pose)

		info = Pose()
		info.position.x = 0.0
		info.position.y = 0.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 1.0  #self.agent_pose[0]
		info.position.y = 0.0  #goal_pose.position.y+1e12
		info.position.z = -goal_pose.position.x  #1e12
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 1.0 #26.0 + 1e12
		info.position.z = 0.0 #1e12
		info.orientation.x = 0.0
		info.orientation.y = 0.0
		info.orientation.z = 0.0
		info.orientation.w = 4.5
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 0.0 #self.agent_pose[0]
		info.position.y =  1.0 #-4.0 + 1e12
		info.position.z = 0.0
		info.orientation.x = 0.0
		info.orientation.y = 0.0
		info.orientation.z = 0.0
		info.orientation.w = -4.5
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
	
	def plot_lanes(self):
		plt.plot(self.vert_lane_lims[0], [20, 80], color='black', linewidth=2.0)
		plt.plot(self.vert_lane_lims[1], [20, 80], color='black', linewidth=1.0)
		plt.plot(self.vert_lane_lims[2], [20, 80], color='black', linewidth=1.0)
		plt.plot(self.vert_lane_lims[3], [20, 80], color='black', linewidth=2.0)

	
	def plot_agents(self):
		for i in range(3):
			obs = plt.Circle((self.obstacles[i][0], self.obstacles[i][1]), 1.0, color='r')
			plt.gca().add_patch(obs)
		agent = plt.Circle((self.agent_pose[0], self.agent_pose[1]), 1.0, color='g')
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

		print(len(path_x))

		## Get Ranks
		y_dist = []
		kkt_cost = []
		cruise_speed = []
		angular_vel = []
		obs_cost = []
		for i in range(self.num_goals):
			res_obs_x = np.linalg.norm(path_x[i*(self.timesteps + 1):i*(self.timesteps + 1) + (self.timesteps + 1)] - self.nearest_obstacles[:3, :(self.timesteps + 1)], axis=1)
			res_obs_y = np.linalg.norm(path_y[i*(self.timesteps + 1):i*(self.timesteps + 1) + (self.timesteps + 1)] - self.nearest_obstacles[:3, (self.timesteps + 1):], axis=1)
			res_obs = np.vstack((res_obs_x, res_obs_y))
			res_obs = np.linalg.norm(res_obs, axis=0)
			print(res_obs)
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
			print(obs_idx[i], cruise_idx[i], ang_idx[i])
			cost = 0*y_idx[i] + 2*obs_idx[i] + 1*cruise_idx[i] + 5*ang_idx[i]
			if cost<min_cost:
				min_cost = cost
				index = i
			print(index)
		for i in range(len(self.lane_y)):
			if i == index:
				plt.plot(path_x[i*(self.timesteps + 1):i*(self.timesteps + 1) + (self.timesteps + 1)], path_y[i*(self.timesteps + 1):i*(self.timesteps + 1) + (self.timesteps + 1)], 'y')
				twist.linear.x = path_v[i*(self.timesteps + 1)+1]
				twist.angular.z = path_w[i*(self.timesteps + 1)+1]
			else:
				plt.plot(path_x[i*(self.timesteps + 1):i*(self.timesteps + 1) + (self.timesteps + 1)], path_y[i*(self.timesteps + 1):i*(self.timesteps + 1) + (self.timesteps + 1)], 'pink')
		
		for i in range(len(self.lane_y)):
			#if i == index:
			plt.plot(self.goal_p.poses[i].position.x, self.goal_p.poses[i].position.y, 'xb')

		self.plot_lanes()
		self.plot_agents()

		# plt.axis('equal')
		plt.ylim(-5+self.agent_pose[1], 15+self.agent_pose[1])
		plt.xlim(-10, 10)
		plt.draw()
		
		plt.pause(0.01)

	
def main(args=None):
	rclpy.init(args=args)

	minimal_subscriber = EmergentEnv()

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