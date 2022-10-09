import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray

import numpy as np
import matplotlib.pylab as plt

class PlotPath(Node):

    def __init__(self):
        super().__init__('plot_path')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.vel_cb, 1)
        self.subs_path = self.create_subscription(PoseArray, 'path', self.path_cb, 1)
        self.publisher_ = self.create_publisher(Odometry, "odom", 1)
        self.publisher_obs_ = self.create_publisher(PoseArray, "obs_pose", 1)
        timer_period = 0.1  # seconds
        self.path_x = []
        self.path_y = []
        self.dt = 0.1
        self.cmd_vel = Twist()
        self.subscription  # prevent unused variable warning
        self.x = 0.0
        self.y = 0.0
        self.agent_p = np.array([self.x, self.y])
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.bot_path = np.array([self.agent_p])
        self.agent_v = [self.v, self.w]
        self.goal = [20.0, 0.0 ]
        self.obs_pos = np.array([[5.0,1.0], [20.0,0.2], [15.0,-1.0]])
        self.obs_vel = np.array([[0.5,0], [-0.6,0], [-0.45,0]])
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vel_cb(self, msg):
        plt.ion()
        plt.show()
        plt.clf()
        self.agent_v = [msg.linear.x, msg.angular.z]
        self.theta = self.theta +self.agent_v[1]*self.dt
        self.x = self.x + self.agent_v[0]*np.cos(self.theta)*self.dt
        self.y = self.y + self.agent_v[0]*np.sin(self.theta)*self.dt
        self.agent_p = np.array([self.x, self.y])
        self.obs_pos = self.obs_pos + np.dot(self.obs_vel, self.dt)
        self.bot_path = np.append(self.bot_path, [self.agent_p], axis=0)
        bot_circle = plt.Circle((self.x, self.y), 0.5, color='b')
        plt.gca().add_patch(bot_circle)
        goal_circle = plt.Circle((self.goal[0], self.goal[1]), 0.5, color='black')
        plt.gca().add_patch(goal_circle)
        for i in range(self.obs_vel.shape[0]):
            obs_circle = plt.Circle((self.obs_pos[i][0], self.obs_pos[i][1]), 0.5, color='r')
            plt.gca().add_patch(obs_circle)
        #plt.arrow(self.x, self.y, 0.5*self.agent_v[0]*np.cos(self.theta), 0.5*self.agent_v[0]*np.sin(self.theta),length_includes_head=True, head_width=0.3, head_length=0.2)
        plt.plot(self.bot_path[:,0], self.bot_path[:,1])
        plt.plot(self.path_x, self.path_y, 'y')
        plt.xlim(-5, 23) 
        plt.ylim(-5, 7)
        plt.title('Path')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.draw()
        plt.pause(0.0001)
    
    def path_cb(self, msg):
        self.path_x = []
        self.path_y = []
        for i in msg.poses:
            self.path_x.append(i.position.x)
            self.path_y.append(i.position.y)
            #print(i.position.x, i.position.y)
        print(self.path_x, self.path_y)
        print(self.x, self.y)

    def timer_callback(self):
        odom = Odometry()
        #print("Pulishing...")
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = self.theta
        odom.twist.twist.linear.x = self.agent_v[0]
        odom.twist.twist.angular.z = self.agent_v[1]
        self.publisher_.publish(odom)
        obs_arr = PoseArray()
        for i in range(self.obs_vel.shape[0]):
            obs_pos = Pose()
            obs_pos.position.x = self.obs_pos[i][0]
            obs_pos.position.y = self.obs_pos[i][1]
            obs_arr.poses.append(obs_pos)
        self.publisher_obs_.publish(obs_arr)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = PlotPath()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()