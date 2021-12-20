from numpy.lib.function_base import select
from numpy.lib.utils import info
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from acado_msgs.srv import GetVelocityCmd
from acado_msgs.msg import OdomArray

import numpy as np
import matplotlib.pylab as plt

class PlotPath(Node):

    def __init__(self):
        super().__init__('plot_path_cli')
        self.cli = self.create_client(GetVelocityCmd, "/get_vel")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.path_x = []
        self.path_y = []
        self.dt = 0.1
        self.cmd_vel = Twist()
        self.x = 0.0
        self.y = -6.0
        self.agent_p = np.array([self.x, self.y])
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.max_vel = 15.0
        self.bot_path = np.array([self.agent_p])
        self.agent_v = [self.v, self.w]
        self.goal = np.array([100, -6.0 ])
        self.intersection_point = np.array([500, -8])
        self.lane_val = 0
        self.index = 100.0
        self.done = [True, True, True, True, True]
        self.val = 1    # for lane change after intersection for ego vehicle
        self.val_obs = [1,1,1,1,1]    # for lane change after intersection for obstacles
        self.obs_pos_x_pos = np.array([[20,-6],[25,-2],[30,-6],[-100,-2.0],[-150,-6.0]])#
        self.obs_vel_x_pos = np.array([[9.0,0],[11.0,0],[13.0,0],[19.0,0],[17.0,0]])#
        self.obs_pos_x_neg = np.array([[-20,-14],[-25,-10],[-30,-14],[100,-10.0],[150,-14.0]])#
        self.obs_vel_x_neg = np.array([[-9.0,0],[-11.0,0],[-13.0,0],[-19.0,0],[-17.0,0]])#
        self.obs_pos_y_pos = np.array([[498.0,12.0],[494.0,17.0],[498,22.0],[494,-108.0],[498,-158.0]])#
        self.obs_vel_y_pos = np.array([[0.0,9.0],[0,11.0],[0,13.0],[0,19.0],[0,17.0]])#
        self.obs_pos_y_neg = np.array([[502.0,12.0],[504.0,17.0],[502,22.0],[504,-108.0],[502,-158.0]])#
        self.obs_vel_y_neg = np.array([[0.0,-9.0],[0,-11.0],[0,-13.0],[0,-19.0],[0,-17.0]])#

        self.sensor_range = 15.0
        self.x_lim_min = -5
        self.x_lim_max = 100
        self.y_lim_min = -50
        self.y_lim_max = 50
        self.req = GetVelocityCmd.Request()
        #while rclpy.ok() or np.linalg.norm(self.agent_p - self.goal) >=1.0:
            #self.plot_client()


    def send_request(self):
        self.req.odom.pose.pose.position.x = self.x
        self.req.odom.pose.pose.position.y = self.y
        self.req.odom.pose.pose.orientation.z = self.theta
        self.req.odom.twist.twist.linear.x = self.agent_v[0]
        self.req.odom.twist.twist.angular.z = self.agent_v[1]
        """obs_arr = PoseArray()
        for i in range(self.obs_vel.shape[0]):
            obs_pos = Pose()
            obs_pos.position.x = self.obs_pos[i][0]
            obs_pos.position.y = self.obs_pos[i][1]
            obs_arr.poses.append(obs_pos)
        print(obs_arr)
        self.req.obs_poses = obs_arr"""
        obs_pos = np.concatenate((self.obs_pos_x_pos, self.obs_pos_x_neg, self.obs_pos_y_pos, self.obs_pos_y_neg))
        obs_vel = np.concatenate((self.obs_vel_x_pos, self.obs_vel_x_neg, self.obs_vel_y_pos, self.obs_vel_y_neg))
        rel_p = obs_pos - self.agent_p
        rel_v = obs_vel - self.agent_v
        index = []
        for i in range(obs_pos.shape[0]):
            same_side = np.dot(rel_p[i], rel_v[i])
            if same_side<0.0:
                index.append(i)
        if len(index) > 10:
            index.sort(key = lambda p: (obs_pos[p][0] - self.agent_p[0])**2 + (obs_pos[p][1] - self.agent_p[1])**2)
        odom_arr = OdomArray()
        for i in range(len(index)):
            #obs_pos = Pose()
            if i>9:
                break
            odom = Odometry()
            odom.pose.pose.position.x = obs_pos[index[i]][0]
            odom.pose.pose.position.y = obs_pos[index[i]][1]
            odom.twist.twist.linear.x = obs_vel[index[i]][0]
            odom.twist.twist.linear.y = obs_vel[index[i]][1]
            print(obs_pos[index[i]][0], obs_pos[index[i]][1], obs_vel[index[i]][0], obs_vel[index[i]][1])
            odom_arr.odom.append(odom)
        self.req.odom_arr = odom_arr
        goal_pose = Pose()
        if ((self.agent_p[0]-self.goal[0])**2 + (self.agent_p[1]-self.goal[1])**2)**0.5 < 80:
            if not self.lane_val:
                self.goal[0] = self.goal[0]+100
                if self.goal[0]%500 == 0:
                    self.goal[1] = -8 - self.goal[1]
            else:
                self.goal[1] = self.goal[1]-100
                if self.goal[1]%-502 == 0 or self.goal[1] == -102:
                    self.goal[0] = self.intersection_point[0]+4 + self.val*2
                self.val = self.val*-1

        print(self.goal, self.agent_p)
        print(obs_pos, self.index)
        int_info = PoseArray()
        if self.index>50:
            info = Pose()
            info.position.x = 0.0
            int_info.poses.append(info)

            info = Pose()
            info.position.x = 0.0
            info.position.y = 1.0
            info.position.z = -self.goal[1]
            int_info.poses.append(info)

            info = Pose()
            info.position.x = -1e12
            info.position.y = -6.9
            int_info.poses.append(info)

            info = Pose()
            info.position.x = 1e12
            info.position.y = -1.1
            int_info.poses.append(info)
        else:
            info = Pose()
            info.position.x = self.index
            int_info.poses.append(info)

            info = Pose()
            info.position.x = 1.0
            info.position.y = 0.0
            info.position.z = -self.goal[0]
            int_info.poses.append(info)

            info = Pose()
            info.position.x = 501.1
            info.position.y = -1e12
            int_info.poses.append(info)

            info = Pose()
            info.position.x = 506.9
            info.position.y = 0.0
            int_info.poses.append(info)
            

        self.req.obs_poses = int_info
                
        goal_pose.position.x = self.goal[0]
        goal_pose.position.y = self.goal[1]

        self.req.goal = goal_pose
        self.future = self.cli.call_async(self.req)

    def plot_path(self, twist, path):
        plt.ion()
        plt.show()
        plt.clf()
        self.path_x = []
        self.path_y = []
        m=-1
        done = False
        for i in path.poses:
            m=m+1
            if i.position.x > 500:
                if not done:
                    self.index = float(m)
                    done = True
                
            self.path_x.append(i.position.x)
            self.path_y.append(i.position.y)
        self.agent_v = [twist.linear.x, twist.angular.z]
        self.theta = self.theta +self.agent_v[1]*self.dt
        self.x = self.x + self.agent_v[0]*np.cos(self.theta)*self.dt
        self.y = self.y + self.agent_v[0]*np.sin(self.theta)*self.dt
        #self.x = self.path_x[1]
        #self.y = self.path_y[1]
        self.agent_p = np.array([self.x, self.y])
        self.obs_pos_x_pos = self.obs_pos_x_pos + np.dot(self.obs_vel_x_pos, self.dt)
        self.obs_pos_x_neg = self.obs_pos_x_neg + np.dot(self.obs_vel_x_neg, self.dt)
        self.obs_pos_y_pos = self.obs_pos_y_pos + np.dot(self.obs_vel_y_pos, self.dt)
        self.obs_pos_y_neg = self.obs_pos_y_neg + np.dot(self.obs_vel_y_neg, self.dt)
        self.bot_path = np.append(self.bot_path, [self.agent_p], axis=0)
        bot_circle = plt.Circle((self.x, self.y), 1.0, color='b')
        plt.gca().add_patch(bot_circle)
        goal_circle = plt.Circle((self.goal[0], self.goal[1]), 1.0, color='black')
        plt.gca().add_patch(goal_circle)
        for i in range(self.obs_vel_x_pos.shape[0]):
            obs_circle = plt.Circle((self.obs_pos_x_pos[i][0], self.obs_pos_x_pos[i][1]), 1.0, color='r')
            plt.gca().add_patch(obs_circle)
        for i in range(self.obs_vel_x_neg.shape[0]):
            obs_circle = plt.Circle((self.obs_pos_x_neg[i][0], self.obs_pos_x_neg[i][1]), 1.0, color='r')
            plt.gca().add_patch(obs_circle)
        for i in range(self.obs_vel_y_pos.shape[0]):
            obs_circle = plt.Circle((self.obs_pos_y_pos[i][0], self.obs_pos_y_pos[i][1]), 1.0, color='r')
            plt.gca().add_patch(obs_circle)
        for i in range(self.obs_vel_y_neg.shape[0]):
            obs_circle = plt.Circle((self.obs_pos_y_neg[i][0], self.obs_pos_y_neg[i][1]), 1.0, color='r')
            plt.gca().add_patch(obs_circle)
        #plt.arrow(self.x, self.y, 0.5*self.agent_v[0]*np.cos(self.theta), 0.5*self.agent_v[0]*np.sin(self.theta),length_includes_head=True, head_width=0.3, head_length=0.2)
        plt.plot(self.bot_path[:,0], self.bot_path[:,1])
        plt.plot(self.path_x, self.path_y, 'y')

        plt.plot([-2000000,492], [0,0], 'black')
        plt.plot([-2000000,492], [-4,-4], linestyle='--', color='black')
        plt.plot([-2000000,492], [-8,-8], 'black', linewidth=3.0)
        plt.plot([-2000000,492], [-12,-12], linestyle='--', color='black')
        plt.plot([-2000000,492], [-16,-16], 'black')

        plt.plot([508,200000], [0,0], 'black')
        plt.plot([508,200000], [-4,-4], linestyle='--', color='black')
        plt.plot([508,200000], [-8,-8], 'black', linewidth=3.0)
        plt.plot([508,200000], [-12,-12], linestyle='--', color='black')
        plt.plot([508,200000], [-16,-16], 'black')


        plt.plot([492,492], [0, 200000], 'black')
        plt.plot([496,496], [0, 200000], linestyle='--', color='black')
        plt.plot([508,508], [0, 200000], 'black')
        plt.plot([504,504], [0, 200000], linestyle='--', color='black')
        plt.plot([500,500], [0, 200000], color='black', linewidth=3.0)

        plt.plot([492,492], [-200000, -16], 'black')
        plt.plot([496,496], [-200000, -16], linestyle='--', color='black')
        plt.plot([508,508], [-200000, -16], 'black')
        plt.plot([504,504], [-200000, -16], linestyle='--', color='black')
        plt.plot([500,500], [-200000, -16], color='black', linewidth=3.0)

        plt.ylim(self.agent_p[1]-40, self.agent_p[1]+40) 
        #plt.xlim(self.agent_p[0]-40, self.agent_p[0]+40)
        plt.xlim(self.agent_p[0]-80, self.agent_p[0]+80)
        plt.tight_layout()
        """if self.x_lim_max < self.agent_p[0]+50:
            self.x_lim_min = self.agent_p[0]-20
            self.x_lim_max = self.agent_p[0] + 100
        plt.xlim(self.x_lim_min, self.x_lim_max) 
        if self.y_lim_max < self.agent_p[1]+50:
            self.y_lim_min = self.agent_p[1]-20
            self.y_lim_max = self.agent_p[1] + 100
        plt.ylim(self.y_lim_min, self.y_lim_max) """

        #plt.xlim(-10, 1100) 
        plt.title('Path')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.text(self.agent_p[0], -12.5, "Velocity = "+str(self.agent_v[0]))
        plt.text(self.agent_p[0], -15, "Goal = ("+str(self.goal[0])+", "+str(self.goal[1])+")")
        plt.draw()
        plt.pause(0.0001)
        for i in range(self.obs_vel_x_pos.shape[0]):
            if ((self.agent_p[0]-self.obs_pos_x_pos[i][0])**2 + (self.agent_p[1]-self.obs_pos_x_pos[i][1])**2)**0.5 < 2.001:
                print(self.obs_pos_x_pos[i])
                quit()
            if self.obs_vel_x_pos[i][0] < self.max_vel:
                if self.obs_pos_x_pos[i][0] < self.agent_p[0] - 80:
                    self.obs_pos_x_pos[i][0] = self.agent_p[0] + 81
                    self.obs_pos_x_pos[i][1] = -8 - self.obs_pos_x_pos[i][1]
            else:
                if self.obs_pos_x_pos[i][0] > self.agent_p[0] + 80:
                    self.obs_pos_x_pos[i][0] = self.agent_p[0] - 81
                    self.obs_pos_x_pos[i][1] = -8 - self.obs_pos_x_pos[i][1]

        for i in range(self.obs_vel_x_neg.shape[0]):
            if ((self.agent_p[0]-self.obs_pos_x_neg[i][0])**2 + (self.agent_p[1]-self.obs_pos_x_neg[i][1])**2)**0.5 < 2.001:
                print(self.obs_pos_x_neg[i])
                quit()
            if self.obs_vel_x_neg[i][0] < self.max_vel:
                if self.obs_pos_x_neg[i][0] < self.agent_p[0] - 80:
                    self.obs_pos_x_neg[i][0] = self.agent_p[0] + 81
            else:
                if self.obs_pos_x_neg[i][0] > self.agent_p[0] - 80:
                    self.obs_pos_x_neg[i][0] = self.agent_p[0] + 81
                    
        for i in range(self.obs_vel_y_pos.shape[0]):
            if ((self.agent_p[0]-self.obs_pos_y_pos[i][0])**2 + (self.agent_p[1]-self.obs_pos_y_pos[i][1])**2)**0.5 < 2.001:
                print(self.obs_pos_y_pos[i])
                quit()
            if self.obs_vel_y_pos[i][1] < self.max_vel:
                if self.obs_pos_y_pos[i][1] < self.agent_p[1] - 80:
                    self.obs_pos_y_pos[i][1] = self.agent_p[1] + 81
                elif self.obs_pos_y_pos[i][1] > self.agent_p[1] + 80:
                    self.obs_pos_y_pos[i][1] = self.agent_p[1] - 81
            else:
                if self.obs_pos_y_pos[i][1] > self.agent_p[1] + 80:
                    self.obs_pos_y_pos[i][1] = self.agent_p[1] - 81
                elif self.obs_pos_y_pos[i][1] < self.agent_p[1] - 80:
                    self.obs_pos_y_pos[i][1] = self.agent_p[1] + 81


        for i in range(self.obs_vel_y_neg.shape[0]):
            if ((self.agent_p[0]-self.obs_pos_y_neg[i][0])**2 + (self.agent_p[1]-self.obs_pos_y_neg[i][1])**2)**0.5 < 2.001:
                print(self.obs_pos_y_neg[i])
                quit()
            if self.obs_vel_y_neg[i][1] < self.max_vel:
                if self.obs_pos_y_neg[i][1] < self.agent_p[1] - 80:
                    self.obs_pos_y_neg[i][1] = self.agent_p[1] + 81
                elif self.obs_pos_y_neg[i][1] > self.agent_p[1] + 80:
                    self.obs_pos_y_neg[i][1] = self.agent_p[1] - 81
            else:
                if self.obs_pos_y_neg[i][1] < self.agent_p[1] - 80:
                    self.obs_pos_y_neg[i][1] = self.agent_p[1] + 81
                if self.obs_pos_y_neg[i][1] > self.agent_p[1] + 80:
                    self.obs_pos_y_neg[i][1] = self.agent_p[1] - 81

        
        if ((self.agent_p[0]-self.intersection_point[0])**2 + (self.agent_p[1]-self.intersection_point[1])**2)**0.5 < 100:
            self.lane_val = 1



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = PlotPath()

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
                    minimal_subscriber.plot_path(response.twist, response.path)
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