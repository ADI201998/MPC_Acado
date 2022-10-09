from matplotlib.pyplot import plot
from numpy.core.defchararray import center
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from acado_msgs.srv import GetVelocityCmd
from acado_msgs.msg import OdomArray

import numpy as np
import random
import matplotlib.pylab as plt

class PlotPath(Node):

    def __init__(self):
        super().__init__('curved_path_plot')
        self.cli = self.create_client(GetVelocityCmd, "/get_vel")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.path_x = []
        self.path_y = []
        self.dt = 0.1
        self.cmd_vel = Twist()
        self.x = 0.0
        self.y = -102.0
        self.agent_p = np.array([self.x, self.y])
        self.theta = 0.0
        self.v = 14.0
        self.w = 0.0
        self.max_vel = 15.0
        self.bot_path = np.array([self.agent_p])
        self.agent_v = [self.v, self.w]

        self.center = [0,0]
        self.num_obs = 5
        self.rr = 108.0
        self.lr = 100.0
        self.mr = 104.0
        self.rm = 106.25
        self.lm = 101.75
        self.theta_g = 300.0
        self.cur_rad = self.rm
        self.obs_rad = np.array([102.0, 106.0, 110.0, 114.0])
        self.index = 1
        self.goal = np.array([self.rm*np.cos(self.theta_g*2*np.pi/360), self.rm*np.sin(self.theta_g*2*np.pi/360) ])
        
        self.obs_pos = np.array([[self.obs_rad[3]*np.cos(180*2*np.pi/360), self.obs_rad[3]*np.sin(180*2*np.pi/360)],
                        [self.obs_rad[3]*np.cos(270*2*np.pi/360), self.obs_rad[3]*np.sin(270*2*np.pi/360)],
                        [self.obs_rad[3]*np.cos(0*2*np.pi/360), self.obs_rad[3]*np.sin(0*2*np.pi/360)],
                        [self.obs_rad[3]*np.cos(90*2*np.pi/360), self.obs_rad[3]*np.sin(90*2*np.pi/360)],
                        ############################
                        [self.obs_rad[2]*np.cos(72*2*np.pi/360), self.obs_rad[2]*np.sin(72*2*np.pi/360)],
                        [self.obs_rad[2]*np.cos(144*2*np.pi/360), self.obs_rad[2]*np.sin(144*2*np.pi/360)],
                        [self.obs_rad[2]*np.cos(216*2*np.pi/360), self.obs_rad[2]*np.sin(216*2*np.pi/360)],
                        [self.obs_rad[2]*np.cos(288*2*np.pi/360), self.obs_rad[2]*np.sin(288*2*np.pi/360)],
                        [self.obs_rad[2]*np.cos(0*2*np.pi/360), self.obs_rad[2]*np.sin(0*2*np.pi/360)],
                        ############################
                        [self.obs_rad[1]*np.cos(60*2*np.pi/360), self.obs_rad[1]*np.sin(60*2*np.pi/360)],
                        [self.obs_rad[1]*np.cos(120*2*np.pi/360), self.obs_rad[1]*np.sin(120*2*np.pi/360)],
                        [self.obs_rad[1]*np.cos(180*2*np.pi/360), self.obs_rad[1]*np.sin(180*2*np.pi/360)],
                        [self.obs_rad[1]*np.cos(240*2*np.pi/360), self.obs_rad[1]*np.sin(240*2*np.pi/360)],
                        [self.obs_rad[1]*np.cos(300*2*np.pi/360), self.obs_rad[1]*np.sin(300*2*np.pi/360)],
                        [self.obs_rad[1]*np.cos(360*2*np.pi/360), self.obs_rad[1]*np.sin(360*2*np.pi/360)],
                        ############################
                        [self.obs_rad[0]*np.cos(51.5*2*np.pi/360), self.obs_rad[0]*np.sin(51.5*2*np.pi/360)],
                        [self.obs_rad[0]*np.cos(51.5*2*2*np.pi/360), self.obs_rad[0]*np.sin(51.5*2*2*np.pi/360)],
                        [self.obs_rad[0]*np.cos(51.5*3*2*np.pi/360), self.obs_rad[0]*np.sin(51.5*3*2*np.pi/360)],
                        [self.obs_rad[0]*np.cos(51.5*4*2*np.pi/360), self.obs_rad[0]*np.sin(51.5*4*2*np.pi/360)],
                        [self.obs_rad[0]*np.cos(51.5*5*2*np.pi/360), self.obs_rad[0]*np.sin(51.5*5*2*np.pi/360)],
                        [self.obs_rad[0]*np.cos(51.5*6*2*np.pi/360), self.obs_rad[0]*np.sin(51.5*6*2*np.pi/360)],
                        [self.obs_rad[0]*np.cos(51.5*7*2*np.pi/360), self.obs_rad[0]*np.sin(51.5*7*2*np.pi/360)]
                        ])
        
        self.theta_obs = np.array([(270)*2*np.pi/360,
                        (0)*2*np.pi/360, 
                        (90)*2*np.pi/360, 
                        (180)*2*np.pi/360,
                        ################
                        (72+90)*2*np.pi/360,
                        (144+90)*2*np.pi/360,
                        (216+90)*2*np.pi/360,
                        (288+90)*2*np.pi/360,
                        (0+90)*2*np.pi/360,
                        ################
                        (60+90)*2*np.pi/360,
                        (120+90)*2*np.pi/360,
                        (180+90)*2*np.pi/360,
                        (240+90)*2*np.pi/360,
                        (300+90)*2*np.pi/360,
                        (360+90)*2*np.pi/360,
                        ################
                        (51.5+90)*2*np.pi/360,
                        (51.5*2+90)*2*np.pi/360,
                        (51.5*3+90)*2*np.pi/360,
                        (51.5*4+90)*2*np.pi/360,
                        (51.5*5+90)*2*np.pi/360,
                        (51.5*6+90)*2*np.pi/360,
                        (51.5*7+90)*2*np.pi/360
                        ])
        self.rads = np.array([self.obs_rad[3], self.obs_rad[3], self.obs_rad[3], self.obs_rad[3],
                            self.obs_rad[2], self.obs_rad[2], self.obs_rad[2], self.obs_rad[2], self.obs_rad[2],
                            self.obs_rad[1], self.obs_rad[1], self.obs_rad[1], self.obs_rad[1], self.obs_rad[1], self.obs_rad[1],
                            self.obs_rad[0], self.obs_rad[0], self.obs_rad[0], self.obs_rad[0], self.obs_rad[0], self.obs_rad[0],self.obs_rad[0]])
        self.obs_vel = np.array([[18.9,0],
                                [19.0,0],
                                [19.1,0],
                                [19.2,0],
                                ############
                                [15.0,0],
                                [15.1,0],
                                [15.2,0],
                                [15.3,0],
                                [15.4,0],
                                ############
                                [11.0,0],
                                [11.1,0],
                                [11.2,0],
                                [11.3,0],
                                [11.4,0],
                                [11.5,0],
                                ############
                                [7.0,0],
                                [7.1,0],
                                [7.2,0],
                                [7.3,0],
                                [7.4,0],
                                [7.5,0],
                                [7.6,0]
                                ])
        
        self.obs_vel[:,1] = self.obs_vel[:,0]/self.rads

        self.dist = np.sqrt((self.obs_pos[:,0] - self.x)**2 + (self.obs_pos[:,1] - self.y)**2)
        #self.centers = np.array([center,center,center,center,center])

        #self.obs_vel = np.array([[0.0,0],[0.0,0],[0.0,0],[0.0,0],[0.0,0]])
        #self.ass_vel = np.array([[float(random.randrange(5,21)),0],[float(random.randrange(5,21)),0],[float(random.randrange(5,21)),0],[float(random.randrange(5,21)),0],[float(random.randrange(5,21)),0]])
        #self.acc = np.array([[1,0],[1,0],[1,0],[1,0],[1,0]])*1.5        


        self.sensor_range = 15.0
        self.val = random.choice(self.obs_rad)
        x_min = np.min(np.hstack((self.obs_pos[:,0],self.agent_p[0],self.goal[0])))
        x_max = np.max(np.hstack((self.obs_pos[:,0],self.agent_p[0],self.goal[0])))
        y_min = np.min(np.hstack((self.obs_pos[:,1],self.agent_p[1],self.goal[1])))
        y_max = np.max(np.hstack((self.obs_pos[:,1],self.agent_p[1],self.goal[1])))
        range_ = np.max(np.array([x_max-x_min, y_max-y_min]))
        self.x_axis_min = (x_min+x_max)/2.-range_/2.-2.
        self.x_axis_max = (x_min+x_max)/2.+range_/2.+2.
        self.y_axis_min = (y_min+y_max)/2.-range_/2.-2.
        self.y_axis_max = (y_min+y_max)/2.+range_/2.+2.

        self.x_lim_min = -5
        self.x_lim_max = 200
        self.req = GetVelocityCmd.Request()
        #while rclpy.ok() or np.linalg.norm(self.agent_p - self.goal) >=1.0:
            #self.plot_client()


    def send_request(self):
        self.req.odom.pose.pose.position.x = self.x
        self.req.odom.pose.pose.position.y = self.y
        self.req.odom.pose.pose.orientation.z = self.theta
        self.req.odom.twist.twist.linear.x = self.agent_v[0]
        self.req.odom.twist.twist.angular.z = self.agent_v[1]

        sorted_obs = self.obs_pos[self.dist.argsort()]
        theta_obs = self.theta_obs[self.dist.argsort()]
        obs_vel = self.obs_vel[self.dist.argsort()]
        odom_arr = OdomArray()
        for i in range(self.num_obs):
            #obs_pos = Pose()
            odom = Odometry()
            odom.pose.pose.position.x = sorted_obs[int(i)][0]
            odom.pose.pose.position.y = sorted_obs[int(i)][1]
            odom.pose.pose.orientation.z = theta_obs[int(i)]
            odom.twist.twist.linear.x = obs_vel[int(i)][0]
            odom.twist.twist.angular.z = obs_vel[int(i)][1]
            odom_arr.odom.append(odom)
        self.req.odom_arr = odom_arr
        goal_pose = Pose()

        theta = np.arctan2(self.y-0, self.x-0)*180/np.pi
        if theta<0:
            theta = 360+theta
        
        if random.random() < 0.4:
            print("Lane Change")
            self.val = random.choice(self.obs_rad)
        self.theta_g = theta+45.0
        self.goal[0] = self.val*np.cos(self.theta_g*2*np.pi/360)
        self.goal[1] = self.val*np.sin(self.theta_g*2*np.pi/360)



        goal_pose.position.x = self.goal[0]
        goal_pose.position.y = self.goal[1]
        goal_pose.position.z = self.val
        print(self.val, self.goal)
        self.req.goal = goal_pose
        self.future = self.cli.call_async(self.req)

    def plot_path(self, twist, path):
        plt.ion()
        plt.show()
        plt.clf()
        self.path_x = []
        self.path_y = []
        for i in path.poses:
            self.path_x.append(i.position.x)
            self.path_y.append(i.position.y)
        self.agent_v = [twist.linear.x, twist.angular.z]
        self.theta = self.theta +self.agent_v[1]*self.dt
        print(self.agent_v)
        #if self.theta>=2*np.pi:
            #self.theta = self.theta - 2*np.pi
        #print(self.theta, 2*np.pi)
        self.x = self.x + self.agent_v[0]*np.cos(self.theta)*self.dt
        self.y = self.y + self.agent_v[0]*np.sin(self.theta)*self.dt
        #self.x = self.path_x[1]
        #self.y = self.path_y[1]
        self.agent_p = np.array([self.x, self.y])

        angle = 180*np.arctan2(self.obs_pos[:,1]-self.center[1], self.obs_pos[:,0]-self.center[0])/np.pi
        for i in range(len(angle)):
            if angle[i]<0:
                angle[i] = 360+angle[i]

        self.theta_obs = self.theta_obs + self.obs_vel[:,1]*0.1
        self.obs_pos[:,1] = self.obs_pos[:,1] + self.obs_vel[:,0]*np.sin(self.theta_obs)*0.1
        self.obs_pos[:,0] = self.obs_pos[:,0] + self.obs_vel[:,0]*np.cos(self.theta_obs)*0.1

        self.bot_path = np.append(self.bot_path, [self.agent_p], axis=0)
        if self.bot_path.shape[0] > 200:
            self.bot_path = self.bot_path[100:,:]
        ego_circle1 = plt.Circle((self.x-2*np.cos(self.theta), self.y-2*np.sin(self.theta)), 1.0, color='b')
        ego_circle2 = plt.Circle((self.x, self.y), 1.0, color='b')
        ego_circle3 = plt.Circle((self.x+2*np.cos(self.theta), self.y+2*np.sin(self.theta)), 1.0, color='b')
        plt.gca().add_patch(ego_circle1)
        plt.gca().add_patch(ego_circle2)
        plt.gca().add_patch(ego_circle3)
        #goal_circle = plt.Circle((self.goal[0], self.goal[1]), 1.0, color='black')
        #plt.gca().add_patch(goal_circle)
        for i in range(self.obs_vel.shape[0]):
            ego_circle1 = plt.Circle((self.obs_pos[i][0]-2*np.cos(self.theta_obs[i]), self.obs_pos[i][1]-2*np.sin(self.theta_obs[i])), 1.0, color='r')
            ego_circle2 = plt.Circle((self.obs_pos[i][0], self.obs_pos[i][1]), 1.0, color='r')
            ego_circle3 = plt.Circle((self.obs_pos[i][0]+2*np.cos(self.theta_obs[i]), self.obs_pos[i][1]+2*np.sin(self.theta_obs[i])), 1.0, color='r')
            plt.gca().add_patch(ego_circle1)
            plt.gca().add_patch(ego_circle2)
            plt.gca().add_patch(ego_circle3)
        
        #plt.plot(self.bot_path[:,0], self.bot_path[:,1])
        plt.plot(self.path_x, self.path_y, 'y')
        lane1 = plt.Circle((0.0, 0.0), 100.0, fill=False, linewidth=2.0)
        lane2 = plt.Circle((0.0, 0.0), 104.0, fill=False, linewidth=2.0)
        lane3 = plt.Circle((0.0, 0.0), 108.0, fill=False, linewidth=2.0)
        lane4 = plt.Circle((0.0, 0.0), 112.0, fill=False, linewidth=2.0)
        lane5 = plt.Circle((0.0, 0.0), 116.0, fill=False, linewidth=2.0)
        self.dist = np.sqrt((self.obs_pos[:,0] - self.x)**2 + (self.obs_pos[:,1] - self.y)**2)
        plt.gca().add_patch(lane1)
        plt.gca().add_patch(lane2)
        plt.gca().add_patch(lane3)
        plt.gca().add_patch(lane4)
        plt.gca().add_patch(lane5)
        """theta = self.theta
        if theta>2*np.pi:
            theta = theta-2*np.pi
        if theta>=0 and theta<=np.pi/2:
            plt.ylim(-self.rr-1, 10) 
            plt.xlim(-10, self.rr+1)
        elif theta>=np.pi/2 and theta<=np.pi:
            plt.ylim(-10, self.rr+1) 
            plt.xlim(-10, self.rr+1)
        elif theta>=np.pi and theta<=3*np.pi/2:
            plt.ylim(-10, self.rr+1) 
            plt.xlim(-self.rr-1, 10)
        else:
            plt.ylim(-self.rr-1, 10) 
            plt.xlim(-self.rr-1, 10)"""
        #plt.ylim(-110, 110) 
        #plt.xlim(-110, 110)
        plt.ylim(self.agent_p[1]-50, self.agent_p[1]+50)
        plt.xlim(self.agent_p[0]-50, self.agent_p[0]+50)
        plt.title('Path')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.draw()
        plt.pause(0.0001)
        for i in range(self.obs_vel.shape[0]):
            if ((self.agent_p[0]-self.obs_pos[i][0])**2 + (self.agent_p[1]-self.obs_pos[i][1])**2)**0.5 < 2.001:
                quit()



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