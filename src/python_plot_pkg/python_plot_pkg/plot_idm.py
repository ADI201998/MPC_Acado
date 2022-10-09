
from math import radians
import random
import numpy as np
from time import time
import matplotlib.pyplot as plt

from matplotlib.patches import Ellipse, Rectangle
from shapely.geometry import Point
from shapely.affinity import scale, rotate

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from acado_msgs.srv import GetVelocityCmd
from acado_msgs.msg import OdomArray


class PlotPath(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.cli = self.create_client(GetVelocityCmd, "/get_vel")
        self.plot_ellipse = 0

        random.seed(0)
        self.a_ell = 5.6
        self.b_ell = 3.0
        self.a_rect = 4.0
        self.b_rect = 1.4

        self.cnt = 1
        self.loop = 0
        self.index = 0
        self.intersection = [False, False, False, False, False, False]
        self.upper = 140 #130 for cruisie
        self.lower_lim = -30
        self.upper_lim = self.upper
        self.pre_x = np.array([])
        self.pre_y = np.array([])
        self.Gotit = 1.0
        self.v_controls = np.array([])
        self.psi_constrols = np.array([])
        self.num_goal = 1
        self.val = 100.0

        total_obs = 18
        self.num_obs = 6
        self.obs = np.zeros([self.num_obs+1, 4])
        self.obs[0] = [0, 10, 14.0, 0.0]#[+0, 10, 15.0, 0.0]

        self.lane_y = [-10.0, -6.0, -2.0, 2.0, 6.0, 10.0]
        self.goal = np.array([100.0, random.choice(self.lane_y) ])
        self.other_vehicles = np.zeros([total_obs, 5])       # x y vx vy dist
        self.other_vehicles[:,1] = (np.hstack((self.lane_y,self.lane_y,self.lane_y)))

        self.other_vehicles[:,0] = np.array([-10, 25,   60, -20, 40, 35,
                                              70, 85,   100, 10, 80, 170,
                                              130, 110, 160, 140, 135, 95 
        ])
                                       

        self.other_vehicles[:,4] = np.sqrt((self.other_vehicles[:,0] - self.obs[0][0])**2 + (self.other_vehicles[:,1] - self.obs[0][1])**2) + (((-self.other_vehicles[:,0] + self.obs[0][0] - 2.5)/(abs(-self.other_vehicles[:,0] + self.obs[0][0] - 2.5)+0.0001)) + 1) * 10000
        
        self.other_vehicles = self.other_vehicles[self.other_vehicles[:, 4].argsort()]

        
        self.other_vehicles_desired = [10, 11, 8.0, 9.5, 8.5, 12, 
                                        7.5, 8.5, 7.0, 9.0, 8.5, 9.0,
                                        10.0, 9.5, 7.2, 9.5, 10, 10.5]
        self.other_vehicles[:,2] = np.roll(self.other_vehicles_desired, 6)
        print(self.other_vehicles[:,2])
        for i in range(self.num_obs):
            self.obs[i+1] = self.other_vehicles[i,:4] 
        
        self.v = self.obs[0][2]
        self.w = 0.0 

        self.prev_psi = 0.0
        self.psi = 0.0
        self.dt = 0.08
    
        self.sim_time = np.array([])

        self.flag = 1
        self.fig = plt.figure(0)
        self.ax = self.fig.add_subplot(111, aspect='equal')
        mng = plt.get_current_fig_manager()
        mng.full_screen_toggle()
        self.fig.set_size_inches(20, 10)
        self.req = GetVelocityCmd.Request()
        print("STARTING SIMULATION")


    def send_request(self):
        self.req.odom.pose.pose.position.x = self.obs[0][0]
        self.req.odom.pose.pose.position.y = self.obs[0][1]
        self.req.odom.pose.pose.orientation.z = self.psi
        self.req.odom.twist.twist.linear.x = self.v
        self.req.odom.twist.twist.angular.z = self.w

        odom_arr = OdomArray()
        for i in range(self.num_obs):
            #obs_pos = Pose()
            odom = Odometry()
            odom.pose.pose.position.x = self.obs[i+1][0]
            odom.pose.pose.position.y = self.obs[i+1][1]
            odom.twist.twist.linear.x = self.obs[i+1][2]
            odom.twist.twist.linear.y = self.obs[i+1][3]
            odom_arr.odom.append(odom)
        self.req.odom_arr = odom_arr
        goal_pose = Pose()
        
        self.goal[0] = self.obs[0][0] + 120
        if self.obs[0][0] > self.val:
            self.goal[1] = random.choice(self.lane_y)
            self.val = self.val + 100.0

        goal_pose.position.x = self.goal[0]
        goal_pose.position.y = self.goal[1]
        self.Gotit = 1
        self.req.goal = goal_pose
        self.future = self.cli.call_async(self.req)
    
    
    def create_ellipse(self, center, axes, inclination):
        p = Point(*center)
        c = p.buffer(1)
        ellipse = scale(c, *axes)
        ellipse = rotate(ellipse, inclination)
        return ellipse

    def checkCollision(self):
        
        obs_ellipse = [self.create_ellipse((self.other_vehicles[i][0], self.other_vehicles[i][1]), (self.a_ell/2, self.b_ell/2), 0) for i in range(self.num_obs)]
        ego_ellipse = self.create_ellipse((self.obs[0][0], self.obs[0][1]), (self.a_ell/2, self.b_ell/2), 0*self.psi*180.0/np.pi)

        self.intersection = [ego_ellipse.intersects(obs_ellipse[i]) for i in range(self.num_obs)]
        for i in range(self.num_obs):
            if self.intersection[i]:
                ptsx, ptsy = ego_ellipse.intersection(obs_ellipse[i]).exterior.coords.xy
                if len(ptsx) < 10:
                    self.intersection[i] = False
        self.intersection = any([inter == True for inter in self.intersection])


    def IDM(self):

        so = 1
        T = 1

        l = self.a_ell

        a = 4
        b = 3
        
        for i in range(len(self.other_vehicles)):
            nearest = -1
            x = self.other_vehicles[i][0]
            y = self.other_vehicles[i][1]
            inLane = self.other_vehicles[:,1] - y * np.ones(len(self.other_vehicles))
            index = np.where( inLane == 0)

            min = 10000
            for k in range(len(index[0])):
                if index[0][k] != i:
                    if x < self.other_vehicles[index[0][k]][0]:
                        dist = self.other_vehicles[index[0][k]][0] - x
                        if dist < min:
                            min = dist
                            nearest = index[0][k]
             
            v_0 = self.other_vehicles[nearest][2]
            x_0 = self.other_vehicles[nearest][0]
            if (y - self.obs[0][1]) <= self.b_ell and x < self.obs[0][0]:
                if nearest != -1:
                    if self.obs[0][0] < self.other_vehicles[nearest][0]: 
                        nearest = 1
                        v_0 = self.obs[0][2]
                        x_0 = self.obs[0][0]
                else:
                    v_0 = self.obs[0][2]
                    x_0 = self.obs[0][0]
            v_1 = self.other_vehicles[i][2]
            x_1 = x

            v_r = self.other_vehicles_desired[i]
            delta_v = v_1 - v_0
            s_alpha = x_0 - x_1 - l

            if nearest != -1:
                s_star = so + v_1 * T + (v_1 * delta_v)/(2 * np.sqrt(a*b))
                decc = a * (1 - (v_1/v_r)**4 - (s_star/s_alpha)**2)
                if decc < -a:
                    decc = -a
                self.other_vehicles[i][2] += decc * self.dt
            else:
                decc = a * (1 - (v_1/v_r)**4)
                if decc > a:
                    decc = a
                self.other_vehicles[i][2] += decc * self.dt

     
#
    def plot_path(self, twist, path):
        self.v = twist.linear.x
        self.w = twist.angular.z
        self.v_controls = np.append(self.v_controls, self.v)
        if self.obs[0][0] < 1000:
            
            if self.Gotit or self.flag:
                t1 = time()
                dt = self.dt
                
                if self.flag == 0:
                    self.loop += 1
                    self.sim_time = np.append(self.sim_time, self.loop * dt)
                    
                    plt.clf()                        
                    self.ax = self.fig.add_subplot(111, aspect='equal')

                    # gray_road = Rectangle((self.lower_lim, -12), width = 200, height = 24, angle=0)
                    # self.ax.add_artist(gray_road)
                    # gray_road.set_facecolor([0.502, 0.502, 0.502])
                    self.path_x = []
                    self.path_y = []
                    for i in path.poses:
                        self.path_x.append(i.position.x)
                        self.path_y.append(i.position.y)
                    plt.plot(self.path_x, self.path_y, 'y')
                        
                        
                    diag = np.sqrt(self.a_rect ** 2 + self.b_rect ** 2)
                    ells = [Ellipse(xy=[self.other_vehicles[i][0], self.other_vehicles[i][1]], width=self.a_ell, height=self.b_ell, angle=0) for i in range(len(self.other_vehicles))]
                    rect = [Rectangle(xy=[self.other_vehicles[i][0] -  self.a_rect/2, self.other_vehicles[i][1] - self.b_rect/2], width=4.0, height=1.4, angle=0) for i in range(len(self.other_vehicles))]
                    mm = 0
                    for e, r in zip(ells, rect):
                        self.ax.add_artist(e)
                        self.ax.add_artist(r)
                        e.set_clip_box(self.ax.bbox)
                        e.set_alpha(1)
                        r.set_alpha(0.5)
                        r.set_facecolor([1, 1, 1])
                        e.set_facecolor([0.0, 0.5, 1])
                        if self.other_vehicles[mm][0] < self.upper_lim - 2.5 and self.other_vehicles[mm][0] > self.lower_lim + 2.5:
                            plt.text(self.other_vehicles[mm][0]-2, self.other_vehicles[mm][1]-0.3, '%s'%(round(self.other_vehicles[mm][2],2)), fontsize=10)
                        mm+=1
                    rob = [Ellipse(xy=[self.obs[0][0], self.obs[0][1]], width=self.a_ell, height=self.b_ell, angle=0*self.psi*180.0/np.pi)]
                    rect = [Rectangle(xy=[self.obs[0][0] - diag/2 * np.cos(self.psi + np.arctan(self.b_rect/self.a_rect)), self.obs[0][1] - diag/2 * np.sin(self.psi + np.arctan(self.b_rect/self.a_rect))], width=4.0, height=1.4, angle=self.psi*180.0/np.pi)]
                    for e, r in zip(rob, rect):
                        self.ax.add_artist(e)
                        self.ax.add_artist(r)
                        e.set_clip_box(self.ax.bbox)
                        e.set_alpha(1)
                        r.set_alpha(0.5)
                        r.set_facecolor([1, 1, 1])
                        e.set_facecolor([1, 0.5, 0.5])
                        plt.text(self.obs[0][0]-2, self.obs[0][1]-0.3, '%s'%(round(self.obs[0][2],2)), fontsize=10)
                    
                    

                    plt.xlabel('Y in m')
                    plt.ylabel('X in m')
                    self.lower_lim = -30 + self.obs[0][0]
                    self.upper_lim = self.upper + self.obs[0][0]
                    
                    
                    if self.flag == 0:
                        plt.text(self.lower_lim+10, 14, 'Collision with obstacle= %s'%((self.intersection)), fontsize=10) 
                        plt.text(self.lower_lim+50, 14, 'Average speed= %s m/s'%(round(self.v_controls.mean(), 3)), fontsize=10)
                        plt.text(self.lower_lim+90, 14, 'Orientation= %s degrees'%(round(self.psi*180/np.pi, 3)), fontsize=10)
                        plt.text(self.lower_lim+130, 14, 'Goal = %s'%(self.goal), fontsize=10)
                        
                    plt.text((self.lower_lim+self.upper_lim)/2 - 15, 20, 'Highway environment', fontsize=14)
                    plt.plot([self.lower_lim, self.upper_lim], [-4, -4], color='black',linestyle='--', alpha=0.2)
                    plt.plot([self.lower_lim, self.upper_lim], [0, 0], color='black',linestyle='--', alpha=0.2)
                    plt.plot([self.lower_lim, self.upper_lim], [4, 4], color='black',linestyle='--', alpha=0.2)
                    plt.plot([self.lower_lim, self.upper_lim], [-8, -8], color='black',linestyle='--', alpha=0.2)
                    plt.plot([self.lower_lim, self.upper_lim], [8, 8], color='black',linestyle='--', alpha=0.2)
                    plt.plot([self.lower_lim, self.upper_lim], [12, 12], color='black',linestyle='-', alpha=0.2)
                    plt.plot([self.lower_lim, self.upper_lim], [-12, -12], color='black',linestyle='-', alpha=0.2)
                    plt.xlim(self.lower_lim, self.upper_lim)
                    plt.ylim(-12, 12)
                    # plt.title('highway env')

                    plt.tight_layout()
                    plt.draw()
                    plt.pause(0.000000000000000001)
                
            
                if self.Gotit:  #ego vehicle update
                    self.psi += self.w * dt
                    self.obs[0][2] = self.v * np.cos(self.psi)    #vx
                    self.obs[0][3] = self.v * np.sin(self.psi)    #vy
                    
                    self.obs[0][0] += self.obs[0][2] * dt    #x
                    self.obs[0][1] += self.obs[0][3] * dt    #y
                
                if self.flag == 0:
                    self.IDM()
                
                self.other_vehicles[:,0] += self.other_vehicles[:,2] * dt    #x
                self.other_vehicles[:,1] += self.other_vehicles[:,3] * dt    #y
                
                self.other_vehicles[:,4] = np.sqrt((self.other_vehicles[:,0] - self.obs[0][0])**2 + (self.other_vehicles[:,1] - self.obs[0][1])**2) + (((-self.other_vehicles[:,0] + self.obs[0][0]-2.5)/(abs(-self.other_vehicles[:,0] + self.obs[0][0]-2.5)+0.0001)) + 1) * 10000
                self.other_vehicles = self.other_vehicles[self.other_vehicles[:, 4].argsort()]

                self.obs[1:] = self.other_vehicles[:len(self.obs)-1,:4]
                self.checkCollision()

                self.Gotit = 0
                self.flag = 0
                
                for i in range(len(self.other_vehicles)):
                    if self.other_vehicles[i][0] < self.lower_lim-6:                        
                        self.other_vehicles[i][0] = self.upper_lim + 5 + self.other_vehicles[i][2]
                        # self.other_vehicles_desired[i] = random.uniform(5,16)
                        self.other_vehicles[i][2] -= 2 * (i%2)

                        
                    # if self.other_vehicles[i][0] > self.upper_lim+6:
                    #     self.other_vehicles[i][0] = self.lower_lim - 5
                    #     self.other_vehicles_desired[i] = random.uniform(10,14)
                    #     self.other_vehicles[i][2] = random.uniform(5, 12)

                # if self.obs[0][0] >= 100 * self.cnt:
                #     self.cnt += 1
                #     self.other_vehicles_desired = np.roll(self.other_vehicles_desired, 2)

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



