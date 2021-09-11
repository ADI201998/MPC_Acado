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
        self.y = -106.0
        self.agent_p = np.array([self.x, self.y])
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.max_vel = 15.0
        self.bot_path = np.array([self.agent_p])
        self.agent_v = [self.v, self.w]

        center = [0,0]
        self.rr = 108.0
        self.lr = 100.0
        self.mr = 104.0
        self.rm = 106.25
        self.lm = 101.75
        self.theta_g = 300.0
        self.cur_rad = self.rm
        self.index = 1
        self.goal = np.array([self.rm*np.cos(self.theta_g*2*np.pi/360), self.rm*np.sin(self.theta_g*2*np.pi/360) ])
        
        self.obs_pos = np.array([[self.rm*np.cos(180*2*np.pi/360), self.rm*np.sin(180*2*np.pi/360)],
                        [self.lm*np.cos(180*2*np.pi/360), self.lm*np.sin(180*2*np.pi/360)],
                        [self.rm*np.cos(280*2*np.pi/360), self.rm*np.sin(280*2*np.pi/360)],
                        [self.lm*np.cos(290*2*np.pi/360), self.lm*np.sin(290*2*np.pi/360)],
                        [self.rm*np.cos(300*2*np.pi/360), self.rm*np.sin(300*2*np.pi/360)]])
        
        self.theta_obs = np.array([(270)*2*np.pi/360, (270)*2*np.pi/360, (10)*2*np.pi/360, (20)*2*np.pi/360, (30)*2*np.pi/360])
        self.rads = np.array([self.rm, self.lm, self.rm, self.lm, self.rm])
        self.obs_vel = np.array([[17.0,0],[19.0,0],[9.0,0],[11.0,0],[13.0,0]])
        self.obs_vel[:,1] = self.obs_vel[:,0]/self.rads
        self.centers = np.array([center,center,center,center,center])

        #self.obs_vel = np.array([[0.0,0],[0.0,0],[0.0,0],[0.0,0],[0.0,0]])
        #self.ass_vel = np.array([[float(random.randrange(5,21)),0],[float(random.randrange(5,21)),0],[float(random.randrange(5,21)),0],[float(random.randrange(5,21)),0],[float(random.randrange(5,21)),0]])
        #self.acc = np.array([[1,0],[1,0],[1,0],[1,0],[1,0]])*1.5
        self.rx_points = []
        self.ry_points = []
        self.lx_points = []
        self.ly_points = []
        self.mx_points = []
        self.my_points = []
        self.angle_res = 0.1
        start_ang = 270
        for i in range(0, 361, 1):
            self.rx_points.append(self.rr*np.cos(i*2*np.pi/360))
            self.ry_points.append(self.rr*np.sin(i*2*np.pi/360) )
            self.lx_points.append(self.lr*np.cos(i*2*np.pi/360))
            self.ly_points.append(self.lr*np.sin(i*2*np.pi/360) )
            self.mx_points.append(self.mr*np.cos(i*2*np.pi/360))
            self.my_points.append(self.mr*np.sin(i*2*np.pi/360) )            


        self.sensor_range = 15.0
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
        """obs_arr = PoseArray()
        for i in range(self.obs_vel.shape[0]):
            obs_pos = Pose()
            obs_pos.position.x = self.obs_pos[i][0]
            obs_pos.position.y = self.obs_pos[i][1]
            obs_arr.poses.append(obs_pos)
        print(obs_arr)
        self.req.obs_poses = obs_arr"""
        obs_arr = PoseArray()
        detected = np.empty(0)
        infront_condition = 0.
        current_rel_p = self.obs_pos - self.agent_p
        current_rel_v = self.obs_vel - self.agent_v
        not_infront = []
        odom_arr = OdomArray()
        for i in range(self.obs_vel.shape[0]):
            #obs_pos = Pose()
            odom = Odometry()
            odom.pose.pose.position.x = self.obs_pos[int(i)][0]
            odom.pose.pose.position.y = self.obs_pos[int(i)][1]
            odom.pose.pose.orientation.z = self.theta_obs[int(i)]
            odom.twist.twist.linear.x = self.obs_vel[int(i)][0]
            odom.twist.twist.angular.z = self.obs_vel[int(i)][1]
            odom_arr.odom.append(odom)
        self.req.odom_arr = odom_arr
        goal_pose = Pose()

        theta = np.arctan2(self.y-0, self.x-0)*180/np.pi
        if theta<0:
            theta = 360+theta
        #print(theta, self.theta_g)
        if abs(theta-self.theta_g) < 360/12:
            self.theta_g = self.theta_g + 30.0
            if self.theta_g>360:
                self.theta_g = self.theta_g - 360
            if self.index%3 == 0:
                self.cur_rad = self.rm if self.cur_rad == self.lm else self.lm
            self.goal[0] = self.cur_rad*np.cos(self.theta_g*2*np.pi/360)
            self.goal[1] = self.cur_rad*np.sin(self.theta_g*2*np.pi/360)
            #print(self.theta_g, self.goal)
            self.index = self.index + 1

        goal_pose.position.x = self.goal[0]
        goal_pose.position.y = self.goal[1]
        goal_pose.position.z = self.cur_rad
        print(self.cur_rad)
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
        #if self.theta>=2*np.pi:
            #self.theta = self.theta - 2*np.pi
        print(self.theta, 2*np.pi)
        self.x = self.x + self.agent_v[0]*np.cos(self.theta)*self.dt
        self.y = self.y + self.agent_v[0]*np.sin(self.theta)*self.dt
        #self.x = self.path_x[1]
        #self.y = self.path_y[1]
        self.agent_p = np.array([self.x, self.y])

        angle = 180*np.arctan2(self.obs_pos[:,1]-self.centers[:,1], self.obs_pos[:,0]-self.centers[:,0])/np.pi
        for i in range(len(angle)):
            if angle[i]<0:
                angle[i] = 360+angle[i]

        self.theta_obs = self.theta_obs + self.obs_vel[:,1]*0.1
        self.obs_pos[:,1] = self.obs_pos[:,1] + self.obs_vel[:,0]*np.sin(self.theta_obs)*0.1
        self.obs_pos[:,0] = self.obs_pos[:,0] + self.obs_vel[:,0]*np.cos(self.theta_obs)*0.1

        self.bot_path = np.append(self.bot_path, [self.agent_p], axis=0)
        if self.bot_path.shape[0] > 200:
            self.bot_path = self.bot_path[100:,:]
        bot_circle = plt.Circle((self.x, self.y), 1.0, color='b')
        plt.gca().add_patch(bot_circle)
        goal_circle = plt.Circle((self.goal[0], self.goal[1]), 1.0, color='black')
        plt.gca().add_patch(goal_circle)
        for i in range(self.obs_vel.shape[0]):
            obs_circle = plt.Circle((self.obs_pos[i][0], self.obs_pos[i][1]), 1.0, color='r')
            #plt.arrow(self.obs_pos[i][0], self.obs_pos[i][1], 0.5*self.obs_vel[i][0]*np.cos(0), 0,length_includes_head=True, head_width=0.3, head_length=0.2)
            plt.gca().add_patch(obs_circle)
        #plt.arrow(self.x, self.y, 0.5*self.agent_v[0]*np.cos(self.theta), 0.5*self.agent_v[0]*np.sin(self.theta),length_includes_head=True, head_width=0.3, head_length=0.2)
        plt.plot(self.bot_path[:,0], self.bot_path[:,1])
        plt.plot(self.path_x, self.path_y, 'y')
        plt.plot(self.rx_points, self.ry_points, 'black')
        plt.plot(self.lx_points, self.ly_points, 'black')
        plt.plot(self.mx_points, self.my_points, linestyle='--', color='black')
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
        plt.ylim(self.agent_p[1]-60, self.agent_p[1]+60)
        plt.xlim(self.agent_p[0]-60, self.agent_p[0]+60)
        plt.title('Path')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        #plt.text(self.agent_p[0], -12.5, "Velocity = "+str(self.agent_v[0]))
        #plt.text(self.agent_p[0], -15, "Goal = ("+str(self.goal[0])+", "+str(self.goal[1])+")")
        #plt.text(self.agent_p[0], 22, "Obs1 speed = ("+str(self.obs_vel[0][0])+")")
        #plt.text(self.agent_p[0], 19, "Obs2 speed = ("+str(self.obs_vel[1][0])+")")
        #plt.text(self.agent_p[0], 16, "Obs3 speed = ("+str(self.obs_vel[2][0])+")")
        #plt.text(self.agent_p[0], 13, "Obs4 speed = ("+str(self.obs_vel[3][0])+")")
        #plt.text(self.agent_p[0], 10, "Obs5 speed = ("+str(self.obs_vel[4][0])+")")
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