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
        """while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')"""
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
        
        self.obs_pos = np.array([[0,-2],[30,-6],[60,-2],[90,-6.0],[120,-2.0]])
        self.obs_vel = np.array([[0.0,0],[0.0,0],[0.0,0],[0.0,0],[0.0,0]])
        self.ass_vel = np.array([[float(random.randrange(5,21)),0],[float(random.randrange(5,21)),0],[float(random.randrange(5,21)),0],[float(random.randrange(5,21)),0],[float(random.randrange(5,21)),0]])
        self.acc = np.array([[1,0],[1,0],[1,0],[1,0],[1,0]])*1.5
        self.rx_points = []
        self.ry_points = []
        self.lx_points = []
        self.ly_points = []
        self.mx_points = []
        self.my_points = []
        self.rx2_points = []
        self.ry2_points = []
        self.lx2_points = []
        self.ly2_points = []
        self.mx2_points = []
        self.my2_points = []
        self.angle_res = 0.1
        self.rr = 58.0
        self.lr = 50.0
        self.mr = 54.0
        self.c_c1 = [0, 58]
        self.c_c2 = [0, 166]
        start_ang = 270
        for i in range(270, 451, 1):
            self.rx_points.append(self.rr*np.cos(i*2*np.pi/360) + self.c_c1[0])
            self.ry_points.append(self.rr*np.sin(i*2*np.pi/360) + self.c_c1[1])
            self.lx_points.append(self.lr*np.cos(i*2*np.pi/360) + self.c_c1[0])
            self.ly_points.append(self.lr*np.sin(i*2*np.pi/360) + self.c_c1[1])
            self.mx_points.append(self.mr*np.cos(i*2*np.pi/360) + self.c_c1[0])
            self.my_points.append(self.mr*np.sin(i*2*np.pi/360) + self.c_c1[1])
        for i in range(270, 90, -1):
            self.rx2_points.append(self.lr*np.cos(i*2*np.pi/360) + self.c_c2[0])
            self.ry2_points.append(self.lr*np.sin(i*2*np.pi/360) + self.c_c2[1])
            self.lx2_points.append(self.rr*np.cos(i*2*np.pi/360) + self.c_c2[0])
            self.ly2_points.append(self.rr*np.sin(i*2*np.pi/360) + self.c_c2[1])
            self.mx2_points.append(self.mr*np.cos(i*2*np.pi/360) + self.c_c2[0])
            self.my2_points.append(self.mr*np.sin(i*2*np.pi/360) + self.c_c2[1])

            


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
            odom.twist.twist.linear.x = self.obs_vel[int(i)][0]
            odom.twist.twist.linear.y = self.obs_vel[int(i)][1]
            odom_arr.odom.append(odom)
        self.req.odom_arr = odom_arr
        goal_pose = Pose()
        if ((self.agent_p[0]-self.goal[0])**2 + (self.agent_p[1]-self.goal[1])**2)**0.5 < 80:
            self.goal[0] = self.goal[0]+100
            if self.goal[0]%400 == 0:
                self.goal[1] = -8 - self.goal[1]

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
        for i in path.poses:
            self.path_x.append(i.position.x)
            self.path_y.append(i.position.y)
        self.agent_v = [twist.linear.x, twist.angular.z]
        self.theta = self.theta +self.agent_v[1]*self.dt
        self.x = self.x + self.agent_v[0]*np.cos(self.theta)*self.dt
        self.y = self.y + self.agent_v[0]*np.sin(self.theta)*self.dt
        #self.x = self.path_x[1]
        #self.y = self.path_y[1]
        self.agent_p = np.array([self.x, self.y])

        self.obs_vel = self.obs_vel + np.dot(self.acc, self.dt)
        self.obs_pos = self.obs_pos + np.dot(self.obs_vel, self.dt)
        self.bot_path = np.append(self.bot_path, [self.agent_p], axis=0)
        bot_circle = plt.Circle((self.x, self.y), 1.0, color='b')
        plt.gca().add_patch(bot_circle)
        goal_circle = plt.Circle((self.goal[0], self.goal[1]), 1.0, color='black')
        plt.gca().add_patch(goal_circle)
        for i in range(self.obs_vel.shape[0]):
            obs_circle = plt.Circle((self.obs_pos[i][0], self.obs_pos[i][1]), 1.0, color='r')
            plt.arrow(self.obs_pos[i][0], self.obs_pos[i][1], 0.5*self.obs_vel[i][0]*np.cos(0), 0,length_includes_head=True, head_width=0.3, head_length=0.2)
            plt.gca().add_patch(obs_circle)
        #plt.arrow(self.x, self.y, 0.5*self.agent_v[0]*np.cos(self.theta), 0.5*self.agent_v[0]*np.sin(self.theta),length_includes_head=True, head_width=0.3, head_length=0.2)
        plt.plot(self.bot_path[:,0], self.bot_path[:,1])
        plt.plot(self.path_x, self.path_y, 'y')
        plt.plot([-2000000,2000000], [0,0], 'black')
        plt.plot([-2000000,2000000], [-8,-8], 'black')
        plt.plot([-2000000,2000000], [-4,-4], linestyle='--', color='black')
        plt.ylim(-30, 30) 
        if self.x_lim_max < self.agent_p[0]+50:
            self.x_lim_min = self.agent_p[0]-20
            self.x_lim_max = self.agent_p[0] + 200
        plt.xlim(self.x_lim_min, self.x_lim_max)
        #plt.xlim(self.agent_p[0]-80, self.agent_p[0]+80) 
        #plt.xlim(-10, 1100) 
        plt.title('Path')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.text(self.agent_p[0], -12.5, "Velocity = "+str(self.agent_v[0]))
        plt.text(self.agent_p[0], -15, "Goal = ("+str(self.goal[0])+", "+str(self.goal[1])+")")
        plt.text(self.agent_p[0], 22, "Obs1 speed = ("+str(self.obs_vel[0][0])+")")
        plt.text(self.agent_p[0], 19, "Obs2 speed = ("+str(self.obs_vel[1][0])+")")
        plt.text(self.agent_p[0], 16, "Obs3 speed = ("+str(self.obs_vel[2][0])+")")
        plt.text(self.agent_p[0], 13, "Obs4 speed = ("+str(self.obs_vel[3][0])+")")
        plt.text(self.agent_p[0], 10, "Obs5 speed = ("+str(self.obs_vel[4][0])+")")
        plt.draw()
        plt.pause(0.0001)
        for i in range(self.obs_vel.shape[0]):
            if ((self.agent_p[0]-self.obs_pos[i][0])**2 + (self.agent_p[1]-self.obs_pos[i][1])**2)**0.5 < 2.001:
                quit()
            if abs(self.ass_vel[i][0] - self.obs_vel[i][0]) < 0.2:
                self.ass_vel[i][0] = float(random.randrange(5,21))
        for i in range(self.obs_vel.shape[0]):
            print(self.ass_vel[i][0] - self.obs_vel[i][0])
            if self.ass_vel[i][0] - self.obs_vel[i][0] < 0:
                self.acc[i][0] = -1.5#self.acc[i][0]*-1
            else:
                self.acc[i][0] = 1.5#abs(self.acc[i][0])
        print(self.obs_vel, self.ass_vel, self.acc)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = PlotPath()

    #rclpy.spin(minimal_subscriber)
    
    x = 0
    y = 6
    theta = 0
    lin = 10
    omega = 0
    center = minimal_subscriber.c_c1
    rad = 52
    sign_ = 1
    obs_pos = np.array([[0.0,6.0],[4.8807215938688415,2.2130969068622477],[9.029705238680359,6.789996843365174],[14.493866525741183,3.908153727812177],[17.785047452934744,9.135983719132753]])
    """plt.plot(minimal_subscriber.rx_points, minimal_subscriber.ry_points, 'black')
    plt.plot(minimal_subscriber.lx_points, minimal_subscriber.ly_points, 'black')
    plt.plot(minimal_subscriber.mx_points, minimal_subscriber.my_points, linestyle='--', color='black')
    plt.plot(minimal_subscriber.rx2_points, minimal_subscriber.ry2_points, 'black')
    plt.plot(minimal_subscriber.lx2_points, minimal_subscriber.ly2_points, 'black')
    plt.plot(minimal_subscriber.mx2_points, minimal_subscriber.my2_points, linestyle='--', color='black')
    for i in range(len(obs_pos)):
        bot_circle = plt.Circle((obs_pos[i][0], obs_pos[i][1]), 1.0, color='b')
        plt.gca().add_patch(bot_circle)
    plt.show()
    quit()"""
    theta = np.array([0, (275-270)*2*np.pi/360, (280-270)*2*np.pi/360, (285-270)*2*np.pi/360, (290-270)*2*np.pi/360])
    rads = np.array([52.0, 56.0, 52.0, 56.0, 52.0])
    obs_vel = np.array([[10.0,0],[12.0,0],[8.0,0],[14.0,0],[6.0,0]])
    signs = [1,1,1,1,1]
    centers = np.array([center,center,center,center,center])
    while True:
        plt.ion()
        plt.show()
        plt.clf()
        #angle = 180*np.arctan2(y-center[1], x-center[0])/np.pi
        angle = 180*np.arctan2(obs_pos[:,1]-centers[:,1], obs_pos[:,0]-centers[:,0])/np.pi
        for i in range(len(angle)):
            if angle[i]<0:
                angle[i] = 360+angle[i]
        print(angle)
        for i in range(len(angle)):
            if angle[i]>89.9 and angle[i] <91.5:
                centers[i] = minimal_subscriber.c_c2
                rads[i] = 52 if rads[i] == 56 else 56
                signs[i] = -1
        omegas = signs*obs_vel[:,0]/rads
        print(omegas)
        theta = theta + omegas*0.1
        obs_pos[:,1] = obs_pos[:,1] + obs_vel[:,0]*np.sin(theta)*0.1
        obs_pos[:,0] = obs_pos[:,0] + obs_vel[:,0]*np.cos(theta)*0.1
        #obs_pos[:,0] = x
        #obs_pos[:,1] = y
        #print(lin, omega)
        #print(x,y,theta)
        plt.plot(minimal_subscriber.rx_points, minimal_subscriber.ry_points, 'black')
        plt.plot(minimal_subscriber.lx_points, minimal_subscriber.ly_points, 'black')
        plt.plot(minimal_subscriber.mx_points, minimal_subscriber.my_points, linestyle='--', color='black')
        plt.plot(minimal_subscriber.rx2_points, minimal_subscriber.ry2_points, 'black')
        plt.plot(minimal_subscriber.lx2_points, minimal_subscriber.ly2_points, 'black')
        plt.plot(minimal_subscriber.mx2_points, minimal_subscriber.my2_points, linestyle='--', color='black')
        for i in range(len(obs_pos)):
            bot_circle = plt.Circle((obs_pos[i][0], obs_pos[i][1]), 1.0, color='b')
            plt.gca().add_patch(bot_circle)
        plt.draw()
        plt.pause(0.00001)


    return


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