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
from acado_msgs.srv import GetVelocityCmd
from acado_msgs.srv import GetControls
from acado_msgs.msg import OdomArray


class LaneID(enum.Enum):
    BR = "BottomLaneRightBound"
    BL = "BottomLaneLeftBound"
    TR = "TopLaneRightBound"
    TL = "TopLaneLeftBound"
    RCI = "RightCircularInnerLane"
    RCO = "RightCircularOuterLane"
    LCI = "LeftCircularInnerLane"
    LCO = "LeftCircularOuterLane"
    LN = "LeftIntersectionLaneNorthBound"
    LS = "LeftIntersectionLaneSouthBound"
    RN = "RightIntersectionLaneNorthBound"
    RS = "RightIntersectionLaneSouthBound"


class Environment(Node):
    def __init__(self):
        super().__init__('combined_plot')
        self.cli = self.create_client(GetControls, "/get_vel")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
        self.lane_ids = []
        self.radius = 116.0
        self.agent_pose = [102.0, 0.0, np.pi/2]
        self.agent_vel = [15.0, 0.0]
        self.dt = 0.1
        self.acc = 0.5
        self.path_index = -1.0
        self.path_x = []
        self.path_y = []
        
        self.circle_lane_center = [0.0, 0.0]
        self.lane_radii = [100.0, 104.0, 108.0, 112.0]#, 116.0, 120.0, 124.0]
        angles = np.arange(0, 360.1, 0.1)
        self.goal_p = Pose()

        in_lane_obs_ang = np.arange(7.5, 360, 15)
        mid_lane_obs_ang = np.arange(10, 360, 20)
        out_lane_obs_ang = np.arange(12.5, 360, 25)
        inner_obs = []
        mid_obs = []
        outer_obs = []
        self.min_obs = []
        idx = 0
        for i in in_lane_obs_ang:
            vel = [7.8, 7.9, 8.0, 8.1, 8.2]
            inner_obs.append([102.0*np.cos(i*np.pi/180), 102.0*np.sin(i*np.pi/180), i*np.pi/180+np.pi/2, vel[idx], vel[idx]/102.0, 8.0, 102.0])
            idx+=1
            if idx == 5:
                idx = 0
        idx = 0
        self.inner_obs = np.array(inner_obs)
        for i in mid_lane_obs_ang:
            vel = [12.8, 12.9, 13.0, 13.1, 13.2]
            mid_obs.append([106.0*np.cos(i*np.pi/180), 106.0*np.sin(i*np.pi/180), i*np.pi/180+np.pi/2, vel[idx], vel[idx]/106.0, 13.0, 106.0])
            idx+=1
            if idx == 5:
                idx = 0
        idx = 0
        self.mid_obs = np.array(mid_obs)
        for i in out_lane_obs_ang:
            vel = [17.8, 17.9, 18.0, 18.1, 18.2]
            outer_obs.append([110.0*np.cos(i*np.pi/180), 110.0*np.sin(i*np.pi/180), i*np.pi/180+np.pi/2, vel[idx], vel[idx]/110.0, 18.0, 110.0])
            idx+=1
            if idx == 5:
                idx = 0
        self.outer_obs = np.array(outer_obs)

        self.obstacles_list = [self.inner_obs, self.mid_obs, self.outer_obs]

        self.use_IDM = ["True", "True", "True", "True"]
        self.circle_x = []
        self.circle_y = []
        for i in self.lane_radii:
            self.circle_x.append([i*np.cos(ang*2*np.pi/360) for ang in angles])
            self.circle_y.append([i*np.sin(ang*2*np.pi/360) for ang in angles])

        self.req = GetControls.Request()

    def send_request_new(self):
        self.req.odom.pose.pose.position.x = self.agent_pose[0]
        self.req.odom.pose.pose.position.y = self.agent_pose[1]
        self.req.odom.pose.pose.orientation.z = self.agent_pose[2]
        self.req.odom.twist.twist.linear.x = self.agent_vel[0]
        self.req.odom.twist.twist.angular.z = self.agent_vel[1]
        print(self.inner_obs.shape, self.mid_obs.shape, self.outer_obs.shape)
        obs_pos = np.concatenate((self.inner_obs, self.mid_obs, self.outer_obs))
        dist = np.sqrt((obs_pos[:,0] - self.agent_pose[0])**2 + (obs_pos[:,1] - self.agent_pose[1])**2)
        sorted_obs = obs_pos[dist.argsort()]
        obstacles = OdomArray()
        m=0
        for i in range(5):
            odom = Odometry()
            odom.pose.pose.position.x = sorted_obs[i][0]
            odom.pose.pose.position.y = sorted_obs[i][1]
            odom.pose.pose.orientation.z = sorted_obs[i][2]
            odom.twist.twist.linear.x = sorted_obs[i][3]
            odom.twist.twist.angular.z = sorted_obs[i][4]
            obstacles.odom.append(odom)
        self.req.odom_arr = obstacles
        goal_pose = Pose()
        goal_pose.position.x = self.agent_pose[0]+100
        goal_pose.position.y = random.choice([14.0, 10.0, 6.0])
        self.req.goal = goal_pose
        self.future = self.cli.call_async(self.req)
    
    def send_request(self):
        self.req.start.pose.pose.position.x = self.agent_pose[0]
        self.req.start.pose.pose.position.y = self.agent_pose[1]
        self.req.start.pose.pose.orientation.z = self.agent_pose[2]
        self.req.start.twist.twist.linear.x = self.agent_vel[0]
        self.req.start.twist.twist.angular.z = self.agent_vel[1]
        #print(self.inner_obs.shape, self.mid_obs.shape, self.outer_obs.shape)
        obs_pos = np.concatenate((self.inner_obs, self.mid_obs, self.outer_obs))
        dist = np.sqrt((obs_pos[:,0] - self.agent_pose[0])**2 + (obs_pos[:,1] - self.agent_pose[1])**2)
        sorted_obs = obs_pos[dist.argsort()]
        self.min_obs.append(np.min(dist))
        print("Min_dist = ", np.min(self.min_obs))
        obstacles = OdomArray()
        for i in range(10):
            odom = Odometry()
            odom.pose.pose.position.x = sorted_obs[i][0]
            odom.pose.pose.position.y = sorted_obs[i][1]
            odom.pose.pose.orientation.z = sorted_obs[i][2]
            odom.twist.twist.linear.x = sorted_obs[i][3]
            odom.twist.twist.angular.z = sorted_obs[i][3]/sorted_obs[i][6]
            #obs = plt.Circle((sorted_obs[i][0], sorted_obs[i][1]), 1.2, color='g')
            #plt.gca().add_patch(obs)
            obstacles.odom.append(odom)
        self.req.obstacles = obstacles
        #print(obstacles)
        goal = Pose()
        lane_cons = PoseArray()
        #if len(self.path_x) == 0:
        lane_cons, goal = self.get_lane_cons()
        #print("Last Point = ", self.path_x[-1], self.path_y[-1])
        #lane_cons, goal = self.br_cons(0.0)
        self.req.goal = goal
        self.req.lane_cons = lane_cons
        #print("Current Lane = ", self.current_laneid)
        #print("Next Lane = ", self.next_laneid)
        print("Goal = ", goal)
        print("Lane Cons == ", self.req.start.pose.pose.orientation.z)
        self.goal_p = goal
        self.future = self.cli.call_async(self.req)
    
    def rci_cons(self, index):
        lane_info = PoseArray()
        goal_pose = Pose()
        #lanes = [102.0, 106.0, 110.0]
        lanes = [106.0]

        theta = np.arctan2(self.agent_pose[1], self.agent_pose[0])*2*180/(2*np.pi)
        if theta<0:
            theta = 360+theta
        if theta>359:
            quit()
        val = random.choice(lanes)
        theta_g = theta+50.0
        goal_pose.position.x = val*np.cos(theta_g*2*np.pi/360)
        goal_pose.position.y = val*np.sin(theta_g*2*np.pi/360)
        t = theta_g*np.pi/180 + np.pi/2
        if self.agent_pose[2]>3*np.pi/2 and t<3*np.pi/2:
            t = 2*np.pi+t
        #if t>2*np.pi:
        #    t = t - 2*np.pi
        goal_pose.orientation.z = t

        info = Pose()
        info.position.x = float(index)
        info.position.y = 1.0
        lane_info.poses.append(info)

        info = Pose()
        info.position.x = 1.0
        info.position.y = 1.0
        info.position.z = val
        lane_info.poses.append(info)

        info = Pose()   #max rad cons
        info.position.x = 1.0  #self.agent_pose[0]
        info.position.y = 0.0 #26.0 + 1e12
        info.position.z = 0.0 #1e12
        info.orientation.x = 1.0
        info.orientation.y = 0.0
        info.orientation.z = 0.0
        info.orientation.w = -101.0**2
        lane_info.poses.append(info)

        info = Pose()   #min rad cons
        info.position.x = 1.0  #self.agent_pose[0]
        info.position.y = 0.0 #26.0 + 1e12
        info.position.z = 0.0 #1e12
        info.orientation.x = 1.0
        info.orientation.y = 0.0
        info.orientation.z = 0.0**2
        info.orientation.w = -111.0**2
        lane_info.poses.append(info)

        return lane_info, goal_pose


    def get_lane_cons(self):
        return self.rci_cons(0)

    def calc_path_index(self, pose, axis):
        for i in range(len(self.path_x)):
            if axis=='x' and self.path_x[i]>pose[0]:
                self.path_index = i
            elif axis=='y' and self.path_y[i]>pose[0]:
                a=4
                


    def update_agent(self, twist):
        self.agent_vel[0] = twist.linear.x
        self.agent_vel[1] = twist.angular.z
        self.agent_pose[2] = self.agent_pose[2] + self.agent_vel[1]*self.dt
        self.agent_pose[0] = self.agent_pose[0] + self.agent_vel[0]*np.cos(self.agent_pose[2])*self.dt
        self.agent_pose[1] = self.agent_pose[1] + self.agent_vel[0]*np.sin(self.agent_pose[2])*self.dt
    


    def update_obstacles(self):
        for i in range(self.inner_obs.shape[0]):
            if self.inner_obs[i][3]<=self.inner_obs[i][5]:
                self.inner_obs[i][3] = self.inner_obs[i][3] + self.acc*self.dt
            else:
                self.inner_obs[i][3] = self.inner_obs[i][3] - self.acc*self.dt
                
            self.inner_obs[i][2] = self.inner_obs[i][2] + (self.inner_obs[i][3]/102.0)*self.dt
            self.inner_obs[i][0] = self.inner_obs[i][0] + \
                                            self.inner_obs[i][3]*np.cos(self.inner_obs[i][2])*self.dt
            self.inner_obs[i][1] = self.inner_obs[i][1] + \
                                            self.inner_obs[i][3]*np.sin(self.inner_obs[i][2])*self.dt
        ##################################
        for i in range(self.mid_obs.shape[0]):
            if self.mid_obs[i][3]<=self.mid_obs[i][5]:
                self.mid_obs[i][3] = self.mid_obs[i][3] + self.acc*self.dt
            else:
                self.mid_obs[i][3] = self.mid_obs[i][3] - self.acc*self.dt

            self.mid_obs[i][2] = self.mid_obs[i][2] + (self.mid_obs[i][3]/106.0)*self.dt
            self.mid_obs[i][0] = self.mid_obs[i][0] + \
                                            self.mid_obs[i][3]*np.cos(self.mid_obs[i][2])*self.dt
            self.mid_obs[i][1] = self.mid_obs[i][1] + \
                                            self.mid_obs[i][3]*np.sin(self.mid_obs[i][2])*self.dt
        #################################
        for i in range(self.outer_obs.shape[0]):
            if self.outer_obs[i][3]<=self.outer_obs[i][5]:
                self.outer_obs[i][3] = self.outer_obs[i][3] + self.acc*self.dt
            else:
                self.outer_obs[i][3] = self.outer_obs[i][3] - self.acc*self.dt

            self.outer_obs[i][2] = self.outer_obs[i][2] + (self.outer_obs[i][3]/110.0)*self.dt
            self.outer_obs[i][0] = self.outer_obs[i][0] + \
                                            self.outer_obs[i][3]*np.cos(self.outer_obs[i][2])*self.dt
            self.outer_obs[i][1] = self.outer_obs[i][1] + \
                                            self.outer_obs[i][3]*np.sin(self.outer_obs[i][2])*self.dt



    def plot_obstacles(self):
        for i in self.inner_obs:
            obs = plt.Circle((i[0], i[1]), 1.0, color='r')
            plt.gca().add_patch(obs)
        for i in self.mid_obs:
            obs = plt.Circle((i[0], i[1]), 1.0, color='r')
            plt.gca().add_patch(obs)
        for i in self.outer_obs:
            obs = plt.Circle((i[0], i[1]), 1.0, color='r')
            plt.gca().add_patch(obs)
        agent = plt.Circle((self.agent_pose[0], self.agent_pose[1]), 1.0, color='g')
        plt.text(self.agent_pose[0], self.agent_pose[1]+55, 'Vel = %s'%(round(self.agent_vel[0],2)), fontsize=10)
        plt.gca().add_patch(agent)


    def plot_lanes(self):
        #   Plot curved lanes
        for i in range(len(self.lane_radii)):
            lw = 1.0
            if i==0 or i==3 or i==6:
                lw = 2.0
            plt.plot(self.circle_x[i], self.circle_y[i], color='black', linewidth=lw)
        
        ########################################################################################

    
    def plot(self, twist, path):
        plt.ion()
        plt.show()
        plt.clf()
        self.path_x = []
        self.path_y = []
        for i in path.poses:
            self.path_x.append(i.position.x)
            self.path_y.append(i.position.y)
        plt.plot(self.path_x, self.path_y, 'y')
        self.update_agent(twist)
        self.update_obstacles()
        self.plot_lanes()
        self.plot_obstacles()
        obs = plt.Circle((self.goal_p.position.x, self.goal_p.position.y), 1.0, color='b')
        plt.gca().add_patch(obs)
        #plt.ylim(self.agent_pose[1]-50, self.agent_pose[1]+50)
        #plt.xlim(self.agent_pose[0]-50, self.agent_pose[0]+150)
        plt.ylim(-50+self.agent_pose[1], 50+self.agent_pose[1])
        plt.xlim(-50+self.agent_pose[0], 50+self.agent_pose[0])
        #plt.ylim(-50, 250)
        #plt.xlim(-350, 350)
        plt.draw()
        plt.pause(0.0001)
        #plt.pause(100)
        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Environment()

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