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
        #self.obs_pos = np.array([[-1.0,-5.0], [2.0,5.0], [100,100]])
        #self.obs_vel = np.array([[0.0,1.0], [0.0,0.2], [0,0]])
        #self.obs_pos = np.array([[-1.5,3.0*3], [0.0,12.0*3], [1.5,17.0*3]])*2
        #self.obs_vel = np.array([[0.0,0.5], [0.0,-0.4], [0.0,-0.5]])*6
        #self.obs_pos = np.array([[3.0*3,1.5], [12.0*3,0.2], [17.0*3,-1.5]])*2
        #self.obs_vel = np.array([[0.5,0.0], [-0.4,0.0], [-0.5,0.0]])*6
        #self.obs_pos = np.array([[5.0*3,-0.1],[11*3,1.5],[13*3,1.33],[13.5*3,3.7],[15*3,3.5],[ 13*3,0.3],[7*3,-1.8],[10*3,-1.7]])*2
        #self.obs_vel = np.array([[0.3,0],[-0.43,0],[-0.23,0],[-0.313,0],[-0.253,0],[0.3,0],[0.27,0],[0.42,0]])*6
        self.obs_pos = np.array([[20,-6],[25,-2],[30,-6],[-100,-2.0],[-150,-6.0]])#
        self.obs_vel = np.array([[9.0,0],[11.0,0],[13.0,0],[19.0,0],[17.0,0]])#
        #self.obs_pos = np.array([[0.1,5.0*3],[-1.5,11*3],[-1.33,13*3],[-3.7,13.5*3],[-3.5,15],[-0.3,13*3],[1.8,7*3],[1.7,10*3]])*2
        #self.obs_vel = np.array([[0.0,0.3],[0.0,-0.43],[0.0,-0.23],[0.0,-0.313],[0.0,-0.253],[0.0,0.3],[0.0,0.27],[0.0,0.42]])*6
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
            if self.goal[0]%500 == 0:
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
        #plt.xlim(self.agent_p[0]-40, self.agent_p[0]+40)
        plt.xlim(self.agent_p[0]-80, self.agent_p[0]+80) 
        #plt.xlim(-10, 1100) 
        plt.title('Path')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.text(self.agent_p[0], -12.5, "Velocity = "+str(self.agent_v[0]))
        plt.text(self.agent_p[0], -15, "Goal = ("+str(self.goal[0])+", "+str(self.goal[1])+")")
        plt.draw()
        plt.pause(0.0001)
        for i in range(self.obs_vel.shape[0]):
            if ((self.agent_p[0]-self.obs_pos[i][0])**2 + (self.agent_p[1]-self.obs_pos[i][1])**2)**0.5 < 2.001:
                quit()
            if self.obs_vel[i][0] < self.max_vel:
                if self.obs_pos[i][0] < self.agent_p[0] - 80:
                    self.obs_pos[i][0] = self.agent_p[0] + 81
                    self.obs_pos[i][1] = -8 - self.obs_pos[i][1]
            else:
                if self.obs_pos[i][0] > self.agent_p[0] + 80:
                    self.obs_pos[i][0] = self.agent_p[0] - 81
                    self.obs_pos[i][1] = -8 - self.obs_pos[i][1]



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