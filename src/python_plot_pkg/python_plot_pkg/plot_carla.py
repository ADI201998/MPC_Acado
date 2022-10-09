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
        super().__init__('plot_carla')
        self.cli = self.create_client(GetVelocityCmd, "/get_vel")
        #while not self.cli.wait_for_service(timeout_sec=1.0):
            #self.get_logger().info('service not available, waiting again...')
        self.path_x = []
        self.path_y = []
        self.odom_msg = Odometry()
        self.got_odom = False
        self.dt = 0.1
        self.cmd_vel = Twist()
        self.x = -230.0
        self.y = -95.0
        self.agent_p = np.array([self.x, self.y])
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.max_vel = 15.0
        self.bot_path = np.array([self.agent_p])
        self.agent_v = [self.v, self.w]
        self.goal = np.array([-130, -95.0 ])
        #self.obs_pos = np.array([[20,-6],[25,-2],[30,-6],[-100,-2.0],[-150,-6.0]])#
        #self.obs_vel = np.array([[9.0,0],[11.0,0],[13.0,0],[19.0,0],[17.0,0]])#
        self.sensor_range = 15.0
        self.subs = self.create_subscription(Odometry, "/carla/ego_vehicle/odometry" , self.cbs, 1)
        self.obs_pose = np.array([[-190,-95],[-175,-91],[-161,-95],[-148,-91.0],[-136,-95.0]])
        self.got_obs_pose = [False] * 5
        self.obs_vel = np.array([[0.0,0],[0.0,0],[0.0,0],[0.0,0],[0.0,0]])
        self.index = 0
        for i in range(5):
            #self.cbs[i] = self.make_cb_functions(i)
            #change Odometry to something else for real robots
            self.create_subscription(Odometry, "/carla/hero"+str(i)+"/odometry" , self.hero_cb, 1)
            #self.publisher_[i] = self.create_publisher(Twist, "/hero"+str(i)+"/cmd_vel", 1)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.req = GetVelocityCmd.Request()
        self.subs
        #while rclpy.ok() or np.linalg.norm(self.agent_p - self.goal) >=1.0:
            #self.plot_client()

    def hero_cb(self, msg):
        """
        callback to receive ego-vehicle info
        """
        #self.loginfo("Actual Vel ={}".format(msg.twist.twist.linear.x))
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        self.obs_pose[int(msg.child_frame_id[4])] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.obs_vel[int(msg.child_frame_id[4])] = msg.twist.twist.linear.x
        if not self.got_obs_pose[int(msg.child_frame_id[4])]:
            self.got_obs_pose[int(msg.child_frame_id[4])] = True
            self.index +=1
        if self.index == 6:
            self.future = self.cli.call_async(self.req)

    def cbs(self, msg):
        """
        callback to receive ego-vehicle info
        """
        #self.loginfo("Actual Vel ={}".format(msg.twist.twist.linear.x))
        self.odom_msg = msg
        #print(msg)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        #self.v = self.odom_msg.twist.twist.linear.x
        #self.w = self.odom_msg.twist.twist.angular.z
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.theta = np.arctan2(siny_cosp, cosy_cosp)
        if not self.got_odom:
            self.got_odom = True
            self.index+=1
        if self.index == 6:
            self.future = self.cli.call_async(self.req)


    def send_request(self):
        """while True:
            print(self.got_odom)
            if self.got_odom:
                break"""
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
        odom_arr = OdomArray()
        for i in range(5):
            #obs_pos = Pose()
            odom = Odometry()
            odom.pose.pose.position.x = self.obs_pose[i][0]
            odom.pose.pose.position.y = self.obs_pose[i][1]
            odom.twist.twist.linear.x = self.obs_vel[i][0]
            odom.twist.twist.linear.y = 0.0
            odom_arr.odom.append(odom)
        self.req.odom_arr = odom_arr
        goal_pose = Pose()
        if ((self.agent_p[0]-self.goal[0])**2 + (self.agent_p[1]-self.goal[1])**2)**0.5 < 80:
            if self.goal[0]<40:
                self.goal[0] = self.goal[0]+100
            else:
                self.goal[0] = 40.0
            if self.goal[0] > -30:
                self.goal[1] = -91

        goal_pose.position.x = self.goal[0]
        goal_pose.position.y = self.goal[1]
        self.req.goal = goal_pose
        

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
        #self.theta = self.theta +self.agent_v[1]*self.dt
        #self.x = self.x + self.agent_v[0]*np.cos(self.theta)*self.dt
        #self.y = self.y + self.agent_v[0]*np.sin(self.theta)*self.dt
        #self.x = self.path_x[1]
        #self.y = self.path_y[1]
        
        #self.x = self.odom_msg.pose.pose.position.x
        #self.y = self.odom_msg.pose.pose.position.y
        #self.theta = self.odom_msg.pose.pose.orientation.z
        self.agent_p = np.array([self.x, self.y])
        #self.obs_pos = self.obs_pos + np.dot(self.obs_vel, self.dt)
        self.bot_path = np.append(self.bot_path, [self.agent_p], axis=0)
        bot_circle = plt.Circle((self.x, self.y), 1.0, color='b')
        plt.gca().add_patch(bot_circle)
        goal_circle = plt.Circle((self.goal[0], self.goal[1]), 1.0, color='black')
        plt.gca().add_patch(goal_circle)
        for i in range(self.obs_vel.shape[0]):
            obs_circle = plt.Circle((self.obs_pose[i][0], self.obs_pose[i][1]), 1.0, color='r')
            #plt.arrow(self.obs_pos[i][0], self.obs_pos[i][1], 0.5*self.obs_vel[i][0]*np.cos(0), 0,length_includes_head=True, head_width=0.3, head_length=0.2)
            plt.gca().add_patch(obs_circle)
        #plt.arrow(self.x, self.y, 0.5*self.agent_v[0]*np.cos(self.theta), 0.5*self.agent_v[0]*np.sin(self.theta),length_includes_head=True, head_width=0.3, head_length=0.2)
        vel_pub = Twist()
        vel_pub = twist
        #if ((self.x - self.goal[0])**2 + (self.y - self.goal[1])**2)**0.5 >70:
            #vel_pub.linear.x = max(3.0, vel_pub.linear.x)
        self.pub.publish(vel_pub)
        plt.plot(self.bot_path[:,0], self.bot_path[:,1])
        plt.plot(self.path_x, self.path_y, 'y')
        plt.plot([-2000000,2000000], [-89,-89], 'black')
        plt.plot([-2000000,2000000], [-97,-97], 'black')
        plt.plot([-2000000,2000000], [-93,-93], linestyle='--', color='black')
        plt.ylim(-130, -60) 
        #plt.xlim(self.agent_p[0]-40, self.agent_p[0]+40)
        plt.xlim(self.agent_p[0]-80, self.agent_p[0]+80) 
        #plt.xlim(-10, 1100) 
        plt.title('Path')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.text(self.agent_p[0], -75.5, "Velocity = "+str(self.agent_v[0]))
        plt.text(self.agent_p[0], -70, "Goal = ("+str(self.goal[0])+", "+str(self.goal[1])+")")
        plt.draw()
        plt.pause(0.0001)
        self.got_obs_pose = [False] * 5
        self.index = 0
        self.got_odom = False
        """for i in range(self.obs_vel.shape[0]):
            if ((self.agent_p[0]-self.obs_pos[i][0])**2 + (self.agent_p[1]-self.obs_pos[i][1])**2)**0.5 < 2.001:
                quit()
            if self.obs_vel[i][0] < self.max_vel:
                if self.obs_pos[i][0] < self.agent_p[0] - 80:
                    self.obs_pos[i][0] = self.agent_p[0] + 81
                    self.obs_pos[i][1] = -8 - self.obs_pos[i][1]
            else:
                if self.obs_pos[i][0] > self.agent_p[0] + 80:
                    self.obs_pos[i][0] = self.agent_p[0] - 81
                    self.obs_pos[i][1] = -8 - self.obs_pos[i][1]"""



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = PlotPath()

    #rclpy.spin(minimal_subscriber)


    while rclpy.ok():
        minimal_subscriber.send_request()
        while rclpy.ok():
            rclpy.spin_once(minimal_subscriber)
            if minimal_subscriber.index < 6:
                continue
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