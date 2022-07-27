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
		self.agent_pose = [0.0, 14.0, 0.0]
		self.agent_vel = [15.0, 0.0]
		self.dt = 0.1
		self.acc = 0.5
		self.path_index = -1.0
		self.path_x = []
		self.path_y = []
		self.lane_radius = 102.0
		self.current_laneid = LaneID.BR
		self.next_laneid = LaneID.BR
		self.laneid_fsm = {LaneID.BR: [LaneID.RN, LaneID.RCI], LaneID.BL: [LaneID.LN, LaneID.RCO], 
							LaneID.TR: [LaneID.RS, LaneID.RCO], LaneID.TL: [LaneID.LS, LaneID.LCO], 
							LaneID.RCI: [LaneID.TL, LaneID.RS], LaneID.RCO: [LaneID.BL, LaneID.RN], 
							LaneID.LCI: [LaneID.BR, LaneID.LN], LaneID.LCO: [LaneID.LS, LaneID.TR], 
							LaneID.LN: [LaneID.LCI, LaneID.TR], LaneID.LS: [LaneID.BR, LaneID.LCO], 
							LaneID.RN: [LaneID.TL, LaneID.RCO], LaneID.RS: [LaneID.BL, LaneID.RCI]}
		self.lane_cons_func = {LaneID.BR: self.br_cons, LaneID.BL: self.bl_cons, LaneID.TR: self.tr_cons, 
								LaneID.TL: self.tl_cons, LaneID.RCI: self.rci_cons, LaneID.RCO: self.rco_cons, 
								LaneID.LCI: self.lci_cons, LaneID.LCO: self.lco_cons, LaneID.LS: self.ls_cons, 
								LaneID.LN: self.ln_cons, LaneID.RS: self.rs_cons, LaneID.RN: self.rn_cons}
		self.right_circle_lane_center = [232.0, self.radius]
		self.left_circle_lane_center = [-232.0, self.radius]
		self.lane_radii = [100.0, 104.0, 108.0, 112.0, 116.0, 120.0, 124.0]
		angles = np.arange(0, 360.1, 0.1)
		angles_left = angles[900:2701]
		angles_right = np.concatenate([angles[2700:3600], angles[0:901]])
		self.goal_p = Pose()
		self.min_obs = []

		self.obstacles_inner_lane = np.array([[-200.0, 14.0, 0.0, 7.8, 0.0, 8.0, 102],
												[-160.0, 14.0, 0.0, 8.3, 0.0, 8.0, 102],
												[-115.0, 14.0, 0.0, 8.2, 0.0, 8.0, 102],
												[-65.0, 14.0, 0.0, 8.1, 0.0, 8.0, 102],
												[-20.0, 14.0, 0.0, 8.3, 0.0, 8.0, 102],
												[25.0, 14.0, 0.0, 7.8, 0.0, 8.0, 102],
												[65.0, 14.0, 0.0, 8.3, 0.0, 8.0, 102],
												[110.0, 14.0, 0.0, 7.7, 0.0, 8.0, 102],
												[155.0, 14.0, 0.0, 7.9, 0.0, 8.0, 102],
												[200.0, 14.0, 0.0, 7.8, 0.0, 8.0, 102],
												[-200.0, 218.0, 180*2*np.pi/360, 7.8, 0.0, 8.0, 102],
												[-160.0, 218.0, 180*2*np.pi/360, 8.3, 0.0, 8.0, 102],
												[-115.0, 218.0, 180*2*np.pi/360, 8.2, 0.0, 8.0, 102],
												[-65.0, 218.0, 180*2*np.pi/360, 8.1, 0.0, 8.0, 102],
												[-20.0, 218.0, 180*2*np.pi/360, 8.2, 0.0, 8.0, 102],
												[25.0, 218.0, 180*2*np.pi/360, 8.3, 0.0, 8.0, 102],
												[65.0, 218.0, 180*2*np.pi/360, 9.8, 0.0, 8.0, 102],
												[110.0, 218.0, 180*2*np.pi/360, 7.9, 0.0, 8.0, 102],
												[155.0, 218.0, 180*2*np.pi/360, 7.8, 0.0, 8.0, 102],
												[200.0, 218.0, 180*2*np.pi/360, 7.7, 0.0, 8.0, 102],
												[102*np.cos(293*2*np.pi/360)+224, 102*np.sin(293*2*np.pi/360)+116, 293*2*np.pi/360 + np.pi/2, 8.0, 8.0/102.0, 8.0, 102],
												[102*np.cos(319*2*np.pi/360)+224, 102*np.sin(319*2*np.pi/360)+116, 319*2*np.pi/360 + np.pi/2, 8.0, 8.0/102.0, 8.0, 102],
												[102*np.cos(347*2*np.pi/360)+224, 102*np.sin(347*2*np.pi/360)+116, 347*2*np.pi/360 + np.pi/2, 8.0, 8.0/102.0, 8.0, 102],
												[102*np.cos(10*2*np.pi/360)+224, 102*np.sin(10*2*np.pi/360)+116, 10*2*np.pi/360 + np.pi/2, 8.0, 8.0/102, 8.0, 102],
												[102*np.cos(38*2*np.pi/360)+224, 102*np.sin(38*2*np.pi/360)+116, 38*2*np.pi/360 + np.pi/2, 8.0, 8.0/102, 8.0, 102],
												[102*np.cos(66*2*np.pi/360)+224, 102*np.sin(66*2*np.pi/360)+116, 66*2*np.pi/360 + np.pi/2, 8.0, 8.0/102, 8.0, 102],
												[102*np.cos(113*2*np.pi/360)-224, 102*np.sin(113*2*np.pi/360)+116, 113*2*np.pi/360 + np.pi/2, 8.0, 8.0/102, 8.0, 102],
												[102*np.cos(139*2*np.pi/360)-224, 102*np.sin(139*2*np.pi/360)+116, 139*2*np.pi/360 + np.pi/2, 8.0, 8.0/102, 8.0, 102],
												[102*np.cos(167*2*np.pi/360)-224, 102*np.sin(167*2*np.pi/360)+116, 167*2*np.pi/360 + np.pi/2, 8.0, 8.0/102, 8.0, 102],
												[102*np.cos(190*2*np.pi/360)-224, 102*np.sin(190*2*np.pi/360)+116, 190*2*np.pi/360 + np.pi/2, 8.0, 8.0/102, 8.0, 102],
												[102*np.cos(218*2*np.pi/360)-224, 102*np.sin(218*2*np.pi/360)+116, 218*2*np.pi/360 + np.pi/2, 8.0, 8.0/102, 8.0, 102],
												[102*np.cos(246*2*np.pi/360)-224, 102*np.sin(246*2*np.pi/360)+116, 246*2*np.pi/360 + np.pi/2, 8.0, 8.0/102, 8.0, 102],
												#######################
												[-200.0, 10.0, 0.0, 12.8, 0.0, 13.0, 106],
												[-140.0, 10.0, 0.0, 13.3, 0.0, 13.0, 106],
												[-75.0, 10.0, 0.0, 13.2, 0.0, 13.0, 106],
												[-15.0, 10.0, 0.0, 13.1, 0.0, 13.0, 106],
												[55.0, 10.0, 0.0, 12.9, 0.0, 13.0, 106],
												[115.0, 10.0, 0.0, 13.2, 0.0, 13.0, 106],
												[185.0, 10.0, 0.0, 12.7, 0.0, 13.0, 106],
												[-200.0, 222.0, 180*2*np.pi/360, 12.8, 0.0, 13.0, 106],
												[-140.0, 222.0, 180*2*np.pi/360, 13.3, 0.0, 13.0, 106],
												[-75.0, 222.0, 180*2*np.pi/360, 13.1, 0.0, 13.0, 106],
												[-15.0, 222.0, 180*2*np.pi/360, 12.7, 0.0, 13.0, 106],
												[55.0, 222.0, 180*2*np.pi/360, 12.9, 0.0, 13.0, 106],
												[115.0, 222.0, 180*2*np.pi/360, 12.8, 0.0, 13.0, 106],
												[185.0, 222.0, 180*2*np.pi/360, 13.2, 0.0, 13.0, 106],
												[106*np.cos(290*2*np.pi/360)+224, 106*np.sin(290*2*np.pi/360)+116, 290*2*np.pi/360 + np.pi/2, 13.0, 13.0/106, 13.0, 106],
												[106*np.cos(325*2*np.pi/360)+224, 106*np.sin(325*2*np.pi/360)+116, 325*2*np.pi/360 + np.pi/2, 13.0, 13.0/106, 13.0, 106],
												[106*np.cos(360*2*np.pi/360)+224, 106*np.sin(360*2*np.pi/360)+116, 360*2*np.pi/360 + np.pi/2, 13.0, 13.0/106, 13.0, 106],
												[106*np.cos(35*2*np.pi/360)+224, 106*np.sin(35*2*np.pi/360)+116, 35*2*np.pi/360 + np.pi/2, 13.0, 13.0/106, 13.0, 106],
												[106*np.cos(70*2*np.pi/360)+224, 106*np.sin(70*2*np.pi/360)+116, 70*2*np.pi/360 + np.pi/2, 13.0, 13.0/106, 13.0, 106],
												[106*np.cos(110*2*np.pi/360)-224, 106*np.sin(110*2*np.pi/360)+116, 110*2*np.pi/360 + np.pi/2, 13.0, 13.0/106, 13.0, 106],
												[106*np.cos(145*2*np.pi/360)-224, 106*np.sin(145*2*np.pi/360)+116, 145*2*np.pi/360 + np.pi/2, 13.0, 13.0/106, 13.0, 106],
												[106*np.cos(180*2*np.pi/360)-224, 106*np.sin(180*2*np.pi/360)+116, 180*2*np.pi/360 + np.pi/2, 13.0, 13.0/106, 13.0, 106],
												[106*np.cos(215*2*np.pi/360)-224, 106*np.sin(215*2*np.pi/360)+116, 215*2*np.pi/360 + np.pi/2, 13.0, 13.0/106, 13.0, 106],
												[106*np.cos(250*2*np.pi/360)-224, 106*np.sin(250*2*np.pi/360)+116, 250*2*np.pi/360 + np.pi/2, 13.0, 13.0/106, 13.0, 106],
												#######################
												[-200.0, 6.0, 0.0, 17.8, 0.0, 18.0, 110],
												[-120.0, 6.0, 0.0, 18.3, 0.0, 18.0, 110],
												[-35.0, 6.0, 0.0, 18.2, 0.0, 18.0, 110],
												[65.0, 6.0, 0.0, 18.1, 0.0, 18.0, 110],
												[145.0, 6.0, 0.0, 17.9, 0.0, 18.0, 110],
												[-200.0, 226.0, 180*2*np.pi/360, 17.8, 0.0, 18.0, 110],
												[-120.0, 226.0, 180*2*np.pi/360, 18.3, 0.0, 18.0, 110],
												[-35.0, 226.0, 180*2*np.pi/360, 18.1, 0.0, 18.0, 110],
												[65.0, 226.0, 180*2*np.pi/360, 17.9, 0.0, 18.0, 110],
												[145.0, 226.0, 180*2*np.pi/360, 17.7, 0.0, 18.0, 110],
												[110*np.cos(315*2*np.pi/360)+224, 110*np.sin(315*2*np.pi/360)+116, 315*2*np.pi/360 + np.pi/2, 18.0, 18.0/110, 18.0, 110],
												[110*np.cos(360*2*np.pi/360)+224, 110*np.sin(360*2*np.pi/360)+116, 360*2*np.pi/360 + np.pi/2, 18.0, 18.0/110, 18.0, 110],
												[110*np.cos(45*2*np.pi/360)+224, 110*np.sin(45*2*np.pi/360)+116, 45*2*np.pi/360 + np.pi/2, 18.0, 18.0/110, 18.0, 110],
												[110*np.cos(135*2*np.pi/360)-224, 110*np.sin(135*2*np.pi/360)+116, 135*2*np.pi/360 + np.pi/2, 18.0, 18.0/110, 18.0, 110],
												[110*np.cos(180*2*np.pi/360)-224, 110*np.sin(180*2*np.pi/360)+116, 180*2*np.pi/360 + np.pi/2, 18.0, 18.0/110, 18.0, 110],
												[110*np.cos(225*2*np.pi/360)-224, 110*np.sin(225*2*np.pi/360)+116, 225*2*np.pi/360 + np.pi/2, 18.0, 18.0/110, 18.0, 110],
												])


		self.obstacles_outer_lane = np.array([[-200.0, 238.0, 0.0, 7.8, 0.0, 8.0, 122],
												[-160.0, 238.0, 0.0, 8.3, 0.0, 8.0, 122],
												[-115.0, 238.0, 0.0, 8.2, 0.0, 8.0, 122],
												[-65.0, 238.0, 0.0, 8.1, 0.0, 8.0, 122],
												[-20.0, 238.0, 0.0, 8.3, 0.0, 8.0, 122],
												[25.0, 238.0, 0.0, 7.8, 0.0, 8.0, 122],
												[65.0, 238.0, 0.0, 8.3, 0.0, 8.0, 122],
												[110.0, 238.0, 0.0, 7.7, 0.0, 8.0, 122],
												[155.0, 238.0, 0.0, 7.9, 0.0, 8.0, 122],
												[200.0, 238.0, 0.0, 7.8, 0.0, 8.0, 122],
												[-200.0, -6.0, 180*2*np.pi/360, 7.8, 0.0, 8.0, 122],
												[-160.0, -6.0, 180*2*np.pi/360, 8.3, 0.0, 8.0, 122],
												[-115.0, -6.0, 180*2*np.pi/360, 8.2, 0.0, 8.0, 122],
												[-65.0, -6.0, 180*2*np.pi/360, 8.1, 0.0, 8.0, 122],
												[-20.0, -6.0, 180*2*np.pi/360, 8.2, 0.0, 8.0, 122],
												[25.0, -6.0, 180*2*np.pi/360, 8.3, 0.0, 8.0, 122],
												[65.0, -6.0, 180*2*np.pi/360, 9.8, 0.0, 8.0, 122],
												[110.0, -6.0, 180*2*np.pi/360, 7.9, 0.0, 8.0, 122],
												[155.0, -6.0, 180*2*np.pi/360, 7.8, 0.0, 8.0, 122],
												[200.0, -6.0, 180*2*np.pi/360, 7.7, 0.0, 8.0, 122],
												[122*np.cos(293*2*np.pi/360)+224, 122*np.sin(293*2*np.pi/360)+116, 293*2*np.pi/360 - np.pi/2, 8.0, -8.0/122, 8.0, 122],
												[122*np.cos(319*2*np.pi/360)+224, 122*np.sin(319*2*np.pi/360)+116, 319*2*np.pi/360 - np.pi/2, 8.0, -8.0/122, 8.0, 122],
												[122*np.cos(347*2*np.pi/360)+224, 122*np.sin(347*2*np.pi/360)+116, 347*2*np.pi/360 - np.pi/2, 8.0, -8.0/122, 8.0, 122],
												[122*np.cos(10*2*np.pi/360)+224, 122*np.sin(10*2*np.pi/360)+116, 10*2*np.pi/360 - np.pi/2, 8.0, -8.0/122, 8.0, 122],
												[122*np.cos(38*2*np.pi/360)+224, 122*np.sin(38*2*np.pi/360)+116, 38*2*np.pi/360 - np.pi/2, 8.0, -8.0/122, 8.0, 122],
												[122*np.cos(66*2*np.pi/360)+224, 122*np.sin(66*2*np.pi/360)+116, 66*2*np.pi/360 - np.pi/2, 8.0, -8.0/122, 8.0, 122],\
												[122*np.cos(113*2*np.pi/360)-224, 122*np.sin(113*2*np.pi/360)+116, 113*2*np.pi/360 - np.pi/2, 8.0, -8.0/122, 8.0, 122],
												[122*np.cos(139*2*np.pi/360)-224, 122*np.sin(139*2*np.pi/360)+116, 139*2*np.pi/360 - np.pi/2, 8.0, -8.0/122, 8.0, 122],
												[122*np.cos(167*2*np.pi/360)-224, 122*np.sin(167*2*np.pi/360)+116, 167*2*np.pi/360 - np.pi/2, 8.0, -8.0/122, 8.0, 122],
												[122*np.cos(190*2*np.pi/360)-224, 122*np.sin(190*2*np.pi/360)+116, 190*2*np.pi/360 - np.pi/2, 8.0, -8.0/122, 8.0, 122],
												[122*np.cos(218*2*np.pi/360)-224, 122*np.sin(218*2*np.pi/360)+116, 218*2*np.pi/360 - np.pi/2, 8.0, -8.0/122, 8.0, 122],
												[122*np.cos(246*2*np.pi/360)-224, 122*np.sin(246*2*np.pi/360)+116, 246*2*np.pi/360 - np.pi/2, 8.0, -8.0/122, 8.0, 122],\
												#######################
												[-200.0, 234.0, 0.0, 12.8, 0.0, 13.0, 118],
												[-140.0, 234.0, 0.0, 13.3, 0.0, 13.0, 118],
												[-75.0, 234.0, 0.0, 13.2, 0.0, 13.0, 118],
												[-15.0, 234.0, 0.0, 13.1, 0.0, 13.0, 118],
												[55.0, 234.0, 0.0, 12.9, 0.0, 13.0, 118],
												[115.0, 234.0, 0.0, 13.2, 0.0, 13.0, 118],
												[185.0, 234.0, 0.0, 12.7, 0.0, 13.0, 118],
												[-200.0, -2.0, 180*2*np.pi/360, 12.8, 0.0, 13.0, 118],
												[-140.0, -2.0, 180*2*np.pi/360, 13.3, 0.0, 13.0, 118],
												[-75.0, -2.0, 180*2*np.pi/360, 13.1, 0.0, 13.0, 118],
												[-15.0, -2.0, 180*2*np.pi/360, 12.7, 0.0, 13.0, 118],
												[55.0, -2.0, 180*2*np.pi/360, 12.9, 0.0, 13.0, 118],
												[115.0, -2.0, 180*2*np.pi/360, 12.8, 0.0, 13.0, 118],
												[185.0, -2.0, 180*2*np.pi/360, 13.2, 0.0, 13.0, 118],
												[118*np.cos(290*2*np.pi/360)+224, 118*np.sin(290*2*np.pi/360)+116, 290*2*np.pi/360 - np.pi/2, 13.0, -13.0/118, 13.0, 118],
												[118*np.cos(325*2*np.pi/360)+224, 118*np.sin(325*2*np.pi/360)+116, 325*2*np.pi/360 - np.pi/2, 13.0, -13.0/118, 13.0, 118],
												[118*np.cos(360*2*np.pi/360)+224, 118*np.sin(360*2*np.pi/360)+116, 360*2*np.pi/360 - np.pi/2, 13.0, -13.0/118, 13.0, 118],
												[118*np.cos(35*2*np.pi/360)+224, 118*np.sin(35*2*np.pi/360)+116, 35*2*np.pi/360 - np.pi/2, 13.0, -13.0/118, 13.0, 118],
												[118*np.cos(70*2*np.pi/360)+224, 118*np.sin(70*2*np.pi/360)+116, 70*2*np.pi/360 - np.pi/2, 13.0, -13.0/118, 13.0, 118],
												[118*np.cos(110*2*np.pi/360)-224, 118*np.sin(110*2*np.pi/360)+116, 110*2*np.pi/360 - np.pi/2, 13.0, -13.0/118, 13.0, 118],
												[118*np.cos(145*2*np.pi/360)-224, 118*np.sin(145*2*np.pi/360)+116, 145*2*np.pi/360 - np.pi/2, 13.0, -13.0/118, 13.0, 118],
												[118*np.cos(180*2*np.pi/360)-224, 118*np.sin(180*2*np.pi/360)+116, 180*2*np.pi/360 - np.pi/2, 13.0, -13.0/118, 13.0, 118],
												[118*np.cos(215*2*np.pi/360)-224, 118*np.sin(215*2*np.pi/360)+116, 215*2*np.pi/360 - np.pi/2, 13.0, -13.0/118, 13.0, 118],
												[118*np.cos(250*2*np.pi/360)-224, 118*np.sin(250*2*np.pi/360)+116, 250*2*np.pi/360 - np.pi/2, 13.0, -13.0/118, 13.0, 118],
												#######################
												[-200.0, 230.0, 0.0, 17.8, 0.0, 18.0, 114],
												[-120.0, 230.0, 0.0, 18.3, 0.0, 18.0, 114],
												[-35.0, 230.0, 0.0, 18.2, 0.0, 18.0, 114],
												[65.0, 230.0, 0.0, 18.1, 0.0, 18.0, 114],
												[145.0, 230.0, 0.0, 17.9, 0.0, 18.0, 114],
												[-200.0, 2.0, 180*2*np.pi/360, 17.8, 0.0, 18.0, 114],
												[-120.0, 2.0, 180*2*np.pi/360, 18.3, 0.0, 18.0, 114],
												[-35.0, 2.0, 180*2*np.pi/360, 18.1, 0.0, 18.0, 114],
												[65.0, 2.0, 180*2*np.pi/360, 17.9, 0.0, 18.0, 114],
												[145.0, 2.0, 180*2*np.pi/360, 17.7, 0.0, 18.0, 114],
												[114*np.cos(270*2*np.pi/360)+224, 114*np.sin(270*2*np.pi/360)+116, 270*2*np.pi/360 - np.pi/2, 18.0, -18.0/114, 18.0, 114],
												[114*np.cos(315*2*np.pi/360)+224, 114*np.sin(315*2*np.pi/360)+116, 315*2*np.pi/360 - np.pi/2, 18.0, -18.0/114, 18.0, 114],
												[114*np.cos(360*2*np.pi/360)+224, 114*np.sin(360*2*np.pi/360)+116, 360*2*np.pi/360 - np.pi/2, 18.0, -18.0/114, 18.0, 114],
												[114*np.cos(45*2*np.pi/360)+224, 114*np.sin(45*2*np.pi/360)+116, 45*2*np.pi/360 - np.pi/2, 18.0, -18.0/114, 18.0, 114],
												[114*np.cos(90*2*np.pi/360)+224, 114*np.sin(90*2*np.pi/360)+116, 90*2*np.pi/360 - np.pi/2, 18.0, -18.0/114, 18.0, 114],
												[114*np.cos(90*2*np.pi/360)-224, 114*np.sin(90*2*np.pi/360)+116, 90*2*np.pi/360 - np.pi/2, 18.0, -18.0/114, 18.0, 114],
												[114*np.cos(135*2*np.pi/360)-224, 114*np.sin(135*2*np.pi/360)+116, 135*2*np.pi/360 - np.pi/2, 18.0, -18.0/114, 18.0, 114],
												[114*np.cos(180*2*np.pi/360)-224, 114*np.sin(180*2*np.pi/360)+116, 180*2*np.pi/360 - np.pi/2, 18.0, -18.0/114, 18.0, 114],
												[114*np.cos(225*2*np.pi/360)-224, 114*np.sin(225*2*np.pi/360)+116, 225*2*np.pi/360 - np.pi/2, 18.0, -18.0/114, 18.0, 114],
												[114*np.cos(270*2*np.pi/360)-224, 114*np.sin(270*2*np.pi/360)+116, 270*2*np.pi/360 - np.pi/2, 18.0, -18.0/114, 18.0, 114],
												])


		self.obstacles_intersection_north = np.array([[202, -22.0, 90*2*np.pi/360, 7.8, 0.0, 8.0, 0],
												[202.0, 38.0, 90*2*np.pi/360, 8.3, 0.0, 8.0, 0],
												[202.0, 98.0, 90*2*np.pi/360, 7.9, 0.0, 8.0, 0],
												#######################
												[206.0, -37.0, 90*2*np.pi/360, 9.8, 0.0, 10.0, 0],
												[206.0, 38.0, 90*2*np.pi/360, 9.9, 0.0, 10.0, 0],
												[206.0, 93.0, 90*2*np.pi/360, 10.3, 0.0, 10.0, 0],
												#######################
												[210.0, 13.0, 90*2*np.pi/360, 12.3, 0.0, 12.0, 0],
												[210.0, 88.0, 90*2*np.pi/360, 11.8, 0.0, 12.0, 0],
												#[210.0, 110.0, 90*2*np.pi/360, 11.9, 0.0, 12.0, 0],
												])


		self.obstacles_intersection_south = np.array([[222.0, 15.0, 270*2*np.pi/360, 12.3, 0.0, 12.0, 0],
												[222.0, 50.0, 270*2*np.pi/360, 11.8, 0.0, 12.0, 0],
												[222.0, 100.0, 270*2*np.pi/360, 11.9, 0.0, 12.0, 0],
												#######################
												[218.0, -15.0, 270*2*np.pi/360, 9.8, 0.0, 10.0, 0],
												[218.0, 50.0, 270*2*np.pi/360, 9.9, 0.0, 10.0, 0],
												[218.0, 95.0, 270*2*np.pi/360, 10.3, 0.0, 10.0, 0],
												######################
												[214.0, -40.0, 270*2*np.pi/360, 7.8, 0.0, 8.0, 0],
												[214.0, 30.0, 270*2*np.pi/360, 8.3, 0.0, 8.0, 0],
												#[214.0, 80.0, 270*2*np.pi/360, 7.9, 0.0, 8.0, 0],
												])

		self.obstacles_list = [self.obstacles_inner_lane, self.obstacles_outer_lane, self.obstacles_intersection_north,
								self.obstacles_intersection_south]
		self.use_IDM = ["True", "True", "True", "True"]
		self.right_circle_x = []
		self.right_circle_y = []
		self.left_circle_x = []
		self.left_circle_y = []
		for i in self.lane_radii:
			self.right_circle_x.append([i*np.cos(ang*2*np.pi/360)+224 for ang in angles_right])
			self.right_circle_y.append([i*np.sin(ang*2*np.pi/360)+116 for ang in angles_right])
			self.left_circle_x.append([i*np.cos(ang*2*np.pi/360)-224 for ang in angles_left])
			self.left_circle_y.append([i*np.sin(ang*2*np.pi/360)+116 for ang in angles_left])

		self.req = GetControls.Request()

	def send_request_new(self):
		self.req.odom.pose.pose.position.x = self.agent_pose[0]
		self.req.odom.pose.pose.position.y = self.agent_pose[1]
		self.req.odom.pose.pose.orientation.z = self.agent_pose[2]
		self.req.odom.twist.twist.linear.x = self.agent_vel[0]
		self.req.odom.twist.twist.angular.z = self.agent_vel[1]
		obs_pos = np.concatenate((self.obstacles_inner_lane, self.obstacles_outer_lane, 
								self.obstacles_intersection_south, self.obstacles_intersection_north))
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

		obs_pos = np.concatenate((self.obstacles_inner_lane, self.obstacles_outer_lane, 
								self.obstacles_intersection_south, self.obstacles_intersection_north))
		dist = np.sqrt((obs_pos[:,0] - self.agent_pose[0])**2 + (obs_pos[:,1] - self.agent_pose[1])**2)
		sorted_obs = obs_pos[dist.argsort()]
		dist = ((self.agent_pose[0] - sorted_obs[:,0])**2 + (self.agent_pose[1] - sorted_obs[:,1])**2)**0.5
		self.min_obs.append(np.min(dist))
		print("Min_dist = ", np.min(self.min_obs))
		obstacles = OdomArray()
		for i in range(10):
			odom = Odometry()
			odom.pose.pose.position.x = sorted_obs[i][0]
			odom.pose.pose.position.y = sorted_obs[i][1]
			odom.pose.pose.orientation.z = sorted_obs[i][2]
			odom.twist.twist.linear.x = sorted_obs[i][3]
			odom.twist.twist.angular.z = sorted_obs[i][4]
			#obs = plt.Circle((sorted_obs[i][0], sorted_obs[i][1]), 1.2, color='g')
			#plt.gca().add_patch(obs)
			obstacles.odom.append(odom)
		self.req.obstacles = obstacles

		goal = Pose()
		lane_cons = PoseArray()
		if len(self.path_x) == 0:
			lane_cons, goal = self.lane_cons_func[self.current_laneid](0.0)
		else:
			lane_cons, goal = self.get_lane_cons()
			print("Last Point = ", self.path_x[-1], self.path_y[-1])
		#lane_cons, goal = self.br_cons(0.0)
		info = Pose()   #min rad cons
		info.position.x = -2.0
		info.position.y = 2.0
		lane_cons.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 2.5*1e3
		info.position.y = 2.5*1e3
		lane_cons.poses.append(info)
		
		info = Pose()   #min rad cons
		info.position.x = 1.0*1e3
		lane_cons.poses.append(info)
		self.req.goal = goal
		self.req.lane_cons = lane_cons
		print("Current Lane = ", self.current_laneid)
		print("Next Lane = ", self.next_laneid)
		print("Goal = ", goal)
		#print("Lane Cons == ", self.req.lane_cons)
		self.goal_p = goal
		self.future = self.cli.call_async(self.req)


	def br_cons(self, index):
		print("br_cons")
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [14.0, 10.0, 6.0]
		lanes = [10.0]
		goal_pose.position.x = self.agent_pose[0] + 85.0
		goal_pose.position.y = random.choice(lanes)
		goal_pose.orientation.z = 0.0
		info = Pose()
		info.position.x = float(index)
		info.position.y = 0.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 1.0  #goal_pose.position.y+1e12
		info.position.z = -goal_pose.position.y  #1e12
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 0.0 #26.0 + 1e12
		info.position.z = 0.0 #1e12
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = -5.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 0.0 #self.agent_pose[0]
		info.position.y =  0.0 #-4.0 + 1e12
		info.position.z = 0.0
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = -15.0
		lane_info.poses.append(info)

		return lane_info, goal_pose

	def bl_cons(self, index):
		print("bl_cons")
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [2.0, -2.0, -6.0]
		lanes = [-2.0]
		goal_pose.position.x = self.agent_pose[0] - 85.0
		goal_pose.position.y = random.choice(lanes)
		goal_pose.orientation.z = np.pi
		info = Pose()
		info.position.x = float(index)
		info.position.y = 0.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 0.0
		info.position.y = 1.0
		info.position.z = -goal_pose.position.y
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 0.0 #26.0 + 1e12
		info.position.z = 0.0 #1e12
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = 7.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 0.0 #self.agent_pose[0]
		info.position.y =  0.0 #-4.0 + 1e12
		info.position.z = 0.0
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = -3.0
		lane_info.poses.append(info)

		return lane_info, goal_pose

	def tr_cons(self, index):
		print("tr_cons")
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [230.0, 234.0, 238.0]
		lanes = [234.0]
		goal_pose.position.x = self.agent_pose[0] + 85.0
		goal_pose.position.y = random.choice(lanes)
		goal_pose.orientation.z = 0.0
		info = Pose()
		info.position.x = float(index)
		info.position.y = 0.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 0.0
		info.position.y = 1.0
		info.position.z = -goal_pose.position.y
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 0.0 #26.0 + 1e12
		info.position.z = 0.0 #1e12
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = -229.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 0.0 #self.agent_pose[0]
		info.position.y =  0.0 #-4.0 + 1e12
		info.position.z = 0.0
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = -239.0
		lane_info.poses.append(info)
		print(lane_info)

		return lane_info, goal_pose

	def tl_cons(self, index):
		print("tl_cons")
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [218.0, 222.0, 226.0]
		lanes = [222.0]
		goal_pose.position.x = self.agent_pose[0] - 85.0
		goal_pose.position.y = random.choice(lanes)
		goal_pose.orientation.z = np.pi
		info = Pose()
		info.position.x = float(index)
		info.position.y = 0.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 0.0
		info.position.y = 1.0
		info.position.z = -goal_pose.position.y
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 0.0 #26.0 + 1e12
		info.position.z = 0.0 #1e12
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = -217.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 0.0 #self.agent_pose[0]
		info.position.y =  0.0 #-4.0 + 1e12
		info.position.z = 0.0
		info.orientation.x = 0.0
		info.orientation.y = 1.0
		info.orientation.z = 0.0
		info.orientation.w = -227.0
		lane_info.poses.append(info)

		return lane_info, goal_pose

	def ls_cons(self, index):
		print("ls_cons")
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [-202.0, -206.0, -210.0]
		lanes = [-206.0]
		goal_pose.position.x = random.choice(lanes)
		goal_pose.position.y = self.agent_pose[1] - 85.0
		goal_pose.orientation.z = 3*np.pi/2
		info = Pose()
		info.position.x = float(index)
		info.position.y = 0.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 1.0
		info.position.y = 0.0
		info.position.z = -goal_pose.position.x
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 1.0 #26.0 + 1e12
		info.position.z = 0.0 #1e12
		info.orientation.x = 0.0
		info.orientation.y = 0.0
		info.orientation.z = 0.0
		info.orientation.w = 211.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 0.0 #self.agent_pose[0]
		info.position.y =  1.0 #-4.0 + 1e12
		info.position.z = 0.0
		info.orientation.x = 0.0
		info.orientation.y = 0.0
		info.orientation.z = 0.0
		info.orientation.w = 201.0
		lane_info.poses.append(info)

		return lane_info, goal_pose

	def ln_cons(self, index):
		print("ln_cons")
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [-214.0, -218.0, -222.0]
		lanes = [-218.0]
		goal_pose.position.x = random.choice(lanes)
		goal_pose.position.y = self.agent_pose[1] + 85.0
		goal_pose.orientation.z = np.pi/2
		info = Pose()
		info.position.x = float(index)
		info.position.y = 0.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 1.0
		info.position.y = 0.0
		info.position.z = -goal_pose.position.x
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 1.0 #26.0 + 1e12
		info.position.z = 0.0 #1e12
		info.orientation.x = 0.0
		info.orientation.y = 0.0
		info.orientation.z = 0.0
		info.orientation.w = 223.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 0.0 #self.agent_pose[0]
		info.position.y =  1.0 #-4.0 + 1e12
		info.position.z = 0.0
		info.orientation.x = 0.0
		info.orientation.y = 0.0
		info.orientation.z = 0.0
		info.orientation.w = 213.0
		lane_info.poses.append(info)
		print(lane_info)

		return lane_info, goal_pose

	def rn_cons(self, index):
		print("rn_cons")
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [202.0, 206.0, 210.0]
		lanes = [206.0]
		goal_pose.position.x = random.choice(lanes)
		goal_pose.position.y = self.agent_pose[1] + 85.0
		dist = ((goal_pose.position.x-self.agent_pose[0])**2 + (goal_pose.position.y-self.agent_pose[1])**2)**0.5
		#if dist>88:
		#    goal_pose.position.x = random.choice(lanes)
		#    goal_pose.position.y = self.agent_pose[1] + 0
		#else:
		#    goal_pose.position.x = random.choice(lanes)
		#    goal_pose.position.y = self.agent_pose[1] + 85.0
		goal_pose.orientation.z = np.pi/2
		info = Pose()
		info.position.x = float(index)
		info.position.y = 0.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 1.0
		info.position.y = 0.0
		info.position.z = -goal_pose.position.x
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 1.0 #26.0 + 1e12
		info.position.z = 0.0 #1e12
		info.orientation.x = 0.0
		info.orientation.y = 0.0
		info.orientation.z = 0.0
		info.orientation.w = -201.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 0.0 #self.agent_pose[0]
		info.position.y =  1.0 #-4.0 + 1e12
		info.position.z = 0.0
		info.orientation.x = 0.0
		info.orientation.y = 0.0
		info.orientation.z = 0.0
		info.orientation.w = -211.0
		lane_info.poses.append(info)

		return lane_info, goal_pose

	def rs_cons(self, index):
		print("rs_cons")
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [214.0, 218.0, 222.0]
		lanes = [218.0]
		goal_pose.position.x = random.choice(lanes)
		goal_pose.position.y = self.agent_pose[1] - 85.0
		goal_pose.orientation.z = 3*np.pi/2
		info = Pose()
		info.position.x = float(index)
		info.position.y = 0.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 1.0
		info.position.y = 0.0
		info.position.z = -goal_pose.position.x
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 0.0  #self.agent_pose[0]
		info.position.y = 1.0 #26.0 + 1e12
		info.position.z = 0.0 #1e12
		info.orientation.x = 0.0
		info.orientation.y = 0.0
		info.orientation.z = 0.0
		info.orientation.w = -213.0
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 0.0 #self.agent_pose[0]
		info.position.y =  1.0 #-4.0 + 1e12
		info.position.z = 0.0
		info.orientation.x = 0.0
		info.orientation.y = 0.0
		info.orientation.z = 0.0
		info.orientation.w = -223.0
		lane_info.poses.append(info)

		return lane_info, goal_pose

	def lci_cons(self, index):
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [102.0, 106.0, 110.0]
		lanes = [106.0]
		

		theta = np.arctan2(self.agent_pose[1]-(116), self.agent_pose[0]-(-224))*2*180/(2*np.pi)
		if theta<0:
			theta = 360+theta
		
		val = random.choice(lanes)
		theta_g = theta+45.0
		goal_pose.position.x = val*np.cos(theta_g*2*np.pi/360) + -224.0
		goal_pose.position.y = val*np.sin(theta_g*2*np.pi/360) + 116.0
		goal_pose.orientation.z = theta_g*np.pi/180 + np.pi/2


		info = Pose()
		info.position.x = float(index)
		info.position.y = 1.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = -224.0
		info.position.y = 116.0
		info.position.z = val
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 1.0  #self.agent_pose[0]
		info.position.y = -2*(-224.0) #26.0 + 1e12
		info.position.z = (-224.0)**2 #1e12
		info.orientation.x = 1.0
		info.orientation.y = -2*116.0
		info.orientation.z = 116.0**2
		info.orientation.w = -101.0**2
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 1.0  #self.agent_pose[0]
		info.position.y = -2*(-224.0) #26.0 + 1e12
		info.position.z = (-224.0)**2 #1e12
		info.orientation.x = 1.0
		info.orientation.y = -2*116.0
		info.orientation.z = 116.0**2
		info.orientation.w = -111.0**2
		lane_info.poses.append(info)

		return lane_info, goal_pose

	def lco_cons(self, index):
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [114.0, 118.0, 122.0]
		lanes = [118.0]

		theta = np.arctan2(self.agent_pose[1]-(116), self.agent_pose[0]-(-224))*2*180/(2*np.pi)
		if theta<0:
			theta = 360+theta
		
		val = random.choice(lanes)
		theta_g = theta-45.0
		goal_pose.position.x = val*np.cos(theta_g*2*np.pi/360) + -224.0
		goal_pose.position.y = val*np.sin(theta_g*2*np.pi/360) + 116.0
		goal_pose.orientation.z = theta_g*np.pi/180 - np.pi/2

		info = Pose()
		info.position.x = float(index)
		info.position.y = 1.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = -224.0
		info.position.y = 116.0
		info.position.z = val
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 1.0  #self.agent_pose[0]
		info.position.y = -2*(-224.0) #26.0 + 1e12
		info.position.z = (-224.0)**2 #1e12
		info.orientation.x = 1.0
		info.orientation.y = -2*116.0
		info.orientation.z = 116.0**2
		info.orientation.w = -113.0**2
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 1.0  #self.agent_pose[0]
		info.position.y = -2*(-224.0) #26.0 + 1e12
		info.position.z = (-224.0)**2 #1e12
		info.orientation.x = 1.0
		info.orientation.y = -2*116.0
		info.orientation.z = 116.0**2
		info.orientation.w = -123.0**2
		lane_info.poses.append(info)

		return lane_info, goal_pose
	
	def rci_cons(self, index):
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [102.0, 106.0, 110.0]
		lanes = [106.0]

		theta = np.arctan2(self.agent_pose[1]-(116), self.agent_pose[0]-(224))*2*180/(2*np.pi)
		if theta<0:
			theta = 360+theta
		
		val = random.choice(lanes)
		theta_g = theta+45.0
		goal_pose.position.x = val*np.cos(theta_g*2*np.pi/360) + 224.0
		goal_pose.position.y = val*np.sin(theta_g*2*np.pi/360) + 116.0
		t = theta_g*np.pi/180 + np.pi/2
		if t>2*np.pi:
			t = t - 2*np.pi
		goal_pose.orientation.z = t

		info = Pose()
		info.position.x = float(index)
		info.position.y = 1.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 224.0
		info.position.y = 116.0
		info.position.z = val
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 1.0  #self.agent_pose[0]
		info.position.y = -2*224.0 #26.0 + 1e12
		info.position.z = 224.0**2 #1e12
		info.orientation.x = 1.0
		info.orientation.y = -2*116.0
		info.orientation.z = 116.0**2
		info.orientation.w = -101.0**2
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 1.0  #self.agent_pose[0]
		info.position.y = -2*224.0 #26.0 + 1e12
		info.position.z = 224.0**2 #1e12
		info.orientation.x = 1.0
		info.orientation.y = -2*116.0
		info.orientation.z = 116.0**2
		info.orientation.w = -111.0**2
		lane_info.poses.append(info)

		return lane_info, goal_pose

	def rco_cons(self, index):
		lane_info = PoseArray()
		goal_pose = Pose()
		#lanes = [114.0, 118.0, 122.0]
		lanes = [118.0]

		theta = np.arctan2(self.agent_pose[1]-(116), self.agent_pose[0]-(224))*2*180/(2*np.pi)
		if theta<0:
			theta = 360+theta
		
		val = random.choice(lanes)
		theta_g = theta-45.0
		goal_pose.position.x = val*np.cos(theta_g*2*np.pi/360) + 224.0
		goal_pose.position.y = val*np.sin(theta_g*2*np.pi/360) + 116.0
		goal_pose.orientation.z = theta_g*np.pi/180 - np.pi/2
		if goal_pose.orientation.z>np.pi:
			goal_pose.orientation.z = 2*np.pi - goal_pose.orientation.z

		info = Pose()
		info.position.x = float(index)
		info.position.y = 1.0
		lane_info.poses.append(info)

		info = Pose()
		info.position.x = 224.0
		info.position.y = 116.0
		info.position.z = val
		lane_info.poses.append(info)

		info = Pose()   #max rad cons
		info.position.x = 1.0  #self.agent_pose[0]
		info.position.y = -2*(224.0) #26.0 + 1e12
		info.position.z = (224.0)**2 #1e12
		info.orientation.x = 1.0
		info.orientation.y = -2*116.0
		info.orientation.z = 116.0**2
		info.orientation.w = -113.0**2
		lane_info.poses.append(info)

		info = Pose()   #min rad cons
		info.position.x = 1.0  #self.agent_pose[0]
		info.position.y = -2*(224.0) #26.0 + 1e12
		info.position.z = (224.0)**2 #1e12
		info.orientation.x = 1.0
		info.orientation.y = -2*116.0
		info.orientation.z = 116.0**2
		info.orientation.w = -123.0**2
		lane_info.poses.append(info)

		return lane_info, goal_pose

	def get_lane_cons(self):
		index = 0
		if self.current_laneid == LaneID.BR:
			print("BR Cons")
			for i in range(len(self.path_x)):
				if self.path_x[i]>-224 and self.path_y[i]<16:
					index = float(i)
					print("br index = ", index)
					break
			lane_cons, goal = self.lane_cons_func[self.current_laneid](index)
			if self.path_x[-1]>200 and self.current_laneid == self.next_laneid:
				self.next_laneid = LaneID.RN#random.choice([LaneID.RN, LaneID.RCI])
			if self.next_laneid == LaneID.RCI:
				self.current_laneid = self.next_laneid if self.path_x[-1]>=226 else self.current_laneid
				return lane_cons, goal
			elif self.next_laneid == LaneID.RN and self.path_x[-1]>=202:
				self.current_laneid = self.next_laneid
			
			return lane_cons, goal
			
		elif self.current_laneid == LaneID.BL:
			print("BL Cons")
			for i in range(len(self.path_x)):
				if self.path_x[i]<224 and self.path_y[i]<4:
					index = float(i)
					print("bl index = ", index)
					break
			lane_cons, goal = self.lane_cons_func[self.current_laneid](index)
			print(lane_cons, goal)
			if self.path_x[-1]<-212 and self.current_laneid == self.next_laneid:
				self.next_laneid = random.choice([LaneID.LN, LaneID.LCO])
			if self.next_laneid == LaneID.LCO:
				self.current_laneid = self.next_laneid if self.path_x[-1]<=-226 else self.current_laneid
				return lane_cons, goal
			elif self.next_laneid == LaneID.LN and self.path_x[-1]<=-214:
				self.current_laneid = self.next_laneid
			return lane_cons, goal

		elif self.current_laneid == LaneID.TR:
			print("TR Cons")
			for i in range(len(self.path_x)):
				if self.path_x[i]>-224 and self.path_y[i]>228:
					index = float(i)
					print("tr index = ", index, self.path_x[i], self.path_y[i])
					break
			lane_cons, goal = self.lane_cons_func[self.current_laneid](index)
			if self.path_x[-1]>212 and self.current_laneid == self.next_laneid:
				self.next_laneid = LaneID.RCO#random.choice([LaneID.RS, LaneID.RCO])
			if self.next_laneid == LaneID.RCO:
				self.current_laneid = self.next_laneid if self.path_x[-1]>=230 else self.current_laneid
				return lane_cons, goal
			elif self.next_laneid == LaneID.RS:
				self.current_laneid = self.next_laneid
			
			return lane_cons, goal

		elif self.current_laneid == LaneID.TL:
			print("TL Cons")
			for i in range(len(self.path_x)):
				if self.path_x[i]<224 and self.path_y[i]>216:
					index = float(i)
					print("tl index = ", index)
					break
			lane_cons, goal = self.lane_cons_func[self.current_laneid](index)
			if self.path_x[-1]<-200 and self.current_laneid == self.next_laneid:
				self.next_laneid = random.choice([LaneID.LS, LaneID.LCI])
			if self.next_laneid == LaneID.LCI:
				self.current_laneid = self.next_laneid if self.path_x[-1]<=-226 else self.current_laneid
				return lane_cons, goal
			elif self.next_laneid == LaneID.LS and self.path_x[-1]<=-202:
				self.current_laneid = self.next_laneid
			
			return lane_cons, goal

		elif self.current_laneid == LaneID.RCI:
			print("RCI Cons")
			for i in range(len(self.path_x)):
				if self.path_x[i]>=224 and self.path_y[i]<16:
					index = float(i)
					print("rci index = ", index)
					break
			lane_cons, goal = self.lane_cons_func[self.current_laneid](index)
			if self.path_x[-1]<=222 and self.path_y[-1]>116 and self.current_laneid == self.next_laneid:
				self.next_laneid = LaneID.RS#random.choice([LaneID.RS, LaneID.TL])
			self.current_laneid = self.next_laneid
			return lane_cons, goal
				
		elif self.current_laneid == LaneID.RCO:
			print("RCO Cons")
			for i in range(len(self.path_x)):
				if self.path_x[i]>=224 and self.path_y[i]>228:
					index = float(i)
					print("rco index = ", index)
					break
			lane_cons, goal = self.lane_cons_func[self.current_laneid](index)
			if self.path_x[-1]<=224 and self.path_y[-1]<116 and self.current_laneid == self.next_laneid:
				self.next_laneid =LaneID.RN#random.choice([LaneID.BL, LaneID.RN])
			if self.next_laneid == LaneID.RN:
				self.current_laneid = self.next_laneid if self.path_x[-1]<=210 else self.current_laneid
				return lane_cons, goal
			elif self.next_laneid == LaneID.BL and self.path_x[-1]<=222:
				self.current_laneid = self.next_laneid
			
			return lane_cons, goal

		elif self.current_laneid == LaneID.LCI:
			print("LCI Cons")
			for i in range(len(self.path_x)):
				if self.path_x[i]<=-224 and self.path_y[i]>216:
					index = float(i)
					print("lci index = ", index)
					break
			lane_cons, goal = self.lane_cons_func[self.current_laneid](index)
			if self.path_x[-1]>=-222 and self.path_y[-1]<116 and self.current_laneid == self.next_laneid:
				self.next_laneid = LaneID.BR#random.choice([LaneID.BR, LaneID.LN])
			self.current_laneid = self.next_laneid
			return lane_cons, goal

		elif self.current_laneid == LaneID.LCO:
			print("LCO Cons")
			for i in range(len(self.path_x)):
				if self.path_x[i]<=-224 and (self.path_y[i]<4 or self.path_y[i]>-8):
					index = float(i)
					print("lco index = ", index)
					break
			lane_cons, goal = self.lane_cons_func[self.current_laneid](index)
			if self.path_x[-1]>=-224 and self.path_y[-1]>116 and self.current_laneid == self.next_laneid:
				self.next_laneid = LaneID.TR#random.choice([LaneID.TR, LaneID.LS])
			if self.next_laneid == LaneID.LS:
				self.current_laneid = self.next_laneid if self.path_x[-1]>=-210 else self.current_laneid
				return lane_cons, goal
			elif self.next_laneid == LaneID.TR:
				self.current_laneid = self.next_laneid
			
			return lane_cons, goal

		elif self.current_laneid == LaneID.RN:
			print("RN Cons")
			for i in range(len(self.path_x)):
				if self.path_x[i]>200 and self.path_x[i]<=212 and self.path_y[i]>4:
					index = float(i)
					print("rn index = ", index)
					break
			lane_cons, goal = self.lane_cons_func[self.current_laneid](index)
			if self.path_y[-1]>=218 and self.current_laneid == self.next_laneid:
				self.next_laneid = LaneID.TL#random.choice([LaneID.TL, LaneID.RCO])
			if self.next_laneid == LaneID.RCO:
				self.current_laneid = self.next_laneid if self.path_y[-1]>=230 else self.current_laneid
				return lane_cons, goal
			elif self.next_laneid == LaneID.TL:
				self.current_laneid = self.next_laneid
			
			return lane_cons, goal

		elif self.current_laneid == LaneID.RS:
			print("RS Cons")
			for i in range(len(self.path_x)):
				if self.path_x[i]>212 and self.path_x[i]<224 and self.path_y[i]<240:
					index = float(i)
					print("rs index = ", index)
					break
			lane_cons, goal = self.lane_cons_func[self.current_laneid](index)
			if self.path_y[-1]<=14 and self.current_laneid == self.next_laneid:
				self.next_laneid = LaneID.BL#random.choice([LaneID.BL, LaneID.RCI])
			if self.next_laneid == LaneID.BL:
				self.current_laneid = self.next_laneid if self.path_y[-1]<=2 else self.current_laneid
				return lane_cons, goal
			elif self.next_laneid == LaneID.RCI:
				self.current_laneid = self.next_laneid
			
			return lane_cons, goal

		elif self.current_laneid == LaneID.LN:
			print("LN Cons")
			for i in range(len(self.path_x)):
				if self.path_x[i]>-224 and self.path_x[i]<-212 and self.path_y[i]>-8:
					index = float(i)
					print("ln index = ", index)
					break
			lane_cons, goal = self.lane_cons_func[self.current_laneid](index)
			if self.path_y[-1]>=216 and self.current_laneid == self.next_laneid:
				self.next_laneid = LaneID.TR#random.choice([LaneID.TR, LaneID.LCI])
			if self.next_laneid == LaneID.TR:
				self.current_laneid = self.next_laneid if self.path_y[-1]>=230 else self.current_laneid
				return lane_cons, goal
			elif self.next_laneid == LaneID.LCI:
				self.current_laneid = self.next_laneid
			
			return lane_cons, goal

		elif self.current_laneid == LaneID.LS:
			print("LS Cons")
			for i in range(len(self.path_x)):
				if self.path_x[i]>-212 and self.path_x[i]<-200 and self.path_y[i]<240:
					index = float(i)
					print("ls index = ", index)
					break
			lane_cons, goal = self.lane_cons_func[self.current_laneid](index)
			if self.path_y[-1]<=16 and self.current_laneid == self.next_laneid:
				self.next_laneid = LaneID.BR#random.choice([LaneID.BR, LaneID.RCO])
			if self.next_laneid == LaneID.RCO:
				self.current_laneid = self.next_laneid if self.path_y[-1]<=2 else self.current_laneid
				return lane_cons, goal
			elif self.next_laneid == LaneID.BR:
				self.current_laneid = self.next_laneid
			
			return lane_cons, goal

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
		for i in range(self.obstacles_inner_lane.shape[0]):
			if self.obstacles_inner_lane[i][3]<=self.obstacles_inner_lane[i][5]:
				self.obstacles_inner_lane[i][3] = self.obstacles_inner_lane[i][3] + self.acc*self.dt
			else:
				self.obstacles_inner_lane[i][3] = self.obstacles_inner_lane[i][3] - self.acc*self.dt

			if self.obstacles_inner_lane[i][1]<116 and self.obstacles_inner_lane[i][0]>=224:
				self.obstacles_inner_lane[i][4] = self.obstacles_inner_lane[i][3]/self.obstacles_inner_lane[i][6]
			elif self.obstacles_inner_lane[i][1]>116 and self.obstacles_inner_lane[i][0]<=224 and self.obstacles_inner_lane[i][0]>220:
				self.obstacles_inner_lane[i][4] = 0.0
				self.obstacles_inner_lane[i][1] = self.obstacles_inner_lane[i][6] + 116.0
				self.obstacles_inner_lane[i][2] = 180*2*np.pi/360
			elif self.obstacles_inner_lane[i][1]>116 and self.obstacles_inner_lane[i][0]<=-224:
				self.obstacles_inner_lane[i][4] = self.obstacles_inner_lane[i][3]/self.obstacles_inner_lane[i][6]
			elif self.obstacles_inner_lane[i][1]<116 and self.obstacles_inner_lane[i][0]>=-224 and self.obstacles_inner_lane[i][0]<-220:
				self.obstacles_inner_lane[i][4] = 0.0
				self.obstacles_inner_lane[i][2] = 0*2*np.pi/360
				self.obstacles_inner_lane[i][1] = 116.0 - self.obstacles_inner_lane[i][6]
				
			self.obstacles_inner_lane[i][2] = self.obstacles_inner_lane[i][2] + self.obstacles_inner_lane[i][4]*self.dt
			self.obstacles_inner_lane[i][0] = self.obstacles_inner_lane[i][0] + \
											self.obstacles_inner_lane[i][3]*np.cos(self.obstacles_inner_lane[i][2])*self.dt
			self.obstacles_inner_lane[i][1] = self.obstacles_inner_lane[i][1] + \
											self.obstacles_inner_lane[i][3]*np.sin(self.obstacles_inner_lane[i][2])*self.dt
		##################################
		for i in range(self.obstacles_outer_lane.shape[0]):
			if self.obstacles_outer_lane[i][3]<=self.obstacles_outer_lane[i][5]:
				self.obstacles_outer_lane[i][3] = self.obstacles_outer_lane[i][3] + self.acc*self.dt
			else:
				self.obstacles_outer_lane[i][3] = self.obstacles_outer_lane[i][3] - self.acc*self.dt

			if self.obstacles_outer_lane[i][1]<116 and self.obstacles_outer_lane[i][0]<=-224:
				self.obstacles_outer_lane[i][4] = -self.obstacles_outer_lane[i][3]/self.obstacles_outer_lane[i][6]
			elif self.obstacles_outer_lane[i][1]>116 and self.obstacles_outer_lane[i][0]>=-224 and self.obstacles_outer_lane[i][0]<-220:
				self.obstacles_outer_lane[i][4] = 0.0
				self.obstacles_outer_lane[i][2] = 0*2*np.pi/360
				self.obstacles_outer_lane[i][1] = self.obstacles_outer_lane[i][6] + 116.0
			elif self.obstacles_outer_lane[i][1]>116 and self.obstacles_outer_lane[i][0]>=224:
				self.obstacles_outer_lane[i][4] = -self.obstacles_outer_lane[i][3]/self.obstacles_outer_lane[i][6]
			elif self.obstacles_outer_lane[i][1]<116 and self.obstacles_outer_lane[i][0]<=224 and self.obstacles_outer_lane[i][0]>220:
				self.obstacles_outer_lane[i][4] = 0.0
				self.obstacles_outer_lane[i][2] = 180*2*np.pi/360
				self.obstacles_outer_lane[i][1] = 116.0 - self.obstacles_outer_lane[i][6]
					
			self.obstacles_outer_lane[i][2] = self.obstacles_outer_lane[i][2] + self.obstacles_outer_lane[i][4]*self.dt
			self.obstacles_outer_lane[i][0] = self.obstacles_outer_lane[i][0] + \
											self.obstacles_outer_lane[i][3]*np.cos(self.obstacles_outer_lane[i][2])*self.dt
			self.obstacles_outer_lane[i][1] = self.obstacles_outer_lane[i][1] + \
											self.obstacles_outer_lane[i][3]*np.sin(self.obstacles_outer_lane[i][2])*self.dt
		##################################
		if self.agent_pose[0]>0 and self.obstacles_intersection_north[0][0]<0.0:
			self.obstacles_intersection_north[:,0] = self.obstacles_intersection_north[:,0] + 424
		elif self.agent_pose[0]<0 and self.obstacles_intersection_north[0][0]>0.0:
			self.obstacles_intersection_north[:,0] = self.obstacles_intersection_north[:,0] - 424
		for i in range(self.obstacles_intersection_north.shape[0]):
			so = 1
			T = 1

			l = 1.5

			a = 4
			b = 3

			nearest = -1
			x = self.obstacles_intersection_north[i][0]
			y = self.obstacles_intersection_north[i][1]
			inLane = self.obstacles_intersection_north[:,0] - x * np.ones(len(self.obstacles_intersection_north))
			index = np.where( inLane == 0)

			min = 10000
			for k in range(len(index[0])):
				if index[0][k] != i:
					if y < self.obstacles_intersection_north[index[0][k]][1]:
						dist = self.obstacles_intersection_north[index[0][k]][1] - y
						if dist < min:
							min = dist
							nearest = index[0][k]
			
			v_0 = self.obstacles_intersection_north[nearest][3]
			x_0 = self.obstacles_intersection_north[nearest][1]
			if abs(x - self.agent_pose[0]) <= 1.5 and y < self.agent_pose[1]:
				if nearest != -1:
					if self.agent_pose[1] < self.obstacles_intersection_north[nearest][1]: 
						nearest = 1
						v_0 = self.agent_vel[0]
						x_0 = self.agent_pose[1]
				else:
					v_0 = self.agent_vel[0]
					x_0 = self.agent_pose[1]
			v_1 = self.obstacles_intersection_north[i][3]
			x_1 = y

			v_r = self.obstacles_intersection_north[i][5]
			delta_v = v_1 - v_0
			s_alpha = x_0 - x_1 - l

			if nearest != -1:
				s_star = so + v_1 * T + (v_1 * delta_v)/(2 * np.sqrt(a*b))
				decc = a * (1 - (v_1/v_r)**4 - (s_star/s_alpha)**2)
				if decc < -a:
					decc = -a
				self.obstacles_intersection_north[i][3] += decc * self.dt
			else:
				decc = a * (1 - (v_1/v_r)**4)
				if decc > a:
					decc = a
				self.obstacles_intersection_north[i][3] += decc * self.dt

			if self.agent_pose[1] - self.obstacles_intersection_north[i][1]>35:
				self.obstacles_intersection_north[i][1] = self.agent_pose[1] + 70.0
			elif self.obstacles_intersection_north[i][1] - self.agent_pose[1]>70:
				self.obstacles_intersection_north[i][1] = self.agent_pose[1] - 35.0
			self.obstacles_intersection_north[i][2] = self.obstacles_intersection_north[i][2] + self.obstacles_intersection_north[i][4]*self.dt
			self.obstacles_intersection_north[i][0] = self.obstacles_intersection_north[i][0] + \
											self.obstacles_intersection_north[i][3]*np.cos(self.obstacles_intersection_north[i][2])*self.dt
			self.obstacles_intersection_north[i][1] = self.obstacles_intersection_north[i][1] + \
											self.obstacles_intersection_north[i][3]*np.sin(self.obstacles_intersection_north[i][2])*self.dt
		##################################
		if self.agent_pose[0]>0 and self.obstacles_intersection_south[0][0]<0.0:
			self.obstacles_intersection_south[:,0] = self.obstacles_intersection_south[:,0] + 424
		elif self.agent_pose[0]<0 and self.obstacles_intersection_south[0][0]>0.0:
			self.obstacles_intersection_south[:,0] = self.obstacles_intersection_south[:,0] - 424
		for i in range(self.obstacles_intersection_south.shape[0]):
			so = 1
			T = 1

			l = 1.5

			a = 4
			b = 3
			
			nearest = -1
			x = self.obstacles_intersection_south[i][0]
			y = self.obstacles_intersection_south[i][1]
			inLane = self.obstacles_intersection_south[:,0] - x * np.ones(len(self.obstacles_intersection_south))
			index = np.where( inLane == 0)

			min = 10000
			for k in range(len(index[0])):
				if index[0][k] != i:
					if y > self.obstacles_intersection_south[index[0][k]][1]:
						dist = self.obstacles_intersection_south[index[0][k]][1] - y
						if dist < min:
							min = dist
							nearest = index[0][k]
			
			v_0 = self.obstacles_intersection_south[nearest][3]
			x_0 = self.obstacles_intersection_south[nearest][1]
			if abs(x - self.agent_pose[0]) <= 1.5 and y > self.agent_pose[1]:
				if nearest != -1:
					if self.agent_pose[1] > self.obstacles_intersection_south[nearest][1]: 
						nearest = 1
						v_0 = self.agent_vel[0]
						x_0 = self.agent_pose[1]
				else:
					v_0 = self.agent_vel[0]
					x_0 = self.agent_pose[1]
			v_1 = self.obstacles_intersection_south[i][3]
			x_1 = y

			v_r = self.obstacles_intersection_south[i][5]
			delta_v = v_1 - v_0
			s_alpha = x_0 - x_1 - l

			if nearest != -1:
				s_star = so + v_1 * T + (v_1 * delta_v)/(2 * np.sqrt(a*b))
				decc = a * (1 - (v_1/v_r)**4 - (s_star/s_alpha)**2)
				if decc < -a:
					decc = -a
				self.obstacles_intersection_south[i][3] += decc * self.dt
			else:
				decc = a * (1 - (v_1/v_r)**4)
				if decc > a:
					decc = a
				self.obstacles_intersection_south[i][3] += decc * self.dt

			if -self.agent_pose[1] + self.obstacles_intersection_south[i][1]>35:
				self.obstacles_intersection_south[i][1] = self.agent_pose[1] - 70.0
			elif -self.obstacles_intersection_south[i][1] + self.agent_pose[1]>70:
				self.obstacles_intersection_south[i][1] = self.agent_pose[1] + 35.0
			self.obstacles_intersection_south[i][2] = self.obstacles_intersection_south[i][2] + self.obstacles_intersection_south[i][4]*self.dt
			self.obstacles_intersection_south[i][0] = self.obstacles_intersection_south[i][0] + \
											self.obstacles_intersection_south[i][3]*np.cos(self.obstacles_intersection_south[i][2])*self.dt
			self.obstacles_intersection_south[i][1] = self.obstacles_intersection_south[i][1] + \
											self.obstacles_intersection_south[i][3]*np.sin(self.obstacles_intersection_south[i][2])*self.dt



	def plot_obstacles(self):
		for i in self.obstacles_inner_lane:
			obs = plt.Circle((i[0], i[1]), 1.0, color='r')
			plt.gca().add_patch(obs)
		for i in self.obstacles_outer_lane:
			obs = plt.Circle((i[0], i[1]), 1.0, color='r')
			plt.gca().add_patch(obs)
		for i in self.obstacles_intersection_south:
			obs = plt.Circle((i[0], i[1]), 1.0, color='r')
			plt.gca().add_patch(obs)
		for i in self.obstacles_intersection_north:
			obs = plt.Circle((i[0], i[1]), 1.0, color='r')
			plt.gca().add_patch(obs)
		agent = plt.Circle((self.agent_pose[0], self.agent_pose[1]), 1.0, color='g')
		plt.text(self.agent_pose[0], self.agent_pose[1]+30, 'Vel = %s'%(round(self.agent_vel[0],2)), fontsize=10)
		plt.gca().add_patch(agent)


	def plot_lanes(self):
		#   Plot curved lanes
		for i in range(len(self.lane_radii)):
			lw = 1.0
			if i==0 or i==3 or i==6:
				lw = 2.0
			plt.plot(self.right_circle_x[i], self.right_circle_y[i], color='black', linewidth=lw)
			plt.plot(self.left_circle_x[i], self.left_circle_y[i], color='black', linewidth=lw)
		
		########################################################################################

		#   Plot intersetion lanes
		#plt.plot([-232, -232], [16, 216], color='black', linewidth=2.0)
		#plt.plot([-228, -228], [16, 216], color='black', linewidth=1.0)
		plt.plot([-224, -224], [16, 216], color='black', linewidth=2.0)
		plt.plot([-220, -220], [16, 216], color='black', linewidth=1.0)
		plt.plot([-216, -216], [16, 216], color='black', linewidth=1.0)
		plt.plot([-212, -212], [16, 216], color='black', linewidth=2.0)
		plt.plot([-208, -208], [16, 216], color='black', linewidth=1.0)
		plt.plot([-204, -204], [16, 216], color='black', linewidth=1.0)
		plt.plot([-200, -200], [16, 216], color='black', linewidth=2.0)

		#plt.plot([232, 232], [16, 216], color='black', linewidth=2.0)
		#plt.plot([228, 228], [16, 216], color='black', linewidth=1.0)
		plt.plot([224, 224], [16, 216], color='black', linewidth=2.0)
		plt.plot([220, 220], [16, 216], color='black', linewidth=1.0)
		plt.plot([216, 216], [16, 216], color='black', linewidth=1.0)
		plt.plot([212, 212], [16, 216], color='black', linewidth=2.0)
		plt.plot([208, 208], [16, 216], color='black', linewidth=1.0)
		plt.plot([204, 204], [16, 216], color='black', linewidth=1.0)
		plt.plot([200, 200], [16, 216], color='black', linewidth=2.0)

		#plt.plot([-232, -232], [248, 3000], color='black', linewidth=2.0)
		#plt.plot([-228, -228], [248, 3000], color='black', linewidth=1.0)
		plt.plot([-224, -224], [240, 3000], color='black', linewidth=2.0)
		plt.plot([-220, -220], [240, 3000], color='black', linewidth=1.0)
		plt.plot([-216, -216], [240, 3000], color='black', linewidth=1.0)
		plt.plot([-212, -212], [240, 3000], color='black', linewidth=2.0)
		plt.plot([-208, -208], [240, 3000], color='black', linewidth=1.0)
		plt.plot([-204, -204], [240, 3000], color='black', linewidth=1.0)
		plt.plot([-200, -200], [240, 3000], color='black', linewidth=2.0)

		#plt.plot([-232, -232], [-16, -600], color='black', linewidth=2.0)
		#plt.plot([-228, -228], [-16, -600], color='black', linewidth=1.0)
		plt.plot([-224, -224], [-8, -600], color='black', linewidth=2.0)
		plt.plot([-220, -220], [-8, -600], color='black', linewidth=1.0)
		plt.plot([-216, -216], [-8, -600], color='black', linewidth=1.0)
		plt.plot([-212, -212], [-8, -600], color='black', linewidth=2.0)
		plt.plot([-208, -208], [-8, -600], color='black', linewidth=1.0)
		plt.plot([-204, -204], [-8, -600], color='black', linewidth=1.0)
		plt.plot([-200, -200], [-8, -600], color='black', linewidth=2.0)

		#plt.plot([232, 232], [248, 3000], color='black', linewidth=2.0)
		#plt.plot([228, 228], [248, 3000], color='black', linewidth=1.0)
		plt.plot([224, 224], [240, 3000], color='black', linewidth=2.0)
		plt.plot([220, 220], [240, 3000], color='black', linewidth=1.0)
		plt.plot([216, 216], [240, 3000], color='black', linewidth=1.0)
		plt.plot([212, 212], [240, 3000], color='black', linewidth=2.0)
		plt.plot([208, 208], [240, 3000], color='black', linewidth=1.0)
		plt.plot([204, 204], [240, 3000], color='black', linewidth=1.0)
		plt.plot([200, 200], [240, 3000], color='black', linewidth=2.0)

		#plt.plot([232, 232], [-16, -600], color='black', linewidth=2.0)
		#plt.plot([228, 228], [-16, -600], color='black', linewidth=1.0)
		plt.plot([224, 224], [-8, -600], color='black', linewidth=2.0)
		plt.plot([220, 220], [-8, -600], color='black', linewidth=1.0)
		plt.plot([216, 216], [-8, -600], color='black', linewidth=1.0)
		plt.plot([212, 212], [-8, -600], color='black', linewidth=2.0)
		plt.plot([208, 208], [-8, -600], color='black', linewidth=1.0)
		plt.plot([204, 204], [-8, -600], color='black', linewidth=1.0)
		plt.plot([200, 200], [-8, -600], color='black', linewidth=2.0)

		########################################################################################

		#   Plot normal lane
		plt.plot([-200, 200], [16, 16], color='black', linewidth=2.0)
		plt.plot([-200, 200], [12, 12], color='black', linewidth=1.0)
		plt.plot([-200, 200], [8, 8], color='black', linewidth=1.0)
		plt.plot([-200, 200], [4, 4], color='black', linewidth=2.0)
		plt.plot([-200, 200], [0, 0], color='black', linewidth=1.0)
		plt.plot([-200, 200], [-4, -4], color='black', linewidth=1.0)
		plt.plot([-200, 200], [-8, -8], color='black', linewidth=2.0)
		#plt.plot([-200, 200], [-12, -12], color='black', linewidth=1.0)
		#plt.plot([-200, 200], [-16, -16], color='black', linewidth=2.0)

		plt.plot([-200, 200], [216, 216], color='black', linewidth=2.0)
		plt.plot([-200, 200], [220, 220], color='black', linewidth=1.0)
		plt.plot([-200, 200], [224, 224], color='black', linewidth=1.0)
		plt.plot([-200, 200], [228, 228], color='black', linewidth=2.0)
		plt.plot([-200, 200], [232, 232], color='black', linewidth=1.0)
		plt.plot([-200, 200], [236, 236], color='black', linewidth=1.0)
		plt.plot([-200, 200], [240, 240], color='black', linewidth=2.0)
		#plt.plot([-200, 200], [244, 244], color='black', linewidth=1.0)
		#plt.plot([-200, 200], [248, 248], color='black', linewidth=2.0)


		#a1 = plt.Circle((102*np.cos(300*2*np.pi/360)+232, 102*np.sin(300*2*np.pi/360)+116), 1.0)
		#b1 = plt.Circle((102*np.cos(330*2*np.pi/360)+232, 102*np.sin(330*2*np.pi/360)+116), 1.0)
		#plt.gca().add_patch(a1)
		#plt.gca().add_patch(b1)
		#a2 = plt.Circle((0.0, 10.0), 1.0)
		#b2 = plt.Circle((40.0, 10.0), 1.0)
		#plt.gca().add_patch(a2)
		#plt.gca().add_patch(b2)
		#a3 = plt.Circle((0.0, 6.0), 1.0)
		#b3 = plt.Circle((90.0, 6.0), 1.0)
		#plt.gca().add_patch(a3)
		#plt.gca().add_patch(b3)
		#a4 = plt.Circle((0.0, 2.0), 1.0)
		#b4 = plt.Circle((120.0, 2.0), 1.0)
		#plt.gca().add_patch(a4)
		#plt.gca().add_patch(b4)
		

	
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
		plt.ylim(-30+self.agent_pose[1], 30+self.agent_pose[1])
		plt.xlim(-30+self.agent_pose[0], 100+self.agent_pose[0])
		#plt.ylim(-50, 250)
		#plt.xlim(-350, 350)
		plt.draw()
		plt.pause(0.0001)
		#if self.agent_pose[1]>100.0:
		#	quit()
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