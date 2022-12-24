#include <memory>
#include<algorithm>
#include <fstream>
#include <iostream>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "acado_msgs/srv/get_velocity_cmd.hpp"
#include "acado_msgs/srv/get_controls_multi.hpp"
#include "acado_msgs/msg/odom_array.hpp"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
//#include "test_car/acado_solver.c"
//#include "test_car/acado_qpoases_interface.hpp"
//#include "test_car/acado_integrator.c"

/*
Much earlier divergence of the trajectories, with reduced occiliations, with higher iterations
*/

#include <stdio.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */
/*extern "C"{
__thread ACADOvariables acadoVariables;
__thread ACADOworkspace acadoWorkspace;
}*/
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class GoalReach : public rclcpp::Node
{
  public:
    GoalReach();
  private:
    void get_vel_cb(const std::shared_ptr<acado_msgs::srv::GetControlsMulti::Request> request,
          const std::shared_ptr<acado_msgs::srv::GetControlsMulti::Response>      response);
    void check_term_dist(double dist);
    nav_msgs::msg::Odometry start;
    geometry_msgs::msg::PoseArray lane_cons;
    acado_msgs::msg::OdomArray obstacles;
    bool got_odom;
    std::vector<int> ns;
    int ni;
    bool got_pa;
    int    i, iter, num_goals;
    double time;
    int mpc_iter;
    acado_timer t;
    std::vector<double> xg; 
    std::vector<double> yg;
    std::vector<double> thetag;
    std::vector<std::vector<float>> multi_goal_state;
    std::vector<std::vector<float>> multi_goal_controls;
    rclcpp::Service<acado_msgs::srv::GetControlsMulti>::SharedPtr service;
    size_t count_;
    std::vector<double> time_arr;
    std::ofstream outdata;
};

GoalReach::GoalReach(): Node("combined_ngsim"), count_(0)
{
    service = this->create_service<acado_msgs::srv::GetControlsMulti>("/get_vel", std::bind(&GoalReach::get_vel_cb, this, _1, _2));
    got_odom = false;
    got_pa = false;
    ns.push_back(10);
    ns.push_back(10);
    ns.push_back(10);
    ns.push_back(10);
    ns.push_back(10);
    ns.push_back(10);
    ni = 5;
    time = 0.0;
    mpc_iter = 0;

    num_goals = 6;

    xg.push_back(0);
    xg.push_back(0);
    yg.push_back(0);
    yg.push_back(0);
    thetag.push_back(0);
    thetag.push_back(0);

    acado_initializeSolver();

    for (i = 0; i < (N + 1); ++i)  
    {
        acadoVariables.x[ NX * i + 0 ] = 0.0; // x
        acadoVariables.x[ NX * i + 1 ] = 14.0; // y
        acadoVariables.x[ NX * i + 2 ] = 0.0; // theta
        acadoVariables.x[ NX * i + 3 ] = 15.0; // v 
        acadoVariables.x[ NX * i + 4 ] = 0.0; // w
    }

    for(int goal_id = 0; goal_id<num_goals; goal_id++)
    {
        std::vector<float> single_goal_state;
        for (i = 0; i < (N + 1); ++i)  
        {
            single_goal_state.push_back(0);
            single_goal_state.push_back(2.0);
            single_goal_state.push_back(0);
            single_goal_state.push_back(15.0);
            single_goal_state.push_back(0);
        }
        multi_goal_state.push_back(single_goal_state);
    }

    for (i = 0; i < N; ++i)  
    {
        acadoVariables.u[ NU * i + 0 ] = 0.0001;
        acadoVariables.u[ NU * i + 1 ] = 0.0001;
    }
    for(int goal_id = 0; goal_id<num_goals; goal_id++)
    {
        std::vector<float> single_goal_control;
        for (i = 0; i < (N + 1); ++i)  
        {
            single_goal_control.push_back(0.0001);
            single_goal_control.push_back(0.0001);
        }
        multi_goal_controls.push_back(single_goal_control);
    }
    /* Initialize the measurements/reference. */
    for (i = 0; i < N; ++i)  
    {
        acadoVariables.y[ NY * i + 0 ] = 100; // xg
        acadoVariables.y[ NY * i + 1 ] = -6.0;	// yg	
        acadoVariables.y[ NY * i + 2 ] = 15.0;	// v	
        acadoVariables.y[ NY * i + 3 ] = 0.0;	// linear acc
        acadoVariables.y[ NY * i + 4 ] = 0.0; 	//  anuglar acc
        //acadoVariables.y[ NY * i + 5 ] = 0.0; 	//  dist to line
        acadoVariables.y[ NY * i + 5 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 6 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 7 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 8 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 9 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 10 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 11 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 12 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 13 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 14 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 15 ] = 0.0; 	//  dist
        acadoVariables.y[ NY * i + 16 ] = 0.0; 	//  dist
        //acadoVariables.y[ NY * i + 4 ] = 0.0;
        //acadoVariables.y[ NY * i + 5 ] = 0.0;
        //acadoVariables.y[ NY * i + 6 ] = 0.0;
    }

    acadoVariables.yN[0] = 200.0;
    acadoVariables.yN[1] = -6.0; // theta
    acadoVariables.yN[2] = 0.0;
    for (int i = 0; i < (N + 1); ++i)
    {
        acadoVariables.od[i * NOD + 0] = 4.0*10;
        acadoVariables.od[i * NOD + 1] = 4.0*10;
        acadoVariables.od[i * NOD + 2] = 0.0;
        acadoVariables.od[i * NOD + 3] = 7.0*10;
        acadoVariables.od[i * NOD + 4] = 0.0;
        acadoVariables.od[i * NOD + 5] = 0.0;
        acadoVariables.od[i * NOD + 6] = 18.0*10;
        acadoVariables.od[i * NOD + 7] = -4.0*10;
        acadoVariables.od[i * NOD + 8] = 0.0;
        acadoVariables.od[i * NOD + 9] = 18.0*10;
        acadoVariables.od[i * NOD + 10] = -4.0*10;
        acadoVariables.od[i * NOD + 11] = 0.0;
        acadoVariables.od[i * NOD + 12] = 18.0*10;

        acadoVariables.od[i * NOD + 13] = 0.0;
        acadoVariables.od[i * NOD + 14] = 0.0;
        acadoVariables.od[i * NOD + 15] = 0.0;
        acadoVariables.od[i * NOD + 16] = 0.0;
        acadoVariables.od[i * NOD + 17] = 1;
        acadoVariables.od[i * NOD + 18] = 0.0;
        acadoVariables.od[i * NOD + 19] = -5;

        acadoVariables.od[i * NOD + 20] = 0.0;
        acadoVariables.od[i * NOD + 21] = 0.0;
        acadoVariables.od[i * NOD + 22] = 0.0;
        acadoVariables.od[i * NOD + 23] = 0.0;
        acadoVariables.od[i * NOD + 24] = 1;
        acadoVariables.od[i * NOD + 25] = 0.0;
        acadoVariables.od[i * NOD + 26] = -15;

        acadoVariables.od[i * NOD + 27] = 4.0*10;
        acadoVariables.od[i * NOD + 28] = 4.0*10;
        acadoVariables.od[i * NOD + 29] = 0.0;
        acadoVariables.od[i * NOD + 30] = 7.0*10;
        acadoVariables.od[i * NOD + 31] = 0.0;
        acadoVariables.od[i * NOD + 32] = 0.0;
        acadoVariables.od[i * NOD + 33] = 18.0*10;
        acadoVariables.od[i * NOD + 34] = -4.0*10;
        acadoVariables.od[i * NOD + 35] = 0.0;
        acadoVariables.od[i * NOD + 36] = 18.0*10;

        acadoVariables.od[i * NOD + 37] = -4.0*10;
        acadoVariables.od[i * NOD + 38] = 0.0;
        acadoVariables.od[i * NOD + 39] = 18.0*10;

        /*acadoVariables.od[i * NOD + 13] = -4.0*10;
        acadoVariables.od[i * NOD + 14] = 0.0;
        acadoVariables.od[i * NOD + 15] = 0;
        acadoVariables.od[i * NOD + 16] = 1;
        acadoVariables.od[i * NOD + 17] = 6;*/
    }
    
    /*//Curved
    for (int i = 0; i < N; i++)
    {
        acadoVariables.W[NY*NY*i + (NY+1)*0] = 1.0;     //x
        acadoVariables.W[NY*NY*i + (NY+1)*1] = 1.0;     //y
        acadoVariables.W[NY*NY*i + (NY+1)*2] = 0;          //v
        //acadoVariables.W[NY*NY*i + (NY+1)*2] = 500;  # Lane change
        //acadoVariables.W[NY*NY*i + (NY+1)*3] = 500;
        acadoVariables.W[NY*NY*i + (NY+1)*3] = 1*1e2;        //a
        acadoVariables.W[NY*NY*i + (NY+1)*4] = 1*1e2;        //j
        //acadoVariables.W[NY*NY*i + (NY+1)*5] = 0.0;         //lane_dist
        acadoVariables.W[NY*NY*i + (NY+1)*5] = 4.0*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*6] = 4.0*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*7] = 4.0*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*8] = 4.0*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*9] = 4.0*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*10] = 4.0*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*11] = 4.0*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*12] = 4.0*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*13] = 4.0*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*14] = 4.0*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*15] = 1.0;
        acadoVariables.W[NY*NY*i + (NY+1)*16] = 0.0;
    }

    acadoVariables.WN[(NYN+1)*0] = 0*2.5*1e3;
	acadoVariables.WN[(NYN+1)*1] = 0*2.5*1e3;
	acadoVariables.WN[(NYN+1)*2] = 0*5*1e5;*/

    
    /*//Combined
    for (int i = 0; i < N; i++)
    {
        acadoVariables.W[NY*NY*i + (NY+1)*0] = 0.0;     //x
        acadoVariables.W[NY*NY*i + (NY+1)*1] = 0.0;     //y
        acadoVariables.W[NY*NY*i + (NY+1)*2] = 0;          //v
        //acadoVariables.W[NY*NY*i + (NY+1)*2] = 500;  # Lane change
        //acadoVariables.W[NY*NY*i + (NY+1)*3] = 500;
        acadoVariables.W[NY*NY*i + (NY+1)*3] = 1*1e3;        //a
        acadoVariables.W[NY*NY*i + (NY+1)*4] = 1*1e5;        //j
        //acadoVariables.W[NY*NY*i + (NY+1)*5] = 0.0;         //lane_dist
        acadoVariables.W[NY*NY*i + (NY+1)*5] = 1*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*6] = 1*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*7] = 1*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*8] = 1*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*9] = 1*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*10] = 1*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*11] = 1*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*12] = 1*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*13] = 1*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*14] = 1*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*15] = 1.0;
        acadoVariables.W[NY*NY*i + (NY+1)*16] = 0.0;
    }*/

    /*
    //Behaviour Tests
    for (int i = 0; i < N; i++)
    {
        acadoVariables.W[NY*NY*i + (NY+1)*0] = 0.0;     //x
        acadoVariables.W[NY*NY*i + (NY+1)*1] = 0.0;     //y
        acadoVariables.W[NY*NY*i + (NY+1)*2] = 0;          //v
        //acadoVariables.W[NY*NY*i + (NY+1)*2] = 500;  # Lane change
        //acadoVariables.W[NY*NY*i + (NY+1)*3] = 500;
        acadoVariables.W[NY*NY*i + (NY+1)*3] = 1*1e3;        //a
        acadoVariables.W[NY*NY*i + (NY+1)*4] = 7*1e3;        //j
        //acadoVariables.W[NY*NY*i + (NY+1)*5] = 0.0;         //lane_dist
        acadoVariables.W[NY*NY*i + (NY+1)*5] = 8*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*6] = 8*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*7] = 8*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*8] = 8*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*9] = 8*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*10] = 8*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*11] = 8*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*12] = 8*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*13] = 8*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*14] = 8*1e3;
        acadoVariables.W[NY*NY*i + (NY+1)*15] = 1.0;
        acadoVariables.W[NY*NY*i + (NY+1)*16] = 0.0;
    }

    acadoVariables.WN[(NYN+1)*0] = 2.5*1e3;
	acadoVariables.WN[(NYN+1)*1] = 2.5*1e3;
	acadoVariables.WN[(NYN+1)*2] = 5*1e5;*/


    for (int i = 0; i < N; i++)
    {
        acadoVariables.W[NY*NY*i + (NY+1)*0] = 0.0;     //x
        acadoVariables.W[NY*NY*i + (NY+1)*1] = 0.0;     //y
        acadoVariables.W[NY*NY*i + (NY+1)*2] = 0;          //v
        //acadoVariables.W[NY*NY*i + (NY+1)*2] = 500;  # Lane change
        //acadoVariables.W[NY*NY*i + (NY+1)*3] = 500;
        acadoVariables.W[NY*NY*i + (NY+1)*3] = 1*1e4;        //a
        acadoVariables.W[NY*NY*i + (NY+1)*4] = 1.25*1e6;        //j
        //acadoVariables.W[NY*NY*i + (NY+1)*5] = 0.0;         //lane_dist
        acadoVariables.W[NY*NY*i + (NY+1)*5] = 1.1*1e6;
        acadoVariables.W[NY*NY*i + (NY+1)*6] = 1.1*1e6;
        acadoVariables.W[NY*NY*i + (NY+1)*7] = 1.1*1e6;
        acadoVariables.W[NY*NY*i + (NY+1)*8] = 1.1*1e6;
        acadoVariables.W[NY*NY*i + (NY+1)*9] = 1.1*1e6;
        acadoVariables.W[NY*NY*i + (NY+1)*10] = 1.1*1e6;
        acadoVariables.W[NY*NY*i + (NY+1)*11] = 1.1*1e6;
        acadoVariables.W[NY*NY*i + (NY+1)*12] = 1.1*1e6;
        acadoVariables.W[NY*NY*i + (NY+1)*13] = 1.1*1e6;
        acadoVariables.W[NY*NY*i + (NY+1)*14] = 1.1*1e6;
        acadoVariables.W[NY*NY*i + (NY+1)*15] = 1000.0;
        acadoVariables.W[NY*NY*i + (NY+1)*16] = 0.0;
    }

    acadoVariables.WN[(NYN+1)*0] = 5*1e9;
	acadoVariables.WN[(NYN+1)*1] = 5*1e9;
	acadoVariables.WN[(NYN+1)*2] = 1*1e10;


    for (int i = 0; i < N; i++)
    {
        //acadoVariables.lbAValues[i*4+0] = -1e12;    //x
        //acadoVariables.lbAValues[i*4+1] = 5.1;     //y
        acadoVariables.lbAValues[i*2+0] = 0;       //v
        acadoVariables.lbAValues[i*2+1] = -0.5;       //w

        //acadoVariables.ubAValues[i*4+0] = 1e12;     //x
        //acadoVariables.ubAValues[i*4+1] = 14.9;     //y
        acadoVariables.ubAValues[i*2+0] = 20;       //v
        acadoVariables.ubAValues[i*2+1] = 0.5;        //w
    }

    for (int i = 0; i < N; i++)
    {
        acadoVariables.lbAValues[i*2+N*2+0] = 0.0;
        acadoVariables.lbAValues[i*2+N*2+1] = -1e12;

        acadoVariables.ubAValues[i*2+N*2+0] = 1e12;
        acadoVariables.ubAValues[i*2+N*2+1] = 0.0;
    }

    /*for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            acadoVariables.lbAValues[i*5+N*4+j] = 2.5;
            acadoVariables.ubAValues[i*5+N*4+j] = 1e12;
        }
    }*/

    for (int i = 0; i < N; i++)
    {
        acadoVariables.lbValues[i*2+0] = -10;
        acadoVariables.lbValues[i*2+1] = -2;

        acadoVariables.ubValues[i*2+0] = 10;
        acadoVariables.ubValues[i*2+1] = 2;
    }


    /*for (int i = 0; i < N; i++)
    {
        RCLCPP_INFO(this->get_logger(), "LB 1 : %f\tLB 2 : %f", acadoWorkspace.lb[i*2+0], acadoWorkspace.lb[i*2+1]);
        RCLCPP_INFO(this->get_logger(), "UB 1 : %d\tUB 2 : %d", acadoWorkspace.ub[i*2+0], acadoWorkspace.ub[i*2+1]);
    }*/

    
    /* MPC: initialize the current state feedback. */

    acadoVariables.x0[ 0 ] = 0.0; // x
    acadoVariables.x0[ 1 ] = -6.0; // y
    acadoVariables.x0[ 2 ] = 0.0; // theta
    acadoVariables.x0[ 3 ] = 0.0; // lienar v
    acadoVariables.x0[ 4 ] = 0.0; // ang v

    outdata.open("crossing-10m.txt");

    RCLCPP_INFO(this->get_logger(), "Initialized");
    //acado_preparationStep();
    //timer_ = this->create_wall_timer(100ms, std::bind(&GoalReach::timer_goal_callback, this));
    //mpc_test();

}

void GoalReach::get_vel_cb(const std::shared_ptr<acado_msgs::srv::GetControlsMulti::Request> request,
          const std::shared_ptr<acado_msgs::srv::GetControlsMulti::Response>      response)
{
    geometry_msgs::msg::PoseArray path_val;
    RCLCPP_INFO(this->get_logger(), "Inside SRV CB %2f",request->goal.poses[0].position.z);
    if (1)
    {
        
        printf("%.2f\n", request->lane_cons.poses[2].position.y);
        printf("%.2f\n", request->lane_cons.poses[2].orientation.y);
        printf("%.2f\n", request->lane_cons.poses[2].orientation.w);
        RCLCPP_INFO(this->get_logger(), "cur = %.2f\n\n", request->lane_cons.poses[0].position.x);

        printf("%.2f\n", request->lane_cons.poses[3].position.y);
        printf("%.2f\n", request->lane_cons.poses[3].orientation.y);
        printf("%.2f\n", request->lane_cons.poses[3].orientation.w);
        RCLCPP_INFO(this->get_logger(), "cur = %.2f\n\n", request->lane_cons.poses[0].position.x);
    }
    
    start = request->start;
    obstacles = request->obstacles;
    lane_cons = request->lane_cons;
    acadoVariables.x0[ 0 ] = start.pose.pose.position.x; // x
    acadoVariables.x0[ 1 ] = start.pose.pose.position.y; // y
    acadoVariables.x0[ 2 ] = start.pose.pose.orientation.z; // theta
    acadoVariables.x0[ 3 ] = start.twist.twist.linear.x; // lienar v
    acadoVariables.x0[ 4 ] = start.twist.twist.angular.z; // ang v

    for(int goal_id = 0; goal_id<num_goals; goal_id++)
    {

        acadoVariables.yN[ 0 ] = request->goal.poses[goal_id].position.x; // xg
        acadoVariables.yN[ 1 ] = request->goal.poses[goal_id].position.y;;	// yg	
        acadoVariables.yN[ 2 ] = request->goal.poses[goal_id].orientation.z;	// yg	

        // acadoVariables.WN[(NYN+1)*0] = lane_cons.poses[5].position.x;
        // acadoVariables.WN[(NYN+1)*1] = lane_cons.poses[5].position.y;

        for (int i = 0; i < (N + 1); ++i)
        {
            if(ns[goal_id] == 5)
            {
                acadoVariables.x[ NX * i + 0 ] = multi_goal_state[goal_id][NX * i + 0]; // x
                acadoVariables.x[ NX * i + 1 ] = multi_goal_state[goal_id][NX * i + 1]; // y
                acadoVariables.x[ NX * i + 2 ] = multi_goal_state[goal_id][NX * i + 2]; // theta
                acadoVariables.x[ NX * i + 3 ] = multi_goal_state[goal_id][NX * i + 3]; // v 
                acadoVariables.x[ NX * i + 4 ] = multi_goal_state[goal_id][NX * i + 4]; // w

                acadoVariables.u[ NU * i + 0 ] = multi_goal_controls[goal_id][ NU * i + 0 ];
                acadoVariables.u[ NU * i + 1 ] = multi_goal_controls[goal_id][ NU * i + 1 ];

            }
            else
            {
                acadoVariables.x[ NX * i + 0 ] = start.pose.pose.position.x;
                acadoVariables.x[ NX * i + 1 ] = 14.0;
                acadoVariables.x[ NX * i + 2 ] = 0.0;
                acadoVariables.x[ NX * i + 3 ] = 15.0;
                acadoVariables.x[ NX * i + 4 ] = 0.0;

                acadoVariables.u[ NU * i + 0 ] = 0.0001;
                acadoVariables.u[ NU * i + 1 ] = 0.0001;
            }

            acadoVariables.y[ NY * i + 0 ] = request->goal.poses[goal_id].position.x; // xg
            acadoVariables.y[ NY * i + 1 ] = request->goal.poses[goal_id].position.y;	// yg	

            acadoVariables.od[i * NOD + 0] = obstacles.odom[0].pose.pose.position.x + obstacles.odom[0].twist.twist.linear.x*0.1*i;
            acadoVariables.od[i * NOD + 1] = obstacles.odom[0].pose.pose.position.y + obstacles.odom[0].twist.twist.linear.y*0.1*i;;
            //acadoVariables.od[i * NOD + 2] = theta0;

            acadoVariables.od[i * NOD + 2] = obstacles.odom[1].pose.pose.position.x + obstacles.odom[1].twist.twist.linear.x*0.1*i;
            acadoVariables.od[i * NOD + 3] = obstacles.odom[1].pose.pose.position.y + obstacles.odom[1].twist.twist.linear.y*0.1*i;
            //acadoVariables.od[i * NOD + 5] = theta1;

            acadoVariables.od[i * NOD + 4] = obstacles.odom[2].pose.pose.position.x + obstacles.odom[2].twist.twist.linear.x*0.1*i;
            acadoVariables.od[i * NOD + 5] = obstacles.odom[2].pose.pose.position.y + obstacles.odom[2].twist.twist.linear.y*0.1*i;
            //acadoVariables.od[i * NOD + 8] = theta2;

            acadoVariables.od[i * NOD + 6] = obstacles.odom[3].pose.pose.position.x + obstacles.odom[3].twist.twist.linear.x*0.1*i;
            acadoVariables.od[i * NOD + 7] = obstacles.odom[3].pose.pose.position.y + obstacles.odom[3].twist.twist.linear.y*0.1*i;
            //acadoVariables.od[i * NOD + 11] = theta3;

            acadoVariables.od[i * NOD + 8] = obstacles.odom[4].pose.pose.position.x + obstacles.odom[4].twist.twist.linear.x*0.1*i;
            acadoVariables.od[i * NOD + 9] = obstacles.odom[4].pose.pose.position.y + obstacles.odom[4].twist.twist.linear.y*0.1*i;
            //acadoVariables.od[i * NOD + 14] = theta4;

            acadoVariables.od[i * NOD + 10] = obstacles.odom[5].pose.pose.position.x + obstacles.odom[5].twist.twist.linear.x*0.1*i;
            acadoVariables.od[i * NOD + 11] = obstacles.odom[5].pose.pose.position.y + obstacles.odom[5].twist.twist.linear.y*0.1*i;

            acadoVariables.od[i * NOD + 12] = obstacles.odom[6].pose.pose.position.x + obstacles.odom[6].twist.twist.linear.x*0.1*i;
            acadoVariables.od[i * NOD + 13] = obstacles.odom[6].pose.pose.position.y + obstacles.odom[6].twist.twist.linear.y*0.1*i;

            acadoVariables.od[i * NOD + 14] = obstacles.odom[7].pose.pose.position.x + obstacles.odom[7].twist.twist.linear.x*0.1*i;
            acadoVariables.od[i * NOD + 15] = obstacles.odom[7].pose.pose.position.y + obstacles.odom[7].twist.twist.linear.y*0.1*i;

            acadoVariables.od[i * NOD + 16] = obstacles.odom[8].pose.pose.position.x + obstacles.odom[8].twist.twist.linear.x*0.1*i;
            acadoVariables.od[i * NOD + 17] = obstacles.odom[8].pose.pose.position.y + obstacles.odom[8].twist.twist.linear.y*0.1*i;

            acadoVariables.od[i * NOD + 18] = obstacles.odom[9].pose.pose.position.x + obstacles.odom[9].twist.twist.linear.x*0.1*i;
            acadoVariables.od[i * NOD + 19] = obstacles.odom[9].pose.pose.position.y + obstacles.odom[9].twist.twist.linear.y*0.1*i;

            if(i<N)
            {
                acadoVariables.lbValues[i*2+1] = lane_cons.poses[4].position.x;
                acadoVariables.ubValues[i*2+1] = lane_cons.poses[4].position.y;
                //acadoVariables.W[NY*NY*i + (NY+1)*3] = lane_cons.poses[6].position.x;        //a
                //acadoVariables.W[NY*NY*i + (NY+1)*4] = 7*1e3;        //j
            }

            if (i >= lane_cons.poses[0].position.x)
            {
                acadoVariables.od[i * NOD + 20] = lane_cons.poses[1].position.x;
                acadoVariables.od[i * NOD + 21] = lane_cons.poses[1].position.y;
                acadoVariables.od[i * NOD + 22] = -request->goal.poses[goal_id].position.y;;
                acadoVariables.od[i * NOD + 23] = lane_cons.poses[1].position.x;
                acadoVariables.od[i * NOD + 24] = lane_cons.poses[1].position.y;
                acadoVariables.od[i * NOD + 25] = -request->goal.poses[goal_id].position.y;;

                acadoVariables.od[i * NOD + 26] = lane_cons.poses[2].position.x;
                acadoVariables.od[i * NOD + 27] = lane_cons.poses[2].position.y;
                acadoVariables.od[i * NOD + 28] = lane_cons.poses[2].position.z;
                acadoVariables.od[i * NOD + 29] = lane_cons.poses[2].orientation.x;
                acadoVariables.od[i * NOD + 30] = lane_cons.poses[2].orientation.y;
                acadoVariables.od[i * NOD + 31] = lane_cons.poses[2].orientation.z;
                acadoVariables.od[i * NOD + 32] = lane_cons.poses[2].orientation.w;

                acadoVariables.od[i * NOD + 33] = lane_cons.poses[3].position.x;
                acadoVariables.od[i * NOD + 34] = lane_cons.poses[3].position.y;
                acadoVariables.od[i * NOD + 35] = lane_cons.poses[3].position.z;
                acadoVariables.od[i * NOD + 36] = lane_cons.poses[3].orientation.x;
                acadoVariables.od[i * NOD + 37] = lane_cons.poses[3].orientation.y;
                acadoVariables.od[i * NOD + 38] = lane_cons.poses[3].orientation.z;
                acadoVariables.od[i * NOD + 39] = lane_cons.poses[3].orientation.w;
                if(i==N)
                    continue;
                // if(lane_cons.poses[0].position.y == 0.0)
                // {
                //     acadoVariables.W[NY*NY*i + (NY+1)*15] = 500.0;
                //     acadoVariables.W[NY*NY*i + (NY+1)*16] = 0.0;
                // }
                // else
                // {
                //     acadoVariables.W[NY*NY*i + (NY+1)*15] = 0.0;
                //     acadoVariables.W[NY*NY*i + (NY+1)*16] = 50.0;
                // }
                

            }
                
        }
        acado_tic( &t );
        for(iter = 0; iter < ns[goal_id]; ++iter)
        {
        /* Perform the feedback step. */
            /*if(ns==1)
            {
                acado_shiftStates(2, 0, 0);
                acado_shiftControls( 0 );
            }*/
            acado_preparationStep();
            acado_feedbackStep( );
            //printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );
            RCLCPP_INFO(this->get_logger(), "Real-Time Iteration %d:  KKT Tolerance = %.3e, Num Iterations %d", iter, acado_getKKT() , ns[goal_id]);
            if(acado_getKKT()>1000000000000)
            {
                RCLCPP_INFO(this->get_logger(), "HIGH COST!!!!!!!!!!!!");
                ns[goal_id] = 5;
                //iter-=1;
                //exit(1);
            }
                
        }
        real_t te = acado_toc( &t );
        for (i = 0; i < (N + 1); ++i)  
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = acadoVariables.x[ NX * i + 0 ];
            pose.position.y = acadoVariables.x[ NX * i + 1 ];
            pose.position.z = acadoVariables.x[ NX * i + 2 ];
            pose.orientation.x = acadoVariables.x[ NX * i + 3 ];
            pose.orientation.y = acadoVariables.x[ NX * i + 4 ];
            path_val.poses.push_back(pose);
            if(acado_getKKT()<1e12)
            {
                multi_goal_state[goal_id][NX * i + 0] = acadoVariables.x[ NX * i + 0 ];
                multi_goal_state[goal_id][NX * i + 1] = acadoVariables.x[ NX * i + 1 ];
                multi_goal_state[goal_id][NX * i + 2] = acadoVariables.x[ NX * i + 2 ];
                multi_goal_state[goal_id][NX * i + 3] = acadoVariables.x[ NX * i + 3 ];
                multi_goal_state[goal_id][NX * i + 4] = acadoVariables.x[ NX * i + 4 ];

                multi_goal_controls[goal_id][ NU * i + 0 ] = acadoVariables.u[ NU * i + 0 ];
                multi_goal_controls[goal_id][ NU * i + 1 ] = acadoVariables.u[ NU * i + 1 ];
            }
            //RCLCPP_INFO(this->get_logger(), "ACC_lin: %.2f, ACC_ang: %.2f", acadoVariables.u[ NU * i + 0], acadoVariables.x[ NU * i + 1 ]);
        }
        if(ns[goal_id]==5)
        //if(ni == 1)
        {
            mpc_iter+=1;
            time+=te;
            time_arr.push_back(te);
            RCLCPP_INFO(this->get_logger(), "Min: %.5f, Max: %.5f, Avg: %.5f",*min_element(time_arr.begin(), time_arr.end()),
                        *max_element(time_arr.begin(), time_arr.end()), time/mpc_iter);
        }
        response->kkt.push_back(acado_getKKT());
        if(acado_getKKT()<1e12)
            ns[goal_id] = 5;
    }
    ni = 2;
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = acadoVariables.x[NX + 3];
    cmd_vel.angular.z = acadoVariables.x[NX + 4];
        //outdata << acadoVariables.x[NX + 0] << " " << acadoVariables.x[NX + 1] << " " << acadoVariables.x[NX + 2] << " " << acadoVariables.x[NX + 3] << " " << acadoVariables.x[NX + 4] << " " << acadoVariables.u[NU + 0] << " " << acadoVariables.u[NU + 1]<< " " << te << " " << mpc_iter <<" " <<obstacles.odom[0].pose.pose.position.x << " " <<obstacles.odom[0].pose.pose.position.y << " " <<obstacles.odom[1].pose.pose.position.x << " " <<obstacles.odom[1].pose.pose.position.y <<" " <<obstacles.odom[2].pose.pose.position.x << " " <<obstacles.odom[2].pose.pose.position.y <<" " <<obstacles.odom[3].pose.pose.position.x <<" " << obstacles.odom[3].pose.pose.position.y <<" " <<obstacles.odom[4].pose.pose.position.x << " " <<obstacles.odom[4].pose.pose.position.y <<" " <<obstacles.odom[5].pose.pose.position.x <<" " << obstacles.odom[5].pose.pose.position.y <<" " <<obstacles.odom[6].pose.pose.position.x << " " <<obstacles.odom[6].pose.pose.position.y <<" " <<obstacles.odom[7].pose.pose.position.x << " " <<obstacles.odom[7].pose.pose.position.y <<" " <<obstacles.odom[8].pose.pose.position.x << " " <<obstacles.odom[8].pose.pose.position.y <<" " <<obstacles.odom[9].pose.pose.position.x <<" " << obstacles.odom[9].pose.pose.position.y <<" "<<lane_cons.poses[7].position.x<<std::endl;
    /*acadoVariables.x0[ 0 ] = acadoVariables.x[NX + 0]; // x*/
    acadoVariables.x0[ 1 ] = acadoVariables.x[NX + 1]; // y
    acadoVariables.x0[ 2 ] = acadoVariables.x[NX + 2]; // theta
    acadoVariables.x0[ 3 ] = acadoVariables.x[NX + 3]; // lienar v
    acadoVariables.x0[ 4 ] = acadoVariables.x[NX + 4]; // ang v*/
    double dist = sqrt(pow(acadoVariables.x0[ 0 ]-acadoVariables.y[ 0 ], 2) + pow(acadoVariables.x0[ 1 ]-acadoVariables.y[ 1 ], 2));
    //check_term_dist(dist);


    RCLCPP_INFO(this->get_logger(), "X: %.2f, Y: %.2f, Iter: %d", path_val.poses[0].position.x, path_val.poses[0].position.y, path_val.poses.size());
    //RCLCPP_INFO(this->get_logger(), "%.2f, %.2f, %.2f, %.2f",
    //                            request->goal.poses[goal_id].position.x, request->goal.poses[goal_id].position.y, start.pose.pose.position.x,
    //                            start.pose.pose.position.y);
    //RCLCPP_INFO(this->get_logger(), "Line Vel: %.2f, Ang Vel: %.2f, Dist = %.2f", acadoVariables.x[NX + 3], acadoVariables.x[NX + 4], dist);
    //RCLCPP_INFO(this->get_logger(), "Goal X: %.2f, Goal Y: %.2f", acadoVariables.yN[ 0 ], acadoVariables.yN[ 1 ]);
    //RCLCPP_INFO(this->get_logger(), "Line acc: %.2f, Ang acc: %.2f", acadoVariables.u[NU + 0], acadoVariables.x[NU + 1]);
    //acadoVariables.y[NY * + 2] = acadoVariables.x[NX + 3];
    //acadoVariables.y[NY * + 3] = acadoVariables.x[NX + 4];
    response->twist = cmd_vel;
    response->path = path_val;
}

void GoalReach::check_term_dist(double dist)
{
    if(dist<30)
    {
        for (int i = 0; i < N; i++)
        {
            acadoVariables.W[NY*NY*i + (NY+1)*0] = 1;
            acadoVariables.W[NY*NY*i + (NY+1)*1] = 1;
            acadoVariables.W[NY*NY*i + (NY+1)*2] = 10;
            //acadoVariables.W[NY*NY*i + (NY+1)*3] = 100/10;
            //acadoVariables.W[NY*NY*i + (NY+1)*4] = 100/10;
        }
        for (i = 0; i < N; ++i)  
        {
            acadoVariables.y[ NY * i + 2 ] = 0.0;	// theta	
        }
        acadoVariables.WN[(NYN+1)*0] = 10;
        acadoVariables.WN[(NYN+1)*1] = 10;
        acadoVariables.WN[(NYN+1)*2] = 1000;

    }
    /*else
    {
        for (int i = 0; i < N; i++)
        {
            acadoVariables.W[NY*NY*i + (NY+1)*0] = 50;
            acadoVariables.W[NY*NY*i + (NY+1)*1] = 50;
            acadoVariables.W[NY*NY*i + (NY+1)*2] = 50;
        }
        for (i = 0; i < N; ++i)  
        {
            acadoVariables.y[ NY * i + 2 ] = 0.0;	// theta	
        }
    }*/
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalReach>());
  rclcpp::shutdown();
  return 0;
}