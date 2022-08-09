#include <memory>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "acado_msgs/srv/get_velocity_cmd.hpp"
#include "acado_msgs/srv/get_controls.hpp"
#include "acado_msgs/srv/get_controls_multi.hpp"
#include "acado_msgs/msg/odom_array.hpp"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <thread>
#include <eigen3/Eigen/Dense>
//#include "test_car/acado_solver.c"
//#include "test_car/acado_qpoases_interface.hpp"
//#include "test_car/acado_integrator.c"



#include <stdio.h>

using namespace std::chrono_literals;
using namespace Eigen;
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
    void optimize(int info);
    void get_ranks();
    nav_msgs::msg::Odometry start;
    geometry_msgs::msg::PoseArray lane_cons;
    acado_msgs::msg::OdomArray obstacles;
    bool got_odom;
    int ns;
    bool got_pa;
    int    i, iter, num_goals;
    bool got_request;
    double time;
    int mpc_iter;
    acado_timer t;
    rclcpp::Service<acado_msgs::srv::GetControlsMulti>::SharedPtr service;
    size_t count_;
    std::vector<double> time_arr;
    std::ofstream outdata;
    std::vector<double> xg; 
    std::vector<double> yg;
    std::vector<double> thetag;
    //std::vector<double> x_path; 
    //std::vector<double> y_path; 
    //std::vector<double> v_path; 
    //std::vector<double> w_path; 
    ArrayXXf x_path, y_path, v_path, w_path, x_obs_pose, y_obs_pose, obs_lin_vel, obs_ang_vel, running, meta_cost, batch_optimal;
    float prev_v_send, prev_w_send, a_obs, b_obs, w0, w1, w2, w3, w4;
};

GoalReach::GoalReach(): Node("acado_circle_lane_srv"), count_(0)
{
    service = this->create_service<acado_msgs::srv::GetControlsMulti>("/get_vel", std::bind(&GoalReach::get_vel_cb, this, _1, _2));
    got_odom = false;
    got_pa = false;
    ns = 5;
    time = 0.0;
    mpc_iter = 0;

    num_goals = 1;

    xg.push_back(0);
    xg.push_back(0);
    yg.push_back(0);
    yg.push_back(0);
    thetag.push_back(0);
    thetag.push_back(0);

    x_path = ArrayXXf(100, num_goals);
    y_path = ArrayXXf(100, num_goals);
    v_path = ArrayXXf(100, num_goals);
    w_path = ArrayXXf(100, num_goals);
    x_obs_pose = ArrayXXf(10, 1);
    y_obs_pose = ArrayXXf(10, 1);
    obs_lin_vel = ArrayXXf(10, 1);
    obs_ang_vel = ArrayXXf(10, 1);
    running = ArrayXXf(num_goals, 1);
    meta_cost = ArrayXXf(num_goals, 9);
    batch_optimal = ArrayXXf(num_goals, 1);

    x_obs_pose = 0;
    y_obs_pose = 0;
    obs_lin_vel = 0;
    obs_ang_vel = 0;
    running = 1;
    got_request = false;

    for (i = 0; i < (N + 1); ++i)  
    {
        acadoVariables.x[ NX * i + 0 ] = 0.0; // x
        acadoVariables.x[ NX * i + 1 ] = 14.0; // y
        acadoVariables.x[ NX * i + 2 ] = 0.0; // theta
        acadoVariables.x[ NX * i + 3 ] = 15.0; // v 
        acadoVariables.x[ NX * i + 4 ] = 0.0; // w
    }
    for (i = 0; i < N; ++i)  
    {
        acadoVariables.u[ NU * i + 0 ] = 0.0001;
        acadoVariables.u[ NU * i + 1 ] = 0.0001;
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
	acadoVariables.WN[(NYN+1)*2] = 5*1e5;


    /*for (int i = 0; i < N; i++)
    {
        acadoVariables.W[NY*NY*i + (NY+1)*0] = 0.0;     //x
        acadoVariables.W[NY*NY*i + (NY+1)*1] = 0.0;     //y
        acadoVariables.W[NY*NY*i + (NY+1)*2] = 0;          //v
        //acadoVariables.W[NY*NY*i + (NY+1)*2] = 500;  # Lane change
        //acadoVariables.W[NY*NY*i + (NY+1)*3] = 500;
        acadoVariables.W[NY*NY*i + (NY+1)*3] = 1*1e3;        //a
        acadoVariables.W[NY*NY*i + (NY+1)*4] = 1*1e5;        //j
        //acadoVariables.W[NY*NY*i + (NY+1)*5] = 0.0;         //lane_dist
        acadoVariables.W[NY*NY*i + (NY+1)*5] = 2*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*6] = 2*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*7] = 2*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*8] = 2*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*9] = 2*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*10] = 2*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*11] = 2*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*12] = 2*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*13] = 2*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*14] = 2*1e4;
        acadoVariables.W[NY*NY*i + (NY+1)*15] = 1.0;
        acadoVariables.W[NY*NY*i + (NY+1)*16] = 0.0;
    }

    acadoVariables.WN[(NYN+1)*0] = 2.5*1e3;
	acadoVariables.WN[(NYN+1)*1] = 2.5*1e3;
	acadoVariables.WN[(NYN+1)*2] = 5*1e5;*/


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

    std::thread Thread[num_goals];	
    for(int i = 0; i < num_goals; i++)
        Thread[i] = std::thread(&GoalReach::optimize, this, i);
    for(int i = 0; i < num_goals; i++)
        Thread[i].detach();

    RCLCPP_INFO(this->get_logger(), "Initialized");
    //acado_preparationStep();
    //timer_ = this->create_wall_timer(100ms, std::bind(&GoalReach::timer_goal_callback, this));
    //mpc_test();

}

void GoalReach::get_ranks()
{
    //printf("inside get ranks\n");
    float v_cruise = 15.0;
    for(int i = 0; i < num_goals; i++)
    {
        meta_cost(i, 1) = (v_path.col(i) - v_cruise).matrix().lpNorm<2>(); // 5
        meta_cost(i, 2) = batch_optimal.row(i).matrix().lpNorm<2>();        // 6
        meta_cost(i, 3) = (y_path.col(i) -  (-10)).matrix().lpNorm<2>(); // 7
        meta_cost(i, 4) = (v_path.col(i) - 24).matrix().lpNorm<2>();                       // 8
        meta_cost(i, 5) = -1;
        meta_cost(i, 6) = -1;
        meta_cost(i, 7) = -1;
        meta_cost(i, 8) = -1;
    }
    //printf("gr1\n");
    float inf = std::numeric_limits<float>::infinity();
    for(int i = 0; i < num_goals; i++)
    {
        //printf("gr2\n");
        float min0 = inf, min1 = inf, min2 = inf, min3 = inf; 
        int index0 = -1, index1 = -1, index2 = -1, index3 = -1;
        for(int j = 0; j < num_goals; j++)
        {
            if(meta_cost(j, 1) < min0 && meta_cost(j, 5) < 0)
            {
                min0 = meta_cost(j, 1);
                index0 = j;
            }
            if(meta_cost(j, 2) < min1 && meta_cost(j, 6) < 0)
            {
                min1 = meta_cost(j, 2);
                index1 = j;
            }
            if(meta_cost(j, 3) < min2 && meta_cost(j, 7) < 0)
            {
                min2 = meta_cost(j, 3);
                index2 = j;
            }
            if(meta_cost(j, 4) < min3 && meta_cost(j, 8) < 0)
            {
                min3 = meta_cost(j, 4);
                index3 = j;
            }
        }
        //printf("gr2 done\n");
        //printf("%i, %i, %i, %i, %i, \n", index0, index1, index2, index3, i);
        meta_cost(index0, 5) = i+1; // cruise     
        meta_cost(index1, 6) = i+1; // optimal
        meta_cost(index2, 7) = i+1; // rightmost lane
        meta_cost(index3, 8) = i+1; // max average velocity
        //printf("gr2 done2\n");
    }
    //printf("get ranks done\n");
}

void GoalReach::optimize(int info)
{
    ns = 5;
    //printf("smfhsadhvc  %i", info);
    acado_initializeSolver();
    while(1)
	{
        if(!got_request)
            continue;
        if(!running(info))
            continue;
        for (i = 0; i < (N + 1); ++i)  
        {
            acadoVariables.x[ i*NX + 0 ] = acadoVariables.x0[ 0 ];	//x 
            acadoVariables.x[ i*NX + 1 ] = acadoVariables.x0[ 1 ];	//y
            acadoVariables.x[ i*NX + 2 ] = acadoVariables.x0[ 2 ];	//theta
            acadoVariables.x[ i*NX + 3 ] = acadoVariables.x0[ 3 ];	//v
            acadoVariables.x[ i*NX + 4 ] = acadoVariables.x0[ 4 ];	//w
        }
        for (i = 0; i < N; ++i)  
        {
            acadoVariables.u[ NU * i + 0 ] = 0.0001;
            acadoVariables.u[ NU * i + 1 ] = 0.0001;
        }
        /* Initialize the measurements/reference. */
        for (i = 0; i < N; ++i)  
        {
            acadoVariables.y[ NY * i + 0 ] = xg[info];//request->goal.position.x; // xg
            acadoVariables.y[ NY * i + 1 ] = yg[info];//request->goal.position.y;	// yg
        }

        acadoVariables.yN[ 0 ] = xg[info];//request->goal.position.x; // xg
        acadoVariables.yN[ 1 ] = yg[info];//request->goal.position.y;;	// yg	
        acadoVariables.yN[ 2 ] = thetag[info];//request->goal.orientation.z;	// yg	
        
        for (int i = 0; i < (N + 1); ++i)
        {
            double theta0 = obstacles.odom[0].pose.pose.orientation.z + obstacles.odom[0].twist.twist.angular.z*0.1*i;
            acadoVariables.od[i * NOD + 0] = obstacles.odom[0].pose.pose.position.x + obstacles.odom[0].twist.twist.linear.x*cos(theta0)*0.1*i;
            acadoVariables.od[i * NOD + 1] = obstacles.odom[0].pose.pose.position.y + obstacles.odom[0].twist.twist.linear.x*sin(theta0)*0.1*i;;
            //acadoVariables.od[i * NOD + 2] = theta0;

            double theta1 = obstacles.odom[1].pose.pose.orientation.z + obstacles.odom[1].twist.twist.angular.z*0.1*i;
            acadoVariables.od[i * NOD + 2] = obstacles.odom[1].pose.pose.position.x + obstacles.odom[1].twist.twist.linear.x*cos(theta1)*0.1*i;
            acadoVariables.od[i * NOD + 3] = obstacles.odom[1].pose.pose.position.y + obstacles.odom[1].twist.twist.linear.x*sin(theta1)*0.1*i;
            //acadoVariables.od[i * NOD + 5] = theta1;

            double theta2 = obstacles.odom[2].pose.pose.orientation.z + obstacles.odom[2].twist.twist.angular.z*0.1*i;
            acadoVariables.od[i * NOD + 4] = obstacles.odom[2].pose.pose.position.x + obstacles.odom[2].twist.twist.linear.x*cos(theta2)*0.1*i;
            acadoVariables.od[i * NOD + 5] = obstacles.odom[2].pose.pose.position.y + obstacles.odom[2].twist.twist.linear.x*sin(theta2)*0.1*i;\
            //acadoVariables.od[i * NOD + 8] = theta2;

            double theta3 = obstacles.odom[3].pose.pose.orientation.z + obstacles.odom[3].twist.twist.angular.z*0.1*i;
            acadoVariables.od[i * NOD + 6] = obstacles.odom[3].pose.pose.position.x + obstacles.odom[3].twist.twist.linear.x*cos(theta3)*0.1*i;
            acadoVariables.od[i * NOD + 7] = obstacles.odom[3].pose.pose.position.y + obstacles.odom[3].twist.twist.linear.x*sin(theta3)*0.1*i;
            //acadoVariables.od[i * NOD + 11] = theta3;

            double theta4 = obstacles.odom[4].pose.pose.orientation.z + obstacles.odom[4].twist.twist.angular.z*0.1*i;
            acadoVariables.od[i * NOD + 8] = obstacles.odom[4].pose.pose.position.x + obstacles.odom[4].twist.twist.linear.x*cos(theta4)*0.1*i;
            acadoVariables.od[i * NOD + 9] = obstacles.odom[4].pose.pose.position.y + obstacles.odom[4].twist.twist.linear.x*sin(theta4)*0.1*i;
            //acadoVariables.od[i * NOD + 14] = theta4;

            double theta5 = obstacles.odom[5].pose.pose.orientation.z + obstacles.odom[5].twist.twist.angular.z*0.1*i;
            acadoVariables.od[i * NOD + 10] = obstacles.odom[5].pose.pose.position.x + obstacles.odom[5].twist.twist.linear.x*cos(theta5)*0.1*i;
            acadoVariables.od[i * NOD + 11] = obstacles.odom[5].pose.pose.position.y + obstacles.odom[5].twist.twist.linear.x*sin(theta5)*0.1*i;

            double theta6 = obstacles.odom[6].pose.pose.orientation.z + obstacles.odom[6].twist.twist.angular.z*0.1*i;
            acadoVariables.od[i * NOD + 12] = obstacles.odom[6].pose.pose.position.x + obstacles.odom[6].twist.twist.linear.x*cos(theta6)*0.1*i;
            acadoVariables.od[i * NOD + 13] = obstacles.odom[6].pose.pose.position.y + obstacles.odom[6].twist.twist.linear.x*sin(theta6)*0.1*i;

            double theta7 = obstacles.odom[7].pose.pose.orientation.z + obstacles.odom[7].twist.twist.angular.z*0.1*i;
            acadoVariables.od[i * NOD + 14] = obstacles.odom[7].pose.pose.position.x + obstacles.odom[7].twist.twist.linear.x*cos(theta7)*0.1*i;
            acadoVariables.od[i * NOD + 15] = obstacles.odom[7].pose.pose.position.y + obstacles.odom[7].twist.twist.linear.x*sin(theta7)*0.1*i;

            double theta8 = obstacles.odom[8].pose.pose.orientation.z + obstacles.odom[8].twist.twist.angular.z*0.1*i;
            acadoVariables.od[i * NOD + 16] = obstacles.odom[8].pose.pose.position.x + obstacles.odom[8].twist.twist.linear.x*cos(theta8)*0.1*i;
            acadoVariables.od[i * NOD + 17] = obstacles.odom[8].pose.pose.position.y + obstacles.odom[8].twist.twist.linear.x*sin(theta8)*0.1*i;

            double theta9 = obstacles.odom[9].pose.pose.orientation.z + obstacles.odom[9].twist.twist.angular.z*0.1*i;
            acadoVariables.od[i * NOD + 18] = obstacles.odom[9].pose.pose.position.x + obstacles.odom[9].twist.twist.linear.x*cos(theta9)*0.1*i;
            acadoVariables.od[i * NOD + 19] = obstacles.odom[9].pose.pose.position.y + obstacles.odom[9].twist.twist.linear.x*sin(theta9)*0.1*i;

            acadoVariables.od[i * NOD + 20] = lane_cons.poses[1].position.x;
            acadoVariables.od[i * NOD + 21] = lane_cons.poses[1].position.y;
            acadoVariables.od[i * NOD + 22] = lane_cons.poses[1].position.z;
            acadoVariables.od[i * NOD + 23] = lane_cons.poses[1].position.x;
            acadoVariables.od[i * NOD + 24] = lane_cons.poses[1].position.y;
            acadoVariables.od[i * NOD + 25] = lane_cons.poses[1].position.z;

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
        }
        
        /*//Curved
        for (int i = 0; i < N; i++)
        {
            acadoVariables.W[NY*NY*i + (NY+1)*3] = lane_cons.poses[6].position.x;;        //a
            if(lane_cons.poses[0].position.y == 0.0)
            {
                acadoVariables.W[NY*NY*i + (NY+1)*15] = 1.0;
                acadoVariables.W[NY*NY*i + (NY+1)*16] = 0.0;
            }
            else
            {
                acadoVariables.W[NY*NY*i + (NY+1)*15] = 0.0;
                acadoVariables.W[NY*NY*i + (NY+1)*16] = 50.0;
            }
        }

        acadoVariables.WN[(NYN+1)*0] = 0*2.5*1e3;
        acadoVariables.WN[(NYN+1)*1] = 0*2.5*1e3;
        acadoVariables.WN[(NYN+1)*2] = 0*5*1e5;*/

        
        /*//Combined
        for (int i = 0; i < N; i++)
        {
            acadoVariables.W[NY*NY*i + (NY+1)*3] = lane_cons.poses[6].position.x;
            if(lane_cons.poses[0].position.y == 0.0)
            {
                acadoVariables.W[NY*NY*i + (NY+1)*15] = 1.0;
                acadoVariables.W[NY*NY*i + (NY+1)*16] = 0.0;
            }
            else
            {
                acadoVariables.W[NY*NY*i + (NY+1)*15] = 0.0;
                acadoVariables.W[NY*NY*i + (NY+1)*16] = 50.0;
            }
        }*/

        
        //Behaviour Tests
        for (int i = 0; i < N; i++)
        {
            acadoVariables.W[NY*NY*i + (NY+1)*3] = lane_cons.poses[6].position.x;
            if(lane_cons.poses[0].position.y == 0.0)
            {
                acadoVariables.W[NY*NY*i + (NY+1)*15] = 1.0;
                acadoVariables.W[NY*NY*i + (NY+1)*16] = 0.0;
            }
            else
            {
                acadoVariables.W[NY*NY*i + (NY+1)*15] = 0.0;
                acadoVariables.W[NY*NY*i + (NY+1)*16] = 50.0;
            }
        }

        acadoVariables.WN[(NYN+1)*0] = lane_cons.poses[5].position.x;
        acadoVariables.WN[(NYN+1)*1] = lane_cons.poses[5].position.y;
        acadoVariables.WN[(NYN+1)*2] = 5*1e5;


        /*for (int i = 0; i < N; i++)
        {
            acadoVariables.W[NY*NY*i + (NY+1)*3] = lane_cons.poses[6].position.x;;        //a
            if(lane_cons.poses[0].position.y == 0.0)
            {
                acadoVariables.W[NY*NY*i + (NY+1)*15] = 1.0;
                acadoVariables.W[NY*NY*i + (NY+1)*16] = 0.0;
            }
            else
            {
                acadoVariables.W[NY*NY*i + (NY+1)*15] = 0.0;
                acadoVariables.W[NY*NY*i + (NY+1)*16] = 50.0;
            }
        }

        acadoVariables.WN[(NYN+1)*0] = 2.5*1e3;
        acadoVariables.WN[(NYN+1)*1] = 2.5*1e3;
        acadoVariables.WN[(NYN+1)*2] = 5*1e5;*/


        for (int i = 0; i < N; i++)
        {
            acadoVariables.lbValues[i*2+1] = lane_cons.poses[4].position.x;

            acadoVariables.ubValues[i*2+1] = lane_cons.poses[4].position.y;
        }

        acadoVariables.x0[ 0 ] = start.pose.pose.position.x; // x
        acadoVariables.x0[ 1 ] = start.pose.pose.position.y; // y
        acadoVariables.x0[ 2 ] = start.pose.pose.orientation.z; // theta
        acadoVariables.x0[ 3 ] = start.twist.twist.linear.x; // lienar v
        acadoVariables.x0[ 4 ] = start.twist.twist.angular.z; // ang v


        acado_tic( &t );
        for(iter = 0; iter < ns; ++iter)
        {
            acado_preparationStep();
            acado_feedbackStep( );
            RCLCPP_INFO(this->get_logger(), "Real-Time Iteration %d:  KKT Tolerance = %.3e", iter, acado_getKKT() );
            if(acado_getKKT()>1000000000000)
            {
                RCLCPP_INFO(this->get_logger(), "HIGH COST!!!!!!!!!!!!");   
                exit(1);
            }
                
        }
        real_t te = acado_toc( &t );
        for (int i = 0; i < (N + 1); ++i)  
        {
            x_path(i, info) = acadoVariables.x[ NX * i + 0 ];
            y_path(i, info) = acadoVariables.x[ NX * i + 1 ];
            v_path(i, info) = acadoVariables.x[ NX * i + 3 ];
            w_path(i, info) = acadoVariables.x[ NX * i + 4 ];
        }

        running(info) = 0;
        ns = 1;
        batch_optimal(info) = acado_getKKT();
    }
}

void GoalReach::get_vel_cb(const std::shared_ptr<acado_msgs::srv::GetControlsMulti::Request> request,
          const std::shared_ptr<acado_msgs::srv::GetControlsMulti::Response>      response)
{
    //RCLCPP_INFO(this->get_logger(), "Inside SRV CB %2f",request->goal.position.z);
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
        //printf("0\n");
    }
    //printf("00\n");
    start = request->start;
    //printf("1\n");
    obstacles = request->obstacles;
    //printf("2\n");
    lane_cons = request->lane_cons;
    //printf("3\n");
    got_request = true;
    xg[0] = request->goal.poses[0].position.x;
    xg[1] = request->goal.poses[1].position.x;
    //printf("4\n");
    yg[0] = request->goal.poses[0].position.y;
    yg[1] = request->goal.poses[1].position.y;
    //printf("5\n");
    //printf("55\n");
    thetag[0] = request->goal.poses[0].orientation.z;
    //printf("5.0\n");
    thetag[1] = request->goal.poses[1].orientation.z;
    //printf("6\n");

    int done = 0;
    while(!done)
    {
        //printf("done = %i", done);
        for(int i=0; i<num_goals; i++)
        {
            if(running(i))
            {
                done = 0;
                break;
            }
            else
                done = 1;
        }
    }
    //printf("done1\n");
    int index = 0;
    get_ranks();
    //printf("done2\n");
    float min = 100000000;
    for(int i = 0; i < num_goals; i++)
    {
        float cost = w0 * meta_cost(i, 5) + w1 * meta_cost(i, 6) + w2 * meta_cost(i, 7) + w3 * meta_cost(i, 8); 
                            // cruise                  optimal                   rightlane             max avg velocity
        if( cost < min)
        {
            min = cost; 
            index = i;
        }    
    }

    geometry_msgs::msg::PoseArray path_val;
    for(int i = 0; i < num_goals; i++)
    {
        for(int j = 0; j < 100; j++)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = x_path(i, j);
            pose.position.y = y_path(i, j);
            path_val.poses.push_back(pose);  
        }
    }
    real_t te = acado_toc( &t );
    mpc_iter+=1;
    time+=te;
    time_arr.push_back(te);
    RCLCPP_INFO(this->get_logger(), "Min: %.5f, Max: %.5f, Avg: %.5f",*min_element(time_arr.begin(), time_arr.end()),
                *max_element(time_arr.begin(), time_arr.end()), time/mpc_iter);
    ns = 1;
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v_path(0, index);
    cmd_vel.angular.z = w_path(0, index);
        outdata << acadoVariables.x[NX + 0] << " " << acadoVariables.x[NX + 1] << " " << acadoVariables.x[NX + 2] << " " << acadoVariables.x[NX + 3] << " " << acadoVariables.x[NX + 4] << " " << acadoVariables.u[NU + 0] << " " << acadoVariables.u[NU + 1]<< " " << te << " " << mpc_iter <<" " <<obstacles.odom[0].pose.pose.position.x << " " <<obstacles.odom[0].pose.pose.position.y << " " <<obstacles.odom[1].pose.pose.position.x << " " <<obstacles.odom[1].pose.pose.position.y <<" " <<obstacles.odom[2].pose.pose.position.x << " " <<obstacles.odom[2].pose.pose.position.y <<" " <<obstacles.odom[3].pose.pose.position.x <<" " << obstacles.odom[3].pose.pose.position.y <<" " <<obstacles.odom[4].pose.pose.position.x << " " <<obstacles.odom[4].pose.pose.position.y <<" " <<obstacles.odom[5].pose.pose.position.x <<" " << obstacles.odom[5].pose.pose.position.y <<" " <<obstacles.odom[6].pose.pose.position.x << " " <<obstacles.odom[6].pose.pose.position.y <<" " <<obstacles.odom[7].pose.pose.position.x << " " <<obstacles.odom[7].pose.pose.position.y <<" " <<obstacles.odom[8].pose.pose.position.x << " " <<obstacles.odom[8].pose.pose.position.y <<" " <<obstacles.odom[9].pose.pose.position.x <<" " << obstacles.odom[9].pose.pose.position.y <<" "<<lane_cons.poses[7].position.x<<std::endl;
    /*acadoVariables.x0[ 0 ] = acadoVariables.x[NX + 0]; // x*/
    acadoVariables.x0[ 1 ] = acadoVariables.x[NX + 1]; // y
    acadoVariables.x0[ 2 ] = acadoVariables.x[NX + 2]; // theta
    acadoVariables.x0[ 3 ] = acadoVariables.x[NX + 3]; // lienar v
    acadoVariables.x0[ 4 ] = acadoVariables.x[NX + 4]; // ang v*/
    double dist = sqrt(pow(acadoVariables.x0[ 0 ]-acadoVariables.y[ 0 ], 2) + pow(acadoVariables.x0[ 1 ]-acadoVariables.y[ 1 ], 2));
    //check_term_dist(dist);


    RCLCPP_INFO(this->get_logger(), "X: %.2f, Y: %.2f, Iter: %.5f", path_val.poses[0].position.x, path_val.poses[0].position.y, mpc_iter);
    running = 1;
    got_request = false;
    response->twist = cmd_vel;
    response->path = path_val;
    response->index = index;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalReach>());
  rclcpp::shutdown();
  return 0;
}