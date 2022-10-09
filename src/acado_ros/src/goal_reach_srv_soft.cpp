#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "acado_msgs/srv/get_velocity_cmd.hpp"
#include "acado_msgs/msg/odom_array.hpp"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
//#include "test_car/acado_solver.c"
//#include "test_car/acado_qpoases_interface.hpp"
//#include "test_car/acado_integrator.c"



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
extern "C"{
__thread ACADOvariables acadoVariables;
__thread ACADOworkspace acadoWorkspace;
}

class GoalReach : public rclcpp::Node
{
  public:
    GoalReach();
  private:
    void get_vel_cb(const std::shared_ptr<acado_msgs::srv::GetVelocityCmd::Request> request,
          const std::shared_ptr<acado_msgs::srv::GetVelocityCmd::Response>      response);
    void check_term_dist(double dist);
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::PoseArray pa;
    acado_msgs::msg::OdomArray odom_arr;
    bool got_odom;
    int ns;
    bool got_pa;
    int    i, iter;
    double time;
    int mpc_iter;
    acado_timer t;
    rclcpp::Service<acado_msgs::srv::GetVelocityCmd>::SharedPtr service;
    size_t count_;
};

GoalReach::GoalReach(): Node("acado_goal_reach_srv"), count_(0)
{
    service = this->create_service<acado_msgs::srv::GetVelocityCmd>("/get_vel", std::bind(&GoalReach::get_vel_cb, this, _1, _2));
    got_odom = false;
    got_pa = false;
    ns = 5;
    time = 0.0;
    mpc_iter = 0;

    acado_initializeSolver();

    for (i = 0; i < (N + 1); ++i)  
    {
        acadoVariables.x[ NX * i + 0 ] = 0.0; // x
        acadoVariables.x[ NX * i + 1 ] = -6.0; // y
        acadoVariables.x[ NX * i + 2 ] = 0.0; // theta
        acadoVariables.x[ NX * i + 3 ] = 0.0; // v 
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
        acadoVariables.y[ NY * i + 0 ] = 200; // xg
        acadoVariables.y[ NY * i + 1 ] = -6.0;	// yg	
        acadoVariables.y[ NY * i + 2 ] = 15.0;	// v	
        acadoVariables.y[ NY * i + 3 ] = 0.0;	// linear acc
        acadoVariables.y[ NY * i + 4 ] = 0.0; 	//  anuglar acc
        acadoVariables.y[ NY * i + 5 ] = 0.0; 	//  dist to line
        acadoVariables.y[ NY * i + 6 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 7 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 8 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 9 ] = 0.0; 	//  obs
        acadoVariables.y[ NY * i + 10 ] = 0.0; 	//  obs
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
        acadoVariables.od[i * NOD + 2] = 7.0*10;
        acadoVariables.od[i * NOD + 3] = 0.0;
        acadoVariables.od[i * NOD + 4] = 18.0*10;
        acadoVariables.od[i * NOD + 5] = -4.0*10;
        acadoVariables.od[i * NOD + 6] = 18.0*10;
        acadoVariables.od[i * NOD + 7] = -4.0*10;
        acadoVariables.od[i * NOD + 8] = 18.0*10;
        acadoVariables.od[i * NOD + 9] = -4.0*10;

        acadoVariables.od[i * NOD + 10] = 0;
        acadoVariables.od[i * NOD + 11] = 1;
        acadoVariables.od[i * NOD + 12] = 6;
    }
    for (int i = 0; i < N; i++)
    {
        acadoVariables.W[NY*NY*i + (NY+1)*0] = 0.01;
        acadoVariables.W[NY*NY*i + (NY+1)*1] = 0.01;
        acadoVariables.W[NY*NY*i + (NY+1)*2] = 50;
        //acadoVariables.W[NY*NY*i + (NY+1)*2] = 500;  # Lane change
        //acadoVariables.W[NY*NY*i + (NY+1)*3] = 500;
        acadoVariables.W[NY*NY*i + (NY+1)*3] = 200;
        acadoVariables.W[NY*NY*i + (NY+1)*4] = 200;
        acadoVariables.W[NY*NY*i + (NY+1)*5] = 20;
        acadoVariables.W[NY*NY*i + (NY+1)*6] = 2000;
        acadoVariables.W[NY*NY*i + (NY+1)*7] = 2000;
        acadoVariables.W[NY*NY*i + (NY+1)*8] = 2000;
        acadoVariables.W[NY*NY*i + (NY+1)*9] = 2000;
        acadoVariables.W[NY*NY*i + (NY+1)*10] = 2000;
    }

    acadoVariables.WN[(NYN+1)*0] = 1/100;
	acadoVariables.WN[(NYN+1)*1] = 1/100;
	acadoVariables.WN[(NYN+1)*2] = 100*0;


    for (int i = 0; i < N; i++)
    {
        acadoVariables.lbAValues[i*4+0] = -1e12;    //x
        acadoVariables.lbAValues[i*4+1] = -6.9;     //y
        acadoVariables.lbAValues[i*4+2] = -5;       //v
        acadoVariables.lbAValues[i*4+3] = -2;       //w

        acadoVariables.ubAValues[i*4+0] = 1e12;     //x
        acadoVariables.ubAValues[i*4+1] = -1.1;     //y
        acadoVariables.ubAValues[i*4+2] = 15;       //v
        acadoVariables.ubAValues[i*4+3] = 2;        //w
    }
    /*for (int i = 0; i < N; i++)
    {
        acadoVariables.lbAValues[i*5+N*4+0] = 2.2;
        acadoVariables.lbAValues[i*5+N*4+1] = 2.2;
        acadoVariables.lbAValues[i*5+N*4+2] = 2.2;
        acadoVariables.lbAValues[i*5+N*4+3] = 2.2;
        acadoVariables.lbAValues[i*5+N*4+4] = 2.2;

        acadoVariables.ubAValues[i*5+N*4+0] = 1e12;
        acadoVariables.ubAValues[i*5+N*4+1] = 1e12;
        acadoVariables.ubAValues[i*5+N*4+2] = 1e12;
        acadoVariables.ubAValues[i*5+N*4+3] = 1e12;
        acadoVariables.ubAValues[i*5+N*4+4] = 1e12;
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


    //acado_preparationStep();
    //timer_ = this->create_wall_timer(100ms, std::bind(&GoalReach::timer_goal_callback, this));
    //mpc_test();

}

void GoalReach::get_vel_cb(const std::shared_ptr<acado_msgs::srv::GetVelocityCmd::Request> request,
          const std::shared_ptr<acado_msgs::srv::GetVelocityCmd::Response>      response)
{
    RCLCPP_INFO(this->get_logger(), "Inside SRV CB");
    odom = request->odom;
    pa = request->obs_poses;
    odom_arr = request->odom_arr;
    acadoVariables.x0[ 0 ] = odom.pose.pose.position.x; // x
    acadoVariables.x0[ 1 ] = odom.pose.pose.position.y; // y
    acadoVariables.x0[ 2 ] = odom.pose.pose.orientation.z; // theta
    acadoVariables.x0[ 3 ] = odom.twist.twist.linear.x; // lienar v
    acadoVariables.x0[ 4 ] = odom.twist.twist.angular.z; // ang v

    acadoVariables.yN[ 0 ] = request->goal.position.x; // xg
    acadoVariables.yN[ 1 ] = request->goal.position.y;;	// yg	

    for (int i = 0; i < (N + 1); ++i)
    {
        /*acadoVariables.od[i * NOD + 0] = pa.poses[0].position.x;
        acadoVariables.od[i * NOD + 1] = pa.poses[0].position.y + (0.5)*0.1*i;
        acadoVariables.od[i * NOD + 2] = pa.poses[1].position.x;
        acadoVariables.od[i * NOD + 3] = pa.poses[1].position.y + (-0.4)*0.1*i;
        acadoVariables.od[i * NOD + 4] = pa.poses[2].position.x;
        acadoVariables.od[i * NOD + 5] = pa.poses[2].position.y + (-0.5)*0.1*i;*/
        acadoVariables.y[ NY * i + 0 ] = request->goal.position.x; // xg
        acadoVariables.y[ NY * i + 1 ] = request->goal.position.y;;	// yg	

        acadoVariables.od[i * NOD + 0] = odom_arr.odom[0].pose.pose.position.x + odom_arr.odom[0].twist.twist.linear.x*0.1*i;
        acadoVariables.od[i * NOD + 1] = odom_arr.odom[0].pose.pose.position.y + odom_arr.odom[0].twist.twist.linear.y*0.1*i;
        acadoVariables.od[i * NOD + 2] = odom_arr.odom[1].pose.pose.position.x + odom_arr.odom[1].twist.twist.linear.x*0.1*i;
        acadoVariables.od[i * NOD + 3] = odom_arr.odom[1].pose.pose.position.y + odom_arr.odom[1].twist.twist.linear.y*0.1*i;
        acadoVariables.od[i * NOD + 4] = odom_arr.odom[2].pose.pose.position.x + odom_arr.odom[2].twist.twist.linear.x*0.1*i;
        acadoVariables.od[i * NOD + 5] = odom_arr.odom[2].pose.pose.position.y + odom_arr.odom[2].twist.twist.linear.y*0.1*i;
        acadoVariables.od[i * NOD + 6] = odom_arr.odom[3].pose.pose.position.x + odom_arr.odom[3].twist.twist.linear.x*0.1*i;
        acadoVariables.od[i * NOD + 7] = odom_arr.odom[3].pose.pose.position.y + odom_arr.odom[3].twist.twist.linear.y*0.1*i;
        acadoVariables.od[i * NOD + 8] = odom_arr.odom[4].pose.pose.position.x + odom_arr.odom[4].twist.twist.linear.x*0.1*i;
        acadoVariables.od[i * NOD + 9] = odom_arr.odom[4].pose.pose.position.y + odom_arr.odom[4].twist.twist.linear.y*0.1*i;

        acadoVariables.od[i * NOD + 10] = 0;
        acadoVariables.od[i * NOD + 11] = 1;
        acadoVariables.od[i * NOD + 12] = -request->goal.position.y;
    }
    acado_tic( &t );
    for(iter = 0; iter < ns; ++iter)
    {
    /* Perform the feedback step. */
        acado_preparationStep();
        acado_feedbackStep( );
        //printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );
        RCLCPP_INFO(this->get_logger(), "Real-Time Iteration %d:  KKT Tolerance = %.3e", iter, acado_getKKT() );
        if(acado_getKKT()>1000000000000)
        {
            RCLCPP_INFO(this->get_logger(), "HIGH COST!!!!!!!!!!!!");
            //exit(1);
        }
            
    }
    //exit(1);
    
    /*acadoVariables.x0[ 0 ] = acadoVariables.x[NX + 0]; // x
    acadoVariables.x0[ 1 ] = acadoVariables.x[NX + 1]; // y
    acadoVariables.x0[ 2 ] = acadoVariables.x[NX + 2]; // theta
    acadoVariables.x0[ 3 ] = acadoVariables.x[NX + 3]; // lienar v
    acadoVariables.x0[ 4 ] = acadoVariables.x[NX + 4]; // ang v*/
    real_t te = acado_toc( &t );
    geometry_msgs::msg::PoseArray path_val;
    for (i = 0; i < (N + 1); ++i)  
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = acadoVariables.x[ NX * i + 0 ];
        pose.position.y = acadoVariables.x[ NX * i + 1 ];
        path_val.poses.push_back(pose);
        //RCLCPP_INFO(this->get_logger(), "ACC_lin: %.2f, ACC_ang: %.2f", acadoVariables.u[ NU * i + 0], acadoVariables.x[ NU * i + 1 ]);
    }
    if(ns==1)
    {
        mpc_iter+=1;
        time+=te;
    }
    ns = 1;
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = acadoVariables.x[NX + 3];
    cmd_vel.angular.z = acadoVariables.x[NX + 4];
    /*acadoVariables.x0[ 0 ] = acadoVariables.x[NX + 0]; // x
    acadoVariables.x0[ 1 ] = acadoVariables.x[NX + 1]; // y
    acadoVariables.x0[ 2 ] = acadoVariables.x[NX + 2]; // theta
    acadoVariables.x0[ 3 ] = acadoVariables.x[NX + 3]; // lienar v
    acadoVariables.x0[ 4 ] = acadoVariables.x[NX + 4]; // ang v*/
    double dist = sqrt(pow(acadoVariables.x0[ 0 ]-acadoVariables.y[ 0 ], 2) + pow(acadoVariables.x0[ 1 ]-acadoVariables.y[ 1 ], 2));
    check_term_dist(dist);


    RCLCPP_INFO(this->get_logger(), "X: %.2f, Y: %.2f, Time: %.5f", acadoVariables.x0[0], acadoVariables.x0[1], time/mpc_iter);
    RCLCPP_INFO(this->get_logger(), "Line Vel: %.2f, Ang Vel: %.2f, Dist = %.2f", acadoVariables.x[NX + 3], acadoVariables.x[NX + 4], dist);
    //RCLCPP_INFO(this->get_logger(), "Goal X: %.2f, Goal Y: %.2f", acadoVariables.yN[ 0 ], acadoVariables.yN[ 1 ]);
    //RCLCPP_INFO(this->get_logger(), "Line acc: %.2f, Ang acc: %.2f", acadoVariables.u[NU + 0], acadoVariables.x[NU + 1]);
    //acadoVariables.y[NY * + 2] = acadoVariables.x[NX + 3];
    //acadoVariables.y[NY * + 3] = acadoVariables.x[NX + 4];
    response->twist = cmd_vel;
    response->path = path_val;
}

void GoalReach::check_term_dist(double dist)
{
    if(dist<50)
    {
        for (int i = 0; i < N; i++)
        {
            acadoVariables.W[NY*NY*i + (NY+1)*0] = 1;
            acadoVariables.W[NY*NY*i + (NY+1)*1] = 1;
            acadoVariables.W[NY*NY*i + (NY+1)*2] = 100;
            //acadoVariables.W[NY*NY*i + (NY+1)*3] = 100/10;
            //acadoVariables.W[NY*NY*i + (NY+1)*4] = 100/10;
        }
        for (i = 0; i < N; ++i)  
        {
            acadoVariables.y[ NY * i + 2 ] = 0.0;	// theta	
        }
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