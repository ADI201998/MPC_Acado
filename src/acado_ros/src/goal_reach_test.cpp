#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
//#include "test_car/acado_solver.c"
//#include "test_car/acado_qpoases_interface.hpp"
//#include "test_car/acado_integrator.c"



#include <stdio.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
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
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void odom_pose_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void timer_goal_callback();
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::PoseArray pa;
    bool got_odom;
    int ns;
    bool got_pa;
    int    i, iter;
    acado_timer t;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subs_obs_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_path_;
    size_t count_;
};

GoalReach::GoalReach(): Node("acado_goal_reach"), count_(0)
{
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/odom", 1, std::bind(&GoalReach::odom_cb, this, _1));
    subs_obs_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
                    "/obs_pose", 1, std::bind(&GoalReach::odom_pose_cb, this, _1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    publisher_path_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/path", 1);
    got_odom = false;
    got_pa = false;
    ns = 30;

    acado_initializeSolver();

    for (i = 0; i < (N + 1); ++i)  
    {
        acadoVariables.x[ NX * i + 0 ] = 0.0; // x
        acadoVariables.x[ NX * i + 1 ] = 0.0; // y
        acadoVariables.x[ NX * i + 2 ] = 0.0; // theta
        acadoVariables.x[ NX * i + 3 ] = 0.0; // v 
        acadoVariables.x[ NX * i + 4 ] = 0.0; // w
    }
    for (i = 0; i < N; ++i)  
    {
        acadoVariables.u[ NU * i + 0 ] = 0.0;
        acadoVariables.u[ NU * i + 1 ] = 0.0;
    }
    /* Initialize the measurements/reference. */
    for (i = 0; i < N; ++i)  
    {
        acadoVariables.y[ NY * i + 0 ] = 20.0; // xg
        acadoVariables.y[ NY * i + 1 ] = 0.0;	// yg	
        acadoVariables.y[ NY * i + 2 ] = 0.0;	// linear acc
        acadoVariables.y[ NY * i + 3 ] = 0.0; 	//  anuglar acc
    }

    acadoVariables.yN[0] = 20.0;
    acadoVariables.yN[1] = 0.0; // theta
    acadoVariables.yN[2] = 0.0;
    for (int i = 0; i < (N + 1); ++i)
    {
        acadoVariables.od[i * NOD + 0] = 4.0;
        acadoVariables.od[i * NOD + 1] = 4.0;
        acadoVariables.od[i * NOD + 2] = 7.0;
        acadoVariables.od[i * NOD + 3] = 0.0;
        acadoVariables.od[i * NOD + 4] = 18.0;
        acadoVariables.od[i * NOD + 5] = -4.0;
    }
    for (int i = 0; i < N; i++)
    {
    acadoVariables.W[NY*NY*i + (NY+1)*0] = 10;
    acadoVariables.W[NY*NY*i + (NY+1)*1] = 10;
    acadoVariables.W[NY*NY*i + (NY+1)*2] = 100;
    acadoVariables.W[NY*NY*i + (NY+1)*3] = 100;
    }

    acadoVariables.WN[NYN*NYN + (NYN+1)*0] = 10;
	acadoVariables.WN[NYN*NYN + (NYN+1)*1] = 10;
	acadoVariables.WN[NYN*NYN + (NYN+1)*2] = 100;

    
    /* MPC: initialize the current state feedback. */

    acadoVariables.x0[ 0 ] = 0.0; // x
    acadoVariables.x0[ 1 ] = 0.0; // y
    acadoVariables.x0[ 2 ] = 0.0; // theta
    acadoVariables.x0[ 3 ] = 0.0; // lienar v
    acadoVariables.x0[ 4 ] = 0.0; // ang v

    acado_preparationStep();
    timer_ = this->create_wall_timer(100ms, std::bind(&GoalReach::timer_goal_callback, this));
    //mpc_test();

}

void GoalReach::timer_goal_callback()
{
    if(!got_odom || !got_pa)
        return;
    acadoVariables.x0[ 0 ] = odom.pose.pose.position.x; // x
    acadoVariables.x0[ 1 ] = odom.pose.pose.position.y; // y
    acadoVariables.x0[ 2 ] = odom.pose.pose.orientation.z; // theta
    acadoVariables.x0[ 3 ] = odom.twist.twist.linear.x; // lienar v
    acadoVariables.x0[ 4 ] = odom.twist.twist.angular.z; // ang v

    for (int i = 0; i < (N + 1); ++i)
    {
        acadoVariables.od[i * NOD + 0] = pa.poses[0].position.x + 0.5*0.1*i;
        acadoVariables.od[i * NOD + 1] = pa.poses[0].position.y;
        acadoVariables.od[i * NOD + 2] = pa.poses[1].position.x + (-0.6)*0.1*i;
        acadoVariables.od[i * NOD + 3] = pa.poses[1].position.y;
        acadoVariables.od[i * NOD + 4] = pa.poses[2].position.x + (-0.45)*0.1*i;
        acadoVariables.od[i * NOD + 5] = pa.poses[2].position.y;
    }
    acado_tic( &t );
    for(iter = 0; iter < ns; ++iter)
    {
    /* Perform the feedback step. */
        acado_feedbackStep( );
        acado_preparationStep();
        //printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );
        RCLCPP_INFO(this->get_logger(), "Real-Time Iteration %d:  KKT Tolerance = %.3e", iter, acado_getKKT() );
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
    publisher_path_->publish(path_val);
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = acadoVariables.x[NX + 3];
    cmd_vel.angular.z = acadoVariables.x[NX + 4];
    publisher_->publish(cmd_vel);
    //RCLCPP_INFO(this->get_logger(), "X: %.2f, Y: %.2f, Time: %.5f", acadoVariables.x0[0], acadoVariables.x0[1], te);
    RCLCPP_INFO(this->get_logger(), "Line Vel: %.2f, Ang Vel: %.2f", acadoVariables.x[NX + 3], acadoVariables.x[NX + 4]);
    ns = 5;
    if((pow(acadoVariables.x0[0] - acadoVariables.y[0], 2) + pow(acadoVariables.x0[1] - acadoVariables.y[1], 2))<0.25)
    {
        RCLCPP_INFO(this->get_logger(), "Done!!!");
        exit(1);
    }
}

void GoalReach::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom = *msg;
    //RCLCPP_INFO(this->get_logger(), "Got Odom!!!");
    got_odom = true;
    //printf("x_pos = %f y_pos = %f theta = %f \n",acadoVariables.x0[ 0 ], acadoVariables.x0[ 1 ], acadoVariables.x0[ 2 ] );

}
void GoalReach::odom_pose_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    pa = *msg;
    //RCLCPP_INFO(this->get_logger(), "Got Odom!!!");
    got_pa = true;
    //printf("x_pos = %f y_pos = %f theta = %f \n",acadoVariables.x0[ 0 ], acadoVariables.x0[ 1 ], acadoVariables.x0[ 2 ] );

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalReach>());
  rclcpp::shutdown();
  return 0;
}