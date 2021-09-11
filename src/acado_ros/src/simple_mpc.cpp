#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
//#include "test_car/acado_solver.c"
//#include "test_car/acado_qpoases_interface.hpp"
//#include "test_car/acado_integrator.c"



#include <stdio.h>

using std::placeholders::_1;
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   1        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class SimpleMPC : public rclcpp::Node
{
  public:
    SimpleMPC();
  private:
    void mpc_test();
    int    i, iter;
    acado_timer t;
    size_t count_;
};

SimpleMPC::SimpleMPC(): Node("acado_simple_mpc"), count_(0)
{
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
        acadoVariables.y[ NY * i + 0 ] = 5.0; // xg
        acadoVariables.y[ NY * i + 1 ] = 5.0;	// yg	
        acadoVariables.y[ NY * i + 2 ] = 0.0;	// linear acc
        acadoVariables.y[ NY * i + 3 ] = 0.0; 	//  anuglar acc
    }
    for (i = 0; i < NYN; ++i)
    {
        acadoVariables.yN[ i ] = 1.5; // theta
    }
    
    /* MPC: initialize the current state feedback. */

    acadoVariables.x0[ 0 ] = 0.0; // x
    acadoVariables.x0[ 1 ] = 0.0; // y
    acadoVariables.x0[ 2 ] = 1.57; // theta
    acadoVariables.x0[ 3 ] = 0.1; // lienar v
    acadoVariables.x0[ 4 ] = 0.0; // ang v

    acado_preparationStep();
    mpc_test();

}

void SimpleMPC::mpc_test()
{
    while(1)
    {
        for(iter = 0; iter < NUM_STEPS; ++iter)
        {
        /* Perform the feedback step. */
            acado_feedbackStep( );
            acado_preparationStep();
        }
        acadoVariables.x0[ 0 ] = acadoVariables.x[NX + 0]; // x
        acadoVariables.x0[ 1 ] = acadoVariables.x[NX + 1]; // y
        acadoVariables.x0[ 2 ] = acadoVariables.x[NX + 2]; // theta
        acadoVariables.x0[ 3 ] = acadoVariables.x[NX + 3]; // lienar v
        acadoVariables.x0[ 4 ] = acadoVariables.x[NX + 4]; // ang v

        RCLCPP_INFO(this->get_logger(), "X: %.2f, Y: %.2f, Theta: %.2f", acadoVariables.x0[0], acadoVariables.x0[1], acadoVariables.x0[2]);
        RCLCPP_INFO(this->get_logger(), "Lin V: %.2f, And W: %.2f", acadoVariables.x0[3], acadoVariables.x0[4]);
        if((pow(acadoVariables.x0[0] - acadoVariables.y[0], 2) + pow(acadoVariables.x0[1] - acadoVariables.y[1], 2))<0.25
            && abs(acadoVariables.x0[2] - acadoVariables.yN[0])<0.05)
        {
            RCLCPP_INFO(this->get_logger(), "Done!!!");
            break;
        }
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleMPC>());
  rclcpp::shutdown();
  return 0;
}