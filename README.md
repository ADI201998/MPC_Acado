# MPC_Acado

Contains packages for integrating and running Acado toolkit with ROS2.

It is implemented as a ROS2 service client.

First, build the workspace using `colcon build`

To run Acado with ros2, run:-

`ros2 run acado_ros goal_reach_srv`

`ros2 run python_plot_pkg plot_client`