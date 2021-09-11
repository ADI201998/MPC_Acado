/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

 /**
 *    \file   examples/code_generation/getting_started.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date   2011-2013
 */

#include <acado_code_generation.hpp>

int main( )
{
	USING_NAMESPACE_ACADO

	double r_rad = 0.5;
	double o_rad = 0.5;

	// Variables:
	DifferentialState   x    ;  // pos
	DifferentialState   y    ;  // pos 
	DifferentialState   theta  ;  // yaw
	DifferentialState 	v; 		// linear vel
	DifferentialState 	w;		// angular vel

	OnlineData xo1;		// x coor obs 1
	OnlineData yo1;		// y coor obs 1
	OnlineData xo2;		// x coor obs 2
	OnlineData yo2;		// y coor obs 2
	OnlineData xo3;		// x coor obs 3
	OnlineData yo3;		// y coor obs 3

	Control             j    ;  // angular acc
	Control 			a	;	// linear acc
			
	// Model equations:
	DifferentialEquation f; 

	f << dot( x ) == v * cos(theta);
	f << dot( y ) == v * sin(theta);
	f << dot( theta ) == w;
	f << dot( v ) == a;
	f << dot( w ) == j;
	
	// num = 10
	// 10*(x[0:9] - xg)**2 + 100*(x[-1] - xg)**2  
	// (theta - theta_g)**2

	// Reference functions and weighting matrices:
	Expression d1 = pow(x-xo1, 2) + pow(y-yo1, 2);
	Expression d2 = pow(x-xo2, 2) + pow(y-yo2, 2);
	Expression d3 = pow(x-xo3, 2) + pow(y-yo3, 2);
	Function h, hN;
	h << x << y << a << j ;//<< 1/d1 << 1/d2 << 1/d3;				// (x - xg)**2 + (y- yg)**2 + (a)**2 + (j)**2
	hN << x << y << theta;

	// Provide defined weighting matrices:
	//DMatrix W = eye<double>( h.getDim() );
	//DMatrix WN = eye<double>( hN.getDim() );

	BMatrix W = eye<bool>( h.getDim() );
	BMatrix WN = eye<bool>( hN.getDim() );
	
	//W(0,0)= 10;
	//W(1,1) = 10;
	//W(2,2) = 100;
	//W(3,3) = 100;
	//W(4,4) = 100;
	//W(5,5) = 100;
	//W(6,6) = 100;

	//WN *= 1;
	

	OCP ocp(0.0, 5.0, 50); // start, final, steps

	ocp.subjectTo( f );
	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);
	ocp.setNOD(6);
	ocp.subjectTo( -3.0+r_rad <= x <= 3.0-r_rad );
	ocp.subjectTo( -2.0 <= w <= 2.0 );
	ocp.subjectTo( -1.5 <= v <= 1.5 );
	ocp.subjectTo(-1 <= a <= 1);
	ocp.subjectTo(-2 <= j <= 2);
	ocp.subjectTo(sqrt(pow(x-xo1, 2) + pow(y-yo1, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-xo2, 2) + pow(y-yo2, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-xo3, 2) + pow(y-yo3, 2)) >= r_rad+o_rad );

	// Export the code:
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
	mpc.set( NUM_INTEGRATOR_STEPS,        30              );

	mpc.set( QP_SOLVER,                   QP_QPOASES      );
// 	mpc.set( HOTSTART_QP,                 YES             );
// 	mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
	mpc.set( GENERATE_TEST_FILE,          NO            );
	mpc.set( GENERATE_MAKE_FILE,          NO             );
	mpc.set( GENERATE_MATLAB_INTERFACE,   NO             );
	mpc.set( GENERATE_SIMULINK_INTERFACE, NO             );

// 	mpc.set( USE_SINGLE_PRECISION,        YES             );

	if (mpc.exportCode( "/home/adi99/ROS2/mpc_ws/src/acado_ros/model/codegen" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}

