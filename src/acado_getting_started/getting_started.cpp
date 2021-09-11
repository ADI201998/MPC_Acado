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

	double rr = 0.5, ro = 0.5;

	OnlineData xo_1;
	OnlineData yo_1;
	OnlineData xo_2;
	OnlineData yo_2;
	OnlineData xo_3;
	OnlineData yo_3;

	// Variables:
	DifferentialState   x    ;  // pos
	DifferentialState   y    ;  // pos 
	DifferentialState   theta  ;  // yaw
	DifferentialState 	v; 		// linear vel
	DifferentialState 	w;

	Control             j    ;  // angular acc
	Control 			a	;	// linear acc
			
	// Model equations:
	DifferentialEquation f; 

	f << dot( x ) == v * cos(theta);
	f << dot( y ) == v * sin(theta);
	f << dot( theta ) == w;
	f << dot( v ) == a;
	f << dot( w ) == j;
	
	
	// Reference functions and weighting matrices:
	Function h, hN;
	h << x << y << a << j;				
	hN << x << y << theta;

	// Provide defined weighting matrices:
	BMatrix W = eye<bool>( h.getDim() );
	BMatrix WN = eye<bool>( hN.getDim() );
		

	OCP ocp(0.0, 5.0, 50); // start, final, steps
	ocp.setNOD(6);
	ocp.subjectTo( f );
	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);
	ocp.subjectTo( -5.0 <= x <= 5.0 );
	ocp.subjectTo( -3.0 <= w <= 3.0 );
	ocp.subjectTo( -1.5 <= v <= 1.5 );
	ocp.subjectTo(-1 <= a <= 1);
	ocp.subjectTo(-2 <= j <= 2);
	
	ocp.subjectTo(sqrt(pow(x - xo_1,2) + pow(y - yo_1,2)) >= rr + ro);
	ocp.subjectTo(sqrt(pow(x - xo_2,2) + pow(y - yo_2,2)) >= rr + ro);
	ocp.subjectTo(sqrt(pow(x - xo_3,2) + pow(y - yo_3,2)) >= rr + ro);
	

	// Export the code:
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
	mpc.set( NUM_INTEGRATOR_STEPS,        30              );
	mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX,         YES);
	mpc.set( QP_SOLVER,                   QP_QPOASES      );
// 	mpc.set( HOTSTART_QP,                 YES             );
// 	mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
	mpc.set( GENERATE_TEST_FILE,          NO             );
	mpc.set( GENERATE_MAKE_FILE,          YES             );
	mpc.set( GENERATE_MATLAB_INTERFACE,   NO             );
	mpc.set( GENERATE_SIMULINK_INTERFACE, NO             );

	if (mpc.exportCode( "/home/adi99/ROS2/mpc_ws/src/acado_ros/model/codegen" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
