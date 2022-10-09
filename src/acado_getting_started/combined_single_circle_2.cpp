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

	double r_rad = 1.2;
	double o_rad = 1.2;

	// Variables:
	DifferentialState   x    ;  // pos
	DifferentialState   y    ;  // pos 
	DifferentialState   theta  ;  // yaw
	DifferentialState 	v; 		// linear vel
	DifferentialState 	w;		// angular vel

	OnlineData xo1;		// x coor obs 1
	OnlineData yo1;		// y coor obs 1
    //OnlineData psi1;		// psi obs 1
	OnlineData xo2;		// x coor obs 2
	OnlineData yo2;		// y coor obs 2
    //OnlineData psi2;		// psi obs 2
	OnlineData xo3;		// x coor obs 3
	OnlineData yo3;		// y coor obs 3
    //OnlineData psi3;		// psi obs 3
	OnlineData xo4;		// x coor obs 4
	OnlineData yo4;		// y coor obs 4
    //OnlineData psi4;		// psi obs 4
	OnlineData xo5;		// x coor obs 5
	OnlineData yo5;		// y coor obs 5
    //OnlineData psi5;		// psi obs 5
	OnlineData xo6;		// x coor obs 5
	OnlineData yo6;
	OnlineData xo7;		// x coor obs 5
	OnlineData yo7;
	OnlineData xo8;		// x coor obs 5
	OnlineData yo8;
	OnlineData xo9;		// x coor obs 5
	OnlineData yo9;
	OnlineData xo10;		// x coor obs 5
	OnlineData yo10;

	OnlineData a0;
	OnlineData b0;
	OnlineData c0;
	OnlineData a1;
	OnlineData b1;
	OnlineData c1;

    OnlineData amin;
    OnlineData bmin;
    OnlineData cmin;
    OnlineData dmin;
    OnlineData emin;
    OnlineData fmin;
    OnlineData gmin;

    OnlineData amax;
    OnlineData bmax;
    OnlineData cmax;
    OnlineData dmax;
    OnlineData emax;
    OnlineData fmax;
    OnlineData gmax;



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
	Expression dist0 = sqrt(pow(a0*x + b0*y + c0, 2))/sqrt(pow(a0, 2) + pow(b0, 2));
	Expression dist1 = sqrt(pow(sqrt(pow(x-a1, 2) + pow(y-b1, 2)) - c1, 2));
	Expression d1 = sqrt(pow(x-xo1, 2) + pow(y-yo1, 2)) - (r_rad + o_rad);
	Expression d2 = sqrt(pow(x-xo2, 2) + pow(y-yo2, 2)) - (r_rad + o_rad);
	Expression d3 = sqrt(pow(x-xo3, 2) + pow(y-yo3, 2)) - (r_rad + o_rad);
	Expression d4 = sqrt(pow(x-xo4, 2) + pow(y-yo4, 2)) - (r_rad + o_rad);
	Expression d5 = sqrt(pow(x-xo5, 2) + pow(y-yo5, 2)) - (r_rad + o_rad);
	Expression d6 = sqrt(pow(x-xo6, 2) + pow(y-yo6, 2)) - (r_rad + o_rad);
	Expression d7 = sqrt(pow(x-xo7, 2) + pow(y-yo7, 2)) - (r_rad + o_rad);
	Expression d8 = sqrt(pow(x-xo8, 2) + pow(y-yo8, 2)) - (r_rad + o_rad);
	Expression d9 = sqrt(pow(x-xo9, 2) + pow(y-yo9, 2)) - (r_rad + o_rad);
	Expression d10 = sqrt(pow(x-xo10, 2) + pow(y-yo10, 2)) - (r_rad + o_rad);

	Function h, hN;
	h << x << y << v << a << j << 1/d1 << 1/d2 << 1/d3 << 1/d4 << 1/d5 << 1/d6 << 1/d7 << 1/d8 << 1/d9 << 1/d10 << dist0 << dist1;				// (x - xg)**2 + (y- yg)**2 + (a)**2 + (j)**2
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
	

	//OCP ocp(0.0, 5.0, 50); // start, final, steps
	OCP ocp(0.0, 5.0, 50); // start, final, steps

	ocp.subjectTo( f );
	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);
	ocp.setNOD(40);

	//ocp.subjectTo( -100 <= x <= 1e12);
	//ocp.subjectTo( -8.0 <= y <= 8.0);
	ocp.subjectTo( -2.0 <= v <= 1.5*6);
	ocp.subjectTo( -2.0 <= w <= 2.0 );
	
	ocp.subjectTo(-5 <= a <= 5);
	ocp.subjectTo(-2 <= j <= 2);

    ocp.subjectTo(amin*pow(x,2) + bmin*x + cmin + dmin*pow(y,2) + emin*y + fmin + gmin <= 0);
    ocp.subjectTo(amax*pow(x,2) + bmax*x + cmax + dmax*pow(y,2) + emax*y + fmax + gmax <= 0);

	/*ocp.subjectTo(sqrt(pow(x-xo1, 2) + pow(y-yo1, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-xo2, 2) + pow(y-yo2, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-xo3, 2) + pow(y-yo3, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-xo4, 2) + pow(y-yo4, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-xo5, 2) + pow(y-yo5, 2)) >= r_rad+o_rad );*/
	
    

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
	mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO );
	mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);

// 	mpc.set( USE_SINGLE_PRECISION,        YES             );

	if (mpc.exportCode( "/home/adi99/ROS2/mpc_ws/src/acado_ros/model/codegen" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	
	if (mpc.exportCode( "/home/adi99/ROS2/mpc_acado_ws/src/mpc_acado/model/codegen" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}