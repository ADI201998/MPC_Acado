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

	double r_rad = 1.1;
	double o_rad = 1.1;

	// Variables:
	DifferentialState   x    ;  // pos
	DifferentialState   y    ;  // pos 
	DifferentialState   theta  ;  // yaw
	DifferentialState 	v; 		// linear vel
	DifferentialState 	w;		// angular vel

	OnlineData xo1;		// x coor obs 1
	OnlineData yo1;		// y coor obs 1
    OnlineData psi1;		// psi obs 1
	OnlineData xo2;		// x coor obs 2
	OnlineData yo2;		// y coor obs 2
    OnlineData psi2;		// psi obs 2
	OnlineData xo3;		// x coor obs 3
	OnlineData yo3;		// y coor obs 3
    OnlineData psi3;		// psi obs 3
	OnlineData xo4;		// x coor obs 4
	OnlineData yo4;		// y coor obs 4
    OnlineData psi4;		// psi obs 4
	OnlineData xo5;		// x coor obs 5
	OnlineData yo5;		// y coor obs 5
    OnlineData psi5;		// psi obs 5

	OnlineData a0;
	OnlineData b0;
	OnlineData c0;

    OnlineData amax;
	OnlineData bmax;
	OnlineData cmax;

    OnlineData amin;
	OnlineData bmin;
	OnlineData cmin;



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
	//Expression d1 = pow(x-xo1, 2) + pow(y-yo1, 2);
	//Expression d2 = pow(x-xo2, 2) + pow(y-yo2, 2);
	//Expression d3 = pow(x-xo3, 2) + pow(y-yo3, 2);
	//Expression dist = sqrt(pow(pow(x - a0, 2) + pow(y - b0, 2) - pow(c0, 2), 2));
    Expression dist = sqrt(pow(sqrt(pow(x-a0, 2) + pow(y-b0, 2)) - c0, 2));
	Function h, hN;
	h << x << y << v << a << j << dist ;				// (x - xg)**2 + (y- yg)**2 + (a)**2 + (j)**2
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
	ocp.setNOD(24);

	ocp.subjectTo( -2.0 <= v <= 1.5*6);
	ocp.subjectTo( -2.0 <= w <= 2.0 );
	
	ocp.subjectTo(-5 <= a <= 5);
	ocp.subjectTo(-2 <= j <= 2);

	ocp.subjectTo(pow(x-amax,2) + pow(y-bmax,2) - pow(cmax,2) <= 0);
    ocp.subjectTo(pow(x-amin,2) + pow(y-bmin,2) - pow(cmin,2) >= 0);

    ocp.subjectTo(sqrt(pow(x-(xo1-2*o_rad*cos(psi1)), 2) + pow(y-(yo1-2*o_rad*sin(psi1)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-xo1, 2) + pow(y-yo1, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-(xo1+2*o_rad*cos(psi1)), 2) + pow(y-(yo1+2*o_rad*sin(psi1)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-(xo1-2*o_rad*cos(psi1)), 2) + pow((y-2*r_rad*sin(theta))-(yo1-2*o_rad*sin(psi1)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-xo1, 2) + pow((y-2*r_rad*sin(theta))-yo1, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-(xo1+2*o_rad*cos(psi1)), 2) + pow((y-2*r_rad*sin(theta))-(yo1+2*o_rad*sin(psi1)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-(xo1-2*o_rad*cos(psi1)), 2) + pow((y+2*r_rad*sin(theta))-(yo1-2*o_rad*sin(psi1)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-xo1, 2) + pow((y+2*r_rad*sin(theta))-yo1, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-(xo1+2*o_rad*cos(psi1)), 2) + pow((y+2*r_rad*sin(theta))-(yo1+2*o_rad*sin(psi1)), 2)) >= r_rad+o_rad );

	ocp.subjectTo(sqrt(pow(x-(xo2-2*o_rad*cos(psi2)), 2) + pow(y-(yo2-2*o_rad*sin(psi2)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-xo2, 2) + pow(y-yo2, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-(xo2+2*o_rad*cos(psi2)), 2) + pow(y-(yo2+2*o_rad*sin(psi2)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-(xo2-2*o_rad*cos(psi2)), 2) + pow((y-2*r_rad*sin(theta))-(yo2-2*o_rad*sin(psi2)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-xo2, 2) + pow((y-2*r_rad*sin(theta))-yo2, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-(xo2+2*o_rad*cos(psi2)), 2) + pow((y-2*r_rad*sin(theta))-(yo2+2*o_rad*sin(psi2)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-(xo2-2*o_rad*cos(psi2)), 2) + pow((y+2*r_rad*sin(theta))-(yo2-2*o_rad*sin(psi2)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-xo2, 2) + pow((y+2*r_rad*sin(theta))-yo2, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-(xo2+2*o_rad*cos(psi2)), 2) + pow((y+2*r_rad*sin(theta))-(yo2+2*o_rad*sin(psi2)), 2)) >= r_rad+o_rad );

	ocp.subjectTo(sqrt(pow(x-(xo3-2*o_rad*cos(psi3)), 2) + pow(y-(yo3-2*o_rad*sin(psi3)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-xo3, 2) + pow(y-yo3, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-(xo3+2*o_rad*cos(psi3)), 2) + pow(y-(yo3+2*o_rad*sin(psi3)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-(xo3-2*o_rad*cos(psi3)), 2) + pow((y-2*r_rad*sin(theta))-(yo3-2*o_rad*sin(psi3)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-xo3, 2) + pow((y-2*r_rad*sin(theta))-yo3, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-(xo3+2*o_rad*cos(psi3)), 2) + pow((y-2*r_rad*sin(theta))-(yo3+2*o_rad*sin(psi3)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-(xo3-2*o_rad*cos(psi3)), 2) + pow((y+2*r_rad*sin(theta))-(yo3-2*o_rad*sin(psi3)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-xo3, 2) + pow((y+2*r_rad*sin(theta))-yo3, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-(xo3+2*o_rad*cos(psi3)), 2) + pow((y+2*r_rad*sin(theta))-(yo3+2*o_rad*sin(psi3)), 2)) >= r_rad+o_rad );

	ocp.subjectTo(sqrt(pow(x-(xo4-2*o_rad*cos(psi4)), 2) + pow(y-(yo4-2*o_rad*sin(psi4)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-xo4, 2) + pow(y-yo4, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-(xo4+2*o_rad*cos(psi4)), 2) + pow(y-(yo4+2*o_rad*sin(psi4)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-(xo4-2*o_rad*cos(psi4)), 2) + pow((y-2*r_rad*sin(theta))-(yo4-2*o_rad*sin(psi4)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-xo4, 2) + pow((y-2*r_rad*sin(theta))-yo4, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-(xo4+2*o_rad*cos(psi4)), 2) + pow((y-2*r_rad*sin(theta))-(yo4+2*o_rad*sin(psi4)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-(xo4-2*o_rad*cos(psi4)), 2) + pow((y+2*r_rad*sin(theta))-(yo4-2*o_rad*sin(psi4)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-xo4, 2) + pow((y+2*r_rad*sin(theta))-yo4, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-(xo4+2*o_rad*cos(psi4)), 2) + pow((y+2*r_rad*sin(theta))-(yo4+2*o_rad*sin(psi4)), 2)) >= r_rad+o_rad );

	ocp.subjectTo(sqrt(pow(x-(xo5-2*o_rad*cos(psi5)), 2) + pow(y-(yo5-2*o_rad*sin(psi5)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-xo5, 2) + pow(y-yo5, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow(x-(xo5+2*o_rad*cos(psi5)), 2) + pow(y-(yo5+2*o_rad*sin(psi5)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-(xo5-2*o_rad*cos(psi5)), 2) + pow((y-2*r_rad*sin(theta))-(yo5-2*o_rad*sin(psi5)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-xo5, 2) + pow((y-2*r_rad*sin(theta))-yo5, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x-2*r_rad*cos(theta))-(xo5+2*o_rad*cos(psi5)), 2) + pow((y-2*r_rad*sin(theta))-(yo5+2*o_rad*sin(psi5)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-(xo5-2*o_rad*cos(psi5)), 2) + pow((y+2*r_rad*sin(theta))-(yo5-2*o_rad*sin(psi5)), 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-xo5, 2) + pow((y+2*r_rad*sin(theta))-yo5, 2)) >= r_rad+o_rad );
	ocp.subjectTo(sqrt(pow((x+2*r_rad*cos(theta))-(xo5+2*o_rad*cos(psi5)), 2) + pow((y+2*r_rad*sin(theta))-(yo5+2*o_rad*sin(psi5)), 2)) >= r_rad+o_rad );

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

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
