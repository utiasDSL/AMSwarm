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
#include <vector>
#include "yaml-cpp/yaml.h"
#include <ros/package.h>
#include <ros/ros.h>


int main( )
{
	std :: string path = ros :: package::getPath("amswarm");
	YAML :: Node params = YAML :: LoadFile(path+"/params/config_acado_swarm.yaml");
	
	int max_drones = params["max_drone"].as<int>();
	int num = params["num"].as<int>();
	float t_plan = params["t_plan"].as<float>();
	float a_drone = params["a_drone"].as<float>();
	float b_drone = params["b_drone"].as<float>();
	float c_drone = params["c_drone"].as<float>();
	
	std :: vector<float> x_lim = params["x_lim"].as<std::vector<float>>();
	std :: vector<float> y_lim = params["y_lim"].as<std::vector<float>>();
	std :: vector<float> z_lim = params["z_lim"].as<std::vector<float>>();

	float x_max = x_lim[1] - a_drone;
	float x_min = x_lim[0] + a_drone;

	float y_max = y_lim[1] - b_drone;
	float y_min = y_lim[0] + b_drone;
	
	float z_max = z_lim[1] - a_drone;
	float z_min = z_lim[0] + a_drone;

	float vel_max = params["vel_max"].as<float>();

	float gravity = params["gravity"].as<float>();
	float f_min = params["f_min"].as<float>()*gravity;
	float f_max = params["f_max"].as<float>()*gravity;
	

	float acc_max = (-(2*gravity) + sqrt(pow(2*gravity,2) - 4*3*(pow(gravity,2) - pow(f_max,2))))/6.0; 
	float z_acc_min =  f_min - gravity;
	ROS_INFO_STREAM(acc_max << " " << z_acc_min);

	USING_NAMESPACE_ACADO

	// OnlineData key("", max_drones-1,1);
	OnlineData xo("", max_drones-1,1);
	OnlineData yo("", max_drones-1,1);
	OnlineData zo("", max_drones-1,1);
	OnlineData ao("", max_drones-1,1);
	OnlineData bo("", max_drones-1,1);
	OnlineData co("", max_drones-1,1);
	OnlineData gamma;


	DifferentialState x;
	DifferentialState y;
	DifferentialState z;
	DifferentialState vx;
	DifferentialState vy;
	DifferentialState vz;
	DifferentialState ax;
	DifferentialState ay;
	DifferentialState az;
	DifferentialState dummy;
	Control jx;
	Control jy;
	Control jz;
	Control slack("", max_drones-1,1);

	Expression dummy_ctrl = slack(0);
	for(int i = 1; i < max_drones-1; i++)
		dummy_ctrl = dummy_ctrl + slack(i);	
	
	DifferentialEquation f;
	f << dot(x) == vx;
	f << dot(y) == vy;
	f << dot(z) == vz;
	f << dot(vx) == ax;
	f << dot(vy) == ay;
	f << dot(vz) == az;
	f << dot(ax) == jx;
	f << dot(ay) == jy;
	f << dot(az) == jz;
	f << dot(dummy) == dummy_ctrl;
	 
	// Reference functions and weighting matrices:
	Function h, hN;
	h << ax << ay << az;
	for(int i = 0; i < max_drones-1; i++) h << slack(i);
	hN << x << y << z;

	// Provide defined weighting matrices:
	BMatrix W = eye<bool>( h.getDim() );
	BMatrix WN = eye<bool>( hN.getDim() );
	
	OCP ocp(0.0, t_plan, num);
	
	ocp.setNOD(6*max_drones-5);
	ocp.subjectTo( f );

	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);

	ocp.subjectTo( -acc_max <= ax <= acc_max );
	ocp.subjectTo( -acc_max <= ay <= acc_max );
	ocp.subjectTo( z_acc_min <= az <= acc_max );
	ocp.subjectTo( -vel_max/sqrt(3) <= vx <= +vel_max/sqrt(3) );
	ocp.subjectTo( -vel_max/sqrt(3) <= vy <= +vel_max/sqrt(3) );
	ocp.subjectTo( -vel_max/sqrt(3) <= vz <= +vel_max/sqrt(3) );
	ocp.subjectTo( x_min <= x <= x_max);
	ocp.subjectTo( y_min <= y <= y_max);
	ocp.subjectTo( z_min <= z <= z_max);
	
	for(int i = 0; i < max_drones-1; i++){
		Expression Bdot = (2*vx*(x - xo(i))/(ao(i)*ao(i)) + 2*vy*(y - yo(i))/(bo(i)*bo(i)) + 2*vz*(z - zo(i))/(co(i)*co(i)));
		Expression B = (pow((x - xo(i))/ao(i), 2) + pow((y - yo(i))/bo(i), 2) + pow((z - zo(i))/co(i), 2) - 1.0);

		ocp.subjectTo((Bdot + gamma*B + slack(i)) >= 0);
		ocp.subjectTo(slack(i) >= 0);	
	}
	
	// Export the code:
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
	mpc.set( NUM_INTEGRATOR_STEPS,        30              );

	mpc.set( QP_SOLVER,                   QP_QPOASES      );
// 	mpc.set( HOTSTART_QP,                 YES             );
// 	mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
	mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES       );
	mpc.set( GENERATE_TEST_FILE,          NO             );
	mpc.set( GENERATE_MAKE_FILE,          NO             );
	mpc.set( GENERATE_MATLAB_INTERFACE,   NO             );
	mpc.set( GENERATE_SIMULINK_INTERFACE, NO             );

// 	mpc.set( USE_SINGLE_PRECISION,        YES             );
	
	if (mpc.exportCode( path+"/submodules/codegen_"+std::to_string(max_drones)+"/" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
