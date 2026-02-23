/*
*	File: mpc_solver_setup.cpp
*	---------------
*   MPC solver setup code generation
*/
#include <acado_code_generation.hpp>
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <vector>
// #include <acado_gnuplot.hpp>
// #include <matrix_vector/vector.hpp>
// #include <stdio.h>

USING_NAMESPACE_ACADO

int main( )
{

    double Ts = 0.1;  // prediction sampling time
    double N  = 30;   // Prediction horizon
    // // double g = 9.8066;
    // // double PI = 3.1415926535897932;
    DifferentialState x;
	DifferentialState y;
	DifferentialState z;
	DifferentialState vx;
	DifferentialState vy;
	DifferentialState vz;
	DifferentialState dummy1;
	DifferentialState dummy2;

	// //Control Input
	Control ax;
	Control ay;
	Control az;
	Control sk_d;
	Control sk_s;

	// // MODEL Definition
	DifferentialEquation f;
	f << dot(x) == vx;
	f << dot(y) == vy;
	f << dot(z) == vz;
	f << dot(vx) == ax; 
	f << dot(vy) == ay;
	f << dot(vz) == az;
	f << dot(dummy1) == sk_d;
	f << dot(dummy2) == sk_s;

	// // Least Square Function
	Function h, hN;
	h << x << y << z << ax << ay << az << sk_d << sk_s;
	hN << x << y << z;

	// // setup OCP
	OCP ocp(0.0, N*Ts, N);
	DMatrix Q(8,8);
	// Q.setIdentity(); Q(0,0) = 1; Q(1,1) = 1; Q(2,2) = 10; Q(3,3) = 1.0; Q(4,4) = 1.0; Q(5,5) = 1.0; 
	Q.setIdentity(); Q(0,0) = 10.0; Q(1,1) = 10.0; Q(2,2) = 10.0; Q(3,3) = 1.0; Q(4,4) = 1.0; Q(5,5) = 1.0; 
	Q(6,6) = 100.0;Q(7,7) = 1000.0;
	
	DMatrix QN(3,3);
	QN.setIdentity(); QN(0,0) = 10.0; QN(1,1) = 10.0; QN(2,2) = 10.0;
	// QN.setIdentity(); QN(0,0) = 1; QN(1,1) = 1; QN(2,2) = 1;
	
	// // ocp.minimizeLSQ(Q, h, r); // Objective
	ocp.minimizeLSQ(Q, h); // Objectives
	ocp.minimizeLSQEndTerm(QN, hN);
	// // Dynamic Constraint
	ocp.subjectTo(f); 


	// State constraint
	// double slackRatio = 0.4;
	// double skLimit = 1 - pow((1 - slackRatio), 2);
	// double skLimit = 1.0;

	OnlineData maxZ;
	OnlineData minZ;
	OnlineData maxVel;
	OnlineData maxAcc;
	OnlineData skLimitStatic;
	OnlineData skLimitDynamic;

	// ocp.subjectTo( 0.8 <= z <= 1.2);
	// ocp.subjectTo( -2 <= vx <= 2 );
	// ocp.subjectTo( -2 <= vy <= 2 );
	// ocp.subjectTo( -2 <= vz <= 2 );
	// ocp.subjectTo( -1.5 <= ax <= 1.5 );
	// ocp.subjectTo( -1.5 <= ay <= 1.5 );
	// ocp.subjectTo( -1.5 <= az <= 1.5 );
	// ocp.subjectTo(0 <= sk_d<= 0.64);
	ocp.subjectTo(z-maxZ <= 0); ocp.subjectTo(z-minZ >= 0);
	ocp.subjectTo(vx-maxVel <= 0); ocp.subjectTo(vx+maxVel>=0);
	ocp.subjectTo(vy-maxVel <= 0); ocp.subjectTo(vy+maxVel>=0);
	ocp.subjectTo(vz-maxVel <= 0); ocp.subjectTo(vz+maxVel>=0);
	ocp.subjectTo(ax-maxAcc <= 0); ocp.subjectTo(ax+maxAcc>=0);
	ocp.subjectTo(ay-maxAcc <= 0); ocp.subjectTo(ay+maxAcc>=0);
	ocp.subjectTo(az-maxAcc <= 0); ocp.subjectTo(az+maxAcc>=0);
	ocp.subjectTo(sk_d >= 0); 
	ocp.subjectTo(sk_d - skLimitDynamic <= 0);
	ocp.subjectTo(sk_s >= 0);
	ocp.subjectTo(sk_s - skLimitStatic <= 0);
	// ocp.subjectTo(0 <= sk_d<= 0.2);

	// Ellipsoid Obstacle Constraint
	int numObstacle = 25;
	int numDynamicOb = 4;
	std::vector<OnlineData> obx(numObstacle);
	std::vector<OnlineData> oby(numObstacle);
	std::vector<OnlineData> obz(numObstacle);
	std::vector<OnlineData> obxsize(numObstacle);
	std::vector<OnlineData> obysize(numObstacle);
	std::vector<OnlineData> obzsize(numObstacle);
	std::vector<OnlineData> yaw(numObstacle);
	ocp.setNOD(6+7*numObstacle);

	for(int i=0;i<numDynamicOb;i++){
		ocp.subjectTo(pow((x-obx[i])*cos(yaw[i])+(y-oby[i])*sin(yaw[i]), 2)/pow(obxsize[i],2) + pow(-(x-obx[i])*sin(yaw[i])+(y-oby[i])*cos(yaw[i]), 2)/pow(obysize[i],2) + pow((z-obz[i]), 2)/pow(obzsize[i],2) -1.0 + sk_d >=  0 );
	}
	for(int i=numDynamicOb;i<numObstacle;i++){
		ocp.subjectTo(pow((x-obx[i])*cos(yaw[i])+(y-oby[i])*sin(yaw[i]), 2)/pow(obxsize[i],2) + pow(-(x-obx[i])*sin(yaw[i])+(y-oby[i])*cos(yaw[i]), 2)/pow(obysize[i],2) + pow((z-obz[i]), 2)/pow(obzsize[i],2) -1.0 + sk_s >=  0 );
	}
	

	// //Linearized Obstacle Constraint
	// int numObstacle = 25;
	// int numDynamicOb = 4;
	// ocp.setNOD(numObstacle*4+9);
	// OnlineData cx;
	// OnlineData cy;
	// OnlineData cz;
	// std::vector<OnlineData> fxyz(numObstacle);
	// std::vector<OnlineData> fxx(numObstacle);
	// std::vector<OnlineData> fyy(numObstacle);
	// std::vector<OnlineData> fzz(numObstacle);
	
	// for (int i=0; i<numDynamicOb;i++){
	// 	ocp.subjectTo(fxyz[i]+fxx[i]*(x-cx)+fyy[i]*(y-cy)+fzz[i]*(z-cz)-1+sk_d>=0);
	// }
	// for (int i=numDynamicOb; i<numObstacle;i++){
	// 	ocp.subjectTo(fxyz[i]+fxx[i]*(x-cx)+fyy[i]*(y-cy)+fzz[i]*(z-cz)-1+sk_s>=0);
	// }


	// Export the code:
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	mpc.set( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
	mpc.set( NUM_INTEGRATOR_STEPS,        30              );

	// mpc.set(FIX_INITIAL_STATE, BT_FALSE);
	mpc.set( QP_SOLVER,                   QP_QPOASES      );
	mpc.set( MAX_NUM_QP_ITERATIONS, 		 1000		   	);
	mpc.set( HOTSTART_QP,                 NO  );
// 	mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
	mpc.set( GENERATE_TEST_FILE,          BT_FALSE             );
	mpc.set( GENERATE_MAKE_FILE,          BT_FALSE             );
	mpc.set( GENERATE_MATLAB_INTERFACE,   BT_FALSE             );
	mpc.set( GENERATE_SIMULINK_INTERFACE, BT_FALSE             );
	mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);

// 	mpc.set( USE_SINGLE_PRECISION,        YES             );

	if (mpc.exportCode( "./src/CERLAB-UAV-Autonomy/trajectory_planner/include/trajectory_planner/mpc_solver" ) != SUCCESSFUL_RETURN){
		exit( EXIT_FAILURE );
	}
		

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}