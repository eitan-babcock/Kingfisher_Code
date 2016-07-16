#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>

#define N_MEAS 3		// number of measurements for vision, imu, optFlow
#define N_STATE 9		// number of states

#define PREDICT_RATE 200	// prediction rate Hz

#define POS_STD 0.5			// pos standard deviation m
#define VEL_STD 3			// vel standard deviation m/s
#define ACC_STD 1			// acc standard deviation m^2/s
#define CAM_MEAS_STD 0.05	// mes standard deviation m
#define IMU_MEAS_STD 0.5	// mes standard deviation m^2/s

/***************************************
			ISSUE LOG
// predict shall not start until first vision CB

***************************************/

// KF variables
// 6 states: pos & vel
Eigen::Matrix<double, N_STATE, 1> x;
Eigen::Matrix<double, N_STATE, N_STATE> F;
Eigen::Matrix<double, N_STATE, N_MEAS> B;
Eigen::Matrix<double, N_MEAS, 1> u;
Eigen::Matrix<double, N_STATE, N_STATE> P;
Eigen::Matrix<double, N_STATE, N_STATE> Q;
Eigen::Matrix<double, N_MEAS, 1> y;
Eigen::Matrix<double, N_MEAS, 1> z;
Eigen::Matrix<double, N_MEAS, N_STATE> H;
Eigen::Matrix<double, N_MEAS, N_MEAS> S;
Eigen::Matrix<double, N_STATE, N_MEAS> K;
Eigen::Matrix<double, N_MEAS, N_MEAS> R;
Eigen::Matrix<double, N_STATE, N_STATE> eye;

// IMU transformation
Eigen::Matrix<double, 4,1> q;

// for dt in transition model F
ros::Time PrevCbTime;
ros::Time CurCbTime;

ros::Publisher state_pub;
ros::Publisher acc_pub;

// output state
geometry_msgs::TwistWithCovarianceStamped state;

// initialization flag
bool initFlag = false;

void initState(double initX, double initY, double initZ){

	// initial state, assume zero vel
	x << initX, initY, initZ, 0, 0, 0, 0, 0, 0;

	std::cout << "init state = " << x(0,0) << ", " << x(1,0) << ", " << x(2,0) << ", " << x(3,0) << ", " << x(4,0) << ", "<< x(5,0) << std::endl;

	initFlag = true;

}

void visionCb(const geometry_msgs::TransformStampedConstPtr & msg){

	// assign measured pos to z matrix
	z << msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z;

	//std::cout << "test matrix z = " << std::endl;
	//std::cout << z << std::endl;

	H.block<3,3> (0,0) = (Eigen::Matrix<double, 3, 1>() << 1, 1, 1).finished().asDiagonal();

	double meas_cov = pow(CAM_MEAS_STD,2);
	R.block<3,3> (0,0) = (Eigen::Matrix<double, 3, 1>() << meas_cov, meas_cov, meas_cov).finished().asDiagonal();

	// TODO: put these in a function and use data struct to pass them
	// UPDATE!
	y = z - H * x;
	S = H * P * H.transpose() + R;
	K = P * H.transpose() * S;
	x = x + K * y;
	P = (eye - K * H) * P;

	/*
	std::cout << "test matrix y = " << std::endl;
	std::cout << y << std::endl;
	std::cout << "test matrix z = " << std::endl;
	std::cout << z << std::endl;
	std::cout << "test matrix H = " << std::endl;
	std::cout << H << std::endl;
	*/

	/*
	std::cout << "test matrix x = " << std::endl;
	std::cout << x << std::endl;
	
	std::cout << "test matrix K = " << std::endl;
	std::cout << K << std::endl;
	/*
	std::cout << "test matrix P = " << std::endl;
	std::cout << P << std::endl;
	std::cout << "test matrix S = " << std::endl;
	std::cout << S << std::endl;
	*/
	
	std::cout << "----UPDATE----" << std::endl;
	std::cout << "pos = " << x(0,0) << ", " << x(1,0) << ", "<< x(2,0) <<std::endl;
	std::cout << "--measurement--" << std::endl;
	std::cout << "pos = " << z(0,0) << ", " << z(1,0) << ", "<< z(2,0) <<std::endl;

	if (!initFlag)
	{
		initState(z(0,0), z(1,0), z(2,0));
	}

}

void predict(){

	// build B matrix
	//*** dont need it for multi sensor update ***//
	//double integral = 0.5 * pow(deltaT,2);
	//B.block<3,3> (0,0) = (Eigen::Matrix<double, 3, 1>() << integral, integral, integral).finished().asDiagonal();
	//B.block<3,3> (3,0) = (Eigen::Matrix<double, 3, 1>() << deltaT, deltaT, deltaT).finished().asDiagonal();

	//std::cout << "test matrix u = " << std::endl;
	//std::cout << u << std::endl;

	// Q, P are initialize in initMatrices()

	// PREDICT!
	x = F * x;
	P = F * P * F.transpose() + Q;

	//std::cout << "test matrix P = " << std::endl;
	//std::cout << P << std::endl;

	//std::cout << " PREDICTION " << std::endl;
	//std::cout << "pos = " << x(0,0) << ", " << x(1,0) << ", "<< x(2,0) <<std::endl;

	state.twist.twist.linear.x = x(0,0);
	state.twist.twist.linear.y = x(1,0);
	state.twist.twist.linear.z = x(2,0);
	state_pub.publish(state);
	
}

void initMatrices(){

	// initialize F matrix
	double dt = 1.0/PREDICT_RATE;
	double ddt = 0.5 * pow(dt,2);
	// for all
	F.block<9,9> (0,0) = (Eigen::Matrix<double, 9, 1>() << 1, 1, 1, 1, 1, 1, 1, 1, 1).finished().asDiagonal();
	// for pos
	F.block<3,3> (0,3) = (Eigen::Matrix<double, 3, 1>() << dt, dt, dt).finished().asDiagonal();
	F.block<3,3> (0,6) = (Eigen::Matrix<double, 3, 1>() << ddt, ddt, ddt).finished().asDiagonal();
	// for vel
	F.block<3,3> (3,6) = (Eigen::Matrix<double, 3, 1>() << dt, dt, dt).finished().asDiagonal();
	
	// initialize P matrix
  	for( int i = 0; i < N_STATE; i++ )
  	{
  		if (i < 3){
  			P(i,i) = pow(POS_STD,2);}
  		else if( (i >= 3) && (i < 6) ){
  			P(i,i) = pow(VEL_STD,2);}
  		else{
  			P(i,i) = pow(ACC_STD,2);}
  	}

  	// initialize Q matrix
  	// TODO

  	// initialize H matrix
  	//*** dont need it for multi sensor update ***//
	//H.block<3,3> (0,0) = (Eigen::Matrix<double, 3, 1>() << 1, 1, 1).finished().asDiagonal();

	// initialize R matrix
	//*** dont need it for multi sensor update ***//
	//double meas_cov = pow(MEAS_STD,2);
	//R.block<3,3> (0,0) = (Eigen::Matrix<double, 3, 1>() << meas_cov, meas_cov, meas_cov).finished().asDiagonal();

	eye.block<9,9> (0,0) = (Eigen::Matrix<double, 9, 1>() << 1, 1, 1, 1, 1, 1, 1, 1, 1).finished().asDiagonal();

	x.setZero();

	//std::cout << "test matrix F = " << std::endl;
	//std::cout << F << std::endl;
}



int main(int argc, char **argv) {

	ros::init(argc, argv, "imu_vision_fusion");
 	ros::NodeHandle nh;

 	// Publishers
 	state_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("state",1); 

 	acc_pub = nh.advertise<geometry_msgs::Twist>("compensatedACC",1); 

 	// Subscribers
  	//ros::Subscriber imu_sub = nh.subscribe("imu/data", 1, imuCb); 

  	ros::Subscriber vision_sub = nh.subscribe("vision_transformed", 1, visionCb); 

  	// the first time stamped
  	PrevCbTime = ros::Time::now();

  	initMatrices();

  	// test
  	//x(3,0) = 0.1;

  	ros::Rate loop_rate(PREDICT_RATE);
    while(ros::ok())
  	{
  		if (initFlag){
  			predict();
  		}
  		
   		ros::spinOnce();
   		loop_rate.sleep();
  	}

  	ros::spin();
}




