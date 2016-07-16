#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>

#define N_MEAS 3		// number of measurements
#define N_STATE 9		// number of states, pos, vel, bias

/*
#define POS_STD 0.5		// pos standard deviation m
#define VEL_STD 3		// vel standard deviation m/s
#define BIAS_STD 0.05	// bias standard deviation m^2/s
#define MEAS_STD 0.001	// mes standard deviation m
*/

double posSTD, velSTD, biasSTD, visionSTD;

// this is not accurate, prediction will drift
#define GRAVITY	9.5		// Gravity value from IMU

// KF variables
// 0 states: pos, vel, bias
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
ros::Publisher bias_pub;
ros::Publisher error_pub;

// output state
geometry_msgs::TwistWithCovarianceStamped state;
geometry_msgs::Twist error;
geometry_msgs::Twist estimatedBias;

// flag for initialization
bool initFlag = false;

double biasNoise;
double posNoise;
double velNoise;
double biasInitValue;

void predict(double deltaT){

	// build F matrix
	F.block<3,3> (0,3) = (Eigen::Matrix<double, 3, 1>() << deltaT, deltaT, deltaT).finished().asDiagonal();
	F.block<3,3> (3,6) = (Eigen::Matrix<double, 3, 1>() << -deltaT, -deltaT, -deltaT).finished().asDiagonal();

	//std::cout << "test matrix F = " << std::endl;
	//std::cout << F << std::endl;

	// build B matrix
	B.block<3,3> (3,0) = (Eigen::Matrix<double, 3, 1>() << deltaT, deltaT, deltaT).finished().asDiagonal();

	//std::cout << "test matrix B = " << std::endl;
	//std::cout << B << std::endl;

	// Q, P are initialize in initMatrices()

	// PREDICT!
	x = F * x + B * u;
	P = F * P * F.transpose() + Q;


	//std::cout << "test matrix P = " << std::endl;
	//std::cout << P << std::endl;

	std::cout << "---PREDICTION---" << std::endl;
	std::cout << "pos = " << x(0,0) << ", " << x(1,0) << ", "<< x(2,0) <<std::endl;

	state.twist.twist.linear.x = x(0,0);
	state.twist.twist.linear.y = x(1,0);
	state.twist.twist.linear.z = x(2,0);
	state.twist.twist.angular.x = x(3,0);
	state.twist.twist.angular.y = x(4,0);
	state.twist.twist.angular.z = x(5,0);
	state_pub.publish(state);

	estimatedBias.linear.x = x(6,0);
	estimatedBias.linear.y = x(7,0);
	estimatedBias.linear.z = x(8,0);
	bias_pub.publish(estimatedBias);
}

void update(){

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
	
	std::cout << "test matrix P = " << std::endl;
	std::cout << P << std::endl;

	std::cout << "test matrix S = " << std::endl;
	std::cout << S << std::endl;

	*/
	std::cout << "----UPDATE----" << std::endl;
	std::cout << "pos = " << x(0,0) << ", " << x(1,0) << ", "<< x(2,0) <<std::endl;
	std::cout << "--measurement--" << std::endl;
	std::cout << "pos = " << z(0,0) << ", " << z(1,0) << ", "<< z(2,0) <<std::endl;

	error.linear.x = z(0,0) - x(0,0);
	error.linear.y = z(1,0) - x(1,0);
	error.linear.z = z(2,0) - x(2,0);
	error_pub.publish(error);

}

void accCompensation(double accX, double accY, double accZ)
{
	// debug use
	geometry_msgs::Twist compensatedACC;

	double g[3]; 
	double linearAccX, linearAccY, linearAccZ;

	// gravity compensation
	g[0] = 2 * (q(1,0) * q(3,0) - q(0,0) * q(2,0));
    g[1] = 2 * (q(0,0) * q(1,0) + q(2,0) * q(3,0));
 	g[2] = q(0,0) * q(0,0) - q(1,0) * q(1,0) - q(2,0) * q(2,0) + q(3,0) * q(3,0);

	// compensated linear acceleration
	linearAccX = accX - g[0]*GRAVITY;
	linearAccY = accY - g[1]*GRAVITY;
	linearAccZ = accZ - g[2]*GRAVITY;
	
	/*
	std::cout << "--compensated Acc--" << std::endl;
	std::cout << "     AccX    : " << linearAccX << std::endl;
	std::cout << "     AccY    : " << linearAccY << std::endl;
	std::cout << "     AccZ    : " << linearAccZ << std::endl;
	std::cout << "------garvity------" << std::endl;
	std::cout << "     g[0]    : " << g[0] << std::endl;
	std::cout << "     g[1]    : " << g[1] << std::endl;
	std::cout << "     g[2]    : " << g[2] << std::endl;
	*/

	u << linearAccX, linearAccY, linearAccZ;

	compensatedACC.linear.x = linearAccX;
	compensatedACC.linear.y = linearAccY;
	compensatedACC.linear.z = linearAccZ;
	acc_pub.publish(compensatedACC);
}


void imuCb(const sensor_msgs::ImuConstPtr & msg){
	
	// calculate dt for transition model F
	CurCbTime = ros::Time::now();
	ros::Duration dt(CurCbTime - PrevCbTime);
	
	// quaternion for IMU gravity compensation
	q << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;

	accCompensation(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

	// dont predict() until the first init() in update()
	if (initFlag)
	{
		predict(dt.toSec());
	}

	PrevCbTime = CurCbTime;	
}


void initState(double initX, double initY, double initZ){

	// initial state, assume zero vel
	x << initX, initY, initZ, 0, 0, 0, biasInitValue, biasInitValue, biasInitValue;

	std::cout << "init state = " << x(0,0) << ", " << x(1,0) << ", " << x(2,0) << ", " << x(3,0) << ", " << x(4,0) << ", "<< x(5,0) << std::endl;

	initFlag = true;

}

void visionCb(const geometry_msgs::TransformStampedConstPtr & msg){

	// assign measured pos to z matrix
	z << msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z;

	//std::cout << "test matrix z = " << std::endl;
	//std::cout << z(0,0) << ", " << z(1,0) << ", " << z(2,0) << std::endl;

	if (!initFlag)
	{
		initState(z(0,0), z(1,0), z(2,0));
	}

	update();


}

void initMatrices(){

	// initialize F matrix (partial)
	F.block<9,9> (0,0) = (Eigen::Matrix<double, 9, 1>() << 1, 1, 1, 1, 1, 1, 1, 1, 1).finished().asDiagonal();
	
	// initialize P matrix
  	for( int i = 0; i < N_STATE; i++ )
  	{
  		if (i < 3){
  			P(i,i) = pow(posSTD,2);}
  		else if( (i >= 3) && (i < 6) ){
  			P(i,i) = pow(velSTD,2);}
  		else{
  			P(i,i) = pow(biasSTD,2);}
  	}
	
  	// initialize Q matrix
  	// To Fix
  	Q.block<3,3> (0,0) = (Eigen::Matrix<double, 3, 1>() << posNoise, posNoise, posNoise).finished().asDiagonal();
  	Q.block<3,3> (4,4) = (Eigen::Matrix<double, 3, 1>() << velNoise, velNoise, velNoise).finished().asDiagonal();
  	Q.block<3,3> (6,6) = (Eigen::Matrix<double, 3, 1>() << biasNoise, biasNoise, biasNoise).finished().asDiagonal();
	
  	// initialize H matrix
	H.block<3,3> (0,0) = (Eigen::Matrix<double, 3, 1>() << 1, 1, 1).finished().asDiagonal();
	
	// initialize R matrix
	double meas_cov = pow(visionSTD,2);
	R.block<3,3> (0,0) = (Eigen::Matrix<double, 3, 1>() << meas_cov, meas_cov, meas_cov).finished().asDiagonal();
	
	eye.block<9,9> (0,0) = (Eigen::Matrix<double, 9, 1>() << 1, 1, 1, 1, 1, 1, 1, 1, 1).finished().asDiagonal();
	
	//std::cout << "test matrix eye = " << std::endl;
	//std::cout << eye << std::endl;
	
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "imu_vision_bias_fusion");
 	ros::NodeHandle nh;

 	state_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("state",1); 

 	acc_pub = nh.advertise<geometry_msgs::Twist>("compensatedACC",1);

 	bias_pub = nh.advertise<geometry_msgs::Twist>("estimatedBias",1); 
 
	error_pub = nh.advertise<geometry_msgs::Twist>("error_vision_state",1); 

  	ros::Subscriber imu_sub = nh.subscribe("imu/data", 1, imuCb); 

  	ros::Subscriber vision_sub = nh.subscribe("vision_transformed", 1, visionCb); 

  	PrevCbTime = ros::Time::now();
 	
 	// INPUT
 	// posSTD, velSTD, bias_STD, visionSTD, biasNoise, biasInitValue;

  	if( argc >= 6)
  	{
    posSTD = atof(argv[1]);
    velSTD = atof(argv[2]);
    biasSTD = atof(argv[3]);
    visionSTD = atof(argv[4]);
    biasNoise = atof(argv[5]);
    posNoise = atof(argv[6]);
    velNoise = atof(argv[7]);
    biasInitValue = atof(argv[8]);
  	}

  	// 0.2 0.05 0.03 0.01 0.1 0.05 0.001 0.05

  	ROS_INFO_STREAM("		Input Parameters		");
  	ROS_INFO_STREAM("posSTD: " << posSTD << "  velSTD: " << velSTD << "  biasSTD: " << biasSTD << "  visionSTD: " << visionSTD);
  	ROS_INFO_STREAM("posNoise: " << posNoise << "  velNoise: " << velNoise << "  biasNoise: " << biasNoise << "  biasInitValue: " << biasInitValue);

  	initMatrices();

  	ros::spin();
}
