#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
//#include <Eigen/Eigen>
//#include <Eigen/Geometry>
//#include <vector>

/***************************************
			ISSUE LOG
1. whats the covariance between x&z and y%z
2. covariance between pos&vel for P or Q
3. how to set Q, white noise mdoel?
4. true gravity value? integral in prediction will explose
5. imu orientation, need acceleration in inertial frame
6. first Current time is not good enough

7. using bag file cannot pause, integral with time will explode
8. who should publish state?
9. tune the balance between prediction and update, update should be way more trusted
10. need to calibrate the gravity (hardware as well)
11. it's not stable, sometimes the value will explode. 



Bias:
1. bias = -(2/3)/(dt^2)*(z-x)
2. calibrate g first? maybe it can be includ in the bias.
***************************************/


#define N_MEAS 3		// number of measurements
#define N_STATE 6		// number of states

#define POS_STD 0.5		// pos standard deviation m
#define VEL_STD 6		// vel standard deviation m/s
#define MEAS_STD 0.1	// mes standard deviation m

// this is not accurate, prediction will drift
#define GRAVITY	9.4		// Gravity value from IMU

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

/* error doest not name a type
x.setZero();
F.setZero();
P.setZero();
Q.setZero();
y.setZero();
z.setZero();
H.setZero();
K.setZero();
R.setZero();
*/

bool initFlag = false;

void reset(double posX, double posY, double posZ){

	for( int i = 0; i < N_STATE; i++ )
  	{
  		if (i < 3)
  		{
  			P(i,i) = pow(POS_STD,2);
  		}
  		else
  		{
  			P(i,i) = pow(VEL_STD,2);
  		}
  	}

  	x << posX, posY, posZ, 0, 0, 0;

  	std::cout << " ** RESET ** " << std::endl;
	std::cout << "state = " << x(0,0) << ", " << x(1,0) << ", "<< x(2,0) << ", "<< x(3,0) << ", " << x(4,0) << ", "<< x(5,0) << std::endl;

}


void predict(double deltaT){

	// build F matrix
	F.block<3,3> (0,3) = (Eigen::Matrix<double, 3, 1>() << deltaT, deltaT, deltaT).finished().asDiagonal();

	//std::cout << "test matrix F = " << std::endl;
	//std::cout << F << std::endl;

	// build B matrix
	double integral = 0.5 * pow(deltaT,2);
	B.block<3,3> (0,0) = (Eigen::Matrix<double, 3, 1>() << integral, integral, integral).finished().asDiagonal();
	B.block<3,3> (3,0) = (Eigen::Matrix<double, 3, 1>() << deltaT, deltaT, deltaT).finished().asDiagonal();

	//std::cout << "test matrix u = " << std::endl;
	//std::cout << u << std::endl;

	// Q, P are initialize in initMatrices()

	// PREDICT!
	x = F * x + B * u;
	P = F * P * F.transpose() + Q;


	//std::cout << "test matrix P = " << std::endl;
	//std::cout << P << std::endl;

	std::cout << " PREDICTION " << std::endl;
	std::cout << "pos = " << x(0,0) << ", " << x(1,0) << ", "<< x(2,0) <<std::endl;

	state.twist.twist.linear.x = x(0,0);
	state.twist.twist.linear.y = x(1,0);
	state.twist.twist.linear.z = x(2,0);
	state_pub.publish(state);
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

}

void initState(double initX, double initY, double initZ){

	// initial state, assume zero vel
	x << initX, initY, initZ, 0, 0, 0;

	std::cout << "init state = " << x(0,0) << ", " << x(1,0) << ", " << x(2,0) << ", " << x(3,0) << ", " << x(4,0) << ", "<< x(5,0) << std::endl;

	initFlag = true;

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

void visionCb(const geometry_msgs::TransformStampedConstPtr & msg){

	// assign measured pos to z matrix
	z << msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z;

	//std::cout << "test matrix z = " << std::endl;
	//std::cout << z << std::endl;

	if (!initFlag)
	{
		initState(z(0,0), z(1,0), z(2,0));
	}

	update();

	// acc drifts bad, try to reset everytime vision update
	reset(z(0,0), z(1,0), z(2,0));
}

void initMatrices(){

	// initialize F matrix (partial)
	F.block<3,3> (3,3) = (Eigen::Matrix<double, 3, 1>() << 1, 1, 1).finished().asDiagonal();
	F.block<3,3> (0,0) = (Eigen::Matrix<double, 3, 1>() << 1, 1, 1).finished().asDiagonal();

	// initialize P matrix
  	for( int i = 0; i < N_STATE; i++ )
  	{
  		if (i < 3)
  		{
  			P(i,i) = pow(POS_STD,2);
  		}
  		else
  		{
  			P(i,i) = pow(VEL_STD,2);
  		}
  	}

  	// initialize Q matrix
  	// TODO

  	// initialize H matrix
	H.block<3,3> (0,0) = (Eigen::Matrix<double, 3, 1>() << 1, 1, 1).finished().asDiagonal();

	// initialize R matrix
	double meas_cov = pow(MEAS_STD,2);
	R.block<3,3> (0,0) = (Eigen::Matrix<double, 3, 1>() << meas_cov, meas_cov, meas_cov).finished().asDiagonal();

	eye.block<6,6> (0,0) = (Eigen::Matrix<double, 6, 1>() << 1, 1, 1, 1, 1, 1).finished().asDiagonal();

	//std::cout << "test matrix I = " << std::endl;
	//std::cout << eye << std::endl;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "imu_vision_fusion");
 	ros::NodeHandle nh;

 	state_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("state",1); 

 	acc_pub = nh.advertise<geometry_msgs::Twist>("compensatedACC",1); 

  	ros::Subscriber imu_sub = nh.subscribe("imu/data", 1, imuCb); 

  	ros::Subscriber vision_sub = nh.subscribe("vision_transformed", 1, visionCb); 

  	PrevCbTime = ros::Time::now();

  	initMatrices();

  	ros::spin();
}
