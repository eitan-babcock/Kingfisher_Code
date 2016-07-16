#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>

#define N_MEAS 3		// number of measurements
#define N_STATE 6		// number of states

#define POS_STD 0.5		// pos standard deviation m
#define VEL_STD 6		// vel standard deviation m/s
#define MEAS_STD 0.1	// mes standard deviation m

// this is not accurate, prediction will drift
#define GRAVITY	9.4		// Gravity value from IMU

class SensorFusion
{
public:

private:
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
	// initiation flag
	bool initFlag = false;
};

SensorFusion::SensorFusion()