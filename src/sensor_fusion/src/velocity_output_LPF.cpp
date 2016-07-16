#include "ros/ros.h"
#include <stdlib.h>
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#define SAMPLING_RATE 100

ros::Publisher vel_pub;

float posX, posY, posZ;
float lastPosX, lastPosY, lastPosZ;
geometry_msgs::Vector3 velocity;


void stateCb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
{
	posX = msg->twist.twist.linear.x;
	posY = msg->twist.twist.linear.y;
	posZ = msg->twist.twist.linear.z;
}


float filter(float input)
{
	float m00, m01, output;
	float a0 = 0;
	float a1 = 0.9981;
	float b1 = -0.001867;

	m00 = input - b1*m01;
	output = a0*m00 + a1*m01;
	m01 = m00;
	
	return output;
}


void calVelocity()
{
	float velx, vely, velz;
	velx = (posX - lastPosX) * SAMPLING_RATE;
	vely = (posY - lastPosY) * SAMPLING_RATE;
	velz = (posZ - lastPosZ) * SAMPLING_RATE;

	velocity.z = vely;
	velocity.y = filter(vely);
	//velocity.x = filter(velx);
	
	//velocity.z = filter(velz);

	lastPosX = posX;
	lastPosY = posY;
	lastPosZ = posZ;

	std::cout << " Velocity in x  : " << velocity.x << std::endl;
	std::cout << " Velocity in y  : " << velocity.y << std::endl;
	std::cout << " Velocity in z  : " << velocity.z << std::endl;

	vel_pub.publish(velocity);
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "velocity_output");
	ros::NodeHandle nh;
	ros::Rate loop_rate(SAMPLING_RATE);

	vel_pub = nh.advertise<geometry_msgs::Vector3>("state_vel",1);

	ros::Subscriber state_sub = nh.subscribe("state",1, stateCb);

	while (ros::ok())
	{
		
		calVelocity();

		ros::spinOnce();
		loop_rate.sleep();
	}
}