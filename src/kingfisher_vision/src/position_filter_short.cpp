#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include <cmath>
#include <iostream>
#include <vector>
#include "geometry_msgs/Vector3.h"

float lx_window[7];
std::vector<float> lx_vec (lx_window, lx_window+7);
std::vector<float> sorted_lx_vec;
float ly_window[7];
std::vector<float> ly_vec (ly_window, ly_window+7);
std::vector<float> sorted_ly_vec;
float lz_window[7];
std::vector<float> lz_vec (lz_window, lz_window+7);
std::vector<float> sorted_lz_vec;
float ax_window[7];
std::vector<float> ax_vec (ax_window, ax_window+7);
std::vector<float> sorted_ax_vec;
float ay_window[7];
std::vector<float> ay_vec (ay_window, ay_window+7);
std::vector<float> sorted_ay_vec;
float az_window[7];
std::vector<float> az_vec (az_window, az_window+7);
std::vector<float> sorted_az_vec;
float heading_window[7];
std::vector<float> heading_vec (heading_window, heading_window+7);
std::vector<float> sorted_heading_vec;

class PositionFilterShort
{

	// Variable Declarations
	std_msgs::Bool filter_flag;
	geometry_msgs::Twist unfiltered_cam_pose, filtered_cam_pose, filtered_pose;
	/*geometry_msgs::Pose2D unfiltered_heading, filtered_heading; */
	std_msgs::Float32 filtered_heading;
	std_msgs::Int32 frame_drop_counter;

	std_msgs::Int16 gimbal_setting;

	float lx, ly, lz, ax, ay, az; //positions and angles of current frame

	int avg_size; //size of moving average window, in frames
	int avg_index; //initialization of index of moving average window
	int heading_index;

	int window_fill_count;

	float sin_cam_pitch;
	float cos_cam_pitch;

	float unfiltered_yaw;

	float imu_roll, imu_pitch, imu_yaw, mount_angle, quadx, quady, quadz;

	ros::NodeHandle n;

	// Subscribers
	ros::Subscriber filter_flag_sub;
	ros::Subscriber unfiltered_cam_pose_sub;
	ros::Subscriber unfiltered_heading_sub;
	ros::Subscriber gimbal_setting_sub;
	ros::Subscriber imu_sub;

	// Publishers
	ros::Publisher filtered_pose_pub;
	ros::Publisher frame_drop_counter_pub;
	ros::Publisher filtered_heading_pub;

public:
	PositionFilterShort()
	: n()
	{

		filter_flag_sub = n.subscribe("filter_flag", 10, &PositionFilterShort::FilterFlag_Callback, this);
		unfiltered_cam_pose_sub = n.subscribe("kingfisher_vision_short/deckPose", 10, &PositionFilterShort::imageCB, this);
		unfiltered_heading_sub = n.subscribe("kingfisher_vision_short/unfilteredHeading", 10, &PositionFilterShort::Heading_Callback, this);
		imu_sub = n.subscribe("accRPY", 10, &PositionFilterShort::imuCb,this);
		
		filtered_pose_pub = n.advertise<geometry_msgs::Twist>("filtered_pose_short", 10);
		frame_drop_counter_pub = n.advertise<std_msgs::Int32>("frame_drop_counter_short", 10);
		filtered_heading_pub = n.advertise<std_msgs::Float32>("filtered_heading_short", 10);

		avg_size = 7;
		avg_index = 0;
		heading_index = 0;
		frame_drop_counter.data = 0;



	}

	~PositionFilterShort()
	{}

	void imageCB(const geometry_msgs::Twist& msg)
	{
		CamPose_Callback(msg);
		//std::cout << "unfiltered_cam_pose: "<< std::endl << unfiltered_cam_pose << std::endl;

		filter_flag.data = true;
		if (filter_flag.data) {

			Orientation_filter();
			Transform();
			Heading_filter();


			filtered_heading_pub.publish(filtered_heading);
			filtered_pose_pub.publish(filtered_pose);
		}
		
		std::cout << "filtered_cam_pose: "<< std::endl << filtered_cam_pose << std::endl;
		std::cout << "filtered_pose: "<< std::endl << filtered_pose << std::endl;
		//std::cout << window_fill_count << std::endl;

		std::cout << "`Distance to Deck: " << filtered_pose.linear.x << std::endl;
		std::cout << "`Height: " << filtered_pose.linear.z << std::endl;
		std::cout << "`Lateral Offset: " << filtered_pose.linear.y << std::endl;
		//std::cout << "`Lateral Heading: " << filtered_heading.data  << " degrees" << std::endl;

		frame_drop_counter_pub.publish(frame_drop_counter);


	}

	// Orientation Filter
	void Orientation_filter(void) {
		
		lx = unfiltered_cam_pose.linear.x;
		ly = unfiltered_cam_pose.linear.y;
		lz = unfiltered_cam_pose.linear.z;
		ax = unfiltered_cam_pose.angular.x;
		ay = unfiltered_cam_pose.angular.y;
		az = unfiltered_cam_pose.angular.z;
		
		if (lz == -1.0){
			frame_drop_counter.data++;
		}
		// Mean Filter
/*		else if(
			fabs(lx - filtered_cam_pose.linear.x) >= 5 ||
			fabs(ly - filtered_cam_pose.linear.y) >= 5 ||
			fabs(lz - filtered_cam_pose.linear.z) >= 5 ||
			fabs(ax - filtered_cam_pose.angular.x) >= 250 ||
			fabs(ay - filtered_cam_pose.angular.y) >= 250 ||
			fabs(az - filtered_cam_pose.angular.z) >= 250
		) {
			frame_drop_counter.data++;
		}
		else {
			filtered_cam_pose.linear.x = (avg_size * filtered_cam_pose.linear.x - lx_window[avg_index] + lx) / avg_size;
			lx_window[avg_index] = lx;
			filtered_cam_pose.linear.y = (avg_size * filtered_cam_pose.linear.y - ly_window[avg_index] + ly) / avg_size;
			ly_window[avg_index] = ly;
			filtered_cam_pose.linear.z = (avg_size * filtered_cam_pose.linear.z - lz_window[avg_index] + lz) / avg_size;
			lz_window[avg_index] = lz;
			filtered_cam_pose.angular.x = (avg_size * filtered_cam_pose.angular.x - ax_window[avg_index] + ax) / avg_size;
			ax_window[avg_index] = ax;
			filtered_cam_pose.angular.y = (avg_size * filtered_cam_pose.angular.y - ay_window[avg_index] + ay) / avg_size;
			ay_window[avg_index] = ay;
			filtered_cam_pose.angular.z = (avg_size * filtered_cam_pose.angular.z - az_window[avg_index] + az) / avg_size;
			az_window[avg_index] = az;
			

			avg_index++;
			avg_index = avg_index % avg_size;
			std::cout << "avg_index" << avg_index << std::endl;
			frame_drop_counter.data = 0;
		} */

		// Median Filter
		else {
			lx_vec[avg_index] = lx;
			sorted_lx_vec = lx_vec;
			std::sort (sorted_lx_vec.begin(), sorted_lx_vec.end());
			filtered_cam_pose.linear.x = sorted_lx_vec[3];

			ly_vec[avg_index] = ly;
			sorted_ly_vec = ly_vec;
			std::sort (sorted_ly_vec.begin(), sorted_ly_vec.end());
			filtered_cam_pose.linear.y = sorted_ly_vec[3];

			lz_vec[avg_index] = lz;
			sorted_lz_vec = lz_vec;
			std::sort (sorted_lz_vec.begin(), sorted_lz_vec.end());
			filtered_cam_pose.linear.z = sorted_lz_vec[3];

			ax_vec[avg_index] = ax;
			sorted_ax_vec = ax_vec;
			std::sort (sorted_ax_vec.begin(), sorted_ax_vec.end());
			filtered_cam_pose.angular.x = sorted_ax_vec[3];

			ay_vec[avg_index] = ay;
			sorted_ay_vec = ay_vec;
			std::sort (sorted_ay_vec.begin(), sorted_ay_vec.end());
			filtered_cam_pose.angular.y = sorted_ay_vec[3];

			az_vec[avg_index] = az;
			sorted_az_vec = az_vec;
			std::sort (sorted_az_vec.begin(), sorted_az_vec.end());
			filtered_cam_pose.angular.z = sorted_az_vec[3];

			//std::cout << "lx: " << lx << std::endl;
			//std::cout << "filtered_cam_pose x: " << filtered_cam_pose.linear.x << std::endl;
			//std::cout << "sorted data: ";
			/*
			for (int i = 0; i < 7; ++i)
			{
				std::cout << " " << sorted_lx_vec[i];
			}
			*/
			//std::cout << std::endl;
			//std::cout << "Median: " << sorted_lx_vec[15] << std::endl;

			avg_index++;
			avg_index = avg_index % avg_size;

		}
	}

	// Transformation from Camera Frame to Quad Frame
	void Transform(void) {

		//for 30 degree from vertical fixed mount tilt camera
		mount_angle = 30*3.14159/180;
		
		quadx = 0.866*filtered_cam_pose.linear.z+0.5*filtered_cam_pose.linear.x;
		quady = -1*filtered_cam_pose.linear.y;
		quadz = 0.5*filtered_cam_pose.linear.z-0.866*filtered_cam_pose.linear.x;
		
		filtered_pose.linear.x = quadx;
		filtered_pose.linear.y = quady;
		filtered_pose.linear.z = quadz;

		//std::cout << "quadx: " << quadx << std::endl;
		//std::cout << "quady: " << quady << std::endl;
		//std::cout << "quadz: " << quadz << std::endl;

		//filtered_pose.linear.x = cos(imu_yaw)*cos(imu_pitch)*quadx + (cos(imu_yaw)*sin(imu_roll)*sin(imu_pitch) - cos(imu_roll)*sin(imu_yaw))*quady + (sin(imu_roll)*sin(imu_yaw)+cos(imu_roll)*cos(imu_yaw)*sin(imu_pitch))*quadz;
		//filtered_pose.linear.y = cos(imu_pitch)*sin(imu_yaw)*quadx + (cos(imu_roll)*cos(imu_yaw)+sin(imu_roll)*sin(imu_yaw)*sin(imu_pitch))*quady + (cos(imu_roll)*sin(imu_yaw)*sin(imu_pitch)-cos(imu_yaw)*sin(imu_roll))*quadz;
		//filtered_pose.linear.z = -sin(imu_pitch)*quadx + cos(imu_pitch)*sin(imu_roll)*quady + cos(imu_roll)*cos(imu_pitch)*quadz;

		//filtered_pose.linear.x = cos(imu_yaw)*cos(imu_pitch)*quadx + cos(imu_pitch)*sin(imu_yaw)*quady - sin(imu_pitch)*quadz;
		//filtered_pose.linear.y = (cos(imu_yaw)*sin(imu_roll)*sin(imu_pitch) - cos(imu_roll)*sin(imu_yaw))*quadx + (cos(imu_roll)*cos(imu_yaw)+sin(imu_roll)*sin(imu_yaw)*sin(imu_pitch))*quady + cos(imu_pitch)*sin(imu_roll)*quadz;
		//filtered_pose.linear.z = (sin(imu_roll)*sin(imu_yaw) + cos(imu_roll)*cos(imu_yaw)*sin(imu_pitch))*quadx + (cos(imu_roll)*sin(imu_yaw)*sin(imu_pitch)-cos(imu_yaw)*sin(imu_roll))*quady + cos(imu_roll)*cos(imu_pitch)*quadz;

	}

	void Heading_Callback(const geometry_msgs::Twist& msg) {
		unfiltered_yaw = msg.angular.x;
	}

	void imuCb(const geometry_msgs::Vector3& msg) {
		imu_pitch = -msg.x*3.14159/180;
		imu_roll = -msg.y*3.14159/180;
		imu_yaw = msg.z*3.14159/180;
	}


	// Heading Filter
	void Heading_filter(void) {	

		heading_vec[heading_index] = unfiltered_yaw;
		sorted_heading_vec = heading_vec;
		std::sort (sorted_heading_vec.begin(), sorted_heading_vec.end());
		filtered_heading.data = sorted_heading_vec[3];

		heading_index++;
		heading_index = heading_index % avg_size;

	}


	// Callbacks

	void FilterFlag_Callback(const std_msgs::Bool& msg)
	{
		filter_flag.data = msg.data;
	}

	void CamPose_Callback(const geometry_msgs::Twist& msg)
	{
		unfiltered_cam_pose.linear.x = msg.linear.x;
		unfiltered_cam_pose.linear.y = msg.linear.y;
		unfiltered_cam_pose.linear.z = msg.linear.z;
		unfiltered_cam_pose.angular.x = msg.angular.x;
		unfiltered_cam_pose.angular.y = msg.angular.y;
		unfiltered_cam_pose.angular.z = msg.angular.z;
	}

};


int avg_size = 7;
int avg_index = 0;
int heading_index = 0;
int window_fill_count = 0;

float lx = 0;
float ly = 0;
float lz = 0;
float ax = 0;
float ay = 0;
float az = 0; //positions and angles of current frame

float sin_cam_pitch = 0;
float cos_cam_pitch = 0;

float unfiltered_yaw = 0;


// Main Function
int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_filter_short");
	PositionFilterShort pfs;
		
	ros::spin(); //<-- this is where publish happens, and when it polls sub

  return 0;
}