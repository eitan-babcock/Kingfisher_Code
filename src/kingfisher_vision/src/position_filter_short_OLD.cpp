#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include <iostream>
#include <math.h>



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

	float lx_window[30];
	float ly_window[30];
	float lz_window[30];
	float ax_window[30];
	float ay_window[30];
	float az_window[30];

	int avg_size; //size of moving average window, in frames
	int avg_index; //initialization of index of moving average window

	int window_fill_count;

	float sin_cam_pitch;
	float cos_cam_pitch;

	ros::NodeHandle n;

	// Subscribers
	ros::Subscriber filter_flag_sub;
	ros::Subscriber unfiltered_cam_pose_sub;
	ros::Subscriber unfiltered_heading_sub;
	ros::Subscriber gimbal_setting_sub;

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
		// unfiltered_heading_sub = n.subscribe("unfiltered_heading", 10, Heading_Callback);
		gimbal_setting_sub = n.subscribe("gimbal_setting", 10, &PositionFilterShort::GimbalSetting_Callback, this);

		filtered_pose_pub = n.advertise<geometry_msgs::Twist>("filtered_pose", 10);
		frame_drop_counter_pub = n.advertise<std_msgs::Int32>("frame_drop_counter", 10);
		/* filtered_heading_pub = n.advertise<geometry_msgs::Pose2D>("filtered_heading", 10); */

		avg_size = 30;
		avg_index = 0;
		window_fill_count = 0;
		frame_drop_counter.data = 0;



	}

	~PositionFilterShort()
	{}

	void imageCB(const geometry_msgs::Twist& msg)
	{
		CamPose_Callback(msg);
		std::cout << "unfiltered_cam_pose: "<< std::endl << unfiltered_cam_pose << std::endl;

		filter_flag.data = true;
		if (filter_flag.data) {
			if (window_fill_count < 30) {
				Fill_queue();
			}
			else {
				Orientation_filter();
				std::cout << "filtered_cam_pose: "<< std::endl << filtered_cam_pose << std::endl;
				Transform();
				Heading_filter();

				gimbal_setting.data = 2;
				if (gimbal_setting.data != 0) {
					filtered_pose_pub.publish(filtered_pose);
				}
			}
			
		}
		std::cout << "filtered pose " << filtered_pose << std::endl;
		std::cout << " window fill count " << window_fill_count << std::endl;

		frame_drop_counter_pub.publish(frame_drop_counter);
		// filtered_heading_pub.publish(filtered_heading);

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
		
		else if(
			fabs(lx - filtered_cam_pose.linear.x) >= 10 ||
			fabs(ly - filtered_cam_pose.linear.y) >= 10 ||
			fabs(lz - filtered_cam_pose.linear.z) >= 10 ||
			fabs(ax - filtered_cam_pose.angular.x) >= 300 ||
			fabs(ay - filtered_cam_pose.angular.y) >= 300 ||
			fabs(az - filtered_cam_pose.angular.z) >= 300
		) {
			frame_drop_counter.data++;
		}
		else {
			//std::cout << "old filtered x " << filtered_cam_pose.linear.x << std::endl;
			filtered_cam_pose.linear.x = (avg_size * filtered_cam_pose.linear.x - lx_window[avg_index] + lx) / avg_size;
			/*std::cout << "filtered x " << filtered_cam_pose.linear.x << std::endl;
			std::cout << "lx " << lx << std::endl;
			std::cout << "old lx " << lx_window[avg_index] << std::endl;
			std::cout << "index " << avg_index << std::endl;*/
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
			frame_drop_counter.data = 0;



		}
	}

	void Fill_queue(void) {
		std::cout << "unfiltered cam pose " << unfiltered_cam_pose << std::endl;
		if (unfiltered_cam_pose.linear.z != -1) {
			lx_window[window_fill_count] = unfiltered_cam_pose.linear.x;
			filtered_cam_pose.linear.x = (filtered_cam_pose.linear.x * window_fill_count + lx_window[window_fill_count]) / (window_fill_count + 1);
			ly_window[window_fill_count] = unfiltered_cam_pose.linear.y;
			filtered_cam_pose.linear.y = (filtered_cam_pose.linear.y * window_fill_count + ly_window[window_fill_count]) / (window_fill_count + 1);
			lz_window[window_fill_count] = unfiltered_cam_pose.linear.z;
			filtered_cam_pose.linear.z = (filtered_cam_pose.linear.z * window_fill_count + lz_window[window_fill_count]) / (window_fill_count + 1);
			ax_window[window_fill_count] = unfiltered_cam_pose.angular.x;
			filtered_cam_pose.angular.x = (filtered_cam_pose.angular.x * window_fill_count + ax_window[window_fill_count]) / (window_fill_count + 1);
			ay_window[window_fill_count] = unfiltered_cam_pose.angular.y;
			filtered_cam_pose.angular.y = (filtered_cam_pose.angular.y * window_fill_count + ay_window[window_fill_count]) / (window_fill_count + 1);
			az_window[window_fill_count] = unfiltered_cam_pose.angular.z;
			filtered_cam_pose.angular.z = (filtered_cam_pose.angular.z * window_fill_count + az_window[window_fill_count]) / (window_fill_count + 1);
			std::cout << "lx window init" << lx_window[window_fill_count] << std::endl;
			window_fill_count++;


		}
	}

	// Transformation from Camera Frame to Quad Frame
	void Transform(void) {

		//not using while gimbal is broken
		/*
		if (gimbal_setting.data == 1) {
			sin_cam_pitch = 0.515;
			cos_cam_pitch = 0.857;
		}
		else {
			sin_cam_pitch = 0.899;
			cos_cam_pitch = 0.438;
			
		}
		*/
		
				//for 30 degree from vertical fixed mount tilt camera
		filtered_pose.linear.x = 0.866*filtered_cam_pose.linear.z-0.5*filtered_cam_pose.linear.y;
		filtered_pose.linear.y = -1*filtered_cam_pose.linear.x;
		filtered_pose.linear.z = 0.5*filtered_cam_pose.linear.z+0.866*filtered_cam_pose.linear.y;
	}


	// Heading Filter
	void Heading_filter(void) {
		filtered_heading.data = filtered_pose.linear.y;
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

	/* 
	void Heading_Callback(const geometry_msgs::Pose2D msg)
	{
		unfiltered_heading.linear.x = msg.linear.x;
		unfiltered_heading.linear.y = msg.linear.y;
		unfiltered_heading.angular.x = msg.angular.x;
		unfiltered_heading.angular.y = msg.angular.y;
	}
	*/

	void GimbalSetting_Callback(const std_msgs::Int16& msg)
	{
		gimbal_setting.data = msg.data;
	}

};


int avg_size = 30;
int avg_index = 0;
int window_fill_count = 0;

float lx = 0;
float ly = 0;
float lz = 0;
float ax = 0;
float ay = 0;
float az = 0; //positions and angles of current frame

float sin_cam_pitch = 0;
float cos_cam_pitch = 0;

float lx_window[30] = {0};
float ly_window[30] = {0};
float lz_window[30] = {0};
float ax_window[30] = {0};
float ay_window[30] = {0};
float az_window[30] = {0};


// Main Function
int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_filter_short");
	PositionFilterShort pfs;
		
	ros::spin(); //<-- this is where publish happens, and when it polls sub

  return 0;
}