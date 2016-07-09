#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include <vector>



float lx_window[30]/* = {2.2, 4, 1, 7, 1, 5, 2, 6, 1, 5.4, 3, 4, 3, 8, 7, 5, 9, 4, 7, 3, 5, 4, 3, 6, 4, 6, 4, 9, 8, 7}*/;
//std::queue<float> lx_vec;
std::vector<float> lx_vec(lx_window, lx_window+30);
std::vector<float> sorted_lx_vec;

//float lx[30] = {2, 4, 1, 7, 1, 5, 2, 6, 1, 5, 3, 4, 3, 8, 7, 5, 9, 4, 7, 3, 5, 4, 3, 6, 4, 6, 4, 9, 8, 7};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sort_test");

	lx_vec[3] = 25;
	sorted_lx_vec = lx_vec;
	std::cout << "Unsorted:";
	for (int i = 0; i < 30; ++i)
	{
		std::cout << " " << lx_vec[i];
	}
	std::cout << "\n";

	std::sort (sorted_lx_vec.begin(), sorted_lx_vec.end());

	std::cout << "Sorted:";
	for (int i = 0; i < 30; ++i)
	{
		std::cout << " " << sorted_lx_vec[i];
	}
	std::cout << "\n";

	std::cout << "Unsorted:";
	for (int i = 0; i < 30; ++i)
	{
		std::cout << " " << lx_vec[i];
	}
	std::cout << "\n";

  return 0;
}