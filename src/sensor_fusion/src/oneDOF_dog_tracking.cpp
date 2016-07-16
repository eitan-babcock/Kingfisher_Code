#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define SAMPLING_TIME 0.01 //sec
#define RUN_TIME 20 //sec

typedef struct
{
  float pos;
  float vel;
  float measure;
}dog;

dog dogMove;

float measurement_var = 2;
float process_var = 1;
float vel = 1;

const int points = RUN_TIME/SAMPLING_TIME;
float dx[points], x[points], measurement[points];
float prediction[points], predict_var[points], state[points], state_var[points];


int i = 1;		// for dog simulation

/********************************************* 
  generate normal distribution random number
*********************************************/
double rand_normal(double mean, double stddev)
{//Box muller method
    static double n2 = 0.0;
    static int n2_cached = 0;
    if (!n2_cached)
    {
        double x, y, r;
        do
        {
            x = 2.0*rand()/RAND_MAX - 1;
            y = 2.0*rand()/RAND_MAX - 1;

            r = x*x + y*y;
        }
        while (r == 0.0 || r > 1.0);
        {
            double d = sqrt(-2.0*log(r)/r);
            double n1 = x*d;
            n2 = y*d;
            double result = n1*stddev + mean;
            n2_cached = 1;
            return result;
        }
    }
    else
    {
        n2_cached = 0;
        return n2*stddev + mean;
    }
}


/********************************************* 
          prediction
*********************************************/
void predict(){
    prediction[i] = state[i-1] + vel*SAMPLING_TIME;
    predict_var[i] = state_var[i-1] + process_var;
}


/********************************************* 
          update
*********************************************/
void update(){
    state[i] = (predict_var[i]*measurement[i] + measurement_var*prediction[i]) / (predict_var[i] + measurement_var);
    state_var[i] = 1 / (1/predict_var[i] + 1/measurement_var);
}


/********************************************* 
  generate dog motion
*********************************************/
dog dogSimulation(){
	
	dx[i] = vel + rand_normal(0,1)*sqrt(process_var);
	x[i] = x[i-1] + dx[i]*SAMPLING_TIME;
	measurement[i] = x[i] + rand_normal(0,1)*sqrt(measurement_var);

	/*
	// debugging use
	std::cout << "dx:           " << dx[i] << std::endl;
	std::cout << "x :           " << x[i] << std::endl;
	std::cout << "measurement : " << measurement[i] << std::endl;
	std::cout << "---------------------" << std::endl;
	*/
	

	if (i == 2000){
		exit(1);
	}

	dogMove.pos = x[i];
	dogMove.vel = dx[i];
	dogMove.measure = measurement[i];


	return dogMove;
}



int main(int argc, char **argv) {

	ros::init(argc, argv, "oneDOF_dog_tracking");
 	ros::NodeHandle nh;

 	ros::Rate loop_rate(1/SAMPLING_TIME);

  geometry_msgs::Twist result;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/oneDOF_KF",100); 

 	dx[0] = 0;
	x[0] = 0;
	measurement[0] = x[0];
  state_var[0] = 400;
  state[0] = 0;

 	while(ros::ok())
	{
 	
 		dogSimulation();

    predict();

    update();

    //std::cout << ""
 		//std::cout << "state_var[0]:           " << state_var[0] << std::endl;

    /*
    // debugging use
 		std::cout << "dx:           " << dogMove.vel << std::endl;
	  std::cout << "x :           " << dogMove.pos << std::endl;
	  std::cout << "measurement : " << dogMove.measure << std::endl;
    std::cout << "prediction:   " << prediction[i] << std::endl;
    std::cout << "estimation :  " << state[i] << std::endl;
	  std::cout << "---------------------" << std::endl;
    */
    result.linear.x = dogMove.pos;
    result.linear.y = dogMove.measure;
    result.linear.z = prediction[i];
    result.angular.x = state[i];
    result.angular.y = predict_var[i];
    result.angular.z = state_var[i];

    pub.publish(result);

    i++;
 		ros::spinOnce();
 		loop_rate.sleep();

	 }

 	return 0;
	
}