#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <math.h>
#include <iostream>

static const std::string OPENCV_WINDOW = "output_video";

static const int NUMOFBEACONS = 4;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher testImage_pub;
  image_transport::Publisher testImage_pub2;

  ros::Publisher deckPose_pub;
  geometry_msgs::Twist deckPose;

  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    testImage_pub = it_.advertise("/image_converter/test_output",1);
    testImage_pub2 = it_.advertise("/image_converter/test_output2",1);
    deckPose_pub = nh_.advertise<geometry_msgs::Twist>("image_converter/deckPose",1);

    cv::namedWindow(OPENCV_WINDOW);



  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr, test_ptr, test_ptr2;
    cv::Mat output_image;
    cv::Mat test_output_image;
    cv::Mat test_output_image2;
    cv::Mat data;
    cv::Mat data2;
    cv::Mat labels;
    cv::Mat centers;
    cv::Mat perspectiveTransformMatrix;
    cv::Mat sourceMatrix;
    cv::Mat targetMatrix;
    cv::Mat cameraMatrix;
    cv::Mat translationVector;
    cv::Mat rotationVector;
    cv::Mat rotationMatrix;
    cv::Mat mtxR;
    cv::Mat mtxQ;
    cv::Mat qX, qY, qZ;
    cv::Vec3d eulerRotationAngles;
    std_msgs::Float64 tempFloat64;


    cv::Mat distortionCoefficients;

    std::vector<cv::Point> nonZeroPointsVector;
    std::vector<cv::Point> cp_vector;
    std::vector<cv::Point2f> source_vector(4);
    std::vector<cv::Point> target_vector (4);
    std::vector<cv::Point> perceived_quad_vector (4);
    std::vector<cv::Point3f> deck_3Dpoints_vector (4);
    cv::Point pt, upperLeftCorner, upperRightCorner, lowerRightCorner, lowerLeftCorner;


    cv::Mat nonZeroLocations;
    int minimumIndex;
    double distance[4], minimumDistance;
    double angle[3], minimumAngle;
    //half the side length of the deck beacons 
    double deckHalfSize = 0.6025; //(m)  





    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      test_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,255,255));


    //threshold image to find bright points
    //////////////////////////////////////////////////////////////
    //TODO TAKE THIS OUT WHEN USING FILTER
    //STILL NEED OUTPUT IMAGE TO BE CREATED THOUGH
    cv::threshold(cv_ptr->image, output_image, 235, 100, 0);
    ////////////////////////////////////////////////////////////////

    cv_ptr->image = output_image;
    //find centroids of each cluster
    cv::findNonZero(output_image,nonZeroLocations);

    nonZeroLocations.convertTo(data, CV_32F);
    //output_image.convertTo(data2, CV_32F);

    //if less than 4 data points remaining, DO NOT RUN K MEANS
    if(data.total() < 4)
        return;

    cv::kmeans(data,NUMOFBEACONS,labels,cv::TermCriteria(0,100,0),10,0,centers);

    //loop through centers and display
        //ROS_INFO("size of output image is:%d x %d",output_image.rows,output_image.cols);
  

    
    for (int i = 0; i<centers.rows;i++)
    {
      pt = centers.at<cv::Point2f>(i,0);
      //debug
      //printf("cp_vector[%d] is: [%d,%d]\n",i,pt.y,pt.x);
      cp_vector.push_back(pt);
      //cv::circle(output_image, pt, 2, 255);
      //ROS_INFO("Center is at: %d,%d", pt.y,pt.x);
    }
    


    //sort centers as: Upper-Left, Upper-Right, Bottom-Right,Bottom-Left
    upperLeftCorner = cv::Point(0,0);
    //upperRightCorner = cv::Point(cv_ptr->image.cols,0);
    //lowerRightCorner = cv::Point(cv_ptr->image.cols,cv_ptr->image.rows);
    //lowerLeftCorner = cv::Point(0,cv_ptr->image.cols);

    //FIND WHICH CENTER SHOULD BE AT POS 0
    for (int i = 0; i < 4; i++)
    {
        distance[i] = sqrt(pow(cp_vector[i].x,2)+pow(cp_vector[i].y,2));
    }

    minimumDistance = 99999999;

    //find minimum distance
    for (int i=0;i<4;i++) {
      if(distance[i] < minimumDistance) {
        minimumDistance = distance[i];
        minimumIndex = i;
        //printf("distance %d is: %f\n",i,distance[i]);
        //printf("minimumDistance is: %f\n",minimumDistance);
        //printf("minimumIndex is: %d\n",minimumIndex);
        }
    }

    //assign to corner
    source_vector[0] = cp_vector[minimumIndex];
    //remove from contention
    cp_vector.erase(cp_vector.begin()+minimumIndex);

    //find other points
    //should be angular clockwise from upperLeft
    angle[0] = fabs(atan2(cp_vector[0].y,cp_vector[0].x));
    angle[1] = fabs(atan2(cp_vector[1].y,cp_vector[1].x));
    angle[2] = fabs(atan2(cp_vector[2].y,cp_vector[2].x));

    //smaller angles should be smaller indices
    //find minimum angle
    minimumAngle = 999;

    for (int i=0;i<3;i++) {
        //printf("angle %d is: %f\n",i,angle[i]);
        //printf("minimumAngle is: %f\n",minimumAngle);

      if(angle[i] < minimumAngle) {
        minimumIndex = i;
        minimumAngle = angle[i];
        }
    }
    //assign point
    source_vector[1] = cp_vector[minimumIndex];
    //remove from contention
    cp_vector.erase(cp_vector.begin()+minimumIndex);

    //find next minimum angle
    angle[0] = atan2(cp_vector[0].y,cp_vector[0].x);
    angle[1] = atan2(cp_vector[1].y,cp_vector[1].x);

    if(angle[0] < angle[1]) {
        source_vector[2] = cp_vector[0];
        source_vector[3] = cp_vector[1];
    }
    else {
        source_vector[2] = cp_vector[1];
        source_vector[3] = cp_vector[0];
    }
/*
    pt = cp_vector[0]- upperRightCorner;
    distance[0] = sqrt(pow(pt.x,2)+pow(pt.y,2));
    pt = cp_vector[1]- upperRightCorner;
    distance[1] = sqrt(pow(pt.x,2)+pow(pt.y,2));
    pt = cp_vector[2]- upperRightCorner;
    distance[2] = sqrt(pow(pt.x,2)+pow(pt.y,2));
    
    minimumDistance = 99999999;

    //find minimum distance
    for (int i=0;i<3;i++) {
      if(distance[i] < minimumDistance)
        minimumIndex = i;
    }

    //assign to corner
    source_vector[1] = cp_vector[minimumIndex];
    //remove from contention
    cp_vector.erase(cp_vector.begin()+minimumIndex);

    //find vector at position 2
    pt = cp_vector[0]- lowerRightCorner;
    distance[0] = sqrt(pow(pt.x,2)+pow(pt.y,2));
    pt = cp_vector[1]- lowerRightCorner;
    distance[1] = sqrt(pow(pt.x,2)+pow(pt.y,2));

    minimumDistance = 99999999;

    //find minimum distance
    for (int i=0;i<2;i++) {
      if(distance[i] < minimumDistance)
        minimumIndex = i;
    }

    //assign to corner
    source_vector[2] = cp_vector[minimumIndex];
    //remove from contention
    cp_vector.erase(cp_vector.begin()+minimumIndex);

    //assign final corner
    source_vector[3] = cp_vector[0];
*/








    /*
    for (int i=0;i<1;i++)
    {
        printf("Source point %d is: [%d,%d]\n",i,source_vector[i].x,source_vector[i].y);
        printf("Target point %d is: [%d,%d]\n",i,target_vector[i].x,target_vector[i].y);
    }
    */

    //debug circles to track which point is which
    for(int i = 0;i<4;i++) {
        for(int j = 0; j < i+1; j++) {
            cv::circle(output_image, source_vector[i], 4*(j+1), 255);
        }
    }


    //debug
/*
    //create target square to match to (10m distance)
    target_vector[0] = cv::Point(460,320);
    target_vector[1] = cv::Point(820,320);
    target_vector[2] = cv::Point(820,704);
    target_vector[3] = cv::Point(460,704);
    
    //create target square to match to (1' distance)
    target_vector[0] = cv::Point(527,392);
    target_vector[1] = cv::Point(753,392);
    target_vector[2] = cv::Point(753,632);
    target_vector[3] = cv::Point(527,632);
    
    
    //use centers for perspective transforms
    sourceMatrix = cv::Mat(source_vector);
    sourceMatrix.convertTo(sourceMatrix,CV_32F);
    targetMatrix = cv::Mat(target_vector);
    targetMatrix.convertTo(targetMatrix,CV_32F);
    perspectiveTransformMatrix = cv::getPerspectiveTransform(sourceMatrix,targetMatrix);


    //test perspective transform

    //unsure if this should be inverse of perspective transform matrix or not
    cv::warpPerspective(test_ptr->image,test_output_image,perspectiveTransformMatrix,test_ptr->image.size());


    //draw output of perspective transforms
    test_ptr->image = test_output_image;
    testImage_pub.publish(test_ptr->toImageMsg());
*/


    //solvePnp
    //make deck 3D points (in meters)
    deck_3Dpoints_vector[0] = cv::Point3f(-deckHalfSize,deckHalfSize,0);
    deck_3Dpoints_vector[1] = cv::Point3f(deckHalfSize,deckHalfSize,0);
    deck_3Dpoints_vector[2] = cv::Point3f(deckHalfSize,-deckHalfSize,0);
    deck_3Dpoints_vector[3] = cv::Point3f(-deckHalfSize,-deckHalfSize,0);

    if(true) {
    //make camera matrix
    cameraMatrix = (cv::Mat_<double>(3,3) << 2810, 0, 640, 0 , 2810, 512, 0, 0, 1);
    }
    else if(false) {
    //make camera matrix
    cameraMatrix = (cv::Mat_<double>(3,3) << 1560, 0, 640, 0 , 1560, 512, 0, 0, 1);    
    }
    else {
        std::cout << "Something is wrong: neither camera flag is set" << std::endl;
        }


    cv::solvePnP(deck_3Dpoints_vector,source_vector,cameraMatrix,distortionCoefficients,rotationVector,translationVector);
    
    //convert rotation vecotr to useable rotation matrix
    cv::Rodrigues(rotationVector,rotationMatrix);
    //extract roll, pitch yaw from rotation matrix
    eulerRotationAngles = cv::RQDecomp3x3(rotationMatrix,mtxR,mtxQ,qX,qY,qZ);
    //print output rotation angles
    std::cout << "eulerRotationAngles: " << eulerRotationAngles << std::endl;

    //print output of solvePnp
    std::cout << "rotationMatrix = " << std::endl << " " << rotationMatrix << std::endl << std::endl; 
    std::cout << "rvec: " << rotationVector << std::endl;
    std::cout << "tvec: " << translationVector << std::endl;


    //publish pose
    //tempFloat64.data = translationVector.at<double>(0);
    //deckPose.linear.x = tempFloat64.data;
    deckPose.linear.x = translationVector.at<double>(0);
    deckPose.linear.y = translationVector.at<double>(1);
    deckPose.linear.z = translationVector.at<double>(2);
    deckPose.angular.x = eulerRotationAngles[0];
    deckPose.angular.y = eulerRotationAngles[1];
    deckPose.angular.z = eulerRotationAngles[2];
    
    deckPose_pub.publish(deckPose);

 
    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(deck_3Dpoints_vector, rotationVector, translationVector, cameraMatrix, distortionCoefficients, projectedPoints);
 
    for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
        std::cout << "Image point: " << source_vector[i] << " Projected to " << projectedPoints[i] << std::endl;
    }
 

    //printf("angle %d is: %f\n",i,angle[i]);
    //printf("minimumAngle is: %f\n",minimumAngle);

    //test_ptr2->image = test_output_image2;


    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
