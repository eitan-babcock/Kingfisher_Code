#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <time.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include "ocam_functions.h"


#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64
#define EPSILON 1E-5

static const std::string OPENCV_WINDOW = "output_video";

static const int NUMOFBEACONS = 4;




class KingfisherVisionShort
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  //image_transport::Subscriber state_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher testImage_pub;
  image_transport::Publisher testImage_pub2;

  ros::Publisher deckPose_pub;
  ros::Publisher unfilteredHeading_pub;
  ros::Publisher quaternion_pub;
  ros::Publisher transformStamped_pub;
  ros::Publisher vision_OK_flag_pub;
  geometry_msgs::Twist deckPose;
  geometry_msgs::Twist unfilteredHeading;
  geometry_msgs::Quaternion quaternion_msg;
  geometry_msgs::Quaternion previous_quaternion_msg;
  geometry_msgs::TransformStamped transformStamped_msg;
  geometry_msgs::TransformStamped previous_transformStamped_msg;
  geometry_msgs::Transform transform_msg;
  geometry_msgs::Transform previous_transform_msg;
  geometry_msgs::Transform predicted_transform_msg;
  geometry_msgs::Vector3 vector3_msg;
  geometry_msgs::TwistWithCovarianceStamped state_msg;
  std_msgs::Header header_msg;
  std_msgs::String cameraFrameString_msg;
  std_msgs::String deckFrameString_msg;
  std_msgs::UInt32 sequence_msg;
  std_msgs::Time stampTime_msg;

  std::vector<cv::Point> tracked_centers_vector;
  std::vector<cv::Point2d> previous_source_vector;
  std::vector<cv::Point3d> deck_3Dpoints_vector;

  std::vector<cv::Point2d> darkIntersections;
  std::vector<cv::Point2d> darkIntersectionParentA;
  std::vector<cv::Point2d> darkIntersectionParentB;
  std::vector<cv::Point2d> darkIntersectionParentC;
  std::vector<cv::Point2d> darkIntersectionParentD;

  std::vector<cv::Point2d> previousDarkIntersections;
  std::vector<cv::Point2d> previousDarkIntersectionParentA;
  std::vector<cv::Point2d> previousDarkIntersectionParentB;
  std::vector<cv::Point2d> previousDarkIntersectionParentC;
  std::vector<cv::Point2d> previousDarkIntersectionParentD;

  cv::Mat previousConnectedComponentLabels;
  cv::Mat previousConnectedComponentCenters;
  cv::Mat previousConnectedComponentStats;
  int prevNumOfConnectedComponents;


  CvMat* mapx_persp;
  CvMat* mapy_persp;
  IplImage src1;
  IplImage* dst_persp;

  int rectificationInitialized;
  int beacons_detected;
  int previous_frame;
  int previous_frame_valid;
  int detectedBeaconRadius;
  int predicted_transform_valid;
  
  std_msgs::Bool vision_OK_flag;

  cv::Mat previous_frame_output_image;

  cv::Mat cameraMatrix;

  double deckHalfSize;


  ros::Time startTime, endTime, totalStartTime, stampTime;
  ros::Duration sectionDuration;

  tf::TransformListener listener;


  
public:
  KingfisherVisionShort()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/short_camera/image_raw", 1, 
      &KingfisherVisionShort::imageCb, this);
    //state_sub = it_.subscribe("/state", 1, &KingfisherVisionShort::stateCb,this);
    image_pub_ = it_.advertise("/kingfisher_vision_short/output_video", 1);
    testImage_pub = it_.advertise("/kingfisher_vision_short/test_output",1);
    testImage_pub2 = it_.advertise("/kingfisher_vision_short/test_output2",1);
    deckPose_pub = nh_.advertise<geometry_msgs::Twist>("kingfisher_vision_short/deckPose",1);
    quaternion_pub = nh_.advertise<geometry_msgs::Quaternion>("kingfisher_vision_short/quaternion",1);
    unfilteredHeading_pub = nh_.advertise<geometry_msgs::Twist>("kingfisher_vision_short/unfilteredHeading",1);
    transformStamped_pub = nh_.advertise<geometry_msgs::TransformStamped>("kingfisher_vision_short/camera2deck_transformStamped",1);
    vision_OK_flag_pub = nh_.advertise<std_msgs::Bool>("/vision_OK_flag",1);

    rectificationInitialized = 0;
    beacons_detected = 0;
    previous_frame = 0;
    previous_frame_valid = 0;
    predicted_transform_valid = 0;
    detectedBeaconRadius = 5;
    vision_OK_flag.data = false;


    cameraFrameString_msg.data = "camera";
    deckFrameString_msg.data = "deck";
    sequence_msg.data = 0;

    cameraMatrix = (cv::Mat_<double>(3,3) << 325.768218, 0, 641.552114, 0 , 325.987229, 511.739130, 0, 0, 1);

    double deckHalfSize = .1524; //(m) 1ft side length


    //cv::namedWindow(OPENCV_WINDOW);

    for (int i = 0; i < NUMOFBEACONS; ++i)
    {
      tracked_centers_vector.push_back(cv::Point(-1,-1));
      previous_source_vector.push_back(cv::Point2d(-1,-1));
      deck_3Dpoints_vector.push_back(cv::Point3d(0,0,0));
    }



    //solvePnp
    //make deck 3D points (in meters)
    //should go from bottom-left, top-left,top-right,bottom-right
    //TEST OFFSET
    //deck_3Dpoints_vector[0] = cv::Point3d(-deckHalfSize,-deckHalfSize+(3*deckHalfSize),0);
    //deck_3Dpoints_vector[1] = cv::Point3d(-deckHalfSize,deckHalfSize+(3*deckHalfSize),0);
    //deck_3Dpoints_vector[2] = cv::Point3d(deckHalfSize,deckHalfSize+(3*deckHalfSize),0);
    //deck_3Dpoints_vector[3] = cv::Point3d(deckHalfSize,-deckHalfSize+(3*deckHalfSize),0);
    //TEST OFFSET

    //NO OFFSET
    deck_3Dpoints_vector[0] = cv::Point3d(-deckHalfSize,-deckHalfSize,0);
    deck_3Dpoints_vector[1] = cv::Point3d(-deckHalfSize,deckHalfSize,0);
    deck_3Dpoints_vector[2] = cv::Point3d(deckHalfSize,deckHalfSize,0);
    deck_3Dpoints_vector[3] = cv::Point3d(deckHalfSize,-deckHalfSize,0);
    //NO OFFSET


    /*
    //this one works for this test case
    //std::cout << "atan2(10,5) = " << atan2(10,5) << std::endl;
    //std::cout << "atan2(15+10,20+5) = " << atan2(15+10,20+5) << std::endl;
    //std::cout << "atan2(20+10,20+5) = " << atan2(20+10,20+5) << std::endl;
    //std::cout << "atan2(25+10,25+5) = " << atan2(25+10,25+5) << std::endl;


    std::cout << "find second point with first at x=5,y=10" << std::endl;
    std::cout << "atan2(15-10,20-5) = " << atan2(15-10,20-5) << std::endl;
    std::cout << "atan2(20-10,20-5) = " << atan2(20-10,20-5) << std::endl;
    std::cout << "atan2(25-10,25-5) = " << atan2(25-10,25-5) << std::endl;

    std::cout << "find third point with second at x=20,y=15" << std::endl;
    std::cout << "atan2(20-15,20-20) = " << atan2(20-15,20-20) << std::endl;
    std::cout << "atan2(25-15,25-20) = " << atan2(25-15,25-20) << std::endl;

    std::cout << "check with new points if skewed the other way" << std::endl;

    std::cout << "find second point with first at x=7,y=13" << std::endl;
    std::cout << "atan2(2-13,15-7) = " << atan2(2-13,15-7) << std::endl;
    std::cout << "atan2(11-13,18-7) = " << atan2(11-13,18-7) << std::endl;
    std::cout << "atan2(30-13,20-7) = " << atan2(30-13,20-7) << std::endl;

    std::cout << "find third point with second at x=15,y=2" << std::endl;
    std::cout << "atan2(11-2,18-15) = " << atan2(11-2,18-15) << std::endl;
    std::cout << "atan2(30-2,20-15) = " << atan2(30-2,20-15) << std::endl;
    */

  }

  ~KingfisherVisionShort()
  {
    cvReleaseMat(&mapx_persp);
    cvReleaseMat(&mapy_persp);
    cvReleaseImage(&dst_persp);  
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  int initializeRectification() 
  {


    struct ocam_model o; // our ocam_model for the fisheye
    get_ocam_model(&o, "../catkin_ws/src/kingfisher_vision/src/calib_results.txt"); //should be in /home folder

   

    mapx_persp = cvCreateMat(src1.height, src1.width, CV_32FC1);
    mapy_persp = cvCreateMat(src1.height, src1.width, CV_32FC1);


    /* --------------------------------------------------------------------  */  
    /* Create Look-Up-Table for perspective undistortion                     */
    /* SF is kind of distance from the undistorted image to the camera       */
    /* (it is not meters, it is justa zoom fator)                            */
    /* Try to change SF to see how it affects the result                     */
    /* The undistortion is done on a  plane perpendicular to the camera axis */
    /* --------------------------------------------------------------------  */
    float sf = 4;
    create_perspecive_undistortion_LUT( mapx_persp, mapy_persp, &o, sf );

    endTime = ros::Time::now();
    sectionDuration = endTime - startTime;
    std::cout << "LUT generation Duration: " << sectionDuration.toSec() << std::endl;
    startTime = ros::Time::now();

    dst_persp   = cvCreateImage( cvGetSize(&src1), 8, 1);   // undistorted perspective and panoramic image

    return 1;

  }
/*
  void stateCb(const geometry_msgs::TwistWithCovarianceStamped& msg)
  {
  }
*/
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr, test_ptr, test_ptr2;
    cv::Mat output_image;
    cv::Mat difference_image;
    cv::Mat test_output_image;
    cv::Mat test_output_image2;
    cv::Mat data;
    cv::Mat data2;
    cv::Mat labels;
    cv::Mat centers;
    cv::Mat connectedComponentLabels;
    cv::Mat connectedComponentCenters;
    cv::Mat connectedComponentCentersFloat;
    cv::Mat connectedComponentStats;
    cv::Mat smallerConnectedComponentCenters;
    cv::Mat largerConnectedComponentCenters;
    cv::Mat perspectiveTransformMatrix;
    cv::Mat sourceMatrix;
    cv::Mat targetMatrix;
    //cv::Mat cameraMatrix;
    cv::Mat translationVector;
    cv::Mat rotationVector;
    cv::Mat cvMat_rotationMatrix;
    cv::Mat cvMat_tVec;
    cv::Mat cvMat_rVec;
    cv::Mat rotationMatrix;
    cv::Mat mtxR;
    cv::Mat mtxQ;
    cv::Mat qX, qY, qZ;
    cv::Mat mtxR2;
    cv::Mat mtxQ2;
    cv::Mat qX2, qY2, qZ2;
    cv::Vec3d eulerRotationAngles;
    cv::Vec3d eulerRotationAngles2;
    std_msgs::Float64 tempFloat64;

    int numOfConnectedComponents;
    int smallerNumOfConnectedComponents;
    int largerNumOfConnectedComponents;
    int temp_beacon_area;



    cv::Mat fisheyeMat;
    CvMat* cvMatRectifiedPtr;


    cv::Mat distortionCoefficients;

    cv::Mat theRightDataType;

    std::vector<cv::Point> nonZeroPointsVector;
    std::vector<cv::Point> cp_vector;

    std::vector<cv::Point> previous_cp_vector;
    std::vector<cv::Point2f> contour_vector;
    std::vector<cv::Point2f> convex_hull_vector;
    std::vector<cv::Point2d> source_vector(NUMOFBEACONS);
    std::vector<cv::Point2d> previous_source_vector(NUMOFBEACONS);
    std::vector<cv::Point> target_vector (NUMOFBEACONS);
    std::vector<cv::Point> perceived_quad_vector (NUMOFBEACONS);
    
    std::vector<cv::Point3d> tracking_3Dpoints_vector (NUMOFBEACONS);
    std::vector<cv::Point2d> tracking_projected_points(NUMOFBEACONS);
    std::vector<cv::Point2d> tracked_points(NUMOFBEACONS);

    cv::Point pt, upperLeftCorner, upperRightCorner, lowerRightCorner, lowerLeftCorner;
    cv::Point2d noisyPixel;

    cv::Mat nonZeroLocations;
    cv::Mat pruned;
    cv::Mat plzTryKmeansAgain;
    int minimumIndex;
    int clusterLabel;
    int clusterCount[NUMOFBEACONS] = {0};
    double distance[NUMOFBEACONS], minimumDistance;
    double angle[NUMOFBEACONS-1], minimumAngle;
    //half the side length of the deck beacons 
    //double deckHalfSize = 0.6025; //(m) 4ft side length

    double unfiltered_yaw;

    double temp;

    tf::Quaternion camera_world_pose;
    geometry_msgs::Vector3 vision_camera_xyz;

    ocam_model* ocam_model_ptr;

    cv::Point2d pt0, pt1, pt2, pt3;
    cv::Point2f pt0f, pt1f,pt2f, pt3f;
    cv::Point2f intersectionFloat;

    cv::Point2d intersection;
    double bbMaxX, bbMinX, bbMaxY, bbMinY;
    cv::Rect intersection_rect;
    cv::Mat intersection_roi;

    double net_dx, this_dx;
    double net_dy, this_dy;
    double intersection_distance_to_edge;

    std::vector<double> dxs, dys;
    double minDistance, delta_x, delta_y, delta_distance;
    double min_delta_x, min_delta_y;

    // // Check flags and publish on vision_OK_flag_pub
    // vision_OK_flag.data = !(rectificationInitialized && beacons_detected && previous_frame && previous_frame_valid && predicted_transform_valid);
    // vision_OK_flag_pub.publish(vision_OK_flag);

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
    //  cv::circle(cv_ptr->image, cv::Point(633, 517), 50, CV_RGB(255,255,255));

    startTime = ros::Time::now();
    totalStartTime = startTime;
    std::cout << "Starting time is: " << startTime.toSec() << std::endl;

    //rectify fisheye image
    //*cvMatFisheyePtr = CvMat(cv_ptr->image);

    /* --------------------------------------------------------------------*/    
    /* Print ocam_model parameters                                         */
    /* --------------------------------------------------------------------*/  
    //int i;
    //printf("pol =\n");    for (i=0; i<o.length_pol; i++){    printf("\t%e\n",o.pol[i]); };    printf("\n");
    //printf("invpol =\n"); for (i=0; i<o.length_invpol; i++){ printf("\t%e\n",o.invpol[i]); }; printf("\n");  
    //printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n",o.xc,o.yc,o.width,o.height);

    /* --------------------------------------------------------------------*/  
    /* Allocate space for the unistorted images                            */
    /* --------------------------------------------------------------------*/ 
    //cv_ptr->image.convertTo(fisheyeMat, CV_32FC1);
    //IplImage src1         = IplImage(fisheyeMat);      // source image 1
    src1         = IplImage(cv_ptr->image);      // source image
    //IplImage *dst_persp   = cvCreateImage( cvGetSize(&src1), 8, 1);   // undistorted perspective and panoramic image



    //build LUT for first image
    if(rectificationInitialized == 0) {
        rectificationInitialized = initializeRectification();
    }


    /* --------------------------------------------------------------------*/  
    /* Undistort using specified interpolation method                      */
    /* Other possible values are (see OpenCV doc):                         */
    /* CV_INTER_NN - nearest-neighbor interpolation,                       */
    /* CV_INTER_LINEAR - bilinear interpolation (used by default)          */
    /* CV_INTER_AREA - resampling using pixel area relation. It is the preferred method for image decimation that gives moire-free results. In case of zooming it is similar to CV_INTER_NN method. */
    /* CV_INTER_CUBIC - bicubic interpolation.                             */
    /* --------------------------------------------------------------------*/
    cvRemap( &src1, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0) ); 

    /* --------------------------------------------------------------------*/
    /* Display image                                                       */
    /* --------------------------------------------------------------------*/  
    //cvNamedWindow( "Original fisheye camera image", 1 );
    //cvShowImage( "Original fisheye camera image", &src1 );

    //cvNamedWindow( "Undistorted Perspective Image", 1 );
    //cvShowImage( "Undistorted Perspective Image", dst_persp );

    endTime = ros::Time::now();
    sectionDuration = endTime - startTime;
    std::cout << "Rectification Duration: " << sectionDuration.toSec() << std::endl;
    startTime = ros::Time::now();


    //threshold image to find bright points
    //////////////////////////////////////////////////////////////
    //TODO TAKE THIS OUT WHEN USING FILTER
    //STILL NEED OUTPUT IMAGE TO BE CREATED THOUGH
    cv::threshold(cv::cvarrToMat(dst_persp), output_image, 230, 100, 0);
    ///////////////////////////////////////////////////////////////


    /* --------------------------------------------------------------------*/    
    /* Free memory                                                         */
    /* --------------------------------------------------------------------*/
    //IplImage *src1ptr = &src1;
    //IplImage **src1ptrptr = &src1ptr;
    //cvReleaseImage(&&src1); 
    //cvReleaseImage(&dst_persp);  
    //cvReleaseMat(&mapx_persp);
    //cvReleaseMat(&mapy_persp);



    // to view rectified without finding beacons uncomment next two lines and comment threshold above
    //output_image = cv::cvarrToMat(dst_persp);
    // /*

    //turn off hot pixels
    for (int i = 0; i < output_image.size().width; ++i)
    {
        output_image.at<uchar>(0,i) = 0;
    }

    for (int i = 0; i < NUMOFBEACONS; ++i)
    {
        source_vector[i].x = 0;
        source_vector[i].y = 0;
    }


    numOfConnectedComponents = cv::connectedComponentsWithStats(output_image,connectedComponentLabels,connectedComponentStats,connectedComponentCenters,8);
    //prevNumOfConnectedComponents = cv::connectedComponentsWithStats(previous_frame_output_image,previousConnectedComponentLabels,previousConnectedComponentStats,previousConnectedComponentCenters,8);
    std::cout << "Found " << numOfConnectedComponents << " connected components..." << std::endl;
    endTime = ros::Time::now();
    sectionDuration = endTime - startTime;
    std::cout << "Connected Components Computation Duration: " << sectionDuration.toSec() << std::endl;
    startTime = ros::Time::now();
    connectedComponentLabels.convertTo(connectedComponentLabels, CV_8UC1);
    connectedComponentCenters.convertTo(connectedComponentCentersFloat,CV_32F);

    //intro of darkness
    if(numOfConnectedComponents < 5) 
    {
      std::cout << "Current frame has not enough light sources..." << std::endl;
      previous_frame = 0;
      previous_frame_valid = 0;
      // Check flags and publish on vision_OK_flag_pub
      vision_OK_flag.data = (rectificationInitialized && beacons_detected && previous_frame && previous_frame_valid && predicted_transform_valid);
      vision_OK_flag_pub.publish(vision_OK_flag);
      return;
    }

    //if need to look for beacons again from scratch
    if(beacons_detected == 0) {

        std::cout << "Looking for beacons from scratch" << std::endl;
        detectedBeaconRadius = 5;

        //find the blinking thing
        // //if no previous frame data
        // if(previous_frame == 0)
        // {
        //   std::cout << "No previous frame, saving this frame and returning..." << std::endl;
        //   output_image.copyTo(previous_frame_output_image);
        //   previous_frame = 1;
        //   previous_frame_valid = 0;
        //   return;
        // }

        //debug
        //test_ptr2->image = previous_frame_output_image;

        //Do Actual blinking algorithm

        // Find net motion of image
        if(previous_frame) 
        {
          std::cout << "Finding Net Motion..." << std::endl;

          std::cout << "Smaller/Larger..." << std::endl;
          if (numOfConnectedComponents < prevNumOfConnectedComponents)
          {
            smallerNumOfConnectedComponents = numOfConnectedComponents;
            connectedComponentCenters.copyTo(smallerConnectedComponentCenters);
            largerNumOfConnectedComponents = prevNumOfConnectedComponents;
            previousConnectedComponentCenters.copyTo(largerConnectedComponentCenters);
          }
          else
          {
            smallerNumOfConnectedComponents = prevNumOfConnectedComponents;
            previousConnectedComponentCenters.copyTo(smallerConnectedComponentCenters);
            largerNumOfConnectedComponents = numOfConnectedComponents;
            connectedComponentCenters.copyTo(largerConnectedComponentCenters);
          }

          std::cout << "Finding Distances..." << std::endl;
          //std::cout << "smallerConnectedComponentCenters: \n" << smallerConnectedComponentCenters << std::endl;
          //std::cout << "largerConnectedComponentCenters: \n" << largerConnectedComponentCenters << std::endl;
          for (int i = 1; i<smallerNumOfConnectedComponents; i++) {
            minDistance = 99999999;
            for (int j = 1; j<largerNumOfConnectedComponents; j++) {
              delta_x = smallerConnectedComponentCenters.at<double>(i,0) - largerConnectedComponentCenters.at<double>(j,0);
              delta_y = smallerConnectedComponentCenters.at<double>(i,1) - largerConnectedComponentCenters.at<double>(j,1);
              delta_distance = sqrt(delta_x*delta_x + delta_y*delta_y);
              //std::cout << "Checking if smaller distance than current min..." << std::endl;
              if (delta_distance < minDistance)
              {
                minDistance = delta_distance;
                min_delta_x = delta_x;
                min_delta_y = delta_y;
              }
            }
            std::cout << "Storing delta_x and delta_y..." << std::endl;
            dxs.push_back(min_delta_x);
            dys.push_back(min_delta_y);
          }

          // Median/Mean
          std::cout << "Sorting..." << std::endl;
          std::sort(dxs.begin(),dxs.end());
          std::sort(dys.begin(),dys.end());
          int median_x_index = (int) dxs.size()/2;
          int median_y_index = (int) dys.size()/2;
          net_dx = (dxs[median_x_index-1] + dxs[median_x_index] + dxs[median_x_index+1]) / 3.0;
          net_dy = (dys[median_x_index-1] + dys[median_x_index] + dys[median_x_index+1]) / 3.0;
        }

        endTime = ros::Time::now();
        sectionDuration = endTime - startTime;
        std::cout << "Blinking Shift Detection Duration: " << sectionDuration.toSec() << std::endl;
        startTime = ros::Time::now();


        //find intersections of blobs in current image
        std::cout << "Finding Intersections..." << std::endl;
        int debugCounter = 0;
        int enclosed = 0;
        cv::Point2f testPoint2f;
        for (int a = 1; a < numOfConnectedComponents-1; a++) {
          for (int b = a+1; b < numOfConnectedComponents; b++) {
            for (int c = 1; c < numOfConnectedComponents-1; c++) {
              for (int d = c+1; d < numOfConnectedComponents; d++) {

                //if sharing an endpoint
                if (a == c || b == c || a == d || b == d)
                  continue;
                // Do logic
                // std::cout << "Intersection parent index a is: " << a << std::endl;
                // std::cout << "Intersection parent index b is: " << b << std::endl;
                // std::cout << "Intersection parent index c is: " << c << std::endl;
                // std::cout << "Intersection parent index d is: " << d << std::endl;
                pt0 = connectedComponentCenters.at<cv::Point2d>(a);
                pt1 = connectedComponentCenters.at<cv::Point2d>(b);
                pt2 = connectedComponentCenters.at<cv::Point2d>(c);
                pt3 = connectedComponentCenters.at<cv::Point2d>(d);

                pt0f = connectedComponentCentersFloat.at<cv::Point2f>(a);
                pt1f = connectedComponentCentersFloat.at<cv::Point2f>(b);
                pt2f = connectedComponentCentersFloat.at<cv::Point2f>(c);
                pt3f = connectedComponentCentersFloat.at<cv::Point2f>(d);

                contour_vector.clear();
                contour_vector.push_back(pt0f);
                contour_vector.push_back(pt1f);
                contour_vector.push_back(pt2f);
                contour_vector.push_back(pt3f);

                cv::convexHull(contour_vector,convex_hull_vector);

                std::cout << "pt0 is: " << pt0 << std::endl;
                std::cout << "pt1 is: " << pt1 << std::endl;
                std::cout << "pt2 is: " << pt2 << std::endl;
                std::cout << "pt3 is: " << pt3 << std::endl;


                


                //compute intersections 
                if (lineIntersection(pt0,pt1,pt2,pt3,intersection))
                {
                  //debug
                  debugCounter++;       
                  std::cout << "Intersection occurs at: " << intersection << std::endl;
                  //end debug
                  intersectionFloat.x = (float) intersection.x;
                  intersectionFloat.y = (float) intersection.y;
                  std::cout << "IntersectionFloat occurs at: " << intersectionFloat << std::endl;
                  intersection_distance_to_edge = cv::pointPolygonTest(convex_hull_vector, intersectionFloat,true);
                  std::cout << "intersection_distance_to_edge: " << intersection_distance_to_edge << std::endl;

                  //if intersection is within convex hull
                  if(intersection_distance_to_edge > 0)
                  {
                    //debug
                    std::cout << "Intersection is within polygon..." << std::endl;

                    enclosed = 0;
                    for (int r = 1; r < numOfConnectedComponents; r++)
                    {
                      //std::cout << "connectedComponentCenters.at<cv::Point2f>(r)" << connectedComponentCentersFloat.at<cv::Point2f>(r) << std::endl;
                      if(cv::pointPolygonTest(convex_hull_vector, connectedComponentCentersFloat.at<cv::Point2f>(r),false) > 0)
                      {
                        enclosed = 1;
                        break;
                      }
                    }
                    //if points in poly or on line, break
                    if(enclosed)
                    {
                      std::cout << "Polygon contains light sources, not a dark intersection..." << std::endl;
                      continue;
                    }



                    intersection_rect = cv::Rect_<double>(intersection.x,intersection.y,1,1);
                    std::cout << "intersection.x: " << intersection.x << std::endl;
                    std::cout << "intersection.y: " << intersection.y << std::endl;
                    intersection_roi = connectedComponentLabels(intersection_rect);

                    intersection_roi.adjustROI(detectedBeaconRadius,detectedBeaconRadius,detectedBeaconRadius,detectedBeaconRadius);
                    cv::findNonZero(intersection_roi,nonZeroLocations);
                    // If intersection is dark
                    std::cout << "nonZeroLocations.total() in intersection_roi: " << nonZeroLocations.total() << std::endl;
                    if (nonZeroLocations.total() == 0 && intersection_distance_to_edge > 2*detectedBeaconRadius)
                    {
                      std::cout << "Intersection is dark, saving location and parents for future checks..." << std::endl;
                      //debug
                      std::cout << "Drawing Intersection " << debugCounter << std::endl;
                      cv::circle(output_image, intersection, 5, 255);
                      cv_ptr->image = output_image;
                      image_pub_.publish(cv_ptr->toImageMsg());
                      //end debug
                      darkIntersections.push_back(intersection);
                      darkIntersectionParentA.push_back(pt0);
                      darkIntersectionParentB.push_back(pt1);
                      darkIntersectionParentC.push_back(pt2);
                      darkIntersectionParentD.push_back(pt3);
                    }
                    else if (previous_frame) // Intersection is light, check if it was dark previously
                    {
                      std::cout << "Intersection has light, checking for matching dark intersection from previous..." << std::endl;
                      std::cout << "previousDarkIntersections.size(): " << previousDarkIntersections.size() << std::endl;
                      for (int t = 0;t < previousDarkIntersections.size(); t++) {
                        std::cout << "previousDarkIntersections[" << t << "] is: " << previousDarkIntersections[t] << std::endl;
                      }
                      for (int i = 0; i < previousDarkIntersections.size(); i++) {
                        this_dx = previousDarkIntersections[i].x - intersection.x;
                        this_dy = previousDarkIntersections[i].y - intersection.y;
                        std::cout << "this_dx: " << this_dx  << std::endl;
                        std::cout << "this_dy: " << this_dy  << std::endl;
                        std::cout << "net_dx: " << net_dx  << std::endl;
                        std::cout << "net_dy: " << net_dy  << std::endl;
                        std::cout << "previousDarkIntersections[" << i << "] is: " << previousDarkIntersections[i] << std::endl;
                        std::cout << "intersection is: " << intersection << std::endl;
                        
                        if (abs(net_dx-this_dx) <= 0.5*abs(net_dx) && abs(net_dy-this_dy) <= 0.5*abs(net_dy)) // you found the beacon
                        {
                          std::cout << "Intersection is correct, saving parents as cp_vector and previous_cp_vector..." << std::endl;
                          previous_cp_vector.push_back(previousDarkIntersectionParentA[i]);
                          previous_cp_vector.push_back(previousDarkIntersectionParentB[i]);
                          previous_cp_vector.push_back(previousDarkIntersectionParentC[i]);
                          previous_cp_vector.push_back(previousDarkIntersectionParentD[i]);

                          pt0 = cv::Point2f(previous_cp_vector[0].x-this_dx,previous_cp_vector[0].y-this_dy);
                          pt1 = cv::Point2f(previous_cp_vector[1].x-this_dx,previous_cp_vector[1].y-this_dy);
                          pt2 = cv::Point2f(previous_cp_vector[2].x-this_dx,previous_cp_vector[2].y-this_dy);
                          pt3 = cv::Point2f(previous_cp_vector[3].x-this_dx,previous_cp_vector[3].y-this_dy);

                          cp_vector.push_back(pt0);
                          cp_vector.push_back(pt1);
                          cp_vector.push_back(pt2);
                          cp_vector.push_back(pt3);


                          for(int q = 0;q<NUMOFBEACONS;q++) {
                            std::cout << "cp_vector[" << q << "] is: " << cp_vector[q] << std::endl;
                            for(int w = 0; w < q+1; w++) {
                              cv::circle(output_image, cp_vector[q], NUMOFBEACONS*(w+3), 200);
                              cv::circle(output_image, previous_cp_vector[q], NUMOFBEACONS*(w+3), 100);
                            }
                          }
                          cv_ptr->image = output_image;
                          image_pub_.publish(cv_ptr->toImageMsg());
                          goto blinkDetected;
                        }
                      }
                    }
                  }
                  else {
                    std::cout << "Intersection is not within convex hull, continuing..." << std::endl;
                  }
                }
              }
            }
          }
        }

        endTime = ros::Time::now();
        sectionDuration = endTime - startTime;
        std::cout << "Blinking Intersections+ConvexHull Duration: " << sectionDuration.toSec() << std::endl;
        startTime = ros::Time::now();

        //if blinking not found
        std::cout << "Blinking not found, saving this frame's CC and intersections and returning..." << std::endl;
        prevNumOfConnectedComponents = numOfConnectedComponents;
        connectedComponentLabels.copyTo(previousConnectedComponentLabels);
        connectedComponentCenters.copyTo(previousConnectedComponentCenters);
        connectedComponentStats.copyTo(previousConnectedComponentStats);

        std::cout << "previousDarkIntersections.size(): " << previousDarkIntersections.size() << std::endl;
        std::cout << "darkIntersections.size(): " << darkIntersections.size() << std::endl;

        for (int t = 0;t < previousDarkIntersections.size(); t++) {
          std::cout << "previousDarkIntersections[" << t << "] is: " << previousDarkIntersections[t] << std::endl;
        }
        for (int t = 0;t < darkIntersections.size(); t++) {
          std::cout << "darkIntersections[" << t << "] is: " << darkIntersections[t] << std::endl;
        }

        previousDarkIntersections = darkIntersections;
        previousDarkIntersectionParentA = darkIntersectionParentA;
        previousDarkIntersectionParentB = darkIntersectionParentB;
        previousDarkIntersectionParentC = darkIntersectionParentC;
        previousDarkIntersectionParentD = darkIntersectionParentD;

        //clear current intersection vectors and parents
        darkIntersections.clear();
        darkIntersectionParentA.clear();
        darkIntersectionParentB.clear();
        darkIntersectionParentC.clear();
        darkIntersectionParentD.clear();


        previous_frame = 1;
        previous_frame_valid = 0;
        // Check flags and publish on vision_OK_flag_pub
        vision_OK_flag.data = (rectificationInitialized && beacons_detected && previous_frame && previous_frame_valid && predicted_transform_valid);
        vision_OK_flag_pub.publish(vision_OK_flag);
        return;

        blinkDetected:
        std::cout << "Blinking thing found, beacons detected..." << std::endl;
        //if blinking thing found
        beacons_detected = 1;

        
        


        if (beacons_detected == 1)
        {


          // //find centroids of each cluster
          // cv::findNonZero(output_image,nonZeroLocations);

          // nonZeroLocations.convertTo(data, CV_32F);
          // //std::cout << "number of nonzero points in data: " << data.total() << std::endl;

          // //output_image.convertTo(data2, CV_32F);

          // //if less than 4 data points remaining, DO NOT RUN K MEANS
          // if(data.total() < 4) 
          // {
          //     //we've lost sight of the beacons
          //     beacons_detected = 0;
          //     previous_frame = 0; 

          //     return;
          // }


          // //std::cout << "data(1,1): " << data.at<cv::Point2d>(1,1) << std::endl;
          // cv::kmeans(data,NUMOFBEACONS,labels,cv::TermCriteria(0,100,0),10,0,centers);
          // //std::cout << "original centers are: " << centers << std::endl;
          // //count the number of points in each cluster
          // //std::cout << "total number of labels: " << labels.total() << std::endl;

          // endTime = ros::Time::now();
          // //std::cout << "Kmeans end time: " << endTime.toSec() << std::endl;
          // sectionDuration = endTime - startTime;
          // std::cout << "Kmeans Duration: " << sectionDuration.toSec() << std::endl;
          // startTime = ros::Time::now();

          // for (int i = 0; i < labels.total(); ++i)
          // {
          //     clusterLabel = labels.at<int>(i);
          //     clusterCount[clusterLabel]++;
          // }

          // detectedBeaconRadius = 0;
          // for (int i = 0; i < NUMOFBEACONS; ++i)
          // {
          //     if(clusterCount[i] > detectedBeaconRadius)
          //         detectedBeaconRadinus = clusterCount[i];
          // }

          // detectedBeaconRadius = (int) sqrt(detectedBeaconRadius/3.14);


          // std::cout << "clusterCount[0]: " << clusterCount[0] << std::endl;
          // std::cout << "clusterCount[1]: " << clusterCount[1] << std::endl;
          // std::cout << "clusterCount[2]: " << clusterCount[2] << std::endl;
          // std::cout << "clusterCount[3]: " << clusterCount[3] << std::endl;

          // std::cout << "detectedBeaconRadius: " << detectedBeaconRadius << std::endl;

          // //prune datapoints from small clusters
          // for (int i = 0; i < NUMOFBEACONS; ++i)
          // {
          //     //throw out anything labeled as cluster i
          //     if(clusterCount[i] < 5){
          //         //std::cout << "Cluster[" << i << "] has fewer than 5 members" << std::endl;

          //         //data.convertTo(theRightDataType,CV_8UC1);
          //         for (int j = 0; j < labels.total(); ++j)
          //         {
          //             if (labels.at<int>(j) == i)
          //             {
          //                 noisyPixel = data.at<cv::Point2d>(1,j);                   
          //                 //std::cout << "noisyPixel: " << noisyPixel << std::endl;
          //                 //std::cout << "output_image.at<char>(noisyPixel): " << output_image.at<char>(noisyPixel) << std::endl;
          //                 output_image.at<uchar>(noisyPixel) = 0;
          //                 //std::cout << "output_image.at<char>(noisyPixel): " << output_image.at<char>(noisyPixel) << std::endl;
          //             }
          //         }
          //         //std::cout << "converting..."<< std::endl;
          //         //nonZeroLocations.convertTo(theRightDataType,CV_8UC1);
          //         //std::cout << "the data type should be converted"<< std::endl;
          //         //if(theRightDataType.type() == CV_8UC1)
          //         //std::cout << "the data type should be right"<< std::endl;
          //         cv::findNonZero(output_image,pruned);
          //         //std::cout << "number of nonzero points in pruned: " << pruned.total() << std::endl;

          //         pruned.convertTo(plzTryKmeansAgain, CV_32F);

          //         //if less than 4 data points remaining, DO NOT RUN K MEANS
          //         if(plzTryKmeansAgain.total() < 4) {

          //             //we've lost sight of the beacons
          //             beacons_detected = 0;
          //             return;
          //         }

          //         //run kmeans again
          //         cv::kmeans(plzTryKmeansAgain,NUMOFBEACONS,labels,cv::TermCriteria(0,20,0),3,0,centers);
          //         //count the number of points in each cluster
          //         //std::cout << "number of labels after pruning: " << labels.total() << std::endl;

          //         //std::cout << "new centers are: " << centers << std::endl;

          //     }
          // }



          //std::cout << "new centers are: " << centers << std::endl;
          //std::cout << "connectedComponentCenters.type(): " << connectedComponentCenters.type() << std::endl;
          
          // for (int i = 1; i<connectedComponentCenters.rows;i++)
          // {
          //   pt = connectedComponentCenters.at<cv::Point2d>(i,0);
          //   //debug
          //   //printf("cp_vector[%d] is: [%d,%d]\n",i,pt.y,pt.x);
          //   std::cout << "pt = (" << pt.x << "," << pt.y << ")" << std::endl;
          //   cp_vector.push_back(pt);
          //   //cv::circle(output_image, pt, 2, 255);
          //   //ROS_INFO("Center is at: %d,%d", pt.y,pt.x);
          // }


          /////////// ASSIGN CENTERS AND TRACK /////////////
          //if this is the first frame with beacons in it
          //if (tracked_centers_vector[0].x == -1)


          //sort centers as: Upper-Left, Upper-Right, Bottom-Right,Bottom-Left
          upperLeftCorner = cv::Point(0,0);
          //upperRightCorner = cv::Point(cv_ptr->image.cols,0);
          //lowerRightCorner = cv::Point(cv_ptr->image.cols,cv_ptr->image.rows);
          //lowerLeftCorner = cv::Point(0,cv_ptr->image.cols);

          //FIND WHICH CENTER SHOULD BE AT POS 0
          for (int i = 0; i < NUMOFBEACONS; i++)
          {
              distance[i] = sqrt(pow(cp_vector[i].x,2)+pow(cp_vector[i].y,2));
              unfiltered_yaw = unfiltered_yaw + cp_vector[i].x;
          }

          //find center of beacons in y(yaw since camera on its side)
          unfiltered_yaw = unfiltered_yaw/NUMOFBEACONS;
          //positive is right, negative is left
          unfiltered_yaw = (unfiltered_yaw-512)/19.5; //degrees

          unfilteredHeading.angular.x = unfiltered_yaw;
              unfilteredHeading.linear.x = 0;
              unfilteredHeading.linear.y = 0;
              unfilteredHeading.linear.z = 0;
              unfilteredHeading.angular.y = 0;
              unfilteredHeading.angular.z = 0;
          unfilteredHeading_pub.publish(unfilteredHeading);

          minimumDistance = 99999999;

          //find minimum distance
          for (int i=0;i<NUMOFBEACONS;i++) {
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

          //find other points from current point
          //should be angular clockwise from upperLeft
          angle[0] = atan2(cp_vector[0].y-source_vector[0].y,cp_vector[0].x-source_vector[0].x);
          angle[1] = atan2(cp_vector[1].y-source_vector[0].y,cp_vector[1].x-source_vector[0].x);
          angle[2] = atan2(cp_vector[2].y-source_vector[0].y,cp_vector[2].x-source_vector[0].x);


          //std::cout << "angle[0] is: " << angle[0] << " at : [" << cp_vector[0].x << "," << cp_vector[0].y << "]" << std::endl;
          //std::cout << "angle[1] is: " << angle[1] << " at : [" << cp_vector[1].x << "," << cp_vector[1].y << "]" << std::endl;
          //std::cout << "angle[2] is: " << angle[2] << " at : [" << cp_vector[2].x << "," << cp_vector[2].y << "]" << std::endl;


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
          angle[0] = atan2(cp_vector[0].y-source_vector[1].y,cp_vector[0].x-source_vector[1].x);
          angle[1] = atan2(cp_vector[1].y-source_vector[1].y,cp_vector[1].x-source_vector[1].x);

          if(angle[0] < angle[1]) {
              source_vector[2] = cp_vector[0];
              source_vector[3] = cp_vector[1];
          }
          else {
              source_vector[2] = cp_vector[1];
              source_vector[3] = cp_vector[0];
          }

          //std::cout << "Solving PnP for current frame..." << std::endl;
          //std::cout << "source_vector[0] = " << source_vector[0] << std::endl;
          //std::cout << "source_vector[1] = " << source_vector[1] << std::endl;
          //std::cout << "source_vector[2] = " << source_vector[2] << std::endl;
          //std::cout << "source_vector[3] = " << source_vector[3] << std::endl;
          // SolvePnP
          cv::solvePnP(deck_3Dpoints_vector,source_vector,cameraMatrix,distortionCoefficients,rotationVector,translationVector);
    
          //convert rotation vecotr to useable rotation matrix
          cv::Rodrigues(rotationVector,rotationMatrix);


          
          //TODO need to convert rotation matrix to quaternion to pass to TF
          //that will allow the final TF node to make the deck orientation in the right frame

          //extract roll, pitch yaw from rotation matrix
          eulerRotationAngles = cv::RQDecomp3x3(rotationMatrix,mtxR,mtxQ,qX,qY,qZ);
          
          

          //std::cout << "qX[0] = " << qX.at<double>(0) << std::endl;
          //std::cout << "qX[1] = " << qX.at<double>(1) << std::endl;
          //std::cout << "qX[2] = " << qX.at<double>(2) << std::endl;
          //std::cout << "qX[3] = " << qX.at<double>(3) << std::endl;
          //std::cout << "qX[4] = " << qX.at<double>(4) << std::endl;
          //std::cout << "qX[5] = " << qX.at<double>(5) << std::endl;
          //std::cout << "qX[6] = " << qX.at<double>(6) << std::endl;
          //std::cout << "qX[7] = " << qX.at<double>(7) << std::endl;
          //std::cout << "qX[8] = " << qX.at<double>(8) << std::endl;



          //std::cout << "qX = " << std::endl << " " << qX << std::endl << std::endl;
          //std::cout << "qY = " << std::endl << " " << qY << std::endl << std::endl; 
          //std::cout << "qZ = " << std::endl << " " << qZ << std::endl << std::endl; 
          //print output rotation angles
          //std::cout << "eulerRotationAngles: " << eulerRotationAngles << std::endl;

          double xThetaRad, yThetaRad, zThetaRad;
          double xThetaDeg, yThetaDeg, zThetaDeg;

          //xThetaRad = acos(qX.at<double>(4));
          //yThetaRad = acos(qY.at<double>(0));
          //zThetaRad = acos(qZ.at<double>(0));

          xThetaRad = -atan2(qX.at<double>(7),qX.at<double>(4));
          yThetaRad = -atan2(qY.at<double>(2),qY.at<double>(0));
          zThetaRad = -atan2(qZ.at<double>(3),qZ.at<double>(0));


          xThetaDeg = xThetaRad*180/3.1415;
          yThetaDeg = yThetaRad*180/3.1415;
          zThetaDeg = zThetaRad*180/3.1415;


          //std::cout << "xTheta (radians): " << xThetaRad << std::endl;
          //std::cout << "yTheta (radians): " << yThetaRad << std::endl;
          //std::cout << "zTheta (radians): " << zThetaRad << std::endl;


          //std::cout << "xTheta (degrees): " << xThetaDeg << std::endl;
          //std::cout << "yTheta (degrees): " << yThetaDeg << std::endl;
          //std::cout << "zTheta (degrees): " << zThetaDeg << std::endl;



          //print output of solvePnp
          //std::cout << "rotationMatrix = " << std::endl << " " << rotationMatrix << std::endl << std::endl; 
          std::cout << "rvec: " << rotationVector << std::endl;
          std::cout << "tvec: " << translationVector << std::endl;


          //publish pose
          //tempFloat64.data = translationVector.at<double>(0);
          //deckPose.linear.x = tempFloat64.data;
          deckPose.linear.x = translationVector.at<double>(0);
          deckPose.linear.y = translationVector.at<double>(1);
          deckPose.linear.z = translationVector.at<double>(2);
          deckPose.angular.x = 0;//eulerRotationAngles[0];
          deckPose.angular.y = 0;//eulerRotationAngles[1];
          deckPose.angular.z = 0;//eulerRotationAngles[2];
          
          //deckPose_pub.publish(deckPose);


          tf::Quaternion quat;
          tf::Matrix3x3 tfMatrix;

          //tfMatrix.setRPY(rotationVector.at<double>(1),rotationVector.at<double>(0),rotationVector.at<double>(2));
          tfMatrix.setRPY(xThetaRad,yThetaRad,zThetaRad);
          tfMatrix.getRotation(quat);

          quaternion_msg.x = quat[0];
          quaternion_msg.y = quat[1];
          quaternion_msg.z = quat[2];
          quaternion_msg.w = quat[3];

          //quaternion_pub.publish(quaternion_msg);

          //std::cout << "quaternion[0] = " << quat[0] << std::endl;
          //std::cout << "quaternion[1] = " << quat[1] << std::endl;
          //std::cout << "quaternion[2] = " << quat[2] << std::endl;
          //std::cout << "quaternion[3] = " << quat[3] << std::endl;


          //publish StampedTransform
          //for SAM
          vector3_msg.x = translationVector.at<double>(0);
          vector3_msg.y = translationVector.at<double>(1);
          vector3_msg.z = translationVector.at<double>(2);

          transform_msg.translation = vector3_msg;
          transform_msg.rotation = quaternion_msg;


          ////////////////////////////////////
          // Do the same thing for previous //
          ////////////////////////////////////
          // for (int i = 1; i<previousConnectedComponentCenters.rows;i++)
          // {
          //   pt = previousConnectedComponentCenters.at<cv::Point2d>(i,0);
          //   //debug
          //   //printf("cp_vector[%d] is: [%d,%d]\n",i,pt.y,pt.x);
          //   previous_cp_vector.push_back(pt);
          //   //cv::circle(output_image, pt, 2, 255);
          //   //ROS_INFO("Center is at: %d,%d", pt.y,pt.x);
          // }


          /////////// ASSIGN CENTERS AND TRACK /////////////
          //if this is the first frame with beacons in it
          //if (tracked_centers_vector[0].x == -1)omy



          //sort centers as: Upper-Left, Upper-Right, Bottom-Right,Bottom-Left
          upperLeftCorner = cv::Point(0,0);
          //upperRightCorner = cv::Point(cv_ptr->image.cols,0);
          //lowerRightCorner = cv::Point(cv_ptr->image.cols,cv_ptr->image.rows);
          //lowerLeftCorner = cv::Point(0,cv_ptr->image.cols);

          //FIND WHICH CENTER SHOULD BE AT POS 0
          for (int i = 0; i < NUMOFBEACONS; i++)
          {
              distance[i] = sqrt(pow(previous_cp_vector[i].x,2)+pow(previous_cp_vector[i].y,2));
              unfiltered_yaw = unfiltered_yaw + previous_cp_vector[i].x;
          }

          //find center of beacons in y(yaw since camera on its side)
          unfiltered_yaw = unfiltered_yaw/NUMOFBEACONS;
          //positive is right, negative is left
          unfiltered_yaw = (unfiltered_yaw-512)/19.5; //degrees

          unfilteredHeading.angular.x = unfiltered_yaw;
              unfilteredHeading.linear.x = 0;
              unfilteredHeading.linear.y = 0;
              unfilteredHeading.linear.z = 0;
              unfilteredHeading.angular.y = 0;
              unfilteredHeading.angular.z = 0;
          unfilteredHeading_pub.publish(unfilteredHeading);

          minimumDistance = 99999999;

          //find minimum distance
          for (int i=0;i<NUMOFBEACONS;i++) {
            if(distance[i] < minimumDistance) {
              minimumDistance = distance[i];
              minimumIndex = i;
              //printf("distance %d is: %f\n",i,distance[i]);
              //printf("minimumDistance is: %f\n",minimumDistance);
              //printf("minimumIndex is: %d\n",minimumIndex);
              }
          }

          //assign to corner
          previous_source_vector[0] = previous_cp_vector[minimumIndex];
          //remove from contention
          previous_cp_vector.erase(previous_cp_vector.begin()+minimumIndex);

          //find other points from current point
          //should be angular clockwise from upperLeft
          angle[0] = atan2(previous_cp_vector[0].y-previous_source_vector[0].y,previous_cp_vector[0].x-previous_source_vector[0].x);
          angle[1] = atan2(previous_cp_vector[1].y-previous_source_vector[0].y,previous_cp_vector[1].x-previous_source_vector[0].x);
          angle[2] = atan2(previous_cp_vector[2].y-previous_source_vector[0].y,previous_cp_vector[2].x-previous_source_vector[0].x);


          //std::cout << "angle[0] is: " << angle[0] << " at : [" << previous_cp_vector[0].x << "," << previous_cp_vector[0].y << "]" << std::endl;
          //std::cout << "angle[1] is: " << angle[1] << " at : [" << previous_cp_vector[1].x << "," << previous_cp_vector[1].y << "]" << std::endl;
          //std::cout << "angle[2] is: " << angle[2] << " at : [" << previous_cp_vector[2].x << "," << previous_cp_vector[2].y << "]" << std::endl;


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
          previous_source_vector[1] = previous_cp_vector[minimumIndex];
          //remove from contention
          previous_cp_vector.erase(previous_cp_vector.begin()+minimumIndex);

          //find next minimum angle
          angle[0] = atan2(previous_cp_vector[0].y-previous_source_vector[1].y,previous_cp_vector[0].x-previous_source_vector[1].x);
          angle[1] = atan2(previous_cp_vector[1].y-previous_source_vector[1].y,previous_cp_vector[1].x-previous_source_vector[1].x);

          if(angle[0] < angle[1]) {
              previous_source_vector[2] = previous_cp_vector[0];
              previous_source_vector[3] = previous_cp_vector[1];
          }
          else {
              previous_source_vector[2] = previous_cp_vector[1];
              previous_source_vector[3] = previous_cp_vector[0];
          }
        }

        //std::cout << "Solving PnP for previous frame..." << std::endl;
        //std::cout << "previous_source_vector[0] = " << previous_source_vector[0] << std::endl;
        //std::cout << "previous_source_vector[1] = " << previous_source_vector[1] << std::endl;
        //std::cout << "previous_source_vector[2] = " << previous_source_vector[2] << std::endl;
        //std::cout << "previous_source_vector[3] = " << previous_source_vector[3] << std::endl;

        // SolvePnP
        cv::solvePnP(deck_3Dpoints_vector,previous_source_vector,cameraMatrix,distortionCoefficients,rotationVector,translationVector);
    
        //convert rotation vecotr to useable rotation matrix
        cv::Rodrigues(rotationVector,rotationMatrix);


        
        //TODO need to convert rotation matrix to quaternion to pass to TF
        //that will allow the final TF node to make the deck orientation in the right frame

        //extract roll, pitch yaw from rotation matrix
        eulerRotationAngles = cv::RQDecomp3x3(rotationMatrix,mtxR,mtxQ,qX,qY,qZ);
        

        double xThetaRad, yThetaRad, zThetaRad;
        double xThetaDeg, yThetaDeg, zThetaDeg;

        //xThetaRad = acos(qX.at<double>(4));
        //yThetaRad = acos(qY.at<double>(0));
        //zThetaRad = acos(qZ.at<double>(0));

        xThetaRad = -atan2(qX.at<double>(7),qX.at<double>(4));
        yThetaRad = -atan2(qY.at<double>(2),qY.at<double>(0));
        zThetaRad = -atan2(qZ.at<double>(3),qZ.at<double>(0));


        xThetaDeg = xThetaRad*180/3.1415;
        yThetaDeg = yThetaRad*180/3.1415;
        zThetaDeg = zThetaRad*180/3.1415;


        //std::cout << "xTheta (radians): " << xThetaRad << std::endl;
        //std::cout << "yTheta (radians): " << yThetaRad << std::endl;
        //std::cout << "zTheta (radians): " << zThetaRad << std::endl;


       // //std::cout << "xTheta (degrees): " << xThetaDeg << std::endl;
        //std::cout << "yTheta (degrees): " << yThetaDeg << std::endl;
        //std::cout << "zTheta (degrees): " << zThetaDeg << std::endl;



        //print output of solvePnp
        //std::cout << "rotationMatrix = " << std::endl << " " << rotationMatrix << std::endl << std::endl; 
        std::cout << "rvec: " << rotationVector << std::endl;
        std::cout << "tvec: " << translationVector << std::endl;


        //publish pose
        //tempFloat64.data = translationVector.at<double>(0);
        //deckPose.linear.x = tempFloat64.data;
        deckPose.linear.x = translationVector.at<double>(0);
        deckPose.linear.y = translationVector.at<double>(1);
        deckPose.linear.z = translationVector.at<double>(2);
        deckPose.angular.x = 0;//eulerRotationAngles[0];
        deckPose.angular.y = 0;//eulerRotationAngles[1];
        deckPose.angular.z = 0;//eulerRotationAngles[2];
        
        //deckPose_pub.publish(deckPose);


        tf::Quaternion quat;
        tf::Matrix3x3 tfMatrix;

        //tfMatrix.setRPY(rotationVector.at<double>(1),rotationVector.at<double>(0),rotationVector.at<double>(2));
        tfMatrix.setRPY(xThetaRad,yThetaRad,zThetaRad);
        tfMatrix.getRotation(quat);

        quaternion_msg.x = quat[0];
        quaternion_msg.y = quat[1];
        quaternion_msg.z = quat[2];
        quaternion_msg.w = quat[3];

        //quaternion_pub.publish(quaternion_msg);


        //std::cout << "quaternion[0] = " << quat[0] << std::endl;
        //std::cout << "quaternion[1] = " << quat[1] << std::endl;
        //std::cout << "quaternion[2] = " << quat[2] << std::endl;
        //std::cout << "quaternion[3] = " << quat[3] << std::endl;


        //publish StampedTransform
        //for SAM
        vector3_msg.x = translationVector.at<double>(0);
        vector3_msg.y = translationVector.at<double>(1);
        vector3_msg.z = translationVector.at<double>(2);

        previous_transform_msg.translation = vector3_msg;
        previous_transform_msg.rotation = quaternion_msg;




        //set previous frame data

        predicted_transform_msg.translation.x = 2*transform_msg.translation.x-previous_transform_msg.translation.x;
        predicted_transform_msg.translation.y = 2*transform_msg.translation.y-previous_transform_msg.translation.y;
        predicted_transform_msg.translation.z = 2*transform_msg.translation.z-previous_transform_msg.translation.z;

        tf::Quaternion previous_quat = tf::Quaternion(previous_transform_msg.rotation.x,previous_transform_msg.rotation.y,previous_transform_msg.rotation.z,previous_transform_msg.rotation.w);
        //copy current quaternion
        tf::Quaternion delta_quat = tf::Quaternion(quat[0],quat[1],quat[2],quat[3]);
        tf::Quaternion predicted_quat = tf::Quaternion(quat[0],quat[1],quat[2],quat[3]);
        delta_quat *= previous_quat.inverse();
        predicted_quat *= delta_quat;

        predicted_transform_msg.rotation.x = predicted_quat[0];
        predicted_transform_msg.rotation.y = predicted_quat[1];
        predicted_transform_msg.rotation.z = predicted_quat[2];
        predicted_transform_msg.rotation.w = predicted_quat[3];

        std::cout << "predicted_transform_msg.translation: \n" << predicted_transform_msg.translation << std::endl;
        std::cout << "predicted_transform_msg.rotation: \n" << predicted_transform_msg.rotation << std::endl;
        std::cout << "predicted_quat[0]: " << predicted_quat[0] << std::endl;
        std::cout << "predicted_quat[1]: " << predicted_quat[1] << std::endl;
        std::cout << "predicted_quat[2]: " << predicted_quat[2] << std::endl;
        std::cout << "predicted_quat[3]: " << predicted_quat[3] << std::endl;
        std::cout << "predicted_transform_msg.rotation: \n" << predicted_transform_msg.rotation << std::endl;

        predicted_transform_valid = 1;
        std::cout << "Predicting Camera Pose for next frame..." << std::endl;

        // Check flags and publish on vision_OK_flag_pub
        vision_OK_flag.data = (rectificationInitialized && beacons_detected && previous_frame && previous_frame_valid && predicted_transform_valid);
        vision_OK_flag_pub.publish(vision_OK_flag);

        endTime = ros::Time::now();
        sectionDuration = endTime - startTime;
        std::cout << "3D Motion Prediction Duration: " << sectionDuration.toSec() << std::endl;
        startTime = ros::Time::now();

        return;
        

        //debug
        //cv_ptr->image = difference_image;
        // Output modified video stream
        //image_pub_.publish(cv_ptr->toImageMsg());

    }
    //beacons were already in view last frame (initial guess for beacon locations valid)
    //this else does tracking
    else if(predicted_transform_valid == 1)//beacons are detected 
    {
        std::cout << "Attempting to predict beacon locations..." << std::endl;
        //get camera motion since last frame
        cvMat_tVec = (cv::Mat_<double>(3,1) << predicted_transform_msg.translation.x, predicted_transform_msg.translation.y, predicted_transform_msg.translation.z);

        camera_world_pose = tf::Quaternion(predicted_transform_msg.rotation.x,predicted_transform_msg.rotation.y,predicted_transform_msg.rotation.z,predicted_transform_msg.rotation.w);

        tf::Matrix3x3 tfRotationMatrix3x3(camera_world_pose);

        cvMat_rotationMatrix = (cv::Mat_<double>(3,3) << tfRotationMatrix3x3[0][0], tfRotationMatrix3x3[0][1], tfRotationMatrix3x3[0][2], tfRotationMatrix3x3[1][0], tfRotationMatrix3x3[1][1], tfRotationMatrix3x3[1][2], tfRotationMatrix3x3[2][0], tfRotationMatrix3x3[2][1], tfRotationMatrix3x3[2][2]);

        cv::Rodrigues(cvMat_rotationMatrix,cvMat_rVec);

        cv::projectPoints(deck_3Dpoints_vector,cvMat_rVec,cvMat_tVec,cameraMatrix,distortionCoefficients,tracking_projected_points);

        //std::cout << "tracking_projected_points are: " << tracking_projected_points << std::endl;


        //make sure projected points are actually in the image
        for(int i = 0;i<NUMOFBEACONS;i++)
        {
          //claim lost beacons if not in image
          if(tracking_projected_points[i].x < 0 || tracking_projected_points[i].x > output_image.size().width-1)
          {
            predicted_transform_valid = 0;
            beacons_detected = 0;
            previous_frame_valid = 0;
            previous_frame = 0;
            std::cout << "Predicted points out of FOV..." << std::endl;
            // Check flags and publish on vision_OK_flag_pub
            vision_OK_flag.data = (rectificationInitialized && beacons_detected && previous_frame && previous_frame_valid && predicted_transform_valid);
            vision_OK_flag_pub.publish(vision_OK_flag);
            return;
          }
          if(tracking_projected_points[i].y < 0 || tracking_projected_points[i].y > output_image.size().height-1)
          {
            predicted_transform_valid = 0;
            beacons_detected = 0;
            previous_frame_valid = 0;
            previous_frame = 0;
            std::cout << "Predicted points out of FOV..." << std::endl;
            // Check flags and publish on vision_OK_flag_pub
            vision_OK_flag.data = (rectificationInitialized && beacons_detected && previous_frame && previous_frame_valid && predicted_transform_valid);
            vision_OK_flag_pub.publish(vision_OK_flag);
            return;
          }


          //force points to be in image
          //tracking_projected_points[i].x = std::max((float)0.0,std::min(tracking_projected_points[i].x,(float)output_image.size().width));
          //tracking_projected_points[i].y = std::max((float)0.0,std::min(tracking_projected_points[i].y,(float)output_image.size().height));
        }



        //dummy = cv::connectedComponents(output_image,labels);
        //cv::connectedComponentsWithStats();
        
        //connectedComponentLabels = cv::Mat(output_image.size(),CV_32S);
        //std::cout << "size of connectedComponentLabels.total(): " << connectedComponentLabels.total() << std::endl;
        //std::cout << "size of stats.total(): " << stats.total() << std::endl;
        //std::cout << "size of connectedComponentCenters.size(): " << connectedComponentCenters.size() << std::endl;
        endTime = ros::Time::now();
        sectionDuration = endTime - startTime;
        std::cout << "Predicted BEacon Reprojection Duration: " << sectionDuration.toSec() << std::endl;
        startTime = ros::Time::now();
        std::cout << "Connecting Components..." << std::endl;
        //numOfConnectedComponents = cv::connectedComponentsWithStats(output_image,connectedComponentLabels,connectedComponentStats,connectedComponentCenters,8);
        //numOfConnectedComponents = cv::connectedComponents(output_image,connectedComponentLabels);
        std::cout << "number of connectedComponents: " << numOfConnectedComponents << std::endl;
        //std::cout << "size of connectedComponentLabels: " << connectedComponentLabels.size() << std::endl;
        //std::cout << "connectedComponentLabels.total(): " << connectedComponentLabels.total() << std::endl;
        //connectedComponentLabels.convertTo(connectedComponentLabels, CV_8UC1);
        //cv::findNonZero(connectedComponentLabels,nonZeroLocations);
        //nonZeroLocations.convertTo(data, CV_32F);
        //std::cout << "data.total(): " << data.total() << std::endl;

        //std::cout << "nonZeroLocations.total(): " << nonZeroLocations.total() << std::endl;
        std::cout << "connectedComponentCenters: \n" << connectedComponentCenters << std::endl;
        std::cout << "tracking_projected_points: \n" << tracking_projected_points << std::endl;

        //search current frame around reprojected points
        cv::Rect roi_rect, mask_rect;
        cv::Mat roi,mask_roi, mask;
        cv::Rect roiToRemove_rect;
        cv::Mat roiToRemove;

        double dx, dy;
        double best_dx, best_dy;
        int best_label;
        int tracking_label;
        double distance;
        cv::Size size;

        for (int i = 0; i<NUMOFBEACONS; i++)
        {
          double minDistance = 999999;
          for (int j = 1; j<numOfConnectedComponents; j++)
          {
            //std::cout << "tracking_projected_points[i].x: " << tracking_projected_points[i].x << std::endl;
            //std::cout << "tracking_projected_points[i].y: " << tracking_projected_points[i].y << std::endl;
            //std::cout << "connectedComponentCenters.type(): " << connectedComponentCenters.type() << std::endl;
            //std::cout << "connectedComponentCenters.at<double>(j,0): " << connectedComponentCenters.at<double>(j,0) << std::endl;
            //std::cout << "connectedComponentCenters.at<double>(j,1): " << connectedComponentCenters.at<double>(j,1) << std::endl;
            dx = connectedComponentCenters.at<double>(j,0) - tracking_projected_points[i].x;
            dy = connectedComponentCenters.at<double>(j,1) - tracking_projected_points[i].y;
            //std::cout << "dx: " << dx << std::endl;
            //std::cout << "dy: " << dy << std::endl;
            distance = sqrt(dx*dx + dy*dy);
            //std::cout << "distance: " << distance << std::endl;
            if (distance < minDistance)
            {
              minDistance = distance;
              best_label = j;
            }
          }
          
          int thisBeaconIndex = -1;
          for (int j = 0; j<NUMOFBEACONS; j++)
          {
            best_dx = connectedComponentCenters.at<double>(best_label,0) - tracking_projected_points[j].x;
            best_dy = connectedComponentCenters.at<double>(best_label,1) - tracking_projected_points[j].y;
            //std::cout << "best_dx: " << best_dx << std::endl;
            //std::cout << "best_dy: " << best_dy << std::endl;
            int beaconCount = 0;

            for (int k = 0; k<NUMOFBEACONS; k++)
            { 
              std::cout << "Checking if beacon " << k << " is at: " << tracking_projected_points[k].x + best_dx <<","<<tracking_projected_points[k].y + best_dy << std::endl;
              std::cout << "Maximum x value is: " << output_image.size().width << std::endl;
              std::cout << "Maximum y value is: " << output_image.size().height << std::endl;
              //claim lost beacons if not in image
              if(tracking_projected_points[k].x+best_dx < 0 || tracking_projected_points[k].x+best_dx > output_image.size().width-1)
              {
                std::cout << "Beacon shift out of FOV" << std::endl;
                break;
              }
              if(tracking_projected_points[k].y+best_dy < 0 || tracking_projected_points[k].y+best_dy > output_image.size().height-1)
              {
                std::cout << "Beacon shift out of FOV" << std::endl;
                break;
              }

              roi_rect = cv::Rect_<double>(tracking_projected_points[k].x + best_dx,tracking_projected_points[k].y + best_dy,1,1);
              roi = connectedComponentLabels(roi_rect);
              roi.adjustROI(3*detectedBeaconRadius,3*detectedBeaconRadius,3*detectedBeaconRadius,3*detectedBeaconRadius);
              //cv::Rect search_rect = cv::Rect_<double>(tracking_projected_points[k].x + best_dx-3*detectedBeaconRadius,tracking_projected_points[k].y + best_dy-3*detectedBeaconRadius,6*detectedBeaconRadius,6*detectedBeaconRadius);
              //cv::rectangle(output_image,search_rect,100);
              //std::cout << "roi " << roi << std::endl;
              //std::cout << "detectedBeaconRadius: " << detectedBeaconRadius << std::endl;
              //std::cout << "roi.total(): " << roi.total() << std::endl;
              cv::findNonZero(roi,nonZeroLocations);
              //std::cout << "nonZeroLocations: " << nonZeroLocations << std::endl;
              //std::cout << "nonZeroLocations.total(): " << nonZeroLocations.total() << std::endl;
              //nonZeroLocations.convertTo(data, CV_32F);
              //std::cout << "data.total(): " << data.total() << std::endl;
              if (nonZeroLocations.total() == 0)
              { 
                std::cout << "Beacon not detected" << std::endl;
                //beaconCount = 0;
                break;
              }
              else
              {
                std::cout << "Beacon is detected" << std::endl;
                // Check which label corresponds to the detected light and save it in order
                
              
                //std::cout << "nonZeroLocations: " << nonZeroLocations << std::endl;
                //std::cout << "nonZeroLocations.at<cv::Point>(0).x: " << nonZeroLocations.at<cv::Point>(0).x << std::endl;

                pt.x = nonZeroLocations.at<cv::Point>(0).x;
                pt.y = nonZeroLocations.at<cv::Point>(0).y;

                //std::cout << "after pt = (" << pt.x << "," << pt.y << ")" << std::endl;
                //std::cout << "connectedComponentLabels.depth() = " << connectedComponentLabels.depth() << std::endl;
                tracking_label = (int) roi.at<char>(pt);
                //std::cout << "tracking_label is: " << tracking_label << std::endl;
                source_vector[k].x = connectedComponentCenters.at<double>(tracking_label,0);
                source_vector[k].y = connectedComponentCenters.at<double>(tracking_label,1);
                //std::cout << "source_vector[" << k << "] = " << source_vector[k] << std::endl;

                temp_beacon_area = connectedComponentStats.at<int>(tracking_label,4);
                
                beaconCount++;
              }
            }
            if (beaconCount == 4)
            {
              thisBeaconIndex = j;
              break;
            }
          }

          if (thisBeaconIndex == -1)
          {
            // remove this connected component and continue to next beacon
            roiToRemove_rect = cv::Rect_<double>(connectedComponentStats.at<int>(best_label,0),
                                                connectedComponentStats.at<int>(best_label,1),
                                                connectedComponentStats.at<int>(best_label,2),
                                                connectedComponentStats.at<int>(best_label,3));
            roiToRemove = output_image(roiToRemove_rect);
            roiToRemove.setTo(0);
            
          }
          else
            break;

        }



        
        ////////////////// DEBUG ///////////////////////
        // cv_ptr->image = output_image;
        // image_pub_.publish(cv_ptr->toImageMsg());
        // return;
        ///////////////////////////////////////////////

/*
        for(int i = 0;i<NUMOFBEACONS;i++)
        {
          std::cout << "Attempting to track beacon " << i << std::endl;
          searchRadius = 1;
          roi_rect = cv::Rect_<float>(tracking_projected_points[i].x,tracking_projected_points[i].y,searchRadius,searchRadius);
          roi = output_image(roi_rect);
          while(searchRadius < maxSearchRadius) {
            //see if roi is nonzero
            cv::findNonZero(roi,nonZeroLocations);
            nonZeroLocations.convertTo(data, CV_32F);

            //if so, snap tracking point to center of region and break
            if(nonZeroLocations.total() > 0) 
            {
              std::cout << "searchRadius: " << searchRadius << std::endl;
              std::cout << "nonZeroLocations.total(): " << nonZeroLocations.total() << std::endl;
              pt = nonZeroLocations.at<cv::Point>(0);
              std::cout << "pt.x: " << pt.x << "  pt.y: " << pt.y << std::endl;
              
              //move tracked points to target
              tracking_projected_points[i].x = tracking_projected_points[i].x-searchRadius+pt.x;
              tracking_projected_points[i].y = tracking_projected_points[i].y-searchRadius+pt.y;

              //build mask
              mask_rect = cv::Rect_<float>(tracking_projected_points[i].x,tracking_projected_points[i].y,1,1);
              mask_roi = mask(mask_rect);
              mask_roi.adjustROI(3*detectedBeaconRadius,3*detectedBeaconRadius,3*detectedBeaconRadius,3*detectedBeaconRadius);

              mask_roi.setTo(255);

              break;
            }
            //if not, expand search radius
            else 
            {
              searchRadius += searchIncrement;
              roi.adjustROI(searchIncrement,searchIncrement,searchIncrement,searchIncrement);
            }

          }

        }
*/
        std::cout << "tracking_projected_points[0].x: " << tracking_projected_points[0].x << "  tracking_projected_points[0].y: " << tracking_projected_points[0].y << std::endl;
        std::cout << "tracking_projected_points[1].x: " << tracking_projected_points[1].x << "  tracking_projected_points[1].y: " << tracking_projected_points[1].y << std::endl;
        std::cout << "tracking_projected_points[2].x: " << tracking_projected_points[2].x << "  tracking_projected_points[2].y: " << tracking_projected_points[2].y << std::endl;
        std::cout << "tracking_projected_points[3].x: " << tracking_projected_points[3].x << "  tracking_projected_points[3].y: " << tracking_projected_points[3].y << std::endl;

        std::cout << "source_vector[0] = " << source_vector[0] << std::endl;
        std::cout << "source_vector[1] = " << source_vector[1] << std::endl;
        std::cout << "source_vector[2] = " << source_vector[2] << std::endl;
        std::cout << "source_vector[3] = " << source_vector[3] << std::endl;

        //if found (bright things in all 4 search areas)
          //suppress everything not in search areas
          //cv::bitwise_and(output_image,mask,output_image);

        endTime = ros::Time::now();
        sectionDuration = endTime - startTime;
        std::cout << "Tracking Duration: " << sectionDuration.toSec() << std::endl;
        startTime = ros::Time::now();

        //if not found, beacons are missing and we need to get new initial positions
        if(source_vector[3].x == 0 && source_vector[3].y == 0) {
          beacons_detected = 0;
          previous_frame = 0;
          previous_frame_valid = 0;
          
          std::cout << "Lost sight of beacons, resetting...." << std::endl;
          // Check flags and publish on vision_OK_flag_pub
          vision_OK_flag.data = (rectificationInitialized && beacons_detected && previous_frame && previous_frame_valid && predicted_transform_valid);
          vision_OK_flag_pub.publish(vision_OK_flag);
          return;
        }





        //output_image.copyTo(previous_frame_output_image);
    }


    //main algorithm

    //put current frame into storage for previous frame
    output_image.copyTo(previous_frame_output_image);


    detectedBeaconRadius = std::max((int) sqrt(temp_beacon_area/3.14),5);
    std::cout << "detectedBeaconRadius: " << detectedBeaconRadius << std::endl;

    ///////////////////////////////////////////////////////////////////////
    // data should now hold only the pixels that we consider to be beacons
    ///////////////////////////////////////////////////////////////////////



    //debug circles to track which point is which
    // for(int i = 0;i<NUMOFBEACONS;i++) {
    //     for(int j = 0; j < i+1; j++) {
    //         //cv::circle(output_image, previous_source_vector[i], NUMOFBEACONS*(j+1), 100);
    //         cv::circle(output_image, tracking_projected_points[i], NUMOFBEACONS*(j+1), 150);
    //     }
    // }

    //circles to track which point is which
    for(int i = 0;i<NUMOFBEACONS;i++) {
        previous_source_vector[i] = source_vector[i];
        for(int j = 0; j < i+1; j++) {
          cv::circle(output_image, source_vector[i], NUMOFBEACONS*(j+1), 255);

        }
    }







    //make camera matrix
    //regular
    //cameraMatrix = (cv::Mat_<double>(3,3) << 1560, 0, 633.167287, 0 , 1560, 517.683457, 0, 0, 1);
    //fisheye:
    //cameraMatrix = (cv::Mat_<double>(3,3) << 325.768218, 0, 641.552114, 0 , 325.987229, 511.739130, 0, 0, 1);
    
    


    cv::solvePnP(deck_3Dpoints_vector,source_vector,cameraMatrix,distortionCoefficients,rotationVector,translationVector);
    


    //convert rotation vecotr to useable rotation matrix
    cv::Rodrigues(rotationVector,rotationMatrix);


    
    //TODO need to convert rotation matrix to quaternion to pass to TF
    //that will allow the final TF node to make the deck orientation in the right frame

    //extract roll, pitch yaw from rotation matrix
    eulerRotationAngles = cv::RQDecomp3x3(rotationMatrix,mtxR,mtxQ,qX,qY,qZ);
    
    

    //std::cout << "qX[0] = " << qX.at<double>(0) << std::endl;
    //std::cout << "qX[1] = " << qX.at<double>(1) << std::endl;
    //std::cout << "qX[2] = " << qX.at<double>(2) << std::endl;
    //std::cout << "qX[3] = " << qX.at<double>(3) << std::endl;
    //std::cout << "qX[4] = " << qX.at<double>(4) << std::endl;
    //std::cout << "qX[5] = " << qX.at<double>(5) << std::endl;
    //std::cout << "qX[6] = " << qX.at<double>(6) << std::endl;
    //std::cout << "qX[7] = " << qX.at<double>(7) << std::endl;
    //std::cout << "qX[8] = " << qX.at<double>(8) << std::endl;



    //std::cout << "qX = " << std::endl << " " << qX << std::endl << std::endl;
    //std::cout << "qY = " << std::endl << " " << qY << std::endl << std::endl; 
    //std::cout << "qZ = " << std::endl << " " << qZ << std::endl << std::endl; 
    //print output rotation angles
    //std::cout << "eulerRotationAngles: " << eulerRotationAngles << std::endl;

    double xThetaRad, yThetaRad, zThetaRad;
    double xThetaDeg, yThetaDeg, zThetaDeg;

    //xThetaRad = acos(qX.at<double>(4));
    //yThetaRad = acos(qY.at<double>(0));
    //zThetaRad = acos(qZ.at<double>(0));

    xThetaRad = -atan2(qX.at<double>(7),qX.at<double>(4));
    yThetaRad = -atan2(qY.at<double>(2),qY.at<double>(0));
    zThetaRad = -atan2(qZ.at<double>(3),qZ.at<double>(0));


    xThetaDeg = xThetaRad*180/3.1415;
    yThetaDeg = yThetaRad*180/3.1415;
    zThetaDeg = zThetaRad*180/3.1415;


    //std::cout << "xTheta (radians): " << xThetaRad << std::endl;
    //std::cout << "yTheta (radians): " << yThetaRad << std::endl;
    //std::cout << "zTheta (radians): " << zThetaRad << std::endl;


    std::cout << "xTheta (degrees): " << xThetaDeg << std::endl;
    std::cout << "yTheta (degrees): " << yThetaDeg << std::endl;
    std::cout << "zTheta (degrees): " << zThetaDeg << std::endl;



    //print output of solvePnp
    //std::cout << "rotationMatrix = " << std::endl << " " << rotationMatrix << std::endl << std::endl; 
    std::cout << "rvec: " << rotationVector << std::endl;
    std::cout << "tvec: " << translationVector << std::endl;


    //publish pose
    //tempFloat64.data = translationVector.at<double>(0);
    //deckPose.linear.x = tempFloat64.data;
    deckPose.linear.x = translationVector.at<double>(0);
    deckPose.linear.y = translationVector.at<double>(1);
    deckPose.linear.z = translationVector.at<double>(2);
    deckPose.angular.x = 0;//eulerRotationAngles[0];
    deckPose.angular.y = 0;//eulerRotationAngles[1];
    deckPose.angular.z = 0;//eulerRotationAngles[2];
    
    deckPose_pub.publish(deckPose);


    tf::Quaternion quat;
    tf::Matrix3x3 tfMatrix;

    //tfMatrix.setRPY(rotationVector.at<double>(1),rotationVector.at<double>(0),rotationVector.at<double>(2));
    tfMatrix.setRPY(xThetaRad,yThetaRad,zThetaRad);
    tfMatrix.getRotation(quat);

    quaternion_msg.x = quat[0];
    quaternion_msg.y = quat[1];
    quaternion_msg.z = quat[2];
    quaternion_msg.w = quat[3];

    quaternion_pub.publish(quaternion_msg);


    //std::cout << "quaternion[0] = " << quat[0] << std::endl;
    //std::cout << "quaternion[1] = " << quat[1] << std::endl;
    //std::cout << "quaternion[2] = " << quat[2] << std::endl;
    //std::cout << "quaternion[3] = " << quat[3] << std::endl;


    //publish StampedTransform
    //for SAM
    vector3_msg.x = translationVector.at<double>(0);
    vector3_msg.y = translationVector.at<double>(1);
    vector3_msg.z = translationVector.at<double>(2);

    transform_msg.translation = vector3_msg;
    transform_msg.rotation = quaternion_msg;

    //set predicted frame data
    if(previous_frame_valid) {  
        predicted_transform_msg.translation.x = 2*transform_msg.translation.x-previous_transform_msg.translation.x;
        predicted_transform_msg.translation.y = 2*transform_msg.translation.y-previous_transform_msg.translation.y;
        predicted_transform_msg.translation.z = 2*transform_msg.translation.z-previous_transform_msg.translation.z;

        tf::Quaternion previous_quat = tf::Quaternion(previous_transform_msg.rotation.x,previous_transform_msg.rotation.y,previous_transform_msg.rotation.z,previous_transform_msg.rotation.w);
        //copy current quaternion
        tf::Quaternion delta_quat = tf::Quaternion(quat[0],quat[1],quat[2],quat[3]);
        tf::Quaternion predicted_quat = tf::Quaternion(quat[0],quat[1],quat[2],quat[3]);
        delta_quat *= previous_quat.inverse();
        predicted_quat *= delta_quat;

        predicted_transform_msg.rotation.x = predicted_quat[0];
        predicted_transform_msg.rotation.y = predicted_quat[1];
        predicted_transform_msg.rotation.z = predicted_quat[2];
        predicted_transform_msg.rotation.w = predicted_quat[3];

        std::cout << "predicted_transform_msg.translation: \n" << predicted_transform_msg.translation << std::endl;
        std::cout << "previous_transform_msg.rotation: \n" << previous_transform_msg.rotation << std::endl;
        std::cout << "previous_quat[0]: " << previous_quat[0] << std::endl;
        std::cout << "previous_quat[1]: " << previous_quat[1] << std::endl;
        std::cout << "previous_quat[2]: " << previous_quat[2] << std::endl;
        std::cout << "previous_quat[3]: " << previous_quat[3] << std::endl;
        std::cout << "predicted_transform_msg.rotation: \n" << predicted_transform_msg.rotation << std::endl;

        predicted_transform_valid = 1;
    }




    previous_transform_msg = transform_msg;

    header_msg.seq = sequence_msg.data; 

    stampTime_msg.data = ros::Time::now();
    header_msg.stamp = stampTime_msg.data;

    header_msg.frame_id = cameraFrameString_msg.data;

    transformStamped_msg.header = header_msg;
    transformStamped_msg.child_frame_id = deckFrameString_msg.data;
    transformStamped_msg.transform = transform_msg;

    transformStamped_pub.publish(transformStamped_msg);

    previous_frame_valid = 1;

    //increment sequence number
    sequence_msg.data = (sequence_msg.data+1)%4294967295; //mod 2^32-1

    //////////////////////////////
    //debug
    //////////////////////////////
    std::vector<cv::Point2d> projectedPoints;

    cv::projectPoints(deck_3Dpoints_vector, rotationVector, translationVector, cameraMatrix, distortionCoefficients, projectedPoints);
    
    for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
        std::cout << "Image point: " << source_vector[i] << " Projected to " << projectedPoints[i] << std::endl;
    }
    

    //printf("angle %d is: %f\n",i,angle[i]);
    //printf("minimumAngle is: %f\n",minimumAngle);

    //test_ptr2->image = test_output_image2;

    //////////////////////////////////////////
    /// END DEBUG
    ///////////////////////////////////////////

    endTime = ros::Time::now();
    sectionDuration = endTime - startTime;
    std::cout << "solvePnp Duration: " << sectionDuration.toSec() << std::endl;

    sectionDuration = endTime - totalStartTime;
    std::cout << "Total Vision Duration: " << sectionDuration.toSec() << std::endl;

    cv_ptr->image = output_image;

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

    // Check flags and publish on vision_OK_flag_pub
    vision_OK_flag.data = (rectificationInitialized && beacons_detected && previous_frame && previous_frame_valid && predicted_transform_valid);
    vision_OK_flag_pub.publish(vision_OK_flag);


  }

  // Determine the intersection point of two lines, if this point exists
  /*! Two lines intersect if they are not parallel (Parallel lines intersect at
  * +/- infinity, but we do not consider this case here).
  *
  * The lines are specified by a pair of points each. If they intersect, then
  * the function returns true, else it returns false.
  *
  * Lines can be specified in the following form:
  *      A1x + B1x = C1
  *      A2x + B2x = C2
  *
  * If det (= A1*B2 - A2*B1) == 0, then lines are parallel
  *                                else they intersect
  *
  * If they intersect, then let us denote the intersection point with P(x, y) where:
  *      x = (C1*B2 - C2*B1) / (det)
  *      y = (C2*A1 - C1*A2) / (det)
  *
  * @param a1 First point for determining the first line
  * @param b1 Second point for determining the first line
  * @param a2 First point for determining the second line
  * @param b2 Second point for determining the second line
  * @param intersection The intersection point, if this point exists
  */
  static bool lineIntersection(const cv::Point2d &a1, const cv::Point2d &b1, const cv::Point2d &a2,
                               const cv::Point2d &b2, cv::Point2d &intersection) {
      double A1 = b1.y - a1.y;
      double B1 = a1.x - b1.x;
      double C1 = (a1.x * A1) + (a1.y * B1);

      double A2 = b2.y - a2.y;
      double B2 = a2.x - b2.x;
      double C2 = (a2.x * A2) + (a2.y * B2);

      double det = (A1 * B2) - (A2 * B1);

      if (!almostEqual(det, 0)) {
          intersection.x = static_cast<double>(((C1 * B2) - (C2 * B1)) / (det));
          intersection.y = static_cast<double>(((C2 * A1) - (C1 * A2)) / (det));

          return true;
      }

      return false;
  }

  //! Check if the two numbers are equal (almost)
  /*!
  * The expression for determining if two real numbers are equal is:
  * if (Abs(x - y) <= EPSILON * Max(1.0f, Abs(x), Abs(y))).
  *
  * @param number1 First number
  * @param number2 Second number
  */
  static bool almostEqual(double number1, double number2) {
      return (std::abs(number1 - number2) <= (EPSILON * std::max(std::max(number1, number2), 1.0)));
  }

};




//------------------------------------------------------------------------------
int get_ocam_model(struct ocam_model *myocam_model, char *filename)
{
 double *pol        = myocam_model->pol;
 double *invpol     = myocam_model->invpol; 
 double *xc         = &(myocam_model->xc);
 double *yc         = &(myocam_model->yc); 
 double *c          = &(myocam_model->c);
 double *d          = &(myocam_model->d);
 double *e          = &(myocam_model->e);
 int    *width      = &(myocam_model->width);
 int    *height     = &(myocam_model->height);
 int *length_pol    = &(myocam_model->length_pol);
 int *length_invpol = &(myocam_model->length_invpol);
 FILE *f;
 char buf[CMV_MAX_BUF];
 int i;
 int returnVal;
 char* returnCharPtr;
 
 //Open file
 if(!(f=fopen(filename,"r")))
 {
   printf("File %s cannot be opened\n", filename);                
   return -1;
 }
 
 //Read polynomial coefficients
 returnCharPtr = fgets(buf,CMV_MAX_BUF,f);
 returnVal = fscanf(f,"\n");
 returnVal = fscanf(f,"%d", length_pol);
 for (i = 0; i < *length_pol; i++)
 {
     returnVal = fscanf(f," %lf",&pol[i]);
 }

 //Read inverse polynomial coefficients
 returnVal = fscanf(f,"\n");
 returnCharPtr = fgets(buf,CMV_MAX_BUF,f);
 returnVal = fscanf(f,"\n");
 returnVal = fscanf(f,"%d", length_invpol);
 for (i = 0; i < *length_invpol; i++)
 {
     returnVal = fscanf(f," %lf",&invpol[i]);
 }
 
 //Read center coordinates
 returnVal = fscanf(f,"\n");
 returnCharPtr = fgets(buf,CMV_MAX_BUF,f);
 returnVal = fscanf(f,"\n");
 returnVal = fscanf(f,"%lf %lf\n", xc, yc);

 //Read affine coefficients
 returnCharPtr = fgets(buf,CMV_MAX_BUF,f);
 returnVal = fscanf(f,"\n");
 returnVal = fscanf(f,"%lf %lf %lf\n", c,d,e);

 //Read image size
 returnCharPtr = fgets(buf,CMV_MAX_BUF,f);
 returnVal = fscanf(f,"\n");
 returnVal = fscanf(f,"%d %d", height, width);

 fclose(f);
 return 0;
}

//------------------------------------------------------------------------------
void cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model)
{
 double *pol    = myocam_model->pol;
 double xc      = (myocam_model->xc);
 double yc      = (myocam_model->yc); 
 double c       = (myocam_model->c);
 double d       = (myocam_model->d);
 double e       = (myocam_model->e);
 int length_pol = (myocam_model->length_pol); 
 double invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

 double xp = invdet*(    (point2D[0] - xc) - d*(point2D[1] - yc) );
 double yp = invdet*( -e*(point2D[0] - xc) + c*(point2D[1] - yc) );
  
 double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
 double zp  = pol[0];
 double r_i = 1;
 int i;
 
 for (i = 1; i < length_pol; i++)
 {
   r_i *= r;
   zp  += r_i*pol[i];
 }
 
 //normalize to unit norm
 double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );
 
 point3D[0] = invnorm*xp;
 point3D[1] = invnorm*yp; 
 point3D[2] = invnorm*zp;
}

//------------------------------------------------------------------------------
void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model)
{
 double *invpol     = myocam_model->invpol; 
 double xc          = (myocam_model->xc);
 double yc          = (myocam_model->yc); 
 double c           = (myocam_model->c);
 double d           = (myocam_model->d);
 double e           = (myocam_model->e);
 int    width       = (myocam_model->width);
 int    height      = (myocam_model->height);
 int length_invpol  = (myocam_model->length_invpol);
 double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
 double theta       = atan(point3D[2]/norm);
 double t, t_i;
 double rho, x, y;
 double invnorm;
 int i;
  
  if (norm != 0) 
  {
    invnorm = 1/norm;
    t  = theta;
    rho = invpol[0];
    t_i = 1;

    for (i = 1; i < length_invpol; i++)
    {
      t_i *= t;
      rho += t_i*invpol[i];
    }

    x = point3D[0]*invnorm*rho;
    y = point3D[1]*invnorm*rho;
  
    point2D[0] = x*c + y*d + xc;
    point2D[1] = x*e + y   + yc;
  }
  else
  {
    point2D[0] = xc;
    point2D[1] = yc;
  }
}
//------------------------------------------------------------------------------
void create_perspecive_undistortion_LUT( CvMat *mapx, CvMat *mapy, struct ocam_model *ocam_model, float sf)
{
     int i, j;
     int width = mapx->cols; //New width
     int height = mapx->rows;//New height     
     float *data_mapx = mapx->data.fl;
     float *data_mapy = mapy->data.fl;
     float Nxc = height/2.0;
     float Nyc = width/2.0;
     float Nz  = -width/sf;
     double M[3];
     double m[2];
     
     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {   
             M[0] = (i - Nxc);
             M[1] = (j - Nyc);
             M[2] = Nz;
             world2cam(m, M, ocam_model);
             *( data_mapx + i*width+j ) = (float) m[1];
             *( data_mapy + i*width+j ) = (float) m[0];
         }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "kingfisher_vision_short");
  KingfisherVisionShort ic;
  ros::spin();
  return 0;
}
