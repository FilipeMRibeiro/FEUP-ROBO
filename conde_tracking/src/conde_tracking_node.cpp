#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <algorithm>
#include <math.h>
#include "geometry_msgs/Twist.h"


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

#include <vector>

using namespace std;
using namespace cv;

bool ipmLeftDone;
bool ipmRightDone;

double angle = 0;
double speedMult = 2.5; //Speed multiplier

/// Intrisic Camera parameters
/*
    [   alpha_x ,   beta    ,   x0
        0       ,   alpha_Y ,   y0
        0       ,   0       ,   1   ]
    alpha_x, alpha_y -> focal length
    x0, y0           -> principal point
*/
//------------------------Camera right parameters----------------------------------------------
cv::Mat cameraRight_intrinsic = (cv::Mat_<double>(3,3) << \
                                 130.81305          , 0                     , 79.81980 ,\
                                 0                  , 131.22421             , 58.91209,\
                                 0                  , 0                     , 1);

cv::Mat cameraRight_T_chess_robot = (cv::Mat_<double>(4,4) <<  \
                                     -1,    0,  0,  0.612,\
                                     0,   -1,  0,  0.0,\
                                     0,    0,  1,  -0.004,\
                                     0,    0,  0,  1);

cv::Mat cameraRight_T_cam_chess = (cv::Mat_<double>(4,4) << \
                                   0.330007,      0.918154,       0.219294,     -0.540327012,\
                                   0.562501,     -0.004706,      -0.826783,     -0.018409465,\
                                   -0.758082,      0.396197,      -0.518016,      1.038223574,\
                                   0       ,      0       ,       0       ,      1);
// Distortion coeficients
cv::Mat cameraRight_dist_coef = (cv::Mat_<double>(1,4) << -0.275678598507515 , 0.045106260288961 ,
                                 0.004883645512607 , 0.001092737340199);

//------------------------Camera left parameters----------------------------------------------

cv::Mat cameraLeft_intrinsic = (cv::Mat_<double>(3,3) << \
                                132.31872          , 0                     , 74.70743 ,\
                                0                  , 132.17822             , 52.77469,\
                                0                  , 0                     , 1);

cv::Mat cameraLeft_T_chess_robot = (cv::Mat_<double>(4,4) <<  \
                                    -1,    0,  0,  0.633,\
                                    0,   -1,  0,  0.72,\
                                    0,    0,  1,  -0.004,\
                                    0,    0,  0,  1);

cv::Mat cameraLeft_T_cam_chess = (cv::Mat_<double>(4,4) << \
                                  -0.436125,      0.850786,      -0.293187,     0.081835848,\
                                  0.553047,     -0.003607,      -0.833142,     0.014687224,\
                                  -0.709883,     -0.525501,      -0.468951,     1.363192844,\
                                  0       ,      0       ,       0       ,     1);
// Distortion coeficients
cv::Mat cameraLeftt_dist_coef = (cv::Mat_<double>(1,4) << -0.275678598507515 , 0.045106260288961 ,
                                 0.004883645512607 , 0.001092737340199);
//--------------------------------------------------------------------------------------------

ros::Publisher dist_angle_pub;
ros::Publisher crossWalk_pub;
ros::Publisher publisher;

///--------------------------------------------------------------------------------------------------
void imageMergeAndTrack()
{
	double distIPM = 0.375;// meters
	double angleIPM = 0; // rad
	
	/// Publish Distance and Angle Info
	std_msgs::Float64MultiArray array;
	array.data.clear();
	array.data.push_back(distIPM);
	array.data.push_back(angleIPM);//angleIPM
	ros::Rate loop_rate(200*speedMult);
	dist_angle_pub.publish(array);
	ros::spinOnce();
	loop_rate.sleep();

}

///------------------------------------------------LEFT--------------------------------------------------
void imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;  
        cv::imshow("left rgb", img_rgb);
        
        ipmLeftDone = true;
        
        uint8_t k = cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    if(ipmRightDone && ipmLeftDone)
    {
        ipmLeftDone = false;
        ipmRightDone = false;
        imageMergeAndTrack();
    }

}
///----------------------------------------------RIGHT-----------------------------------------------

Point history(14, 40);
double angular = 0; // robot's angular velocity
int numberLines = 15;
int numberCols = 25; 

Mat rotate(Mat src, double angle)
{
    Mat dst;
    Point2f pt(src.cols/2, src.rows/2);
    Mat r = getRotationMatrix2D(pt, angle, 1.0);
    warpAffine(src, dst, r, Size(src.cols, src.rows));
    return dst;
}

void imageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        Mat img_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
        ////////////////////////////////////////////////////////////////////////////////////////////////
        
        Mat ROI(img_rgb, Rect(30, 0, 50, 55));
        Mat croppedImage, croppedImageGray;
        ROI.copyTo(croppedImage);
        Mat croppedImageRot = rotate(croppedImage, -14);
        
        for (int i = 0; i < numberLines; i++)
        {
            for (int j = 0; j < croppedImageRot.cols; j++ )
            {
                Vec3b color = croppedImageRot.at<Vec3b>(Point(j,i));
                color[0] = 0;
                color[1] = 0;
                color[2] = 0;
                croppedImageRot.at<Vec3b>(Point(j,i)) = color;
            }
        }
        
        for (int i = 0; i < croppedImageRot.rows; i++)
        {
            for (int j = numberCols; j < croppedImageRot.cols; j++ )
            {
                Vec3b color = croppedImageRot.at<Vec3b>(Point(j,i));
                color[0] = 0;
                color[1] = 0;
                color[2] = 0;
                croppedImageRot.at<Vec3b>(Point(j,i)) = color;
            }
        }

        for (int i = 0; i < croppedImageRot.cols; i++)
        {
            Vec3b color;
            color[0] = 0;
            color[1] = 0;
            color[2] = 0;
            croppedImageRot.at<Vec3b>(Point(i,26)) = color;
            croppedImageRot.at<Vec3b>(Point(i,27)) = color;
            croppedImageRot.at<Vec3b>(Point(i,28)) = color;
            croppedImageRot.at<Vec3b>(Point(i,36)) = color;
            croppedImageRot.at<Vec3b>(Point(i,37)) = color;
            croppedImageRot.at<Vec3b>(Point(i,46)) = color;
        }

        for (int i = 0; i < croppedImageRot.rows; i++)
        {
            for (int j = 35; j < croppedImageRot.cols; j++ )
            {
                Vec3b color = croppedImageRot.at<Vec3b>(Point(j,i));
                color[0] = 0;
                color[1] = 0;
                color[2] = 0;
                croppedImageRot.at<Vec3b>(Point(j,i)) = color;
            }
        }

        // convert image into a binary image
        cvtColor(croppedImageRot, croppedImageGray, CV_BGR2GRAY);
        threshold(croppedImageGray, croppedImageGray, 250, 255, THRESH_BINARY);
        
        // detect edges
        vector<Vec4i> lines;
        Canny(croppedImageGray, croppedImageGray, 128, 128, 3, false);

        // find image contours
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(croppedImageGray, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

        // calculate contour with greatest area
        double max = 0;
        int index;
        for(int i = 0; i < contours.size(); i++)
        {
            double temp = contourArea(contours[i]);
            if (temp > max)
            {
                max = temp;
                index = i;
            }
        }

        Point point;
        // get center of mass of greatest countour
        Moments mu = moments(contours[index], false);
        point.x = mu.m10/mu.m00;
        point.y = mu.m01/mu.m00;

        line(croppedImageRot, point, point, Scalar(0,0,255), 5);
        imshow("crop", croppedImageRot);

        // calculate angle
        double delta_x = point.x - 14;

        double distance = cv::norm(point - history);
        
        // verify point precision
        if (distance > 1000) {
            point.x = history.x;
            point.y = history.y;
            ROS_ERROR("NO POINT DETECTED.DEFAULT.");
        } else if (distance < 10) {
            // verify if turn is needed
            if (abs(delta_x) >= 0 && abs(delta_x) <= 5.0) 
                angular = 0;
            else 
                angular = -0.01 * delta_x;

            // update point coordinates
            history.x = point.x;
            history.y = point.y;
        }

        geometry_msgs::Twist velocity_msg;
        velocity_msg.linear.x = 0.10 * speedMult;
        velocity_msg.angular.z = angular * speedMult;
        publisher.publish(velocity_msg);
        ROS_ERROR("Distance : %f", distance);

        if (point.y >= 40 ) {
            numberCols = 35;
            numberLines = 25;
        } else {
            numberCols = 25;
            numberLines = 15;
        }

        imshow("right rgb", img_rgb);
        
        /////////////////////////////////////////////////////////////////////////////////////////////////

        ipmRightDone = true;
        
        uint8_t k = cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    if(ipmRightDone && ipmLeftDone)
    {
        ipmLeftDone = false;
        ipmRightDone = false;
        imageMergeAndTrack();
    }
    return;
}

///--------------------------------------------------------------------------------------------------
///--------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{

    /// init variables
    ros::init(argc, argv, "conde_tracking_node");
    ros::NodeHandle nh("~");
    cv::namedWindow("right rgb");
    cv::namedWindow("left rgb");

    std::string cameraRightTopic;
    std::string cameraLeftTopic;

    if(!(nh.getParam("camera_right", cameraRightTopic))){
        std::cerr << "Parameter (camera_right) not found" << std::endl;
        return 0;
    }

    std::cout << "Parameter camera_right: " << cameraRightTopic <<  std::endl;

    if(!(nh.getParam("camera_left", cameraLeftTopic))){
        std::cout << "Parameter (camera_left) not found" << std::endl;
        return 0;
    }
    std::cout << "Parameter camera_left: " << cameraLeftTopic <<  std::endl;

    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subRight = it.subscribe(cameraRightTopic, 1, imageRightCallback);
    image_transport::Subscriber subLeft = it.subscribe(cameraLeftTopic, 1, imageLeftCallback);

    dist_angle_pub = nh.advertise<std_msgs::Float64MultiArray>("/conde_dist_angle", 1);
    crossWalk_pub = nh.advertise<std_msgs::Bool>("/crossWalk", 1);
    
    // publish velocity
    publisher = nh.advertise<geometry_msgs::Twist>("/conde_tracking_info", 100);

    //ros::spin();
    
    ros::Rate r(2000*speedMult);
    while(ros::ok())
    {
      
      // wait until it's time for another iteration
    	ros::spinOnce();
    	r.sleep();
    }
    
    cv::destroyWindow("view");
}

