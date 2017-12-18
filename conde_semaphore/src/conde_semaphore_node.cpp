#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <opencv2/xfeatures2d/nonfree.hpp>

ros::Publisher semaphore_pub;

using namespace cv;
using namespace std; 
using namespace cv::xfeatures2d;

vector<DMatch> getGoodMatches(Mat descriptors_database, Mat descriptors_scene)
{
	double max_dist = 0;
	double min_dist = 100;
	vector<DMatch> matches;
	vector<DMatch> good_matches;
	FlannBasedMatcher matcher;
  	
	matcher.match( descriptors_database,descriptors_scene, matches );

	for (unsigned i = 0; i < matches.size(); i++)
	{
		double dist = matches[i].distance;

		if (dist < min_dist)
			min_dist = dist;

		if (dist > max_dist)
			max_dist = dist;
	}

	for (unsigned i = 0; i < matches.size(); i++)
	{
		if (matches[i].distance < 3 * min_dist)
		{
			good_matches.push_back(matches[i]);
		}
	}

	return good_matches;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
	cv::Mat img_gray;
	cv::Mat resize;	
	cv::Mat croppedImage;
	
	cv::Mat img_descriptors, data_descriptors;
	cv::Mat img_2 = imread( "/home/jorge/catkin_ws/src/FEUP-ROBO-master/gazebo_semaphore/semaphores_pics/stop.png", IMREAD_GRAYSCALE );
	cv::resize(img_2,resize,Size(),0.5,0.5);
        cv::Mat img_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
	cv::Mat ROI(img_rgb, Rect(90, 0, 230, 150));
	ROI.copyTo(croppedImage);
	cvtColor(img_rgb, img_gray, CV_BGR2GRAY);

	Ptr<SIFT> detector = SIFT::create( );
	std::vector<cv::KeyPoint> keypoints_original, keypoints_database;
    	detector->detectAndCompute( croppedImage, Mat(), keypoints_original, img_descriptors );
  	detector->detectAndCompute( img_2, Mat(), keypoints_database, data_descriptors );
	
	vector<DMatch> good_matches = getGoodMatches(data_descriptors,img_descriptors);

	Mat img_matches;
  	drawMatches(img_2 ,keypoints_database , croppedImage, keypoints_original, good_matches, img_matches );
 
  	imshow("Matches", img_matches );

	std::cout<<(good_matches.size()*100/keypoints_database.size())<<std::endl;

	//cv::Mat output;
    	//cv::drawKeypoints(img_gray, keypoints, output);
	//cv::imshow("OUTPUT",output);
        //std::string semaphore_data("false");

        /*if (semaphore_data != "false")
        {
            /// Publish Semaphore Info
            std_msgs::String msg;
            msg.data.clear();
            msg.data = semaphore_data;
            ros::Rate loop_rate(200);
            semaphore_pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();

        }*/

        cv::imshow("rgb",img_rgb);
	cv::imshow("crop",croppedImage);

        uint8_t k = cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "conde_semaphore_node");
    ros::NodeHandle nh("~");
    cv::namedWindow("rgb");

    std::string camera;

    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/conde_camera_signalling_panel/image_raw", 1, imageCallback);

    //semaphore_pub = nh.advertise<std_msgs::String>("/conde_semaphore_info", 1);

    ros::spin();
}
