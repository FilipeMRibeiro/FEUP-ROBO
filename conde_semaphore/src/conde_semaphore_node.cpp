#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <unistd.h>

#include <sys/types.h>
#include <dirent.h>

ros::Publisher semaphore_pub;

using namespace cv;
using namespace std; 
using namespace cv::xfeatures2d;

vector<string> signs; // signal image paths
vector<Mat> image_descriptors; // signal image descriptors
string path = "/home/pedro/Documents/FEUP/ROBO/conde_simulator/src/gazebo_semaphore/semaphores_pics/"; // absolute path of image dataset

// process image key points
vector<DMatch> getGoodMatches(Mat descriptors_database, Mat descriptors_scene)
{
	double max_dist = 0;
	double min_dist = 100;
	vector<DMatch> matches;
	vector<DMatch> good_matches;
	BFMatcher matcher;
  	
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

// obtain detected signal, if any
string getSign(vector<KeyPoint> keypoints_database, Mat descriptors)
{
	string sign = "undetected";	// signal name
	int max_confidence = 0;

	// for each existent signal
	for(int i=0; i<image_descriptors.size(); i++)
	{
		
		// obtain matches between obtained image and signal image
		vector<DMatch> good_matches = getGoodMatches(image_descriptors[i], descriptors);
		
		// calculate certainty
		int confidence;
		if (keypoints_database.size() != 0)
			confidence = (good_matches.size() * 100) / keypoints_database.size();
		
		// update value		
		if (confidence >= max_confidence)
		{
			max_confidence = confidence;
			sign = signs[i];
		}
	}
	
	return sign;
}

// callback function to process the upper camera images
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
		Mat img_gray;
		Mat croppedImage;

		// read and process upper camera image	
		Mat img_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
		cvtColor(img_rgb, img_gray, CV_BGR2GRAY);
		Mat ROI(img_rgb, Rect(90, 0, 230, 150));
		ROI.copyTo(croppedImage);
			
		// detect key points
		Mat img_descriptors;
		vector<KeyPoint> keypoints_original;
		Ptr<SIFT> detector = SIFT::create();
		detector->detectAndCompute(croppedImage, Mat(), keypoints_original, img_descriptors);
		
		// publish detected signal
		std_msgs::String detected;
        detected.data = getSign(keypoints_original, img_descriptors);
        semaphore_pub.publish(detected);

		// display the obtained images
		cv::imshow("rgb", img_rgb);
		cv::imshow("crop", croppedImage);

		uint8_t k = cv::waitKey(1);
	}
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char** argv)
{

	///////////////////////////////////////////////////////////////////////////////
	///																			///								
	///							OPENCV INITIALIZATION							///
	///																			///
	///////////////////////////////////////////////////////////////////////////////
	
	
	// open signal image dataset
	DIR *dir_ptr = opendir(path.c_str());

	// initialize vector of signal images
	struct dirent *dir;
    while(true)
    {  
        dir = readdir(dir_ptr);
        
		if(dir == NULL)
            break;

		string name = dir->d_name;

		if (name.size() > 2)
		{
			int pos = name.find(".");
			name = name.substr(0,pos); 
			signs.push_back(name);
		}
	} 

    // close signal image dataset
	closedir(dir_ptr);

	// print vector content
	for (int i = 0; i < signs.size(); i++)
		cout << signs[i] << endl;
	
	// for each of the existing signal images
	Mat resize_temp;
	Mat data_descriptors;
	vector<KeyPoint> keypoints_database;
	for (int i = 0; i < signs.size(); i++)
	{
		// open and process the signal image
		Mat database_image = imread(path + signs[i] + ".png", IMREAD_GRAYSCALE);
		resize(database_image,resize_temp,Size(),0.5,0.5);
		
		// obtain signal image key points
		Ptr<SIFT> detector = SIFT::create();
		detector->detectAndCompute(resize_temp, Mat(), keypoints_database, data_descriptors);

		image_descriptors.push_back(data_descriptors);
	}

	///////////////////////////////////////////////////////////////////////////////
	///																			///								
	///							ROS NODE INITIALIZATION							///
	///																			///
	///////////////////////////////////////////////////////////////////////////////

	// initialize ROS node
    ros::init(argc, argv, "conde_semaphore_node");
    ros::NodeHandle nh("~");
    cv::namedWindow("rgb");

    std::string camera;

	// initialize cameras
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/conde_camera_signalling_panel/image_raw", 1, imageCallback);

	// prepare topic to advertise signal
    semaphore_pub = nh.advertise<std_msgs::String>("/conde_semaphore_info", 1);

    ros::spin();
}
