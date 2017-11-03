#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <limits>
#include <sensor_msgs/LaserScan.h>
#include <string>

using namespace std;

// handle conversion between radians and degrees
#define PI 3.14159265358979323846
float toDegrees(float angle) { return angle * (180 / PI); }
float toRadians(float angle) { return angle * (PI / 180); }

ros::Publisher publisher;

// callback function to process arriving laser readings
void laserReadingReceived(const sensor_msgs::LaserScan& msg)
{
    geometry_msgs::Twist response;
		
    float distanceRightSide = numeric_limits<float>::max(); 
    float distanceLeftSide = numeric_limits<float>::max();
    float robotAngle = 0.0;
    	
	// for each of the laser rays...
    for(int i = 0; i < msg.ranges.size(); i++)
    {
		float distance = msg.ranges[i];
		float rayAngle = -100 + toDegrees(msg.angle_increment) * i;
	
		// obtain minimum distance from robot's right side
		if (rayAngle >= -100 && rayAngle <= 0) 
		{
			if(distance < distanceRightSide)
			{
				distanceRightSide = distance;
				robotAngle = rayAngle;
			}
		}

		// obtain minimum distance from robot's left side
		else if (rayAngle > 0 && rayAngle <= 100) 
		{
			if(distance < distanceLeftSide)
			{
				distanceLeftSide = distance;
				robotAngle = rayAngle;
			}	
		} 
	}
		
	float minWallDistance = min(distanceRightSide, distanceLeftSide);
	float alpha = 90.0 - abs(robotAngle);
		
	// check if wall is within the sensor range
	if(minWallDistance <= msg.range_max)
	{
		// follow the wall
		response.linear.x = 0.4;	
		response.angular.z = (-20 * (sin(toRadians(alpha)) - (minWallDistance - 1.46))) * response.linear.x;	
	}
	else 
	{
		// go forward
		response.linear.x = 0.4;	
		response.angular.z = 0.0;
	}
			
	publisher.publish(response);
}

// main function
int main(int argc, char** argv)
{

	// verify if number of arguments passed is valid
	if(argc != 3)
	{
		ROS_ERROR_STREAM("Usage : reactive_robot <STDR_robot_id> <STDR_laser_frame_id>");
		exit(-1);
	}

	// initialize the ROS system and become a node
	ros::init(argc, argv, "reactive_robot");
	ros::NodeHandle nh;

	// subscribes to topic '/<STDR_robot_id>/<STDR_laser_frame_id>' 
	// to obtain laser readings
	string subTopic = "/"+ string(argv[1]) + "/" + string(argv[2]); 
  	ros::Subscriber subscriber = nh.subscribe(subTopic, 1, &laserReadingReceived);
	
	// advertise to topic '/<STDR_robot_id>/cmd_vel' 
	// in order to change robot orientation and velocity
	string pubTopic = string("/") + string(argv[1]) + string("/cmd_vel");
	publisher = nh.advertise<geometry_msgs::Twist>(pubTopic, 1);
	
	// let ROS take over
	ros::spin();
	
	return 0;
}
