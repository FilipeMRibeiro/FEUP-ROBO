#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

ros::Publisher dist_angle_pub;
ros::Publisher publisher;
bool stop;

void crossWalkCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->data);
}

//////////////////////////////////////////////////////////////////////////////////////// 

void semaphoreCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());

  if (msg->data == "stop")
    stop = true;
  else
    stop = false;
}

void trackingCallback(const geometry_msgs::Twist msg)
{
  geometry_msgs::Twist new_msg;
  
  if (stop) {
    new_msg.linear.x = 0;
    new_msg.angular.z = 0;
  } else {
    new_msg.linear.x = msg.linear.x;
    new_msg.angular.z = msg.angular.z;
  }
  publisher.publish(new_msg);
}

//////////////////////////////////////////////////////////////////////////////////////// 

void velocityCallback(const std_msgs::Float64MultiArray::ConstPtr& array)    {
    std::cout << array->data[0] << std::endl;       // Distance
    std::cout << array->data[1] << std::endl;       // Angle

    double distance = array->data[0];
    double angle = array->data[1];

    /// Publish Distance and Angle Info
    std_msgs::Float64MultiArray array_out;
    array_out.data.clear();
    array_out.data.push_back(distance);         //distanceIPM
    array_out.data.push_back(angle);            //angleIPM
    array_out.data.push_back(0.5);              //velocity
    array_out.data.push_back(0.375);            //reference distance
    array_out.data.push_back(0);                //reference angle
    ros::Rate loop_rate(200);
    dist_angle_pub.publish(array_out);
    ros::spinOnce();
    loop_rate.sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "conde_decision");

  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("/crossWalk", 5, crossWalkCallback);
  ros::Subscriber sub2 = n.subscribe("/conde_dist_angle", 100, velocityCallback);

//////////////////////////////////////////////////////////////////////////////////////// 
  
  ros::Subscriber sub3 = n.subscribe("/conde_semaphore_info", 100, semaphoreCallback);
  ros::Subscriber sub4 = n.subscribe("/conde_tracking_info", 100, trackingCallback);

  publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 2000);

////////////////////////////////////////////////////////////////////////////////////////

  dist_angle_pub = n.advertise<std_msgs::Float64MultiArray>("/conde_msg", 1);

  ros::spin();

  return 0;
}
