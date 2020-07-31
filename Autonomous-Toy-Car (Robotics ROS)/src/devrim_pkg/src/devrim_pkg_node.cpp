#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

ros::Publisher scan_pub;
std_msgs::Float32MultiArray arr;

void poseMessageReceived(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	float firstAverage = 0;
	float secondAverage = 0;
	float thirdAverage = 0;
	float fourthAverage = 0;
	float fifthAverage = 0;
  
	//add first 20 value that corresponds to 0-10 degrees
	for(int i = 160; i < 180; i++){
		firstAverage += scan->ranges[i];
	}
	firstAverage = firstAverage / 20.0;

	//add second 20 value that corresponds to 40-50 degrees
	for(int i = 80; i < 100; i++){
		secondAverage += scan->ranges[i];
	}
	secondAverage = secondAverage / 20.0;
  
	//add third 20 value that corresponds to 85-95 degrees
	for(int i = 710; i < 730; i++){
		thirdAverage += scan->ranges[i % 720];
	}
	thirdAverage = thirdAverage / 20.0;
	
	//add fourth 20 value that corresponds to 140-150 degrees
	for(int i = 620; i < 640; i++){
		fourthAverage += scan->ranges[i];
	}
	fourthAverage = fourthAverage / 20.0;	

	//add fifth 20 value that corresponds to 170-180 degrees
	for(int i = 520; i < 540; i++){
		fifthAverage += scan->ranges[i];
	}
	fifthAverage = fifthAverage / 20.0;
	
	arr.data.clear();
	
	arr.data.push_back(firstAverage);
	arr.data.push_back(secondAverage);
	arr.data.push_back(thirdAverage);
	arr.data.push_back(fourthAverage);
	arr.data.push_back(fifthAverage);
	
	scan_pub.publish(arr);
	
	/*
	  std::cout << "en sol: " << firstAverage << 
	" hafif sol " << secondAverage << 
	" orta " << thirdAverage << 
	" hafif sağ " << fourthAverage << 
	" en sağ " << fifthAverage << std::endl;
	*/
}
 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "scan_node");
   ros::NodeHandle n("~");
   ros::Subscriber sub = n.subscribe("/scan", 10, poseMessageReceived);
   
   scan_pub = n.advertise<std_msgs::Float32MultiArray>("scan_distances", 1);
   ros::Rate loop_rate(50);
   
   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
}

