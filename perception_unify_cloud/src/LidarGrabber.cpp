/*
 * LidarGrabber.cpp
 *
 *  Created on: Jun 22, 2014
 *      Author: bpwiselybabu
 */

#include "lidar_grabber.h"
#include <Eigen/Core>
#include <wrecs_common/WRECS_Names.h>
#define LASER_BUF_SIZE 100
#define SCAN_TIME_MS   25


namespace drc_perception
{

LidarGrabber::LidarGrabber() {
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	std::string laser_scan;
	if(!pnh.getParam("laser_scan",laser_scan))
		laser_scan=WRECS_NAMES::MULTISENSE_LASER_CLOUD_TOPIC;
	sub_=nh.subscribe(laser_scan,10,&LidarGrabber::myCallback,this);

	ROS_INFO_STREAM("LiddarGrabber using the topic: "<<sub_.getTopic());
}
void LidarGrabber::myCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
	if(cloud_buffer_.size()>LASER_BUF_SIZE)
	{
		ROS_WARN_STREAM("Buffer Overrun on Laser Grabber!");
		cloud_buffer_.pop();
	}
	cloud_buffer_.push(cloud);
}
std::vector<sensor_msgs::PointCloud2ConstPtr> LidarGrabber::getScansForFrame(ros::Time frame_time)
{
	std::vector<sensor_msgs::PointCloud2ConstPtr> cloud_vector;

	while(!cloud_buffer_.empty())
	{
		if(cloud_buffer_.front()->header.stamp<=frame_time+ros::Duration(SCAN_TIME_MS/1000))
		{
			cloud_vector.push_back(cloud_buffer_.front());
			cloud_buffer_.pop();
		}
		else
			break;
	}

	return cloud_vector;
}
LidarGrabber::~LidarGrabber()
{
	// TODO Auto-generated destructor stub
}

}
