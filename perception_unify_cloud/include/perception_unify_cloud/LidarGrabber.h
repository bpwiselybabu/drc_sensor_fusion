/*
 * LidarAssembler.h
 *
 *  Created on: Jun 22, 2014
 *      Author: bpwiselybabu
 */

#ifndef LIDARGRABBER_H_
#define LIDARGRABBER_H_

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <sensor_msgs/PointCloud2.h>
#include <queue>
//subscribe to the laser scans and assemble it
//subscribe to the laser scan using cache mechanism
namespace drc_perception
{

class LidarGrabber
{
	std::queue<sensor_msgs::PointCloud2ConstPtr> 			cloud_buffer_;    //contains all the cloud from the previous frame to the frame no
	ros::Subscriber											sub_;

	void myCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
public:
	LidarGrabber();
	std::vector<sensor_msgs::PointCloud2ConstPtr> getScansForFrame(ros::Time frame_time);
	virtual ~LidarGrabber();
};
}
#endif /* LIDARASSEMBLER_H_ */
