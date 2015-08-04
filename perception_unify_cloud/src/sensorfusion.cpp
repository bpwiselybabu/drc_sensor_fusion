/*
 * sensorfusion.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: bpwiselybabu
 */

#include <ros/ros.h>
#include <perception_unify_cloud/SensorFusion.h>


using namespace drc_perception;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"cloud_unifier");
	SensorFusion sensor;

	//sensor.startSensorFusion(atof(argv[1]),atof(argv[2]));
	sensor.startSensorFusion(3,0.05);

    return 0;
}


