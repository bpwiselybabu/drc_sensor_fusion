#ifndef __EIGEN_TO_ROS_MSGS
#define __EIGEN_TO_ROS_MSGS 

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Geometry>

template<typename T> void EV3ToRV3(const Eigen::Vector3d &ev3, T &rv3)
{
	rv3.x = ev3[0];
	rv3.y = ev3[1];
	rv3.z = ev3[2];
}

template<typename T> void RV3ToEV3(const T &rv3, Eigen::Vector3d &ev3)
{
	ev3[0] = rv3.x;
	ev3[1] = rv3.y;
	ev3[2] = rv3.z;
}

inline void EQuatToRQuat(const Eigen::Quaterniond &eq, geometry_msgs::Quaternion &rq)
{
	rq.w = eq.w();	
	rq.x = eq.x();	
	rq.y = eq.y();	
	rq.z = eq.z();	
}

inline void RQuatToEQuat(const geometry_msgs::Quaternion &rq, Eigen::Quaterniond &eq)
{
	eq.w() = rq.w;	
	eq.x() = rq.x;	
	eq.y() = rq.y;	
	eq.z() = rq.z;
	eq.normalize();
}

#endif  
