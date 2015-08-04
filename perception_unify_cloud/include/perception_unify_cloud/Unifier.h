/**
 ********************************************************************************************************
 * @file    Unifier.h
 * @brief   Unifier.h class is 
 * @details Used to ...
 ********************************************************************************************************
 */

#ifndef UNIFIER_H_
#define UNIFIER_H_

#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/ImageHelper.h>
#include <opencv2/gpu/gpu.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>

namespace drc_perception {

/**
 * @brief this function is used to unify the laser with the stereo cloud
 */
class Unifier {

	std::shared_ptr<MultisenseImage> 			image_source_;
	std::shared_ptr<MultisensePointCloud>		cloud_source_;

	StereoPointCloud::Ptr 						scloud_;
	LaserPointCloud::Ptr 						lcloud_;

	image_transport::Subscriber 				cost_subsciber_;
	ros::NodeHandle 							nh_;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
	image_transport::ImageTransport 			it_;

	octomap::OcTree								map_;

	ros::Publisher 								debug_publisher_;

	cv::Mat										cost_img_;
	std_msgs::Header							cost_header_;
	/**
	 * @brief the callback function to get the cost map of the stereo rectification
	 * @param img the image which is the cost of stereo rectification
	 */
	void costImageCb(const sensor_msgs::ImageConstPtr &img);

public:
	Unifier();
	/**
	 * @brief this function sets the image source for the point cloud
	 * @param ptr the image source class
	 */
	void setImageSource(std::shared_ptr<MultisenseImage> &ptr);
	/**
	 * @brief this is the function to set the point cloud source
	 * @param ptr the point cloud source class
	 */
	void setCloudSource(std::shared_ptr<MultisensePointCloud> &ptr);
	/**
	 * @brief this function applies the bilateral filter to the disparity image
	 */
	void disparityBiLateralFilter();
	/**
	 * @brief this function filters the point cloud
	 * @param img_cols the width of the image
	 * @param img_rows the height of the image
	 * @param cmat the instrinsic matrix of the camera
	 */
	void filterCloud(int img_cols, int img_rows, cv::Mat cmat);
	/**
	 * @brief this function converts the cloud into disparity image
	 */
	void convert2Disparity();

	/**
	 * @brief this function is used to get the laser cloud that corresponds to a given FOV
	 * TODO to be moved to CloudHelper!!!!!!!!!!!!!!!!!!!!!!!!
	 * @param hfov the horizontal field of view
	 * @param vfov the vertical field of view
	 * @param img_sz the image size
	 * @param inp    the input laser from which the data is to be segmenented
	 * @param laserSubCloud the laser cloud that comprises onlt the given field of view
	 */
	void getLaserFOV(const float hfov, float vfov,
			  	  	 const cv::Size img_sz,
			  	  	 const pcl::PointCloud<LaserPoint>::Ptr inp,
			  	  	 pcl::PointCloud<pcl::PointXYZ>::Ptr &laserSubCloud);

	/**
	 * @brief this function is used to publish the debug cloud so can be viewed in rviz
	 * @param cloud
	 */
	void publishDebugCloud(StereoPointCloud::Ptr cloud);

	virtual ~Unifier();
};

} /* namespace drc_perception */

#endif /* UNIFIER_H_ */
