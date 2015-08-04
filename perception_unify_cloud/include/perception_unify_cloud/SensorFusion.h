/*
 * SensorFusion.h
 *
 *  Created on: Jan 27, 2015
 *      Author: bpwiselybabu
 *
 *  This class implements the sensor fusion algorithm
 */

#ifndef SENSORFUSION_H_
#define SENSORFUSION_H_

#include <ros/ros.h>
#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/chrono.h>
#include "DMR/filter.h"

namespace drc_perception {

class SensorFusion {

public:
	/**
		 * @brief This function converts a disparity map into a depth map.
		 * @param disp 			- The disparity map
		 * @param cam			- The camera config matrix
		 * @param baselength	- The baselength
		 * @param depth			- The depth Image
		 */
		void convertDisp2Depth(const cv::Mat &disp,
							   const cv::Mat &cam,
							   const float &baselength,
							   cv::Mat &depth);
		/**
		 * @brief This function applies cross bilateral filter to smooth the laser point cloud.
		 * @param src			- The lab color image
		 * @param laser_depth	- The laser depth image
		 * @param intensity		- The intensity image
		 * @param intensity_new	- The filtered intensity image
		 * @param dst			- The filtered laser depth image
		 * @param sigmaR		- The settings for depth variance
		 * @param sigmaD		- The settings for spatial variance
		 */
		void crossBilateralFilter(const cv::Mat &src,
		   	   	    			  const cv::Mat &laser_depth,
		   	   	    			  const cv::Mat& intensity,
		   	   	    			  const float& sigmaR,
		   	   	    			  const float& sigmaD,
		   	   	    			  cv::Mat &intensity_new,
		   	   	    			  cv::Mat &dst);

		/**
		 * @brief this function combines the laser data with the stereo data.
		 * @param laser_src		- the laser depth data
		 * @param intensity		- the intensity image
		 * @param stereo_depth	- the stereo depth image
		 * @param filtered_depth - the fused output depth image
		 */
		void fuseStereoDepth(const cv::Mat &laser_src,
							 const cv::Mat &intensity,
							 const cv::Mat &stereo_depth,
							 cv::Mat &filtered_depth,
							 cv::Mat &weight_map);

		/**
		 * @brief this function converts a point cloud into a depth image and an intensity image
		 * @param laser			- the laser point cloud
		 * @param image			- the color image, for the limits of the FOV
		 * @param camera		- the camera parameters for projection
		 * @param depth			- the output depth image
		 * @param intensity		- the output intensity image
		 */
		void convertPointCloud2Depth(const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser,
				                     const cv::Mat& image,
				                     const cv::Mat& camera,
				                     cv::Mat &depth,
				                     cv::Mat &intensity);

		/**
		 * @brief this function converts a depth image into point cloud for display in RVIZ and other
		 * 		  pcl operations
		 * @param depth			- the depth image that is to be converted into point cloud
		 * @param intensity		- the intensity image
		 * @param cam			- the intrinsic camera parameter
		 * @param pc			- the output point cloud
		 */
		void convertDepth2PointCloud(const cv::Mat& depth,
									 const cv::Mat& intensity,
									 const cv::Mat& cam,
									 pcl::PointCloud<pcl::PointXYZI>::Ptr &pc);

		void createPreMat(const cv::Mat &cam,
										const float &width,
										const float &height,
										cv::Mat &preMat);

		void convertDepth2PointCloud_optimized(const cv::Mat& depth,
									 const cv::Mat& intensity,
									 const cv::Mat& preMat,
									 pcl::PointCloud<pcl::PointXYZI>::Ptr &pc);


		/**
		 * @brief This function applies fast cross bilateral filter to smooth the laser point cloud.
		 * @param src			- The lab color image
		 * @param laser_depth	- The laser depth image
		 * @param intensity		- The intensity image
		 * @param intensity_new	- The filtered intensity image
		 * @param dst			- The filtered laser depth image
		 * @param sigmaR		- The settings for depth variance
		 * @param sigmaD		- The settings for spatial variance
		 */
		void fastCrossBilateralFilter(const cv::Mat &src,
	    			  	  	  	  	  const cv::Mat &laser_depth,
	    			  	  	  	  	  const cv::Mat& intensity,
	    			  	  	  	  	  const float& sigmaR,
	    			  	  	  	  	  const float& sigmaD,
	    			  	  	  	  	  cv::Mat &intensity_new,
	    			  	  	  	  	  cv::Mat &dst);
		/**
		 * @brief this function adjusts the image balance and displays it
		 * @param image 		- the image that needs to be displayed
		 * @param text			- the text for the image title
		 */
		void scaleAndDisplay(const cv::Mat &image,
				             const std::string &text);

		void projectLaser(pcl::PointCloud<pcl::PointXYZI>::Ptr & laser);

		void filterDepthMap(const cv::Mat &depth, const cv::Mat &cam, cv::Mat &filtered);
		void shadowFilter(const cv::Mat &depth, const cv::Mat &cam, cv::Mat &filtered);

	SensorFusion();
	void startSensorFusion(double sigmaS, double sigmaR);
	/**
	 * @brief this function converts the depth image into a color point cloud
	 * @param depth			- the depth image
	 * @param color			- the color image
	 * @param cam			- the camera intrinsic matrix
	 * @param pc			- the output point cloud
	 */
	void convertDepth2ColorPointCloud(const cv::Mat& depth,
									  const cv::Mat& color,
									  const cv::Mat& cam,
									  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc);

	virtual ~SensorFusion();
};

} /* namespace drc_perception */

#endif /* SENSORFUSION_H_ */

