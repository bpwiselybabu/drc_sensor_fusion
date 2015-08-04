/**
 ********************************************************************************************************
 * @file    assemblePC_color.cpp
 * @brief   assemblePC_color.cpp class is 
 * @details Used to ...
 ********************************************************************************************************
 */


#include <iostream>
#include <visualization_msgs/Marker.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/project_inliers.h>
#include <perception_common/MultisenseImage.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/transforms.h>
#include <wrecs_common/WRECS_Names.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>
#include <perception_common/chrono.h>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
//the objective of this file is to assembled all the point clouds recieved on the ocu end and give it as
//a single cloud.


static pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_giant;
static ros::Publisher giant_pub;
static tf::TransformListener 				*listener;

void ndtTransformAndAdd(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& A,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& B)
{
	 pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;
	 // Setting scale dependent NDT parameters
	 // Setting minimum transformation difference for termination condition.
	 ndt.setTransformationEpsilon (0.01);
	 // Setting maximum step size for More-Thuente line search.
	 ndt.setStepSize (0.1);
	 //Setting Resolution of NDT grid structure (VoxelGridCovariance).
	 ndt.setResolution (1.0);

	  // Setting max number of registration iterations.
	  ndt.setMaximumIterations (35);

	  // Setting point cloud to be aligned.
	 ndt.setInputSource (B);
	  // Setting point cloud to be aligned to.
	 ndt.setInputTarget (A);

	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	 Eigen::Affine3f estimate;
	 estimate.setIdentity();
	 ndt.align (*output_cloud, estimate.matrix());

	 std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
	           << " score: " << ndt.getFitnessScore () << std::endl;

	   // Transforming unfiltered, input cloud using found transform.
	 pcl::transformPointCloud (*B, *output_cloud, ndt.getFinalTransformation ());

	 *A=*A+*output_cloud;
}

void addCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	//transform the cloud to the cmu_root
	tf::StampedTransform stamped_tf;
	Eigen::Affine3d transform;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudW(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWperipheral(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWFoveal(new pcl::PointCloud<pcl::PointXYZRGB>);

	try{
	listener->waitForTransform("cmu_root",cloud->header.frame_id,ros::Time().fromNSec(cloud->header.stamp*10e2), ros::Duration(1.0));
	listener->lookupTransform("cmu_root",cloud->header.frame_id,ros::Time().fromNSec(cloud->header.stamp*10e2), stamped_tf);
	}catch(tf::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
	}
	tf::transformTFToEigen(stamped_tf,transform);
	pcl::transformPointCloud (*cloud, *cloudW, transform);


	//if you need to assembled this addition needs to be replaced
	if(final_giant->points.size()==0)
	{
		*final_giant=*final_giant+*cloudW;
	}
	else
		ndtTransformAndAdd(final_giant,cloudW);

	pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (0.01, 0.01, 0.01);
	approximate_voxel_filter.setInputCloud (final_giant);
	approximate_voxel_filter.filter (*cloudWFiltered);

	Eigen::Vector4f robot=Eigen::Vector4f(transform.translation()[0],transform.translation()[1],0,0);
	Eigen::Vector4f minPoint(-4,-2,-2.5,1);
	Eigen::Vector4f maxPoint(4,2,3,1);
	minPoint+=robot;
	maxPoint+=robot;
	pcl::CropBox<pcl::PointXYZRGB> cropFilter;
	cropFilter.setInputCloud (cloudWFiltered);
	cropFilter.setMin(minPoint);
	cropFilter.setMax(maxPoint);
	cropFilter.setTransform(transform.cast<float>());
	cropFilter.setNegative(false);
	cropFilter.filter (*cloudWFoveal);
	cropFilter.setNegative(true);
	cropFilter.filter (*cloudWperipheral);

	approximate_voxel_filter.setLeafSize (0.1, 0.1, 0.1);
	approximate_voxel_filter.setInputCloud (cloudWperipheral);
	approximate_voxel_filter.filter (*cloudWFiltered);
	*final_giant=*cloudWFiltered+*cloudWFoveal;

	pcl::PCLPointCloud2 output;
	pcl::toPCLPointCloud2(*final_giant,output);
	output.header.frame_id="map";
	output.header.stamp=cloud->header.stamp;
	giant_pub.publish(output);

}

void colorCloudSub(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *laser_cloud);
	addCloud(laser_cloud);
}

void laserCloudSub(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *laser_cloud);
    copyPointCloud(*laser_cloud, *color_cloud);
	addCloud(color_cloud);
}



void clearCb()
{
	final_giant.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

int main(int argc, char**argv)
{
	ros::init(argc,argv,"ocu_total_assembler");
	ros::NodeHandle nh;

	listener=new tf::TransformListener(nh);

	//subscrible to the laser cloud
	ros::Subscriber laserSub=nh.subscribe<sensor_msgs::PointCloud2>("/ocu/laser/assembled", 1, &laserCloudSub);
	//subscribe to the fused point cloud
	ros::Subscriber fuseSub=nh.subscribe<sensor_msgs::PointCloud2>("/ocu/fused_color_cloud", 1, &colorCloudSub);
	final_giant=pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	giant_pub=nh.advertise<pcl::PCLPointCloud2> (std::string("/ocu/the_giant_cloud"), 1);

	ros::spin();
}


