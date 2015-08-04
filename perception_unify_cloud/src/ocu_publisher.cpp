/**
 ********************************************************************************************************
 * @file    ocu_publisher.cpp
 * @brief   ocu_publisher.cpp class is 
 * @details Used to ...
 ********************************************************************************************************
 */

#include <perception_common/MultisenseImage.h>
#include <perception_unify_cloud/SensorFusion.h>
#include <tf/transform_broadcaster.h>
#include <wrecs_msgs/sf_state_est.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

static bool g_have_cam_info;
static cv::Mat g_cam;
static tf::TransformBroadcaster *br;
static bool g_egress;


using namespace cv;

void cb_left_camera_info(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	g_have_cam_info=true;
	g_cam=cv::Mat::zeros(3,3,CV_32FC1);
	g_cam.at<float>(0,0)=msg->P[0];
	g_cam.at<float>(1,1)=msg->P[5];
	g_cam.at<float>(0,2)=msg->P[2];
	g_cam.at<float>(1,2)=msg->P[6];
	g_cam.at<float>(2,2)=msg->P[10];
}

void cb_tf_broadcaster(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z) );
	tf::Quaternion q(msg->transform.rotation.x,msg->transform.rotation.y,msg->transform.rotation.z,msg->transform.rotation.w);
	transform.setRotation(q);
	br->sendTransform(tf::StampedTransform(transform,ros::Time::now(), "cmu_root", "left_camera_optical_sweep_fixed"));
}

void cb_cmu_pose(const wrecs_msgs::sf_state_estConstPtr& pose)
{
	if(pose->contact_state==wrecs_msgs::sf_state_est::Egress)
		g_egress=true;
	else
		g_egress=false;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"sensor_fusion");
	ros::NodeHandle nh;
	drc_perception::SensorFusion sensor;
	g_egress=false;
	br=new tf::TransformBroadcaster;

	g_have_cam_info=false;
	drc_perception::MultisenseImage image_sub(nh);
	cv::Mat image,depth;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc;
	pcl::PCLPointCloud2 output;
	ros::Subscriber camera_model = nh.subscribe("/ocu/multisense/left/camera_info", 5, cb_left_camera_info);
	ros::Subscriber assembled_transform = nh.subscribe("/ocu/laser_assembled/transform", 5, cb_tf_broadcaster);
	ros::Subscriber contact_mode = nh.subscribe("/ocu/cmu/robot_pose", 5, cb_cmu_pose);
	ros::Publisher cloud_pub = nh.advertise<pcl::PCLPointCloud2> (std::string("/ocu/fused_color_cloud"), 1);
	image_transport::ImageTransport it(nh);
  	image_transport::Publisher pub_histeq = it.advertise("/ocu/multisense/left/hist_eq_image", 1);

	ros::Rate r (30);
	cv::Mat lab,equalizedImage;
	sensor_msgs::ImagePtr msg;
	while(ros::ok())
	{
		ros::spinOnce();
		if(!g_have_cam_info)
			continue;
		if(image_sub.giveLeftColorImage(image))
		{
			if(image_sub.giveDepthImage(depth))
			{
				sensor.convertDepth2ColorPointCloud(depth,image,g_cam,pc);
				pcl::toPCLPointCloud2(*pc,output);
				if(g_egress)
					output.header.frame_id=std::string("egress_real_robot/left_camera_optical_frame");
				else
					output.header.frame_id=std::string("left_camera_optical_frame");
				output.header.stamp=ros::Time::now().toNSec()/10e2;
				cloud_pub.publish(output);
			}
			//convert to lab
			cvtColor(image,lab,CV_RGB2Lab);
			//split into 3
			Mat channel[3], lequalized;
		   // The actual splitting.
    		split(lab, channel);
			//histeq l
			equalizeHist( channel[0], lequalized );
			lequalized.copyTo(channel[0]);
			//merge the 3 together
			merge(channel,3,lab);
			//create a new image
			cvtColor(lab,equalizedImage,CV_Lab2BGR);
			//publish
			if(!equalizedImage.empty()) 
			{
      			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", equalizedImage).toImageMsg();
      			pub_histeq.publish(msg);
    		}
			
		}
		r.sleep();
	}
}
