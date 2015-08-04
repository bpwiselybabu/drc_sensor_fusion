#include <ros/ros.h>
#include <perception_unify_cloud/Unifier.h>
#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>


using namespace drc_perception;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"unify_filter");
	ros::NodeHandle nh;
	std::shared_ptr<MultisenseImage> mi=std::make_shared<MultisenseImage>(nh);
	std::shared_ptr<MultisensePointCloud> mc=std::make_shared<MultisensePointCloud>(nh);
	Unifier uc;

	uc.setCloudSource(mc);
	uc.setImageSource(mi);
	cv::Mat image,cmat;
	while(!mi->giveImage(image))
	if(!ros::ok())
		break;
	else
		ros::spinOnce();

	while(!mi->giveCameraInfo(cmat))
	if(!ros::ok())
		break;
	else
		ros::spinOnce();


	while(ros::ok())
	{
		uc.filterCloud(image.cols,image.rows,cmat);
		ros::spinOnce();
	}
    ros::spin();
    return 0;
}
