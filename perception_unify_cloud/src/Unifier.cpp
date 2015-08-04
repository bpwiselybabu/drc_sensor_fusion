/**
 ********************************************************************************************************
 * @file    Unifier.cpp
 * @brief   Unifier.cpp class is 
 * @details Used to ...
 ********************************************************************************************************
 */

#include <perception_unify_cloud/Unifier.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>


#define HFOV 35*M_PI/180
#define VFOV 25*M_PI/180
#define SIGMA 0.1		//2cm
#define WORKSPACE_RADIUS 1*1	//3m
namespace drc_perception {

Unifier::Unifier() : scloud_(new StereoPointCloud),
					 lcloud_(new LaserPointCloud),
					 viewer_(new pcl::visualization::PCLVisualizer ("3D Viewer")),
					 it_(nh_),
					 map_(0.1)
{

	debug_publisher_ = nh_.advertise<pcl::PCLPointCloud2> ("/debug/cloud", 1);
	//cost_subsciber_ = it_.subscribe(std::string("/multisense/left/cost"), 1, &Unifier::costImageCb,this);

	viewer_->setBackgroundColor (0, 0, 0);
	//viewer_->addCoordinateSystem()
	viewer_->setCameraPosition(0,0,0,0,0,1,0,-1,0);

}
void Unifier::setImageSource(std::shared_ptr<MultisenseImage> &ptr)
{
	image_source_=ptr;
}
void Unifier::setCloudSource(std::shared_ptr<MultisensePointCloud> &ptr)
{
	cloud_source_=ptr;
}

void Unifier::costImageCb(const sensor_msgs::ImageConstPtr &img)
{
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		int source_type = cv_bridge::getCvType(img->encoding);
		if(source_type==CV_8UC1)
		{
			cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
		}
		cost_img_=cv_ptr->image;
		cost_header_=img->header;

		ROS_INFO_ONCE("Received new cost image size: %d x %d",cost_img_.rows,cost_img_.cols);
	}
	catch (cv_bridge::Exception& e)
	{
		 ROS_ERROR_STREAM("Exception: " << e.what());
	}

}
void Unifier::disparityBiLateralFilter()
{
	cv::Mat disp_image;
	cv::Mat image;
	cv::gpu::DisparityBilateralFilter filter;
	cv::gpu::GpuMat d_img;
	cv::gpu::GpuMat d_dimg;
	cv::gpu::GpuMat d_res;
	cv::gpu::GpuMat d_show;
	if(image_source_->giveSyncImages(image,disp_image))
	{
		d_img.upload(image);

		cv::Mat disdata_16s;
		disp_image.convertTo(disdata_16s,CV_16S);
		d_dimg.upload(disdata_16s);
		filter(d_dimg,d_img,d_res);
		cv::gpu::drawColorDisp(d_res,d_show,32);
		cv::Mat h_show;
		d_show.download(h_show);
		cv::imshow("with bilateral",h_show);
		cv::Mat *color;
		ImageHelper::colorDisparity(disp_image,color,16.0f);
		cv::imshow("without bilateral",*color);
	}

	cv::waitKey(10);
}
float giveScanAngle(const Eigen::Vector4f &pt_start, const Eigen::Vector4f &pt_end)
{
	Eigen::Vector4f pt1_norm=pt_start/sqrt(pt_start.x()*pt_start.x()+pt_start.y()*pt_start.y()+pt_start.z()*pt_start.z());
	Eigen::Vector4f pt2_norm=pt_end/sqrt(pt_start.x()*pt_start.x()+pt_start.y()*pt_start.y()+pt_start.z()*pt_start.z());
	return (atan2(pt2_norm.y() - pt1_norm.y(), pt2_norm.x() - pt1_norm.x()));
}

//find points that are close to/has corresponding disparity/stereo image
//Move it to laser helper later
void Unifier::getLaserFOV(const float hfov, float vfov,
						  const cv::Size img_sz,
						  const pcl::PointCloud<LaserPoint>::Ptr inp,
						  pcl::PointCloud<pcl::PointXYZ>::Ptr &subCloud)
{
	cout<<"Hfov: "<<hfov<<" "<<"VFov: "<<vfov<<endl;

	if((!inp)||(inp->empty()))
		return;

	float sangle=atan2(img_sz.height/2.0f,img_sz.width/2.0f)*180/M_PI;
	int hwidth;
	int startIdx,endIdx;
	float theta,angle;
	pcl::PointCloud<pcl::PointXYZ>::Ptr laserSubCloud (new pcl::PointCloud<pcl::PointXYZ>);

	for(uint32_t j=0;j<inp->height;j++)
	{
		//theta=(360.0f/inp->height)*j;

		Eigen::Vector4f start_pt=Eigen::Map<const Eigen::Vector4f>(reinterpret_cast<const float*>(&inp->points[j*inp->width+0]));
		Eigen::Vector4f end_pt=Eigen::Map<const Eigen::Vector4f>(reinterpret_cast<const float*>(&inp->points[j*inp->width+1080]));
		float theta=giveScanAngle(start_pt,end_pt)*180/M_PI;

		if(theta>180)
			theta=theta-180;

		if(theta==0 || theta==-180)
		{
			//std::cout<<"angle : "<<theta<<std::endl;
			hwidth=(hfov*180/(0.28*M_PI));
		}
		else if(theta<=sangle&&theta>0)
		{
			//std::cout<<"angle : "<<theta<<std::endl;
			hwidth=(hfov*180/(0.28*M_PI))/cos(theta*M_PI/180);
		}
		else if((theta>sangle)&&(theta<=90))
		{
			angle=theta-90;
			//std::cout<<"angle : "<<angle<<std::endl;
			hwidth=(vfov*180/(0.28*M_PI))/cos(angle*M_PI/180);
		}
		else if((theta>90)&&(theta<180-sangle))
		{
			angle=theta-90;
			//std::cout<<"angle : "<<angle<<std::endl;
			hwidth=(vfov*180/(0.28*M_PI))/cos(angle*M_PI/180);
		}
		else
		{
			angle=theta-180;
			hwidth=-(hfov*180/(0.28*M_PI))/cos(theta*M_PI/180);
			//getchar();
		}

		startIdx=inp->width/2-hwidth;
		endIdx=inp->width/2+hwidth;
//		std::cout<<theta<<" : ";
//		std::cout<<sangle<<" : ";
//		std::cout<<startIdx<<" : "<<endIdx<<std::endl;


		auto L2norm=[](pcl::PointXYZI spt)
					{
					    float dx=spt.x;
						float dy=spt.y;
						float dz=spt.z;
						return sqrt(dx*dx+dy*dy+dz*dz);
					};
		for(int i=startIdx;i<endIdx;i++)
		{

			if(L2norm(inp->at(i,j))<5.0f)
				laserSubCloud->points.push_back(pcl::PointXYZ(inp->at(i,j).x,inp->at(i,j).y,inp->at(i,j).z));
		}


	}

	std::cout<<"Sub cloud size: "<<laserSubCloud->points.size()<<std::endl;
	subCloud=laserSubCloud;

	publishDebugCloud(laserSubCloud);

}
//find the error vector

//diffuse the error vector

//interpolate laser only points

//add to the cloud

//combine cloud
//void Unifier::superpixelInterpolation()
//{	
	//
//}

void Unifier::filterCloud(int img_cols, int img_rows, cv::Mat cmat)
{
	ros::Time start=ros::Time::now();
	ROS_INFO("Started");

	pcl::PointCloud<pcl::PointXYZI>::Ptr residuals(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfromLaser(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>(true));
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	auto checkfov=[](pcl::PointXYZ x)
				  {
					if((fabs(atan2(x.y,x.z))<VFOV)&&(fabs(atan2(x.x,x.z))<HFOV))
						return true;
					return false;
				  };

	auto L2norm=[](pcl::PointXYZ spt,StereoPoint pt)
				{
					float dx=spt.x-pt.x;
					float dy=spt.y-pt.y;
					float dz=spt.z-pt.z;
					return sqrt(dx*dx+dy*dy+dz*dz);
				};



	float hfov=img_cols/(2*cmat.at<float>(0,2));
	float vfov=img_rows/(2*cmat.at<float>(1,2));

	int hwidth;
	pcl::PointCloud<pcl::PointXYZ>::Ptr laserSubCloud;

	if(cloud_source_->giveStereoCloud(scloud_))
	{
		if(scloud_->empty())
			std::cout<<"you cheated me"<<std::endl;

		if(cloud_source_->giveLaserCloud(lcloud_))
		{
			residuals->width=scloud_->width;
			residuals->height=scloud_->height;
			residuals->resize(scloud_->size());

			getLaserFOV(hfov,vfov,
						cv::Size(img_cols,img_rows),
						lcloud_,
						laserSubCloud);

			if(laserSubCloud->empty())
				std::cout<<"Dont cheat me again"<<std::endl;

//			pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
//			pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//
//			mls.setComputeNormals (true);
//
//			// Set parameters
////			mls.setInputCloud (laserSubCloud);
////			mls.setPolynomialFit (true);
////			mls.setSearchMethod (tree);
////			mls.setSearchRadius (0.03);
//		  // Reconstruct
//            mls.process (*mls_points);


			tree->setInputCloud(scloud_);
//
			for(size_t j=0;j<laserSubCloud->points.size();j++)
			{
				LaserPoint pt=lcloud_->at(j);
				octomap::point3d endpoint (pt.x,pt.y,pt.z);
				map_.updateNode(endpoint, true); // integrate 'occupied' measurement

				pcl::PointXYZ spt(pt.x,pt.y,pt.z);
				if(checkfov(spt))
				{
					//std::cout<<" Pt new "<<std::endl;
					std::vector<int> pointIdxNKNSearch;
					std::vector<float> pointNKNSquaredDistance;
					if(tree->radiusSearch(spt,.01,pointIdxNKNSearch,pointNKNSquaredDistance))
					{
						float error=0;
						//for(auto idx:pointIdxNKNSearch)
								error+=pointNKNSquaredDistance[0];

						//error/=pointNKNSquaredDistance.size();

						int v_idx=0;
						for(auto idx:pointIdxNKNSearch)
						{
							float deltaX=scloud_->points[idx].x-spt.x;
							float deltaY=scloud_->points[idx].y-spt.y;
							float deltaZ=scloud_->points[idx].z-spt.z;
							residuals->points[idx].x+=error*(exp(-deltaX*deltaX/(2*SIGMA*SIGMA))/(SIGMA*sqrt(2*M_PI)));
							residuals->points[idx].y+=error*(exp(-deltaY*deltaY/(2*SIGMA*SIGMA))/(SIGMA*sqrt(2*M_PI)));
							residuals->points[idx].z+=error*(exp(-deltaZ*deltaZ/(2*SIGMA*SIGMA))/(SIGMA*sqrt(2*M_PI)));
							residuals->points[idx].intensity+=1;
							v_idx++;
						}
					}
					else
					{
						std::vector<int> pointIdxNKNSearch2;
						std::vector<float> pointNKNSquaredDistance2;
						if(tree->nearestKSearch(spt,1,pointIdxNKNSearch2,pointNKNSquaredDistance2)>0)
						{
							//std::cout<<pointNKNSquaredDistance2[0]<<" ";
							float distance=fabs(pointNKNSquaredDistance2[0]);
							distance=sqrt(distance);
							if((distance>0.01))	//20cm
							{
								cloudfromLaser->push_back(spt);
							}
						}
					}

				}
			}
			//perform adjustment
			for(size_t i=0;i<scloud_->points.size();i++)
			{
				pcl::PointXYZI res=residuals->points[i];
				if(res.intensity>0)
				{
					scloud_->points[i].x+=res.x/res.intensity;
					scloud_->points[i].y+=res.y/res.intensity;
					scloud_->points[i].z+=res.z/res.intensity;
				}
			}

			//*scloud_=*scloud_+*cloudfromLaser;
			viewer_->removeAllPointClouds(0);
			viewer_->addPointCloud<pcl::PointXYZ>(scloud_,std::string("test"),0);
			viewer_->spinOnce();
//			convert2Disparity();
			//publishDebugCloud(scloud_);

		}
	}

	ros::Time end=ros::Time::now();
	ros::Duration elapsed=end-start;
	ROS_INFO_STREAM("Time elapsed: "<<elapsed);
//	char ch='k';
//	while(ch!='q')
//	{
//
//		cin>>ch;
//	}


}
void Unifier::convert2Disparity()
{

	cv::Mat image, disp_image, camera;
	cv::gpu::DisparityBilateralFilter filter;//(64,3,10);
	if(!image_source_->giveSyncImages(image,disp_image))
	{

		return;
	}

	std::cout<<"entered"<<std::endl;
	cv::Mat *rawcolor;
	ImageHelper::colorDisparity(disp_image,rawcolor,16.0f);
	cv::imshow("without bilateral",*rawcolor);
	//d=bf/z
	//u=f*x/z
	//v=f*y/z
	if(!image_source_->giveCameraInfo(camera))
		return;
	float fx=camera.at<float>(0,0);
	float fy=camera.at<float>(1,1);
	float baselength=0.0700324326754;
	cv::Mat mydisp(disp_image.rows,disp_image.cols,CV_32F,0.0f);
	for(size_t i=0;i<scloud_->points.size();i++)
	{
		int u=fx*scloud_->points[i].x/scloud_->points[i].z+camera.at<float>(0,2);
		int v=fy*scloud_->points[i].y/scloud_->points[i].z+camera.at<float>(1,2);

		if((u>0)&&(u<disp_image.cols)&&(v>0)&&(v<disp_image.rows))
		{
			mydisp.at<float>(v,u)=baselength*(fx)/(2*scloud_->points[i].z);
		}
	}

	cv::Mat *color;
	cv::Mat filter_disp;
	filter_disp=mydisp;
	//cv::medianBlur(mydisp,filter_disp,5);
	//cv::bilateralFilter(mydisp, filter_disp,5, 10,3/2 );
	ImageHelper::colorDisparity(filter_disp,color,16.0f);
	cv::imshow("my disp",*color);

	cv::gpu::GpuMat d_img,d_dimg,d_res,d_show;
	d_img.upload(image);
	cv::Mat disdata_16s;
	filter_disp.convertTo(disdata_16s,CV_16S);
	d_dimg.upload(disdata_16s);
	filter(d_dimg,d_img,d_res);
	cv::gpu::drawColorDisp(d_res,d_show,32);
	cv::Mat h_show;
	d_show.download(h_show);
	cv::imshow("with bilateral",h_show);
	cv::waitKey(20);
}

void Unifier::publishDebugCloud(StereoPointCloud::Ptr cloud)
{
	pcl::PCLPointCloud2 output;
	pcl::toPCLPointCloud2(*cloud,output);
	output.header.frame_id=std::string("left_camera_optical_frame");
	output.header.stamp=ros::Time::now().toNSec()/10e3;
	debug_publisher_.publish(output);
}

Unifier::~Unifier() {
	// TODO Auto-generated destructor stub
}

} /* namespace drc_perception */
