/*
 * SensorFusion.cpp
 *
 *  Created on: Jan 27, 2015
 *      Author: bpwiselybabu
 */

#include "perception_unify_cloud/SensorFusion.h"

#include <sensor_msgs/image_encodings.h>

//#define DEBUG_MODE

namespace drc_perception {

SensorFusion::SensorFusion() {
	// TODO Auto-generated constructor stub

}
void SensorFusion::convertDisp2Depth(const cv::Mat &disp,
									 const cv::Mat &cam,
									 const float &baselength,
									 cv::Mat &depth)
{
	const int rows=disp.rows;
	const int cols=disp.cols;
	depth=cv::Mat::zeros(rows,cols,CV_32FC1);

	float *disp_data=(float*)disp.data;
	float *depth_data=(float*)depth.data;

	int index;
	float fx=cam.at<float>(0,0);

	//std::cout<<"fx: "<<fx<<" baselength: "<<baselength<<std::endl;
	for(int i=0;i<rows;++i)
	{
		for(int j=0;j<cols;++j)
		{
			index=disp.step/sizeof(float)*i+j;
			if(disp_data[index]>2.0f && !isnan(disp_data[index]) && disp_data[index]<(fx*baselength/0.05f))// 1pix disp=39.7m 795pix disp = 0.05m
				depth_data[index]=fx*baselength/disp_data[index];
			else
				depth_data[index]=NAN;

		}
	}
}

void SensorFusion::crossBilateralFilter(const cv::Mat &src,
						   	   	   	    const cv::Mat &laser_depth,
						   	   	   	    const cv::Mat& intensity,
						   	   	   	    const float& sigmaR,
						   	   	   	    const float& sigmaD,
						   	   	   	    cv::Mat &intensity_new,
						   	   	   	    cv::Mat &dst)
{
	const int ncols=src.cols;
	const int nrows=src.rows;

	/*Result initialization*/
	dst=cv::Mat::zeros(nrows,ncols,CV_32FC1);
	intensity_new=cv::Mat::zeros(nrows,ncols,CV_32FC1);

	/*The pointers to all the images*/
	unsigned char *input = (unsigned char*)(src.data);
	float *output = (float*)(dst.data);
	float *intens_new=(float*)(intensity_new.data);
	float *dpt = (float*)(laser_depth.data);
	float *intens =(float*)(intensity.data);

	// iterators
	int i,j,ii,jj;


	unsigned char 	center_px=0,window_px=0;
	int 			index_w, index_d;
	float 			w,new_px,total_w,depth_px,intensity_px, new_intensity;
	float 			Sdist2,Rdist2,Gs,Gr,Rint,Gi,center_intensity;
	cv::Vec3i 		center_color_px,window_color_px;

	for(i=sigmaR;i<nrows-sigmaR;++i)
	{
		for(j=sigmaR;j<ncols-sigmaR;++j)
		{

			if(src.channels()==1)
				center_px= input[src.step * i + j ] ;
			else
			{
				center_color_px.val[0]=input[src.step * i + j*src.channels()+0 ];
				center_color_px.val[1]=input[src.step * i + j*src.channels()+1 ];
				center_color_px.val[2]=input[src.step * i + j*src.channels()+2 ];
			}
			center_intensity = intens[(laser_depth.step/sizeof(float))*i+j];
			//inner loop
			w=0;
			new_px=0;
			total_w=0;
			Sdist2=0;
			Rdist2=0;
			Rint=0;
			new_intensity=0;
			//trying box filter first
			for(ii=-sigmaR;ii<sigmaR;ii++)
			{
				for(jj=-sigmaR;jj<sigmaR;jj++)
				{

					if(src.channels()==1)
						index_w=(i+ii)*src.step+j+jj;
					else
						index_w=(i+ii)*src.step+(j+jj)*src.channels();

					index_d=(laser_depth.step/sizeof(float))*(i+ii)+jj+j;

					if(src.channels()==1)
						window_px=input[index_w];
					else
					{
						window_color_px.val[0]=input[index_w+0];
						window_color_px.val[1]=input[index_w+1];
						window_color_px.val[2]=input[index_w+2];
					}
					depth_px=dpt[index_d];
					intensity_px=intens[index_d];

					//spatial dist
					Sdist2=ii*ii+jj*jj;
					Gs=exp(-Sdist2/(2*sigmaR*sigmaR));
					//range distance
					if((src.channels()==1))
					{
						Rdist2=fabs(window_px-center_px);
					}
					else
					{
//						Rdist2=((window_color_px[0]-center_color_px[0])*(window_color_px[0]-center_color_px[0])+
//						 	 (window_color_px[1]-center_color_px[1])*(window_color_px[1]-center_color_px[1])+
//						 	 (window_color_px[2]-center_color_px[2])*(window_color_px[2]-center_color_px[2]))/9;
//
						Rdist2=(fabs(window_color_px[0]-center_color_px[0])+
								fabs(window_color_px[1]-center_color_px[1])+
								fabs(window_color_px[2]-center_color_px[2]))/3;
					}
					Gr=exp(-Rdist2/(2*sigmaD*sigmaD));

//					if(intensity_px!=0)
//					{
//						Rint=intensity_px-center_intensity;
//						Gi=exp(-fabs(Rint)/(2*0.5*0.5));
//					}
//					else
//						Gi=1.0;

					w=Gs*Gr;

					if(depth_px!=0)
						total_w+=w;

					new_px+=depth_px*w;
					new_intensity+=intensity_px*w;
				}
			}
			if(total_w!=0.0f)
			{
				output[dst.step/sizeof(float) * i + j ]=new_px/total_w;
				intens_new[intensity_new.step/sizeof(float)*i+j]=new_intensity/total_w;
			}
			//std::cout<<output[dst.step/sizeof(float) * i + j ]<<" ";
		}
		//std::cout<<std::endl;
	}
}

void SensorFusion::fuseStereoDepth(const cv::Mat &laser_src,
				     	 	 	   const cv::Mat &intensity,
				     	 	 	   const cv::Mat &stereo_depth,
				     	 	 	   cv::Mat &filtered_depth,
				     	 	 	   cv::Mat &weight_map)
{
	int cols=laser_src.cols;
	int rows=laser_src.rows;

	cv::Mat weight;
	filtered_depth=cv::Mat::zeros(rows,cols,CV_32F);
	weight=cv::Mat::zeros(rows,cols,CV_32F);

	float *laser_data=(float*)laser_src.data;
	float *stereo_data=(float*) stereo_depth.data;
	float *filtered_data=(float*) filtered_depth.data;
	float *intensity_data=(float*) intensity.data;
	float *weight_data=(float*) weight.data;
	uchar *old_weight_data=(uchar*) weight_map.data;

	int index;
	float stereo_weight;
	float laser_weight;
	int win=3;
	int tw;
	for(int i=0;i<rows;++i)
	{
		for(int j=0;j<cols;++j)
		{
			index=laser_src.step/sizeof(float)*i+j;
			if(laser_data[index]>=0.1 && !isnan(laser_data[index]))
			{
				filtered_data[index]=laser_data[index];
				weight_data[index]=100.0f;
			}
			else if(!isnan(stereo_data[index])&& stereo_data[index]>=0.1)
			{
#ifdef GAZEBO_SIMULATION
				weight_data[index]=0.7f;
#else
				weight_data[index]=(255.0f-old_weight_data[index])/255.0f;
#endif
				filtered_data[index]=stereo_data[index];
			}
			else
			{
				filtered_data[index]=0.0f;
				weight_data[index]=0.0;
			}

		}
	}
	//imshow("weight_map",weight_map);
	weight_map=weight;
	//imshow("weight",weight_map);
}


void SensorFusion::convertPointCloud2Depth(const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser,
		                     const cv::Mat& image,
		                     const cv::Mat& camera,
		                     cv::Mat &depth,
		                     cv::Mat &intensity)
{
	const int rows=image.rows;
	const int cols=image.cols;

	depth=cv::Mat::zeros(rows,cols,CV_32FC1);
	intensity=cv::Mat::zeros(rows,cols,CV_32FC1);

	std::vector<cv::Point3f> pts;
	pts.resize(laser->points.size());

	for(size_t i=0;i<laser->points.size();i++)
	{
		pts[i]=cv::Point3f(laser->points[i].x,laser->points[i].y,laser->points[i].z);
	}

	cv::Mat uv(laser->points.size(),2,CV_32F);

	cv::Mat rvec=cv::Mat::zeros(1,3,CV_32F);
	cv::Mat tvec=cv::Mat::zeros(1,3,CV_32F);

	cv::Mat dist_coeff=(cv::Mat_<float>(1,5)<<0,0,0,0,0);
	cv::projectPoints(cv::Mat(pts),rvec,tvec, camera, dist_coeff, uv);

	float *output = (float*)(depth.data);
	float *output_intensity = (float*)(intensity.data);
	int x,y;
	int index;
	for(int i=0;i<uv.rows;i++)
	{
		x=floor(uv.at<float>(i,0));
		y=floor(uv.at<float>(i,1));

		if((x>=0)&&(x<image.cols))
		{
			if((y>=0)&&(y<image.rows))
			{
				index=depth.step/sizeof(float) * y + x;
				if(laser->points[i].z>0.1f)
				{
					if(output[index]==0.0f)
					{
						output[index]=(float)laser->points[i].z;
						output_intensity[index]=laser->points[i].intensity;
					}
					else
					{
						if(output[index]>laser->points[i].z)
						{
							output[index]=laser->points[i].z;
							output_intensity[index]=laser->points[i].intensity;
						}
					}
				}
			}
		}
	}
#ifdef DEBUG_MODE
	imshow("laser image", depth);
#endif
}



void SensorFusion::convertDepth2PointCloud(const cv::Mat& depth,
							 const cv::Mat& intensity,
							 const cv::Mat& cam,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr &pc)
{
	float *d=(float*)depth.data;
	float *in=(float*)intensity.data;

	pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);

	const int cols=depth.cols;
	const int rows=depth.rows;
	const float fx=cam.at<float>(0,0);
	const float fy=cam.at<float>(1,1);
	const float cx=cam.at<float>(0,2);
	const float cy=cam.at<float>(1,2);

	filtered->height=rows;
	filtered->width=cols;
	filtered->points.resize(rows*cols);

	cv::Mat holes(cv::Mat::zeros(rows,cols,CV_8UC1));
	uchar *holes_data=(uchar*)holes.data;
	int index,uindex;
	int k=0;
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;++j)
		{
			index=i*depth.step/(sizeof(float))+j;
			if(isnan(d[index])||d[index]<=0.001)
			{

				filtered->points[k].x=0;
				filtered->points[k].y=0;
				filtered->points[k].z=0;
				k++;
				continue;
			}

			filtered->points[k].x=-(cx-j)*d[index]/fx;
			filtered->points[k].y=-(cy-i)*d[index]/fy;
			filtered->points[k].z=d[index];
			filtered->points[k].intensity=in[index];
#ifdef DEBUG_MODE
			uindex=i*holes.step+j;
			if(d[index]==0.0f)
				holes_data[uindex]=255;
#endif
			k++;
		}
	}
	pc=filtered;
#ifdef DEBUG_MODE
	cv::imshow("holes",holes);
#endif
}

void SensorFusion::convertDepth2PointCloud_optimized(const cv::Mat& depth,
							 const cv::Mat& intensity,
							 const cv::Mat& preMat,
							 pcl::PointCloud<pcl::PointXYZI>::Ptr &pc)
{
	float *d=(float*)depth.data;
	float *in=(float*)intensity.data;
	float *preMat_ptr=(float*)preMat.data;

	pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);

	const int cols=depth.cols;
	const int rows=depth.rows;

	filtered->height=rows;
	filtered->width=cols;
	filtered->points.resize(rows*cols);

	cv::Mat holes(cv::Mat::zeros(rows,cols,CV_8UC1));
	uchar *holes_data=(uchar*)holes.data;
	int index,uindex,i_premat;
	int k=0;
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;++j)
		{
			index=i*depth.step/(sizeof(float))+j;
			i_premat=i*preMat.step/(sizeof(float))+j*4;

			Eigen::Map<Eigen::Vector4f>(filtered->points[k].data_c)=
					Eigen::Map<Eigen::Vector4f>(&preMat_ptr[i_premat],4)*d[index];
			filtered->points[k].intensity=in[index];
#ifdef DEBUG_MODE
			uindex=i*holes.step+j;
			if(d[index]==0.0f)
				holes_data[uindex]=255;
#endif
			k++;
		}
	}
	pc=filtered;
#ifdef DEBUG_MODE
	cv::imshow("holes",holes);
#endif
}

void SensorFusion::convertDepth2ColorPointCloud(const cv::Mat& depth,
								  const cv::Mat& color,
								  const cv::Mat& cam,
								  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc)
{

	const float *d=(float*)depth.data;
	const uchar *in=(uchar*)color.data;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	const int cols=depth.cols;
	const int rows=depth.rows;
	const float fx=cam.at<float>(0,0);
	const float fy=cam.at<float>(1,1);
	const float cx=cam.at<float>(0,2);
	const float cy=cam.at<float>(1,2);

	filtered->height=rows;
	filtered->width=cols;
	filtered->points.resize(rows*cols);

#ifdef DDEBUG_MODE
	cv::Mat holes(cv::Mat::zeros(rows,cols,CV_8UC1));
	uchar *holes_data=(uchar*)holes.data;
#endif

	int index,uindex,color_index;
	int k=0;
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;++j)
		{
			index=i*depth.step/(sizeof(float))+j;
			if(isnan(d[index])||d[index]<=0.001)
			{
				filtered->points[k].x=0;
				filtered->points[k].y=0;
				filtered->points[k].z=0;
				k++;
				continue;
			}


			color_index=i*color.step+j*color.channels();
			filtered->points[k].x=-(cx-j)*d[index]/fx;
			filtered->points[k].y=-(cy-i)*d[index]/fy;
			filtered->points[k].z=d[index];
			/*
			uchar r=in[color_index+2];
			uchar g=in[color_index+1];
			uchar b=in[color_index+0];
			uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
			filtered->points[k].rgb = *reinterpret_cast<float*>(&rgb);
			*/
			filtered->points[k].r=in[color_index+2];
			filtered->points[k].g=in[color_index+1];
			filtered->points[k].b=in[color_index+0];
#ifdef DDEBUG_MODE
			uindex=i*holes.step+j;
			if(d[index]==0.0f)
				holes_data[uindex]=255;
#endif
			k++;
		}
	}
	pc=filtered;
#ifdef DDEBUG_MODE
	cv::imshow("holes",holes);
#endif
}

//projects the laser onto current time, given the transform from previous transform
void SensorFusion::projectLaser(pcl::PointCloud<pcl::PointXYZI>::Ptr & laser)
{

}


void SensorFusion::fastCrossBilateralFilter(const cv::Mat &src,
	  	  	  	  	  	  	  	  	  	  	const cv::Mat &laser_depth,
	  	  	  	  	  	  	  	  	  	  	const cv::Mat& intensity,
	  	  	  	  	  	  	  	  	  	  	const float& sigmaR,
	  	  	  	  	  	  	  	  	  	  	const float& sigmaD,
	  	  	  	  	  	  	  	  	  	  	cv::Mat &intensity_new,
	  	  	  	  	  	  	  	  	  	  	cv::Mat &dst)
{

	const int samplingSpatial=4;
	const int samplinghRange=4;

	const int height  = src.rows;
	const int width = src.cols;

	const int padding_s = 2;
	const int padding_r = 2;

    //find min max
	uchar cmax[3],cmin[3];
	cmax[0]=cmax[1]=cmax[2]=0;
	cmin[0]=cmin[1]=cmin[2]=255;
	const uchar*  lab_data=(uchar*) src.data;

	int index;

	for(int i=0;i<height;++i)
	{
		for(int j=0;j<width;++j)
		{
			index=i*src.step+j*src.channels();
			for(int k=0;k<3;++k)
			{
				cmin[k]=std::min(cmin[k],lab_data[index+k]);
				cmax[k]=std::max(cmax[k],lab_data[index+k]);
			}
		}
	}
	//create padded image
	int downsampledWidth=static_cast<int>((width-1)/sigmaR);
	int downsampledHeight=static_cast<int>((height-1)/sigmaR);

	int downsampledl=static_cast<int>((cmax[0]-cmin[0])/sigmaD);
	int downsampleda=static_cast<int>((cmax[1]-cmin[1])/sigmaD);
	int downsampledb=static_cast<int>((cmax[2]-cmin[2])/sigmaD);
	//downsampling

	int sz[] = {downsampledWidth+1+2*padding_s,
				downsampledHeight+1+2*padding_s,
				downsampledl+1+2*padding_r,
				downsampleda+1+2*padding_r,
				downsampledb+1+2*padding_r,
			   };
	cv::Mat wi(5,sz,CV_32F, cv::Scalar::all(0));
	cv::Mat w(5,sz,CV_32F, cv::Scalar::all(0));

	float* wi_data=(float*)wi.data;
	float* w_data=(float*)w.data;

	float* laser_data=(float*)laser_depth.data;

	float l,a,b,laser;
	int index_arr[5];
	for(int i=0;i<height;++i)
	{
		index_arr[0] = static_cast<int>(1.0*i / sigmaR + 0.5) + padding_s;
		for(int j=0;j<width;++j)
		{
			index_arr[1] = static_cast<int>(1.0*j / sigmaR + 0.5) + padding_s;
			laser=laser_data[i*laser_depth.step/(sizeof(float))+j];
			index=i*src.step+j*src.channels();
			for(int k=0;k<3;++k)
			{

				index_arr[2+k] = static_cast<int>((lab_data[index+k] - cmin[k]) / sigmaD + 0.5) + padding_r;
			}
			const int w_index=index_arr[0]*wi.step[0]/sizeof(float)+
							  index_arr[1]*wi.step[1]/sizeof(float)+
							  index_arr[2]*wi.step[2]/sizeof(float)+
							  index_arr[3]*wi.step[3]/sizeof(float)+
							  index_arr[4];
			wi_data[w_index]+=laser;
			w_data[w_index]+=1.0f;
		}
	}

	//split into respective arrays

	//convolution

	//nonlinearities
}
void SensorFusion::scaleAndDisplay(const cv::Mat &image,
		                           const std::string &text)
{

	double min;
	double max;
	cv::minMaxIdx(image, &min, &max);
	cv::Mat adjMap;
	// expand your range to 0..255. Similar to histEq();
	image.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);

	// this is great. It converts your grayscale image into a tone-mapped one,
	// much more pleasing for the eye
	// function is found in contrib module, so include contrib.hpp
	// and link accordingly
	cv::Mat falseColorsMap;
	applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_HSV);
	cv::imshow(text.c_str(), falseColorsMap);

	std::stringstream ss;
	ss<<text.c_str()<<".png";
	if(cv::waitKey(3)=='s')
		cv::imwrite(ss.str().c_str(),adjMap);
}
void SensorFusion::createPreMat(const cv::Mat &cam,
								const float &width,
								const float &height,
								cv::Mat &preMat)
{
	preMat=cv::Mat::zeros(height,width,CV_32FC4);
	float* preMat_ptr=(float*)preMat.data;

	int index;
	Eigen::Vector4f val(1,1,1,1);
	const float fx=cam.at<float>(0,0);
	const float fy=cam.at<float>(1,1);
	const float cx=cam.at<float>(0,2);
	const float cy=cam.at<float>(1,2);
	int noChannels=preMat.channels();

	int i,j;
	for(int i=0;i<height;++i)
	{
		val(1)=-(cy-i)/fy;
		for(int j=0;j<width;++j)
		{
			val(0)=-(cx-j)/fx;
			index=i*preMat.step/sizeof(float)+j*noChannels;
			Eigen::Map<Eigen::Vector4f>(&preMat_ptr[index],4)=val;
		}
	}
}

void SensorFusion::filterDepthMap(const cv::Mat &depth, const cv::Mat &cam, cv::Mat &filtered)
{

	filtered=cv::Mat::zeros(depth.rows,depth.cols,CV_32FC1);

	const float *d=(float*)depth.data;
	float *out=(float*)filtered.data;
	const int cols=depth.cols;
	const int rows=depth.rows;
	const float fx=cam.at<float>(0,0);
	const float fy=cam.at<float>(1,1);
	const float cx=cam.at<float>(0,2);
	const float cy=cam.at<float>(1,2);

	const int dx4[4] = {-1,  0,  1,  0};
	const int dy4[4] = { 0, -1,  0,  1};


	Mat grad_x, grad_y;

	/// Gradient X
	//Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
	Sobel( depth, grad_x, CV_32F, 1, 0, 3, 1, 0, BORDER_DEFAULT );

	/// Gradient Y
	//Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
	Sobel( depth, grad_y, CV_32F, 0, 1, 3, 1, 0, BORDER_DEFAULT );


	const float *dx=(float*)grad_x.data;
	const float *dy=(float*)grad_y.data;

	int index,oindex;
	int k=0;
	Eigen::Vector3f ptA,temp;

	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;++j)
		{
			index=i*depth.step/(sizeof(float))+j;
			oindex=index;
			if(isnan(d[index])||d[index]<=0.001)
			{
				out[oindex]=-1;
				continue;
			}

			temp[0]=-dy[index]*d[index]/fy;
			temp[1]=-dx[index]*d[index]/fx;
			temp[2]=1;
			temp.normalize();

			ptA[0]=-(cx-j)*d[index]/fx;
			ptA[1]=-(cy-i)*d[index]/fy;
			ptA[2]=d[index];

			ptA.normalize();

			//if((0.5*dx[index]+0.5*dy[index])<=0.0001)
			//{
				out[oindex]=ptA.dot(temp);//d[index];
			//}
		}
	}
	//filtered=grad_x;
}

void SensorFusion::shadowFilter(const cv::Mat &depth, const cv::Mat &cam, cv::Mat &filtered)
{
	//the angle with respect to the principle axis must be constrained
}


void SensorFusion::startSensorFusion(double sigmaS, double sigmaR)
{
	//float sigmaS=atof(argv[1]);
	//float sigmaR=atof(argv[2]);

	ros::NodeHandle nh;

	ros::Duration(2).sleep();
	MultisenseImage mi(nh);
	MultisensePointCloud mp(nh);


	LaserPointCloud::Ptr laser_cloud,filtered_cloud;
	StereoPointCloudColor::Ptr color_cloud;

	ros::Publisher laser_publisher = nh.advertise<pcl::PCLPointCloud2> (ros::this_node::getName()+std::string("/laser_cloud"), 1);
	ros::Publisher color_publisher = nh.advertise<pcl::PCLPointCloud2> (ros::this_node::getName()+std::string("/color_cloud"), 1);

	image_transport::ImageTransport it(nh);
	image_transport::Publisher depth_pub = it.advertise(ros::this_node::getName()+std::string("/depth_image"), 1);


	Chrono timer_crossbf("crossBilateralFilter");
	Chrono timer_disp2depth("convertDisp2Depth");
	Chrono timer_pc2depth("convertPointCloud2Depth");
	Chrono timer_fusedepth("fuseDepth");
	Chrono timer_depth2pc("Depth2pc");
	Chrono timer_total("TotalPeriod");

	cv::Mat image,disp,	BI_cross,BI_cross2, final_depth, cam,intensity_new,	intensity_new2;
	cv::Mat depth,intensity,stereo_depth,fused_depth,lab,labf,overseg_image,gray, depth_w;
	cv::Mat preMat;
	float baselength;
	ros::Time time;
	while(ros::ok())
	{
		ros::spinOnce();
		mi.giveCameraInfo(cam);
#ifdef GAZEBO_SIMULATION
		if(!mi.giveSyncImageswTime(image,disp,time))
		{
			ros::Duration(0.02).sleep();
			continue;
		}
#ifdef DEBUG_MODE
		cv::imshow("input",image);
#endif
		baselength=mi.giveBaseLength();

		timer_disp2depth.start();
		convertDisp2Depth(disp,cam,baselength,stereo_depth);
		timer_disp2depth.stop();
#else
		ROS_INFO_ONCE("Node running on real robot");
		if(!mi.giveSyncDepthImageswTime(image,stereo_depth,depth_w,time))
		{
			ros::Duration(0.02).sleep();
			continue;
		}
#endif
		//TODO: The camera and the laser need not be synchornized????
		if(!mp.giveLaserCloudForTime(time,laser_cloud))
		{
			ros::Duration(0.02).sleep();
			continue;
		}
		timer_pc2depth.start();
		convertPointCloud2Depth(laser_cloud,image,cam,depth,intensity);
		timer_pc2depth.stop();

		if(!image.empty())
		{
			timer_total.start();
			cvtColor(image,lab,CV_BGR2Lab);
			lab.convertTo(labf,CV_32F);

			timer_fusedepth.start();
			fuseStereoDepth(depth,intensity_new,stereo_depth,fused_depth,depth_w);
			timer_fusedepth.stop();

			timer_crossbf.start();
			weightedJointBilateralFilter(fused_depth,depth_w,labf, BI_cross2, cv::Size(5,5),2*sigmaR,sigmaS,BILATERAL_NORMAL);
			timer_crossbf.stop();

			//blurRemoveMinMaxBF(BI_cross2,final_depth,5,0);
			//ointColorDepthFillOcclusion(BI_cross2,labf,final_depth,cv::Size(5,5),10);
			//filterDepthMap(BI_cross2,cam,final_depth);

			//self filter to remove the robot
			//


			timer_depth2pc.start();
			//if(preMat.empty())
			//	createPreMat(cam,BI_cross2.rows,BI_cross2.cols,preMat);
			convertDepth2PointCloud(BI_cross2,intensity, cam,filtered_cloud);
			convertDepth2ColorPointCloud(BI_cross2,image, cam,color_cloud);
			timer_depth2pc.stop();
			timer_total.stop();





#ifdef DEBUG_MODE
			scaleAndDisplay(stereo_depth,"Filtered Depth");
#endif
//			std::cout<<timer_crossbf.report()<<std::endl;
//			std::cout<<timer_fusedepth.report()<<std::endl;
//			std::cout<<timer_disp2depth.report()<<std::endl;
//			std::cout<<timer_pc2depth.report()<<std::endl;
//			std::cout<<timer_depth2pc.report()<<std::endl;
//			std::cout<<timer_total.report()<<std::endl;

			pcl::PCLPointCloud2 output;
			if(color_cloud!=NULL)
			{
				pcl::toPCLPointCloud2(*filtered_cloud,output);
				output.header.frame_id=std::string("left_camera_optical_frame");
				output.header.stamp=time.toNSec()/10e2;//laser_cloud->header.stamp;
				laser_publisher.publish(output);

				pcl::toPCLPointCloud2(*color_cloud,output);
				output.header.frame_id=std::string("left_camera_optical_frame");
				output.header.stamp=time.toNSec()/10e2;//laser_cloud->header.stamp;
				color_publisher.publish(output);
				std_msgs::Header h;
				h.stamp=time;//ros::Time::now();
				h.frame_id="/left_camera_optical_frame";
				sensor_msgs::ImagePtr msg = cv_bridge::CvImage(h,  sensor_msgs::image_encodings::TYPE_32FC1, BI_cross2).toImageMsg();

				depth_pub.publish(msg);
			}
		}
	}
}
SensorFusion::~SensorFusion() {
	// TODO Auto-generated destructor stub
}

} /* namespace drc_perception */
