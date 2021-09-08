#include <iostream>
#include "ros/ros.h"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>


#include <sensor_msgs/PointCloud2.h>


#include <lidar_camera_fusion/fusion.h>
#include <lidar_camera_fusion/BoundingBoxes.h>

using namespace std;

#define Hmax 1080
#define Wmax 1440
#define H Hmax
#define W Wmax

typedef pcl::PointXYZI PointType;

livox_lidar_color::livox_lidar_color(ros::NodeHandle* nh)
{
	nh_ = nh;
	max_depth_ = 200;
	min_depth_ = 0.5;
	stampedPoints_.xmin.push_back(0);		//initialize
	stampedPoints_.xmax.push_back(0);
	stampedPoints_.ymin.push_back(0);
	stampedPoints_.ymax.push_back(0);
	stampedPoints_.label.push_back(0);
	stampedPoints_.size = stampedPoints_.label.size();

	image_transport::ImageTransport it(*nh_);
	pubImage_ = it.advertise("fusion/fusion_image",10);
	pubCloud_ = nh_->advertise<sensor_msgs::PointCloud2>("fusion/fusion_cloud",5);

	getParameters();
	calibrationData();
	printMat();

}

void livox_lidar_color::getParameters(){
	ROS_INFO("Get parameters from launch file.");

	if(!ros::param::get("intrinsic_path",intrinsic_path_)){
		ROS_INFO("can not get the value of intrinsic_path");
		exit(1);
	}
	if(!ros::param::get("extrinsic_path",extrinsic_path_)){
		ROS_INFO("can not get the value of extrinsic_path");
		exit(1);
	}
}

void livox_lidar_color::calibrationData(){
	vector<float> intr;
	getIntrinsic(intrinsic_path_, intr);
	vector<float> distortion;
	getDistortion(intrinsic_path_, distortion);
	vector<float> extrinsic;
	getExtrinsic(extrinsic_path_, extrinsic);

	intrinsic_.at<double>(0, 0) = intr[0];
	intrinsic_.at<double>(0, 2) = intr[2];
	intrinsic_.at<double>(1, 1) = intr[4];
	intrinsic_.at<double>(1, 2) = intr[5];

	intrinsicMat_ = cv::Mat::zeros(3,4,CV_64F);
    intrinsicMat_.at<double>(0, 0) = intr[0];
	intrinsicMat_.at<double>(0, 2) = intr[2];
	intrinsicMat_.at<double>(1, 1) = intr[4];
	intrinsicMat_.at<double>(1, 2) = intr[5];
	intrinsicMat_.at<double>(2, 2) = 1.000000e+00;

	distCoeffs_ = cv::Mat::zeros(5,1,CV_64F);
    distCoeffs_.at<double>(0, 0) = distortion[0];
    distCoeffs_.at<double>(1, 0) = distortion[1];
    distCoeffs_.at<double>(2, 0) = distortion[2];
    distCoeffs_.at<double>(3, 0) = distortion[3];
    distCoeffs_.at<double>(4, 0) = distortion[4];
	
    extrinsicMat_RT_ = cv::Mat::zeros(4,4,CV_64F);
	extrinsicMat_RT_.at<double>(0, 0) = extrinsic[0];
	extrinsicMat_RT_.at<double>(0, 1) = extrinsic[1];
	extrinsicMat_RT_.at<double>(0, 2) = extrinsic[2];
	extrinsicMat_RT_.at<double>(0, 3) = extrinsic[3];
	extrinsicMat_RT_.at<double>(1, 0) = extrinsic[4];
	extrinsicMat_RT_.at<double>(1, 1) = extrinsic[5];
	extrinsicMat_RT_.at<double>(1, 2) = extrinsic[6];
	extrinsicMat_RT_.at<double>(1, 3) = extrinsic[7];
	extrinsicMat_RT_.at<double>(2, 0) = extrinsic[8];
	extrinsicMat_RT_.at<double>(2, 1) = extrinsic[9];
	extrinsicMat_RT_.at<double>(2, 2) = extrinsic[10];
	extrinsicMat_RT_.at<double>(2, 3) = extrinsic[11];
	extrinsicMat_RT_.at<double>(3, 0) = 0.0;
	extrinsicMat_RT_.at<double>(3, 1) = 0.0;
	extrinsicMat_RT_.at<double>(3, 2) = 0.0;
	extrinsicMat_RT_.at<double>(3, 3) = 1.0;
}

void livox_lidar_color::printMat()
{
	cout<<"=============extrinsic==============="<<endl;
	cout<<extrinsicMat_RT_.at<double>(0, 0)<<"\t"; 
	cout<<extrinsicMat_RT_.at<double>(0, 1)<<"\t";
	cout<<extrinsicMat_RT_.at<double>(0, 2)<<"\t";
	cout<<extrinsicMat_RT_.at<double>(0, 3)<<endl;
	cout<<extrinsicMat_RT_.at<double>(1, 0) <<"\t";
	cout<<extrinsicMat_RT_.at<double>(1, 1) <<"\t";
	cout<<extrinsicMat_RT_.at<double>(1, 2) <<"\t";
	cout<<extrinsicMat_RT_.at<double>(1, 3) <<endl;
	cout<<extrinsicMat_RT_.at<double>(2, 0) <<"\t";
	cout<<extrinsicMat_RT_.at<double>(2, 1) <<"\t";
	cout<<extrinsicMat_RT_.at<double>(2, 2) <<"\t";
	cout<<extrinsicMat_RT_.at<double>(2, 3) <<endl;
	cout<<extrinsicMat_RT_.at<double>(3, 0) <<"\t";
	cout<<extrinsicMat_RT_.at<double>(3, 1) <<"\t";
	cout<<extrinsicMat_RT_.at<double>(3, 2) <<"\t";
	cout<<extrinsicMat_RT_.at<double>(3, 3) <<endl;

	cout<<"==================intrinsic==========="<<endl;
	cout<<intrinsicMat_.at<double>(0, 0) <<"\t"; 
	cout<<intrinsicMat_.at<double>(0, 1) <<"\t"; 
	cout<<intrinsicMat_.at<double>(0, 2) <<"\t"; 
	cout<<intrinsicMat_.at<double>(0, 3) <<endl;
	cout<<intrinsicMat_.at<double>(1, 0) <<"\t"; 
	cout<<intrinsicMat_.at<double>(1, 1) <<"\t"; 
	cout<<intrinsicMat_.at<double>(1, 2) <<"\t"; 
	cout<<intrinsicMat_.at<double>(1, 3) <<endl;
	cout<<intrinsicMat_.at<double>(2, 0) <<"\t"; 
	cout<<intrinsicMat_.at<double>(2, 1) <<"\t"; 
	cout<<intrinsicMat_.at<double>(2, 2) <<"\t"; 
	cout<<intrinsicMat_.at<double>(2, 3) <<endl;
	cout<<"==================disCoeffs==========="<<endl;
	cout<<distCoeffs_.at<double>(0) <<"\t"; 
	cout<<distCoeffs_.at<double>(1) <<"\t"; 
	cout<<distCoeffs_.at<double>(2) <<"\t"; 
	cout<<distCoeffs_.at<double>(3) <<"\t"; 
	cout<<distCoeffs_.at<double>(4) <<endl;

	cout<<"==================intrinsic_3x3==========="<<endl;
	cout<<intrinsic_.at<double>(0, 0) <<"\t"; 
	cout<<intrinsic_.at<double>(0, 1) <<"\t"; 
	cout<<intrinsic_.at<double>(0, 2) <<"\n"; 
	cout<<intrinsic_.at<double>(1, 0) <<"\t"; 
	cout<<intrinsic_.at<double>(1, 1) <<"\t"; 
	cout<<intrinsic_.at<double>(1, 2) <<"\n"; 
	cout<<intrinsic_.at<double>(2, 0) <<"\t"; 
	cout<<intrinsic_.at<double>(2, 1) <<"\t"; 
	cout<<intrinsic_.at<double>(2, 2) <<endl; 
}

void livox_lidar_color::getColor(int &result_r, int &result_g, int &result_b, float cur_depth) {
    float scale = (max_depth_ - min_depth_)/10;
    if (cur_depth < min_depth_) {
        result_r = 0;
        result_g = 0;
        result_b = 0xff;
    }
    else if (cur_depth < min_depth_ + scale) {
        result_r = 0;
        result_g = int((cur_depth - min_depth_) / scale * 255) & 0xff;
        result_b = 0xff;
    }
    else if (cur_depth < min_depth_ + scale*2) {
        result_r = 0;
        result_g = 0xff;
        result_b = (0xff - int((cur_depth - min_depth_ - scale) / scale * 255)) & 0xff;
    }
    else if (cur_depth < min_depth_ + scale*4) {
        result_r = int((cur_depth - min_depth_ - scale*2) / scale * 255) & 0xff;
        result_g = 0xff;
        result_b = 0;
    }
    else if (cur_depth < min_depth_ + scale*7) {
        result_r = 0xff;
        result_g = (0xff - int((cur_depth - min_depth_ - scale*4) / scale * 255)) & 0xff;
        result_b = 0;
    }
    else if (cur_depth < min_depth_ + scale*10) {
        result_r = 0xff;
        result_g = 0;
        result_b = int((cur_depth - min_depth_ - scale*7) / scale * 255) & 0xff;
    }
    else {
        result_r = 0xff;
        result_g = 0;
        result_b = 0xff;
    }

}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg, livox_lidar_color *llc)
{
	llc->pointCloudProcess(laserCloudMsg);	
}

void livox_lidar_color::pointCloudProcess(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
	pcl::fromROSMsg(*laserCloudMsg, *raw_pcl_ptr_);										   //把msg消息指针转化为PCL点云
	cv::Mat X(4, 1, cv::DataType<double>::type);
	cv::Mat Y(3, 1, cv::DataType<double>::type);
	uint32_t myCount=0;
	cv::Mat src_img(imageMat_);
	pcl::PointCloud<pcl::PointXYZL>::Ptr pcl_ptr(new pcl::PointCloud<pcl::PointXYZL>);

	double k1,k2,k3,p1,p2;
	k1 = distCoeffs_.at<double>(0,0);
	k2 = distCoeffs_.at<double>(1,0);
	p1 = distCoeffs_.at<double>(2,0);
	p2 = distCoeffs_.at<double>(3,0);
	k3 = distCoeffs_.at<double>(4,0);

	for (int i = 0; i < raw_pcl_ptr_->points.size(); i++)
	{
		X.at<double>(0, 0) = raw_pcl_ptr_->points[i].x;
		X.at<double>(1, 0) = raw_pcl_ptr_->points[i].y;
		X.at<double>(2, 0) = raw_pcl_ptr_->points[i].z;
		X.at<double>(3, 0) = 1;

		cv::Mat X_fromCa(3,1,cv::DataType<double>::type);
		X_fromCa = extrinsicMat_RT_ * X;
		
		double xnor = X_fromCa.at<double>(0,0)/X_fromCa.at<double>(2,0);
		double ynor = X_fromCa.at<double>(1,0)/X_fromCa.at<double>(2,0);
		double r = std::hypot(xnor, ynor);
		double x_distorted = xnor * (1 + k1 * pow(r, 2) + k2 * pow(r, 4) + k3 * pow(r, 6)) + 2 * p1 * xnor * ynor + 
			p2 * (pow(r, 2) + 2 * pow(xnor, 2));
		double y_distorted = ynor * (1 + k1 * pow(r, 2) + k2 * pow(r, 4) + k3 * pow(r, 6)) + 2 * p2 * xnor * ynor + 
			p1 * (pow(r, 2) + 2 * pow(ynor, 2));

		cv::Mat X_Normal(3,1,cv::DataType<double>::type);
		X_Normal.at<double>(0,0) = x_distorted;
		X_Normal.at<double>(1,0) = y_distorted;
		X_Normal.at<double>(2,0) = 1;

		Y = intrinsic_*X_Normal; //雷达坐标转换到相机坐标，相机坐标投影到像素坐标

		cv::Point pt;						   // (x,y) 像素坐标
		pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
		pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);			
		pt.y = pt.y+10;
        double rat = 0.1;

        if ((pt.x >= 0) && (pt.x < W) && (pt.y >= 0) && (pt.y < H) && (raw_pcl_ptr_->points[i].x > 0)) //&& raw_pcl_ptr_->points[i].x>0去掉图像后方的点云
		{
			

        	for (int j=0;j<stampedPoints_.size;j++)
        	{
        		double xmin = double(stampedPoints_.xmin[j]);
        		double xmax = double(stampedPoints_.xmax[j]);
        		double ymin = double(stampedPoints_.ymin[j]);
        		double ymax = double(stampedPoints_.ymax[j]);

        		double xdiff = xmax - xmin;
        		xmax = xmax + xdiff * rat;
        		xmin = xmin - xdiff * rat;

        		double ydiff = ymax - ymin;
        		ymax = ymax + ydiff * rat;
        		ymin = ymin - ydiff * rat;


        		
        		if((xmin<pt.x) && (pt.x< xmax) && (ymin<pt.y) && (pt.y< ymax))         		//ROS_INFO("dist is %lf",dist);
        		{
        			int u = floor(pt.x + 0.5);
        			int v = floor(pt.y + 0.5);

        			int r,g,b;
        			getColor(r, g, b, X.at<double>(0, 0));
        			cv::Point p(u, v);

        			cv::circle(src_img, p, 10, cv::Scalar(b, g, r), -1);
        			++myCount;

        			pcl::PointXYZL pc;					//TODO : rename
        			pc.x = raw_pcl_ptr_->points[i].x;
        			pc.y = raw_pcl_ptr_->points[i].y;
        			pc.z = raw_pcl_ptr_->points[i].z;
        			pc.label = stampedPoints_.label[j];

        			pcl_ptr->points.push_back(pc);
        			break;
        		}


        	}
        	
        	
    	}
    	if (myCount > threshold_lidar_) {
            	break;
        	}
	}
	//cout<<"=========the image is over==================="<<endl;
	//publish image
	sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",src_img).toImageMsg();
	pubImage_.publish(msg);

	//publish cloud
	pcl_ptr->width = pcl_ptr->points.size();
	pcl_ptr->height = 1;
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*pcl_ptr,output);
	output.header.stamp = stampedPoints_.header.stamp;
	output.header.frame_id = "livox_frame";
	pubCloud_.publish(output);

}

void imageCallback(const sensor_msgs::ImageConstPtr &msg, livox_lidar_color *llc)
{
	try
	{
		llc->imageProcess(msg);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not conveextrinsicMat_RT from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void livox_lidar_color::imageProcess(const sensor_msgs::ImageConstPtr &msg) {

	cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image; 

	//去畸变，可选
	// cv::Mat map1, map2;
	// cv::Size imageSize = image.size();		
	// cv::initUndistortRectifyMap(intrinsic_, distCoeffs_, cv::Mat(), 
	// 	cv::getOptimalNewCameraMatrix(intrinsic_, distCoeffs_, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
	// cv::remap(image, image, map1, map2, cv::INTER_LINEAR); // correct the distortion
	//cv::imwrite("1.bmp",image);	
	imageMat_ = image;


}

void boxCallback(const lidar_camera_fusion::BoundingBoxesConstPtr& boxmsg, livox_lidar_color *llc)
{
	llc->boxProcess(boxmsg);
}

void livox_lidar_color::boxProcess(const lidar_camera_fusion::BoundingBoxesConstPtr& boxmsg)
{

	stampedPoints_.xmin.clear();
	stampedPoints_.xmax.clear();
	stampedPoints_.ymin.clear();
	stampedPoints_.ymax.clear();
	stampedPoints_.label.clear();

	stampedPoints_.header = boxmsg->header;
	for(int i=0;i<boxmsg->bounding_boxes.size();i++)
	{
		stampedPoints_.xmin.push_back(boxmsg->bounding_boxes[i].xmin);
		stampedPoints_.xmax.push_back(boxmsg->bounding_boxes[i].xmax);
		stampedPoints_.ymin.push_back(boxmsg->bounding_boxes[i].ymin);
		stampedPoints_.ymax.push_back(boxmsg->bounding_boxes[i].ymax);		
        stampedPoints_.label.push_back(boxmsg->bounding_boxes[i].id);

	}

	stampedPoints_.size = stampedPoints_.label.size();
	// for(int i=0;i<stampedPoints_.size;i++)
	// {
	// 	ROS_INFO("cenx ceny is (%d,%d)  label is %d:", stampedPoints_.cenx[i],stampedPoints_.ceny[i],stampedPoints_.label[i]);

	// }

	// cout<<"=========the image is over==================="<<endl;

}



int main(int argc, char *argv[])
{
	ros::init(argc,argv,"fusion_node");
	ros::NodeHandle nh;

	livox_lidar_color llc(&nh); 
	ros::Subscriber subCloud = nh.subscribe<sensor_msgs::PointCloud2>("/perception/livox_cluster",5,boost::bind(pointCloudCallback,_1,&llc));
	ros::Subscriber subBox = nh.subscribe<lidar_camera_fusion::BoundingBoxes>("/darknet_ros0/bounding_boxes",10,boost::bind(boxCallback,_1,&llc));



	image_transport::ImageTransport itc(nh);
	image_transport::Subscriber subImage = itc.subscribe("/darknet_ros0/detection_image",20,boost::bind(imageCallback,_1,&llc)) ;


	
	ROS_INFO("start working...");
	ros::AsyncSpinner spinner(4);
	spinner.start();



	ros::waitForShutdown();
	return 0;

}

