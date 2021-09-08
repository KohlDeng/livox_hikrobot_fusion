#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <map_to_cloud/BoundingBoxes.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <math.h>


#include <map_to_cloud/common.h>


using namespace std;

#define SH 0.3
#define BH 0.7

//define global parameters the camera is frame_id ,the lidar is child_frame_id
cv::Mat distCoeffs(5,1,cv::DataType<double>::type);
cv::Mat intrinsic = cv::Mat::eye(3,3,CV_64F);
cv::Mat rotation(3,3,cv::DataType<double>::type);
cv::Mat translation(3,1,cv::DataType<double>::type);

cv::Mat intrInv(3,3,CV_64F);
cv::Mat rotaInv(3,3,CV_64F);

cv::Mat extrinsicMat_RT(4, 4, cv::DataType<double>::type); // 外参旋转矩阵3*3和平移向量3*1


string intrinsic_path, extrinsic_path;

void getParameters()
{
	cout<<"get the parameters from the launch file"<<endl;

	if(!ros::param::get("intrinsic_path",intrinsic_path)){
		cout<<"can not get the value of intrinsic_path"<<endl;
		exit(1);
	}
	if(!ros::param::get("extrinsic_path",extrinsic_path)){
		cout<<"can not get the value of extrinsic_path"<<endl;
		exit(1);
	}
}

void CalibrationData(void)
{
	vector<float> intr;
	getIntrinsic(intrinsic_path, intr);
	vector<float> distortion;
	getDistortion(intrinsic_path,distortion);
	vector<float> extrinsic;
	getExtrinsic(extrinsic_path,extrinsic);


	intrinsic.at<double>(0, 0) = intr[0];
	intrinsic.at<double>(0, 2) = intr[2];
	intrinsic.at<double>(1, 1) = intr[4];
	intrinsic.at<double>(1, 2) = intr[5];

	intrInv = intrinsic.inv();


	distCoeffs = cv::Mat::zeros(5,1,CV_64F);
    distCoeffs.at<double>(0, 0) = distortion[0];
    distCoeffs.at<double>(1, 0) = distortion[1];
    distCoeffs.at<double>(2, 0) = distortion[2];
    distCoeffs.at<double>(3, 0) = distortion[3];
    distCoeffs.at<double>(4, 0) = distortion[4];
	
    rotation = cv::Mat::zeros(3,3,CV_64F);
    translation = cv::Mat::zeros(3,1,CV_64F);

    rotation.at<double>(0, 0) = extrinsic[0];
	rotation.at<double>(0, 1) = extrinsic[1];
	rotation.at<double>(0, 2) = extrinsic[2];
	translation.at<double>(0,0) = extrinsic[3];
	rotation.at<double>(1, 0) = extrinsic[4];
	rotation.at<double>(1, 1) = extrinsic[5];
	rotation.at<double>(1, 2) = extrinsic[6];
	translation.at<double>(1,0) = extrinsic[7];
	rotation.at<double>(2, 0) = extrinsic[8];
	rotation.at<double>(2, 1) = extrinsic[9];
	rotation.at<double>(2, 2) = extrinsic[10];
	translation.at<double>(2, 0) = extrinsic[11];

	rotaInv = rotation.inv();



    extrinsicMat_RT = cv::Mat::zeros(4,4,CV_64F);
	extrinsicMat_RT.at<double>(0, 0) = extrinsic[0];
	extrinsicMat_RT.at<double>(0, 1) = extrinsic[1];
	extrinsicMat_RT.at<double>(0, 2) = extrinsic[2];
	extrinsicMat_RT.at<double>(0, 3) = extrinsic[3];
	extrinsicMat_RT.at<double>(1, 0) = extrinsic[4];
	extrinsicMat_RT.at<double>(1, 1) = extrinsic[5];
	extrinsicMat_RT.at<double>(1, 2) = extrinsic[6];
	extrinsicMat_RT.at<double>(1, 3) = extrinsic[7];
	extrinsicMat_RT.at<double>(2, 0) = extrinsic[8];
	extrinsicMat_RT.at<double>(2, 1) = extrinsic[9];
	extrinsicMat_RT.at<double>(2, 2) = extrinsic[10];
	extrinsicMat_RT.at<double>(2, 3) = extrinsic[11];
	extrinsicMat_RT.at<double>(3, 0) = 0.0;
	extrinsicMat_RT.at<double>(3, 1) = 0.0;
	extrinsicMat_RT.at<double>(3, 2) = 0.0;
	extrinsicMat_RT.at<double>(3, 3) = 1.0;
}

void printMat()
{

	cout<<"==================intrinsic3x3==========="<<endl;
	cout<<intrinsic.at<double>(0, 0) <<"\t"; 
	cout<<intrinsic.at<double>(0, 1) <<"\t"; 
	cout<<intrinsic.at<double>(0, 2) <<"\n"; 
	cout<<intrinsic.at<double>(1, 0) <<"\t"; 
	cout<<intrinsic.at<double>(1, 1) <<"\t"; 
	cout<<intrinsic.at<double>(1, 2) <<"\n"; 
	cout<<intrinsic.at<double>(2, 0) <<"\t"; 
	cout<<intrinsic.at<double>(2, 1) <<"\t"; 
	cout<<intrinsic.at<double>(2, 2) <<endl; 


	cout<<"==================intrInv3x3==========="<<endl;
	cout<<intrInv.at<double>(0, 0) <<"\t"; 
	cout<<intrInv.at<double>(0, 1) <<"\t"; 
	cout<<intrInv.at<double>(0, 2) <<"\n"; 
	cout<<intrInv.at<double>(1, 0) <<"\t"; 
	cout<<intrInv.at<double>(1, 1) <<"\t"; 
	cout<<intrInv.at<double>(1, 2) <<"\n"; 
	cout<<intrInv.at<double>(2, 0) <<"\t"; 
	cout<<intrInv.at<double>(2, 1) <<"\t"; 
	cout<<intrInv.at<double>(2, 2) <<endl; 

	cout<<"==================disCoeffs==========="<<endl;
	cout<<distCoeffs.at<double>(0) <<"\t"; 
	cout<<distCoeffs.at<double>(1) <<"\t"; 
	cout<<distCoeffs.at<double>(2) <<"\t"; 
	cout<<distCoeffs.at<double>(3) <<"\t"; 
	cout<<distCoeffs.at<double>(4) <<endl;

	cout<<"==================rotation3x3==========="<<endl;
	cout<<rotation.at<double>(0, 0) <<"\t"; 
	cout<<rotation.at<double>(0, 1) <<"\t"; 
	cout<<rotation.at<double>(0, 2) <<"\n"; 
	cout<<rotation.at<double>(1, 0) <<"\t"; 
	cout<<rotation.at<double>(1, 1) <<"\t"; 
	cout<<rotation.at<double>(1, 2) <<"\n"; 
	cout<<rotation.at<double>(2, 0) <<"\t"; 
	cout<<rotation.at<double>(2, 1) <<"\t"; 
	cout<<rotation.at<double>(2, 2) <<endl; 

	cout<<"==================rotaInv3x3==========="<<endl;
	cout<<rotaInv.at<double>(0, 0) <<"\t"; 
	cout<<rotaInv.at<double>(0, 1) <<"\t"; 
	cout<<rotaInv.at<double>(0, 2) <<"\n"; 
	cout<<rotaInv.at<double>(1, 0) <<"\t"; 
	cout<<rotaInv.at<double>(1, 1) <<"\t"; 
	cout<<rotaInv.at<double>(1, 2) <<"\n"; 
	cout<<rotaInv.at<double>(2, 0) <<"\t"; 
	cout<<rotaInv.at<double>(2, 1) <<"\t"; 
	cout<<rotaInv.at<double>(2, 2) <<endl; 	


	cout<<"=============translation3x1==============="<<endl;
	cout<<translation.at<double>(0,0)<<endl;
	cout<<translation.at<double>(1,0)<<endl;	
	cout<<translation.at<double>(2,0)<<endl;		



}




class BoundingMap
{
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;

public:
	BoundingMap(ros::NodeHandle nh):n(nh)
	{
		pub=n.advertise<sensor_msgs::PointCloud2>("boundingbox/point_cloud",5);
		sub=n.subscribe<map_to_cloud::BoundingBoxes>("/darknet_ros0/bounding_boxes",10, &BoundingMap::callback,this);
	}

	void callback(const map_to_cloud::BoundingBoxesConstPtr& msg)
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>);


		for(int i=0;i<msg->bounding_boxes.size();i++)
		{
			int xmin = msg->bounding_boxes[i].xmin;
			int ymin = msg->bounding_boxes[i].ymin;
			int xmax = msg->bounding_boxes[i].xmax;
			int ymax = msg->bounding_boxes[i].ymax;

			cv::Mat center_uv(3,1,cv::DataType<double>::type);
			cv::Mat center_xyz(3,1,cv::DataType<double>::type);
			cv::Mat delta(3,1,cv::DataType<double>::type);
			cv::Mat delta_xyz(3,1,cv::DataType<double>::type);
			cv::Mat position(3,1,cv::DataType<double>::type);

			center_uv.at<double>(0,0)=double((xmax+xmin)/2);      //目标框中心点坐标，在uv坐标系下
			center_uv.at<double>(1,0)=double((ymax+ymin)/2);
			center_uv.at<double>(2,0)=1.0;

			cv::Mat top(3,1,cv::DataType<double>::type);
			cv::Mat bottom(3,1,cv::DataType<double>::type);
			cv::Mat top_normal(3,1,cv::DataType<double>::type);
			cv::Mat bottom_normal(3,1,cv::DataType<double>::type);				

			top.at<double>(0,0)=double(xmin);			
			top.at<double>(1,0)=double(ymin);				
			top.at<double>(2,0)=1.0;

			bottom.at<double>(0,0)=double(xmax);			
			bottom.at<double>(1,0)=double(ymax);				
			bottom.at<double>(2,0)=1.0;	

			top_normal = intrInv * top;
			bottom_normal = intrInv * bottom;
			double dy_normal = bottom_normal.at<double>(1,0)-top_normal.at<double>(1,0);
			double depth = SH/(dy_normal);  // SH/(dY/Z) = depth
			depth = abs(depth);



			//calculate the position of the centor in the camera frame_id
			center_xyz = intrInv*center_uv*depth;					
			// tramsform from camera to lidar

			position = rotaInv*(center_xyz-translation);

			pcl::PointXYZ p;


			p.x = position.at<double>(0,0);
			p.y = position.at<double>(1,0);
			p.z = position.at<double>(2,0);

			pcl_ptr->points.push_back(p);
			// cout<<"x:"<<xmax<<","<<xmin<<endl;
			// cout<<"y:"<<ymax<<","<<ymin<<endl;			
			// cout<<"delta is "<<delta<<endl;
			// cout<<"the depth is "<<depth<<endl;
			ROS_INFO("dy_normal is %lf", dy_normal);
			// ROS_INFO("delta is (%lf, %lf, %lf)", delta.at<double>(0, 0), delta.at<double>(1, 0), delta.at<double>(2, 0));
			ROS_INFO("delta_xyz is (%lf, %lf, %lf)", delta_xyz.at<double>(0, 0), delta_xyz.at<double>(1, 0), delta_xyz.at<double>(2, 0));
			ROS_INFO("depth is %lf", depth);
			// // cout<<"center_xyz in the camera frame is " <<center_xyz<<endl;
			// cout<<"the position in lidar frame is "<<position<<endl;
			
		}
		cout<<"=========the image is over==================="<<endl;

		pcl_ptr->width = pcl_ptr->points.size();
		pcl_ptr->height = 1;
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(*pcl_ptr,output);

		output.header.stamp = msg->header.stamp;
		output.header.frame_id = "livox_frame";
		pub.publish(output);
	}


};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"image_depth_node");
	ros::NodeHandle nh;

	getParameters();
	CalibrationData();
	printMat();

	cout<<"start working: "<<endl;

	BoundingMap llc(nh);

	ros::spin();
	return 0;

}