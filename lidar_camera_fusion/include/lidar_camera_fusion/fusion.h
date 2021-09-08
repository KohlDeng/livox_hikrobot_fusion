#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <Eigen/Core>
#include <chrono>

#include <std_msgs/Header.h>
#include <lidar_camera_fusion/BoundingBoxes.h>

using namespace std;

string float2str(float num);
float str2float(string str);

double str2double(string str);
string double2str(double num);

int str2int(string str);
string int2str(int num);

string long2str(long num);

void getIntrinsic(const string path, vector<float> &intrinsic);
void getDistortion(const string path, vector<float> &distortion);
void getExtrinsic(const string path, vector<float> &extrinsic);

void rotation2angle(Eigen::Matrix4d rot);

string configPath = "config";

// convert a int to a string
string int2str(int num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[float2str] - Code error" << endl;
        exit(0);
    }
}

int str2int(string str) {
    int d;
    stringstream sin(str);
    if(sin >> d) {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to int" << endl;
    exit(0);
}

string float2str(float num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[float2str] - Code error" << endl;
        exit(0);
    }
}

float str2float(string str) {
    float d;
    stringstream sin(str);
    if(sin >> d) {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to float" << endl;
    exit(0);
}

string double2str(double num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[double2str] - Code error" << endl;
        exit(0);
    }
}

double str2double(string str) {
    double d;
    stringstream sin(str);
    if(sin >> d) {
        return d;
    }
    cout << str << endl;
    cout << "Can not convert a string to double" << endl;
    exit(0);
}

string long2str(long num) {
    ostringstream oss;
    if (oss << num) {
        string str(oss.str());
        return str;
    }
    else {
        cout << "[long2str] - Code error" << endl;
        exit(0);
    }
}

void getIntrinsic(const string path, vector<float> &intrinsic) {
    ifstream inFile;
    inFile.open(path);
    if (!inFile.is_open()) {
        cout << "Can not open file " << path << endl; 
        exit(1);
    }

    string lineStr;
    getline(inFile, lineStr);
    for (uint i = 0; i < 3; ++i) {
        getline(inFile, lineStr);
        stringstream line(lineStr);
        string str;
        
        line >> str;
        intrinsic.push_back(str2double(str));
        line >> str;
        intrinsic.push_back(str2double(str));
        line >> str;
        intrinsic.push_back(str2double(str));
    }
}

void getDistortion(const string path, vector<float> &distortion) {
    ifstream inFile;
    inFile.open(path);
    if (!inFile.is_open()) {
        cout << "Can not open file " << path << endl; 
        exit(1);
    }
    string lineStr;
    for (uint i = 0; i < 6; ++i) {
        getline(inFile, lineStr);
    }
    
    getline(inFile, lineStr);
    stringstream line(lineStr);
    string str;
        
    line >> str;
    distortion.push_back(str2double(str));
    line >> str;
    distortion.push_back(str2double(str));
    line >> str;
    distortion.push_back(str2double(str));
    line >> str;
    distortion.push_back(str2double(str));
    line >> str;
    distortion.push_back(str2double(str));
}

void getExtrinsic(const string path, vector<float> &extrinsic) {
    ifstream inFile;
    inFile.open(path);
    if (!inFile.is_open()) {
        cout << "Can not open file " << path << endl; 
        exit(1);
    }

    string lineStr;
    getline(inFile, lineStr);
    for (uint i = 0; i < 3; ++i) {
        getline(inFile, lineStr);
        stringstream line(lineStr);
        string str;
        
        line >> str;
        extrinsic.push_back(str2double(str));
        line >> str;
        extrinsic.push_back(str2double(str));
        line >> str;
        extrinsic.push_back(str2double(str));
        line >> str;
        extrinsic.push_back(str2double(str));
    }
}

void rotation2angle(Eigen::Matrix4d rot) {
    double sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if(!singular) {
        x = atan2(rot(2, 1), rot(2, 2)) * 180 / M_PI;
        y = atan2(-rot(2, 0), sy) * 180 / M_PI;
        z = atan2(rot(1, 0), rot(0, 0)) * 180 / M_PI;
    }
    else {
        x = atan2(-rot(1, 2), rot(1, 1)) * 180 / M_PI;
        y = atan2(-rot(2, 0), sy) * 180 / M_PI;
        z = 0;
    }
    cout << x << " " << y << " " << z << endl << endl; // roll pitch yaw
}

struct StampedPoints
{
    std_msgs::Header header;
    int size;
    vector<int>xmin;
    vector<int>xmax;
    vector<int>ymin;
    vector<int>ymax;
    vector<int>label;
};


class lidar_camera_fusion
{
public:
	lidar_camera_fusion(ros::NodeHandle* nh);
	void getParameters();
	void calibrationData();
	void printMat();
	void getColor(int &result_r, int &result_g, int &result_b, float cur_depth);
	
	void imageProcess(const sensor_msgs::ImageConstPtr& msg);
	void pointCloudProcess(const sensor_msgs::PointCloud2ConstPtr& laserCloudmsg);
    void boxProcess(const lidar_camera_fusion::BoundingBoxesConstPtr& boxmsg);


private:
	ros::NodeHandle* nh_;

    image_transport::Publisher pubImage_;
    ros::Publisher pubCloud_;
    cv::Mat imageMat_;
    struct StampedPoints stampedPoints_;

	
	image_transport::Publisher pubImage1_;
    ros::Publisher pubCloud1_;
	cv::Mat imageMat1_;
    struct StampedPoints stampedPoints1_;




	const int threshold_lidar_=15000;
	float max_depth_;
	float min_depth_;
	string intrinsic_path_,extrinsic_path_;

	cv::Mat intrinsic_= cv::Mat::eye(3,3,CV_64F);
	cv::Mat distCoeffs_ = cv::Mat(5,1,cv::DataType<double>::type);
	cv::Mat intrinsicMat_ = cv::Mat(3,4,cv::DataType<double>::type);
	cv::Mat extrinsicMat_RT_ = cv::Mat(4,4,cv::DataType<double>::type);

    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>); //livox点云消息包含xyz和intensity 

};


#endif // COMMON_H