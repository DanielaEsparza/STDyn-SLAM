/*
 *--------------------------------------------------------------------------------------------------
 * DS-SLAM: A Semantic Visual SLAM towards Dynamic Environments
　*　Author(s):
 * Chao Yu, Zuxin Liu, Xinjun Liu, Fugui Xie, Yi Yang, Qi Wei, Fei Qiao qiaofei@mail.tsinghua.edu.cn
 * Created by Yu Chao@2018.12.03
 * --------------------------------------------------------------------------------------------------
 * DS-SLAM is a optimized SLAM system based on the famous ORB-SLAM2. If you haven't learn ORB_SLAM2 code, 
 * you'd better to be familiar with ORB_SLAM2 project first. Compared to ORB_SLAM2, 
 * we add anther two threads including semantic segmentation thread and densemap creation thread. 
 * You should pay attention to Frame.cc, ORBmatcher.cc, Pointcloudmapping.cc and Segment.cc.
 * 
 *　@article{murORB2,
 *　title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
　*　author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
　* journal={IEEE Transactions on Robotics},
　*　volume={33},
　* number={5},
　* pages={1255--1262},
　* doi = {10.1109/TRO.2017.2705103},
　* year={2017}
 *　}
 * --------------------------------------------------------------------------------------------------
 * Copyright (C) 2018, iVip Lab @ EE, THU (https://ivip-tsinghua.github.io/iViP-Homepage/) and 
 * Advanced Mechanism and Roboticized Equipment Lab. All rights reserved.
 *
 * Licensed under the GPLv3 License;
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * https://github.com/ivipsourcecode/DS-SLAM/blob/master/LICENSE
 *--------------------------------------------------------------------------------------------------
 */
 
/*
* This file is part of STDyn-SLAM.
*
*    STDyn-SLAM is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    STDyn-SLAM is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with STDyn-SLAM.  If not, see <https://www.gnu.org/licenses/>.
* 
*    Copyright (C) 2020 Daniela Esparza, (Center for Research in Optics)
*    Perception and Robotics LAB, https://github.com/DanielaEsparza/STDyn-SLAM
*    
*/  

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <ros/ros.h>
#include <octomap/octomap.h>    
#include <octomap/ColorOcTree.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
 
#include <../../../include/System.h>
#include <iostream>

using namespace octomap;

ros::Publisher CamPose_Pub;
ros::Publisher Camodom_Pub;
ros::Publisher odom_pub;

geometry_msgs::PoseStamped Cam_Pose;
geometry_msgs::PoseWithCovarianceStamped Cam_odom;


boost::shared_ptr<PointCloudMapping> tumPointCloudMapping;

cv::Mat Camera_Pose;
tf::Transform orb_slam;
tf::TransformBroadcaster * orb_slam_broadcaster;
std::vector<float> Pose_quat(4);
std::vector<float> Pose_trans(3);

ros::Time current_time, last_time;
double lastx=0,lasty=0,lastth=0;
unsigned int a =0,b=0; 
octomap::ColorOcTree tree( 0.05 );

void Pub_CamPose(cv::Mat &pose);

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
    
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};


typedef octomap::ColorOcTree::leaf_iterator it_t;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_DS_SLAM");
    ros::start();
    ::google::InitGoogleLogging(argv[0]);
   
    if(argc != 6)
    {
        cerr << endl << "Usage: TUM path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_prototxt path_to_caffemodel path_to_pascal.png" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<double> vTimestamps;
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::Viewer *viewer;
    viewer = new ORB_SLAM2::Viewer();
    ORB_SLAM2::System SLAM(argv[1],argv[2], argv[3],argv[4],argv[5],ORB_SLAM2::System::RGBD, viewer);
    usleep(50);

    ImageGrabber igb(&SLAM);

   
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    ros::Rate loop_rate(50);
    ros::NodeHandle nh;

    CamPose_Pub = nh.advertise<geometry_msgs::PoseStamped>("/Camera_Pose",1);
    Camodom_Pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/Camera_Odom", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

	
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgbd_image", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));


    ros::spin();
    
    // Stop all threads
    SLAM.Shutdown();
    

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");	
    
    ros::shutdown();
    return 0;
}

void Pub_CamPose(cv::Mat &pose)
{
    cv::Mat Rwc(3,3,CV_32F);
	cv::Mat twc(3,1,CV_32F);
	Eigen::Matrix<double,3,3> rotationMat;
	orb_slam_broadcaster = new tf::TransformBroadcaster;
	if(pose.dims<2 || pose.rows < 3)
	{
        	Rwc = Rwc;
		twc = twc;
		cout << pose.dims << endl;
	}
	else
	{
		Rwc = pose.rowRange(0,3).colRange(0,3).t();
		twc = -Rwc*pose.rowRange(0,3).col(3);
		
		rotationMat << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
					Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
					Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);
		Eigen::Quaterniond Q(rotationMat);

		Pose_quat[0] = Q.x(); Pose_quat[1] = Q.y();
		Pose_quat[2] = Q.z(); Pose_quat[3] = Q.w();
		
		Pose_trans[0] = twc.at<float>(0);
		Pose_trans[1] = twc.at<float>(1);
		Pose_trans[2] = twc.at<float>(2);
		
		orb_slam.setOrigin(tf::Vector3(Pose_trans[2], -Pose_trans[0], -Pose_trans[1]));
		orb_slam.setRotation(tf::Quaternion(Q.z(), -Q.x(), -Q.y(), Q.w()));
		orb_slam_broadcaster->sendTransform(tf::StampedTransform(orb_slam, ros::Time::now(), "/map", "/base_link"));
		
		Cam_Pose.header.stamp = ros::Time::now();
		Cam_Pose.header.frame_id = "/map";
		tf::pointTFToMsg(orb_slam.getOrigin(), Cam_Pose.pose.position);
		tf::quaternionTFToMsg(orb_slam.getRotation(), Cam_Pose.pose.orientation);
		
		Cam_odom.header.stamp = ros::Time::now();
		Cam_odom.header.frame_id = "/map";
		tf::pointTFToMsg(orb_slam.getOrigin(), Cam_odom.pose.pose.position);
		tf::quaternionTFToMsg(orb_slam.getRotation(), Cam_odom.pose.pose.orientation);
		Cam_odom.pose.covariance = {0.01, 0, 0, 0, 0, 0,
									0, 0.01, 0, 0, 0, 0,
									0, 0, 0.01, 0, 0, 0,
									0, 0, 0, 0.01, 0, 0,
									0, 0, 0, 0, 0.01, 0,
									0, 0, 0, 0, 0, 0.01};
		
		CamPose_Pub.publish(Cam_Pose);
		Camodom_Pub.publish(Cam_odom);
		
		nav_msgs::Odometry odom;
		odom.header.stamp =ros::Time::now();
		odom.header.frame_id = "/map";

		// Set the position
		odom.pose.pose.position = Cam_odom.pose.pose.position;
		odom.pose.pose.orientation = Cam_odom.pose.pose.orientation;

		// Set the velocity
		odom.child_frame_id = "/base_link";
		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		double vx = (Cam_odom.pose.pose.position.x - lastx)/dt;
		double vy = (Cam_odom.pose.pose.position.y - lasty)/dt;
		double vth = (Cam_odom.pose.pose.orientation.z - lastth)/dt;
		
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		// Publish the message
		odom_pub.publish(odom);
		
		last_time = current_time;
		lastx = Cam_odom.pose.pose.position.x;
		lasty = Cam_odom.pose.pose.position.y;
		lastth = Cam_odom.pose.pose.orientation.z;
	}
}


void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
	std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
	Camera_Pose =  mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
	std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
	double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count();
    cout << "SLAM TrackRGBD all time =" << ttrack*1000 << endl << endl;
	Pub_CamPose(Camera_Pose);
	
	bool flag_pc_finish=tumPointCloudMapping->flagFinishPC();
	
	while(!flag_pc_finish)
	{
		flag_pc_finish=tumPointCloudMapping->flagFinishPC();
	}
}
