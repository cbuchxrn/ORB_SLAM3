/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM,ros::Publisher* pPublisher):mpSLAM(pSLAM),mpPublisher(pPublisher){}
    void PublishPose(cv::Mat* originalMat, ORB_SLAM3::System* mpSLAM);
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;
    ros::Publisher* mpPublisher;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

   

    ros::NodeHandle nh;
    //advertise camera pose topic for each image
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pose_cam_0", 1);
    ImageGrabber igb(&SLAM,&pub);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, argv[3], 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, argv[4], 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));


    

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
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
    cv::Mat TCam;
    TCam = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    this->PublishPose(&TCam,this->mpSLAM);
}



void ImageGrabber::PublishPose(cv::Mat* originalMat, ORB_SLAM3::System* mpSLAM)
{
    if (originalMat->dims == 0 )
        return;
        
    cv::Rect rotSel(0,0,3,3);
    cv::Rect transSel(3,0,1,3);
    cv::Mat_<double> trans = (*originalMat)(transSel);
    cv::Mat_<double> rot = (*originalMat)(rotSel);

  
    // fill msg header
    geometry_msgs::PoseStamped pStamped;
    pStamped.header.stamp = ros::Time::now(); // timestamp of creation of the msg
    pStamped.header.frame_id = "pose_cam_"+std::to_string(mpSLAM->getSysId()); // frame id 
    geometry_msgs::Pose pose_msg; 

    
    cv::Mat mat = originalMat->clone();

    //extract Transformation
    tf2::Matrix3x3 tf2_rot( rot(0,0), rot(0,1) ,rot(0,2),
                            rot(1,0), rot(1,1) ,rot(1,2),
                            rot(2,0), rot(2,1) ,rot(2,2));
    
    tf2::Vector3 tf2_tran(trans(0,0),trans(1,0),trans(2,0));
    tf2::Transform tf2_transform(tf2_rot, tf2_tran);
    tf2::Quaternion q;
    tf2_rot.getRotation(q);


    // fill pose appropriately
    tf2::toMsg(tf2_transform, pose_msg);
    pStamped.pose = pose_msg;

    this->mpPublisher->publish(pStamped);

    
}
