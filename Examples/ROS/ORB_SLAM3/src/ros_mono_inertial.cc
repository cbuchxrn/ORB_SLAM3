/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <System.h>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Core>

#include<opencv2/core/core.hpp>
#include<opencv2/core/eigen.hpp>

#include "../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM,ros::Publisher* pPublisher):mpSLAM(pSLAM),mpPublisher(pPublisher){}
    void PublishPose(cv::Mat* originalMat, ORB_SLAM3::System* mpSLAM);
    void GrabMonoIMU(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImuConstPtr& msgIMU);

    ORB_SLAM3::System* mpSLAM;
    ros::Publisher* mpPublisher;
};
const vector<ORB_SLAM3::IMU::Point>& vImuMeas = vector<ORB_SLAM3::IMU::Point>();
string filename="asdsa";
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Inertial");
    ros::start();

    
    if((argc <= 6)&&(argc >= 5))
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings rgb_topic imu_topic map_file_name [do_equalize]" << endl;        
        ros::shutdown();
        return 1;
    }    

    // verbose input data 
    std::cout <<"Use Vocabulary at: " << argv[1]<< std::endl;
    std::cout <<"Use Settings at: " << argv[2]<< std::endl;
    std::cout <<"Subscribe to Camera: " << argv[3]<< std::endl;
    std::cout <<"Subscribe to IMU: " << argv[4]<< std::endl;  
    std::cout <<"Use Map named: " << argv[5]<< std::endl;
    std::cout <<"Equalise: " << argv[6]<< std::endl;

    bool bEqual = false;
    if(argc==6)
    {
        std::string sbEqual(argv[6]);
        if(sbEqual == "true")
        bEqual = true;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,true,0, argv[5]);

   

    ros::NodeHandle nh;
    //advertise camera pose topic for each image
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pose_cam_0", 1);
    ImageGrabber igb(&SLAM,&pub);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, argv[3], 10);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, argv[4], 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Imu> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(100), rgb_sub,imu_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabMonoIMU,&igb,_1,_2));


    

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabMonoIMU(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImuConstPtr& msgIMU)
{   
    double tIm = 0;
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
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

    tIm = msgIMU->header.stamp.toSec();
    
    cv::Point3f acc(msgIMU->linear_acceleration.x, msgIMU->linear_acceleration.y, msgIMU->linear_acceleration.z);
    cv::Point3f gyr(msgIMU->angular_velocity.x, msgIMU->angular_velocity.y, msgIMU->angular_velocity.z);
    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,tIm));

    std::cout << vImuMeas.front().a << std::endl;
    std::cout << vImuMeas.front().w << std::endl;
    Sophus::SE3f Tcw_SE3f = mpSLAM->TrackMonocular(cv_ptrRGB->image,tIm,vImuMeas,filename);
    Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();

    cv::Mat TCam;
    cv::eigen2cv(Tcw_Matrix, TCam); 
    if (TCam.empty())
        return;

    this->PublishPose(&TCam,this->mpSLAM);
    
}



void ImageGrabber::PublishPose(cv::Mat* originalMat, ORB_SLAM3::System* mpSLAM)
{
    if (originalMat->dims == 0 )
        return;
    //std::cout<<"\nTCam : \n" << *originalMat <<std::endl;    
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