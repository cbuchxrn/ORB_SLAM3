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
#include <cv_bridge/cv_bridge.h>

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

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void PublishPose(cv::Mat* originalMat, ORB_SLAM3::System* mpSLAM);
    ORB_SLAM3::System* mpSLAM;
    ros::Publisher* mpPublisher;
};
const vector<ORB_SLAM3::IMU::Point>& vImuMeas = vector<ORB_SLAM3::IMU::Point>();
string filename="asdsa";
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    cout << endl <<"argc = " << argc;
    if(argc != 6)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings resource_topic_name map_file_name" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle nodeHandler;
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true,0, argv[5]);
    ros::Publisher pub = nodeHandler.advertise<geometry_msgs::PoseStamped>(argv[4], 1);
    ImageGrabber igb(&SLAM,&pub);
    ros::Subscriber sub = nodeHandler.subscribe(argv[3], 1, &ImageGrabber::GrabImage,&igb);

    /*ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pose_cam_0", 1);
    ImageGrabber igb(&SLAM,&pub);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, argv[3], 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, argv[4], 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    */
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
   
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat TCam ;
    Sophus::SE3f Tcw_SE3f = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec(),vImuMeas,filename);
    Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
    cv::eigen2cv(Tcw_Matrix, TCam);
    if (TCam.empty())
        return;

    this->PublishPose(&TCam,this->mpSLAM);
    //cout << TCam.data << endl;
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


