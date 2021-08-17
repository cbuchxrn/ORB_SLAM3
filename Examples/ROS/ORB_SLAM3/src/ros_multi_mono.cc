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

#include<vector>
#include<string> 

#include<ros/ros.h>
#include<ros/message.h>

#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/Float32.h>
#include<cv_bridge/cv_bridge.h>
#include<message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include<opencv2/core/affine.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "../../../include/System.h"

using namespace std;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM,ros::Publisher* pPublisher):mpSLAM(pSLAM),mpPublisher(pPublisher){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    
    void PublishPoseMat(cv::Mat* originalMat);

    void PublishPose(cv::Mat* originalMat, ORB_SLAM3::System* mpSLAM);
    ORB_SLAM3::System* mpSLAM;
    ros::Publisher* mpPublisher;
};

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
    cv::Mat TCam;
    TCam = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    this->PublishPose(&TCam,this->mpSLAM);
    //cout << TCam.data << endl;
}

void ImageGrabber::PublishPoseMat(cv::Mat* originalMat)
{
    if (originalMat->dims == 0 )
        return;
        
    std_msgs::Float32MultiArray sendMat;
    sendMat.layout = std_msgs::MultiArrayLayout();

    std::vector<std_msgs::MultiArrayDimension> mydim(2);;
    sendMat.layout.dim =  mydim;
    
    sendMat.layout.dim[0].label     = "size1";
    sendMat.layout.dim[0].size      = 4;
    sendMat.layout.dim[0].stride    = 16;
    
    sendMat.layout.dim[1].label     = "size2";
    sendMat.layout.dim[1].size      = 4;
    sendMat.layout.dim[1].stride    = 4;
    

    sendMat.layout.data_offset      = 0; 

    sendMat.data.assign(originalMat->begin<float>(), originalMat->end<float>());

    this->mpPublisher->publish(sendMat);

    
}

void ImageGrabber::PublishPose(cv::Mat* originalMat, ORB_SLAM3::System* mpSLAM)
{
    if (originalMat->dims == 0 )
        return;
        
    cv::Rect rotSel(0,0,3,3);
    cv::Rect transSel(3,0,1,3);
    
    cv::Mat trans = (*originalMat)(transSel);
    cv::Mat rot = (*originalMat)(rotSel);

  
    // fill msg header
    geometry_msgs::PoseStamped pStamped;
    pStamped.header.stamp = ros::Time::now(); // timestamp of creation of the msg
    pStamped.header.frame_id = "pose_cam_"+std::to_string(mpSLAM->getSysId()); // frame id 
    geometry_msgs::Pose pose_msg; 


    //extract Transformation
    tf2::Matrix3x3 tf2_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                       rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                       rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));
    
    tf2::Vector3 tf2_tran(trans.at<double>(0,0),trans.at<double>(1,0),trans.at<double>(2,0));
  
    tf2::Transform tf2_transform(tf2_rot, tf2_tran);
    

    // fill pose appropriately
    tf2::toMsg(tf2_transform, pose_msg);
    pStamped.pose = pose_msg;

    this->mpPublisher->publish(pStamped);

    
}

int main(int argc, char **argv)
{
    //argv[0] ... programm name char[]
    //argv[1] ... vocaluary BoW filepath
    //argv[2] ... cam config filepath
    //argv[3] ... number following of cam topics
    //argv[4] ... cam topic 1
    //argv[5] ... cam topic 2
    // ...
    // ..
    
    ros::init(argc, argv, "MultiMono");
    ros::start();
    cout << "Start MultiMono" << endl;
    if(argc < 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 MultiMono path_to_vocabulary path_to_settings number_cam_topics resource_topic_name" << endl;        
        ros::shutdown();
        return 1;
    }  
    // Create objects dependent on input 
    string vocabulary = argv[1];
    string camConfig  = argv[2];
    int nCams = stoi(argv[3]);

    if(argc != nCams+4)
    {
        cerr << endl << "number_cam_topics does not match with the number of input arguments " << endl;        
        ros::shutdown();
        return 1;
    }

    if(nCams > 8)
    {
        //limited by synchronizer which takes only 8 topics
        // TODO maybe drop synch
        cerr << endl << "number_cam_topics greater then 8 is not supported at the moment" << endl;        
        ros::shutdown();
        return 1;
    }
    ros::Publisher pub[nCams];
    ros::Subscriber sub[nCams];
    ORB_SLAM3::System* slamsSys[nCams];
    ImageGrabber*  imgGrabber[nCams];
    ros::NodeHandle nodeHandler;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    
    for(int iCam = 0; iCam < nCams; iCam++)
    {
        //create slam system for each cam topic
        if(iCam ==0)
            slamsSys[iCam] = new ORB_SLAM3::System(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
        else
            slamsSys[iCam] = new ORB_SLAM3::System(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false);

        //advertise camera pose topic for each image
        pub[iCam] = nodeHandler.advertise<geometry_msgs::PoseStamped>("pose_cam_"+std::to_string(iCam), 1);

        //create a image grabber for each cam topic
        imgGrabber[iCam] = new ImageGrabber(slamsSys[iCam],& pub[iCam]);

        //create a subscriber for each cam topic
        sub[iCam] = nodeHandler.subscribe(argv[4+iCam], 1, &ImageGrabber::GrabImage,imgGrabber[iCam]);
    }

    
    // publish transformation for each camera
    //ros::Publisher cam_pose_pub = n.advertise<std_msgs::>("", 1);

    //while true loop
    ros::spin();
    for (int iCam = 0; iCam < nCams; iCam++){

        // Stop all threads
        slamsSys[iCam]->Shutdown();

        // Save camera trajectory
        slamsSys[iCam]->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

        ros::shutdown();
    }

    return 0;
}


