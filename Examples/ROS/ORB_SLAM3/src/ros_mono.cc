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
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM,ros::Publisher* pPublisher):mpSLAM(pSLAM),mpPublisher(pPublisher){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void PublishPose(cv::Mat* originalMat);
    ORB_SLAM3::System* mpSLAM;
    ros::Publisher* mpPublisher;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    cout << endl <<"argc = " << argc;
    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings resource_topic_name" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle nodeHandler;
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false);
    ros::Publisher pub = nodeHandler.advertise<std_msgs::Float32MultiArray>(argv[4], 1);
    ImageGrabber igb(&SLAM,&pub);
    ros::Subscriber sub = nodeHandler.subscribe(argv[3], 1, &ImageGrabber::GrabImage,&igb);

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
    cv::Mat TCam;
    TCam = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    this->PublishPose(&TCam);
    //cout << TCam.data << endl;
}

void ImageGrabber::PublishPose(cv::Mat* originalMat)
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


