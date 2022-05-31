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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <opencv2/opencv.hpp>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Imu.h>


#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "../../../include/System.h"

#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;

class ImageGrabber
{
public:

    ImageGrabber(ORB_SLAM3::System* pSLAM,ros::Publisher* pPublisher,message_filters::Cache<sensor_msgs::Imu>* imu_cache):mpSLAM(pSLAM),mpPublisher(pPublisher),mp_imu_cache(imu_cache),lastTime(ros::Time::now()){}
    vector<ORB_SLAM3::IMU::Point> GetIMUData(const ros::Time& start, const ros::Time& end);
    ORB_SLAM3::IMU::Point convIMUData(const sensor_msgs::ImuConstPtr &ROSimu);
    void PublishPose(cv::Mat* originalMat, ORB_SLAM3::System* mpSLAM);
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;

    ros::Time lastTime;
    ros::Publisher* mpPublisher;
    message_filters::Cache<sensor_msgs::Imu>* mp_imu_cache;
};
const vector<ORB_SLAM3::IMU::Point>& vImuMeas = vector<ORB_SLAM3::IMU::Point>();
string filename="rgbd_imu";
int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Inertial");
    ros::start();

    if(argc != 7)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD_Inertial path_to_vocabulary path_to_settings rgb_topic d_topic imu_topic map_file_name" << endl;        
        ros::shutdown();
        return 1;
    }    

    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_RGBD,true,0, argv[6]);

   

    ros::NodeHandle nh;
    //advertise camera pose topic for each image
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pose_cam_0", 1);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, argv[5], 5);
    message_filters::Cache<sensor_msgs::Imu> imu_cache(imu_sub, 1000);
    ImageGrabber igb(&SLAM,&pub,&imu_cache);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, argv[3], 5);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, argv[4], 5);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(40), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));


    //ros::Subscriber sub_imu = n.subscribe(argv[4], 1000, &ImuGrabber::GrabImu, &imugb); 

    

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}


cv::Mat shiftFrame(cv::Mat frame, int xpixels, int ypixels)
{
    //create a same sized temporary Mat with all the pixels flagged as invalid (-1)
    cv::Mat temp    = cv::Mat::zeros(frame.size(), frame.type());
    int newWidth    = frame.cols - std::abs(xpixels);
    int newHeight   = frame.rows - std::abs(ypixels);

    cv::Rect srcArea(0,0,newWidth,newHeight);
    cv::Rect dstArea(srcArea);

    if (ypixels > 0){
        dstArea.y  = std::abs(ypixels);
    }else{
        srcArea.y   = std::abs(ypixels);
    }

    if (xpixels > 0){
        dstArea.x  = std::abs(xpixels);
    }else{
        srcArea.x   = std::abs(xpixels);
    }

    frame(srcArea).copyTo(temp(dstArea));
    
    return temp;
}

/*converts ros msg to imu ORB class*/
ORB_SLAM3::IMU::Point ImageGrabber::convIMUData(const sensor_msgs::ImuConstPtr &ROSimu)
{   
    double t = ROSimu->header.stamp.toSec();
    cv::Point3f acc(ROSimu->linear_acceleration.x, ROSimu->linear_acceleration.y, ROSimu->linear_acceleration.z);
    cv::Point3f gyr(ROSimu->angular_velocity.x, ROSimu->angular_velocity.y, ROSimu->angular_velocity.z);
    return(ORB_SLAM3::IMU::Point(acc,gyr,t));
}

vector<ORB_SLAM3::IMU::Point> ImageGrabber::GetIMUData(const ros::Time& start, const ros::Time& stop)
{
    std::vector<sensor_msgs::ImuConstPtr> rosImuMsgs;
    rosImuMsgs = this->mp_imu_cache->getInterval(start,stop);

    vector<ORB_SLAM3::IMU::Point> orbImuMeasures;

    for (sensor_msgs::ImuConstPtr element :rosImuMsgs)
    {   ORB_SLAM3::IMU::Point orbImuMeas(convIMUData(element));
        orbImuMeasures.push_back(orbImuMeas);
    }
    return (orbImuMeasures);
    
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

    // get rgbd data
    cv::Size rgbSize    = cv_ptrRGB->image.size();
    cv::Mat dImg        = cv::Mat(rgbSize,cv_ptrRGB->image.type());
    cv::resize(cv_ptrD->image,dImg,rgbSize);
   
    // get all imu data between last and recent time 
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    vImuMeas.clear();
    vImuMeas            = this->GetIMUData(this->lastTime,cv_ptrRGB->header.stamp);
    cout << to_string(vImuMeas.size()) << endl;
    cout<< to_string(this->lastTime.toSec())<<endl;
    // update last message time stamp
    this->lastTime      = cv_ptrRGB->header.stamp;
    cout<< to_string(this->lastTime.toSec())<<endl;
    //get actuall transform 
    cv::Mat TCam;
    Sophus::SE3f Tcw_SE3f = mpSLAM->TrackRGBD(cv_ptrRGB->image,dImg,cv_ptrRGB->header.stamp.toSec(),vImuMeas,filename);
    Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
    cv::eigen2cv(Tcw_Matrix, TCam); 
    
    dImg.release();
    if (TCam.empty())
        return;
    
    this->PublishPose(&TCam,this->mpSLAM);   
    TCam.release();
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
