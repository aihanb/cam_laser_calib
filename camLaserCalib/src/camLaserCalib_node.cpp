#include <ros/ros.h>
#include "camLaserCalib.h"
#include <dynamic_reconfigure/server.h>
#include <cam_laser_calib/CamLaserCalibConfig.h>
#include "std_msgs/String.h"
#include <iostream>
using namespace message_filters;
using namespace sensor_msgs;
using namespace std;
Parameter param;

void Callback(cam_laser_calib::CamLaserCalibConfig &config, uint32_t level)
{
    param.x_min           = config.x_min;
    param.x_max           = config.x_max;
    param.y_min           = config.y_min;
    param.y_max           = config.y_max;
    param.z_min           = config.z_min;
    param.z_max           = config.z_max;
}

void chatterCallback1(const sensor_msgs::ImageConstPtr& img)
{
    ROS_INFO("img");
    //ROS_INFO_STREAM("Image received at " << img->header.stamp.toSec());
    ROS_INFO("Camera Timestamp= %i.%i", img->header.stamp.sec, img->header.stamp.nsec);
}

void chatterCallback2(const sensor_msgs::PointCloud2ConstPtr& pc2)
{
    ROS_INFO("pc2");
    //ROS_INFO_STREAM("PointCloud2 received at " << pc2->header.stamp.toSec());
    ROS_INFO("LiDAR Timestamp= %i.%i",  pc2->header.stamp.sec, pc2->header.stamp.nsec);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cam_laser_calib");
    //cout<<"argc: "<<argc<<endl;
    //cout<<"argv[1]: "<<argv[1]<<endl;
    //cout<<"argv[2]: "<<argv[2]<<endl;
    if(argc!=3)
    {
        cerr<<endl<<"Usage: rosrun cam_laser_calib cam_laser_calib_node path_to_pointsFile path_to_calibFile";
        ros::shutdown();
        return 1;
    }
    ros::NodeHandle nh;

    camLaserCalib::CamLaserCalib CamLaserCalib(nh,argv[1],argv[2]);
    /*************************** test timestamp ***********************************/
    //ros::Subscriber sub = nh.subscribe("/camera/image_color", 1, chatterCallback1);
    //ros::Subscriber sub1 = nh.subscribe("/pandar_points", 1, chatterCallback2);
    //ros::Subscriber sub1 = nh.subscribe("/velodyne_points", 1000, chatterCallback2);
    /*************************** message filter ***********************************/
    message_filters::Subscriber<PointCloud2> laser40(nh, CamLaserCalib.strSub_pc2_.c_str(), 1);
    //message_filters::Subscriber<PointCloud2> laser40(nh,"pandar_points",100);
    message_filters::Subscriber<Image> img(nh, CamLaserCalib.strSub_img_.c_str() , 1);
    //message_filters::Subscriber<Image> img(nh,"camera/image_color",100);
    typedef sync_policies::ApproximateTime<Image, PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img, laser40);
    //ROS_INFO("Ready to recieve messages");
    sync.registerCallback(boost::bind(&camLaserCalib::CamLaserCalib::img_pc2_Callback, &CamLaserCalib, _1, _2));
    //sync.registerCallback(boost::bind(&Callback1, _1, _2));
       
    /*************************** dynamic_reconfigure ***********************************/
    dynamic_reconfigure::Server<cam_laser_calib::CamLaserCalibConfig> server;
    dynamic_reconfigure::Server<cam_laser_calib::CamLaserCalibConfig>::CallbackType f;
    f = boost::bind(&Callback, _1, _2);
    server.setCallback(f);
    ros::Rate loop(10);
    while(ros::ok())
    {
        ros::spinOnce();
        CamLaserCalib.cfgCallback(param);
    }

    return 0;
}
