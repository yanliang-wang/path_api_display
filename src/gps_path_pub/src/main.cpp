#include <iostream>
#include <ros/ros.h>//
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>//
#include <geometry_msgs/PoseStamped.h>//
#include <tf/transform_broadcaster.h>//
#include <GPSConversion.h>

#define REF_POINT_GET
#ifdef REF_POINT_GET

#include <get_path/Refpointget.h>

#endif

using namespace std;
/*
四个功能：
1.订阅/fix，获取wgs84表示的坐标位置
2.发布wgs84和gcj02表示的起点坐标，gcj02表示的实时更新坐标，路径信息
3.发布gps，map坐标系的tf关系
4.提供 返回wgs84表示的起点坐标的服务
*/

//全局变量定义
/*
定义四个话题发布器: pub_gpsref_gcj02, pub_gps_gcj02,pub_gpsref_wgs84 , pub_hist
pub_gpsref_gcj02 发布gcj02格式的参考坐标（起点坐标）
pub_gps_gcj02    发布gcj02格式的坐标（实时更新的坐标）
pub_gpsref_wgs84 发布wgs84格式的参考坐标（起点坐标）
pub_hist    发布机器人移动的路径（实时更新）
*/
ros::Publisher pub_gpsref_gcj02,  pub_gps_gcj02,  pub_gpsref_wgs84,  pub_hist;

/*
定义pub_gpsref_gcj02, pub_gps_gcj02,pub_gpsref_wgs84 , pub_hist发布的消息
消息类型为sensor_msgs/NavSatFix，nav_msgs/Path
*/
sensor_msgs::NavSatFix gpsref_gcj02;
sensor_msgs::NavSatFix gps_gcj02;
sensor_msgs::NavSatFix gpsref_wgs84;
nav_msgs::Path arrGps;

//定义广播map和gps坐标系关系的广播器
tf::TransformBroadcaster *mTfBr;

// 处理 “服务收到的信息” 的回调函数，service callback
#ifdef REF_POINT_GET

bool refpointCallback(get_path::Refpointget::Request& reque,get_path::Refpointget::Response& respo)
{  //返回起点的wgs84坐标

    //输出服务调用成功信息
    static int num = 0;
    num++;
    ROS_INFO("Service is being called %dth",num);

    //起点坐标
    respo.origin_point = {gpsref_wgs84.latitude,gpsref_wgs84.longitude,gpsref_wgs84.altitude};
    
    //输出服务调用成功信息
    ROS_INFO("Service has been called %dth sucessfully",num);
    return true;
}

#endif

// 处理 “订阅的gps位置信息” 的回调函数，topic callback
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

int main(int argc, char** argv) {
    // 输出节点启动信息
    ROS_INFO("Starting up");
    // 初始化节点名称为：gps_path_pub
    ros::init(argc, argv, "gps_path_pub");
    //实例化一个节点对象
    ros::NodeHandle nh;

    //定义话题的订阅器，并声明订阅的话题名称为：/fix ，已经回调函数
    ros::Subscriber sub_gps = nh.subscribe("/fix", 10, gpsCallback);
    // 输出订阅话题的名称
    ROS_INFO("Subscribed: %s", sub_gps.getTopic().c_str());

    //定义话题的广播器，sensor_msgs/NavSatFix是广播消息的类型，
    pub_gpsref_gcj02 = nh.advertise<sensor_msgs::NavSatFix>("/gps_path_pub/gpsref_gcj02", 2);
    ROS_INFO("Publishing: %s", pub_gpsref_gcj02.getTopic().c_str());
    pub_gps_gcj02 = nh.advertise<sensor_msgs::NavSatFix>("/gps_path_pub/gps_gcj02", 2);
    ROS_INFO("Publishing: %s", pub_gps_gcj02.getTopic().c_str());
    pub_gpsref_wgs84 = nh.advertise<sensor_msgs::NavSatFix>("/gps_path_pub/gpsref_wgs84", 2);
    ROS_INFO("Publishing: %s", pub_gpsref_wgs84.getTopic().c_str());
    pub_hist = nh.advertise<nav_msgs::Path>("/gps_path_pub/line", 2);
    ROS_INFO("Publishing: %s", pub_hist.getTopic().c_str());

    //定义提供的服务，并声明名称为：get_ref_point，以及回调函数
    #ifdef REF_POINT_GET
        ros::ServiceServer service= nh.advertiseService("get_ref_point",refpointCallback);
        ROS_INFO("Service: %s",service.getService().c_str());
    #endif
    // 申请一个tf::TransformBroadcaster型的地址空间
    mTfBr = new tf::TransformBroadcaster();

    // 输出等待信息
    ROS_INFO("Ready. Waiting for messages...");

    // ROS spin
    ros::spin();

    // 输出推出信息
    ROS_INFO("DONE EXITING");
    return EXIT_SUCCESS;

}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
/*
根据得到的wgs84格式的坐标，做两件事
一是将wgs84坐标转换为gcj02的坐标，并发布
二是将gcj84坐标点集转换为ENU坐标表示，进而获取路径，并发布
*/



    //定义接gps_wgs84保存接收到的位置
    sensor_msgs::NavSatFix gps_wgs84;

    //坐标位置的标号
    unsigned int poses_seq = 0;
    //判断起始位置的标志
    static bool is_first = true;

    // 时刻更新接受的位置信息到gps_wgs84
    gps_wgs84 = *msg; // 从话题/fix订阅的位置坐标格式是wgs84的
    double after_longitude=(*msg).longitude;
    double after_latitude=(*msg).latitude;    

    //after_longitude，after_latitude初始为wgs84型的坐标，经过wgs84_to_gcj02函数（函数传入为引用）更改为gcj02坐标
    GPSConversion::wgs84_to_gcj02(after_longitude,after_latitude);

    //初始位置
    if(is_first) {
        gpsref_gcj02 = gps_wgs84;
        gpsref_gcj02.longitude=after_longitude;
        gpsref_gcj02.latitude=after_latitude;

        gpsref_wgs84 = gps_wgs84;
        is_first = false;
        return;
    }

    //gcj02坐标的更新
    gps_gcj02 = gps_wgs84;
    gps_gcj02.longitude=after_longitude;
    gps_gcj02.latitude=after_latitude;


    //将经纬度坐标转换为 相对于起始点的ENU坐标
    //定义path下的poses成分，类型为包含多个geometry_msgs/PoseStamped类型信息的数组
    static vector<geometry_msgs::PoseStamped> poses;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";//pose坐标系是map
    //将经纬度坐标转换为 相对于起始点的ENU坐标，并保存在pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
    GPSConversion::GeodeticToEnu(gps_wgs84.latitude, gps_wgs84.longitude, gps_wgs84.altitude,
                                 gpsref_wgs84.latitude, gpsref_wgs84.longitude, gpsref_wgs84.altitude,
                                 pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    // 把pose加到poses数组的最后
    poses.push_back(pose);
    // 实例化一个 path 对象并对其赋值
    arrGps.header.stamp = ros::Time::now();
    arrGps.header.seq = poses_seq;//序号
    arrGps.header.frame_id = "map";
    arrGps.poses = poses;

    /*
    发布gps，map坐标系的关系，每订阅到一个位置信息更新一次
    */ 
    tf::StampedTransform tfLinM;
    tfLinM.stamp_ = ros::Time::now();
    tfLinM.frame_id_ = "map";
    tfLinM.child_frame_id_ = "gps";
    tf::Quaternion quat(0,0,0,1);
    tfLinM.setRotation(quat);
    tf::Vector3 orig(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    tfLinM.setOrigin(orig);
    mTfBr->sendTransform(tfLinM);
    //广播四个话题
    pub_gpsref_gcj02.publish(gpsref_gcj02);
    pub_gps_gcj02.publish(gps_gcj02);
    pub_gpsref_wgs84.publish(gpsref_wgs84);
    pub_hist.publish(arrGps);

    // Move forward
    poses_seq++;
    
    // 每接收到100个位置坐标，输出一个消息
    if(poses_seq%100==0) {
        ROS_INFO("Published %d GPS path messages",poses_seq);
    }
}
