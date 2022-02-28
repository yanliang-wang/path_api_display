#!/usr/bin/env python
# coding:utf-8

#实现功能：
#1.订阅/move_base_simple/goal话题更新终点坐标
#2.订阅/fix话题，更新起点坐标
#3.调用/path_get_service服务，获取起点到终点的路径（gcj02表示）
#4.调用/get_ref_point服务，获取起点坐标（wgs84表示），将起点到终点的路径（gcj02表示）转换为wgs84表示
#然后将路径用ENU表示，然后通过/path_info话题将路径发出去
import rospy
from get_path.srv import *
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from geo import enu_to_geodetic,geodetic_to_enu
from coordTransform_utils import wgs84_to_gcj02,gcj02_to_wgs84
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix

def swap_eo(c):
    """互换列表内两个元素的位置"""
    return [c[1],c[0]]

def get_path_points():
    global destination_enu
    global origin_enu
    global pub
    global ref_point
    global origin_point

    """当参考点无效时，#获取参考坐标的位置，用于发布规划的路径"""
    if ref_point[0] > 999999999:
        try:
            ref_point_client = rospy.ServiceProxy("/get_ref_point", Refpointget)  # 创建（返回起点值）服务的client
            resp1 = ref_point_client.call()  # 调用服务，得到起点经纬度的wgs84表示
            ref_point = list(resp1.origin_point)  # 本来得到数据类型的是元组，(lat0,lon0,h0)
            rospy.loginfo("/get_ref_point Service call successfully")
        except rospy.ServiceException, e:
            rospy.logwarn("/get_ref_point Service call failed: %s" % e)

    # 将起点和终点的ENU坐标转换为wgs84表示的终点经纬度坐标
    destination_wgs84 = list(enu_to_geodetic(destination_enu[0], destination_enu[1], 0, \
                                             ref_point[0], ref_point[1], ref_point[2]))  # return lat, lon, h
    origin_wgs84 = list(enu_to_geodetic(origin_enu[0], origin_enu[1], 0, \
                                             ref_point[0], ref_point[1], ref_point[2]))  # return lat, lon, h
    # 将起点和终点的wgs84坐标转换为gcj02表示的经纬度坐标
    origin_gcj02 = swap_eo(wgs84_to_gcj02(origin_wgs84[1], origin_wgs84[0]))
    destination_gcj02 = swap_eo(wgs84_to_gcj02(destination_wgs84[1], destination_wgs84[0]))
    try:
        # ServiceProxy是rospy里面与client有关的函数,"Pathget"是服务的名称
        path_client = rospy.ServiceProxy("path_get_service", Pathget)
        resp2 = path_client.call(origin_gcj02, destination_gcj02)
        response_path = resp2.pathfeedback
        response_num = resp2.points_num
        rospy.loginfo("There is %d points in the path" % response_num)
        # print(response_path)
        poses_planning = []

        for i in response_path:
            # [lon,lat]
            pose_wgs84 = gcj02_to_wgs84(i.longitude, i.latitude)  # [lon,lat]

            [x1, y1, z1] = geodetic_to_enu(pose_wgs84[1], pose_wgs84[0], 0, ref_point[0], ref_point[1], ref_point[2])
            pose_planning = PoseStamped()
            pose_planning.pose.position.x = x1
            pose_planning.pose.position.y = y1
            pose_planning.pose.position.z = 0
            poses_planning.append(pose_planning)
        arrGps_planning = Path()
        arrGps_planning.header.stamp = rospy.Time.now()
        arrGps_planning.header.seq = 0;  # 序号
        arrGps_planning.header.frame_id = "map"
        arrGps_planning.poses = poses_planning
        # 发布path
        pub.publish(arrGps_planning)
        rospy.loginfo("published sucessfully")
    except rospy.ServiceException, e:
        rospy.logwarn("path_get_service Service call failed: %s" % e)

def get_destination(destination):
    """根据rviz的2D Nav Goal 得到的终点（ENU表示）"""
    #得到enu坐标，作为终点
    global destination_enu
    global origin_enu
    destination_enu = [destination.pose.position.x,destination.pose.position.y]

    if origin_enu[0] < 999999999 :
        get_path_points();

def get_origin(origin):
    global destination_enu
    global origin_enu
    origin_enu = [origin.pose.pose.position.x, origin.pose.pose.position.y]
    if destination_enu[0] < 999999999 :
        get_path_points();

if __name__=="__main__":
    destination_enu = [float("inf"),float("inf")]
    origin_enu =  [float("inf"),float("inf")]
    ref_point = [float("inf"),float("inf"),float("inf")]

    dflag = 0#终点位置是否更新的标志
    rflag = 0#起点位置是否更新的标志

    rospy.init_node('path_client', anonymous=True)
    
    #等待需要是要的服务
    rospy.loginfo("Waiting Service : /get_ref_point");
    rospy.wait_for_service("/get_ref_point")#阻塞等待，检测到有名为“path_get_service”的service后，才往下执行程序
    rospy.loginfo("Ready Service : /get_ref_point");

    rospy.loginfo("Waiting Service : /path_get_service");
    rospy.wait_for_service("/path_get_service")#阻塞等待，检测到有名为“path_get_service”的service后，才往下执行程序
    rospy.loginfo("Ready Service : /path_get_service");

    #Publisher函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
    pub = rospy.Publisher('/path_info', Path, queue_size=100)

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, get_destination)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, get_origin)
    
    rospy.spin()#spin()反复检查是否有消息并调用callback_client_srv