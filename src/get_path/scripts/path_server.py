#!/usr/bin/env python
# coding:utf-8


import rospy
from get_path.srv import *
from sensor_msgs.msg import NavSatFix
#import baidu_planning_riding_func as bdplan  #baidu_map_planning
#经过百度地图和高德地图的对比，决定使用高德地图的步行路径规划
import gaode_planning_buxing_func as gdplan
def server_srv():
    rospy.init_node("path_server")
    s = rospy.Service("path_get_service", Pathget, handle_function) 
    #输出准备信息
    rospy.loginfo("Ready to handle the request:")
    # 阻塞程序结束
    rospy.spin()

# Define the handle function to handle the request inputs
def handle_function(req):
    rospy.loginfo( 'The origin point is %f,%f ;The destination point is %f,%f', \
                 req.origin_point[0],req.origin_point[1],req.destination_point[0],req.destination_point[1])

    fix_path = []
    
    result_path = gdplan.get_path(req.origin_point,req.destination_point)

    for point in result_path:
        fix_point = NavSatFix()#实例化一个navsatfix对象
        fix_point.longitude = point[1]
        fix_point.latitude = point[0]
        fix_path.append(fix_point)
    point_num = len(fix_path)
    print(fix_path)
    return PathgetResponse(fix_path,point_num)

# 如果单独运行此文件，则将上面定义的server_srv作为主函数运行
if __name__=="__main__":
    server_srv()
