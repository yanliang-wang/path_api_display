# -*- coding: utf-8 -*-
import json
from urllib import urlopen
 
def pathstr_to_pathlist(path_str):
    """将字符串表示的“long1,lat1;long2,lat2”转换为[[long1,lat1],[long2,lat2]] """
    path_list1 = path_str.split(";")  #[“long1,lat1","long2,lat2”]
    path_list = []
    for point in path_list1:
        point_list = point.split(",")
        mid_var = float(point_list[0]) #str转为float
        point_list[0] = float(point_list[1]) 
        point_list[1] = mid_var
        path_list.append(point_list)
    return path_list

def get_path(origin_point,destination_point):
    """输入起始点，终止点坐标[lat,long]（gcj02格式），输出path路径点[[long1,lat1],[long2,lat2]]（gcj02格式）"""
    #输入为gcj02，输出为gcj02
    #使用的高德提供的步行路径规划api
    #url_walk = r'https://restapi.amap.com/v3/direction/walking?origin=116.434307,39.90909&destination=116.434446,39.90816&key=<用户的key>'  # 步行路径规划api
    #测试点[纬度，精度]，输入为wgs84
    #origin_point = [41.6526785357,123.4134637847]
    #destination_point = [41.6543619048,123.4148370757]
    url_ride = r'http://restapi.amap.com/v3/direction/walking?'
    key = '99efee3f5c01bbb737193667a7e420dd' #开发者秘钥
    ak_walking = r'&key=' + key
    origin = r"origin=" + str(origin_point[1]) + ',' + str(origin_point[0])  # 起点
    destination = r"&destination=" + str(destination_point[1]) + ',' + str(destination_point[0])  # 终点
    aurl_walking = url_ride + origin + destination + ak_walking  #驾车规划网址
    #print(aurl_walking) 输出URL，供调试程序

    res_walking = urlopen(aurl_walking)  #打开网页
    cet_walking = res_walking.read()  #解析内容
    res_walking.close()  #关闭

    result_riding = json.loads(cet_walking)  # json转dict
    #print(result_riding) #输出json格式，供调试
    status = result_riding['status']
    #print('查询状态码', status)
    path_result = []
    #print(type(status))
    if status == '1':  # 状态码为1：成功 ，0：失败
        riding_routes = result_riding['route']['paths'][0] #包含道路信息的字典
        steps = riding_routes['steps']
        step_num = len(steps) #显示路径分为几段
        #print("the number of steps is %d" %step_num)
        first_flag = True

        for step in steps:
            path = pathstr_to_pathlist(step['polyline'])
            if first_flag:
                path_result = path
                first_flag = False
            else:
                del(path[0])#下一个路径的起点 与上一个路径的重点重合，所以从第二段开始要删除路径的起点
                path_result = path_result +path
        #print(path_result)
    else:
        print("error\n")
    return path_result
