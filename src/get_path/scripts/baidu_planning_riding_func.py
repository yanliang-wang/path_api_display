# -*- coding: utf-8 -*-
import time#time.sleep(s),可以延迟时间
import json
from urllib import urlopen
import urllib
 
def pathstr_to_pathlist(path_str):
    """将“long1,lat1;long2,lat2”转换为[[long1,lat1],[long2,lat2]] """
    path_list1 = path_str.split(";")  #[“long1,lat1","long2,lat2”]
    path_list = []
    for point in path_list1:
        point_list = point.split(",")
        point_list[0] = float(point_list[0]) #str转为float
        point_list[1] = float(point_list[1]) 
        path_list.append(point_list)
    return path_list

def get_path(origin_point,destination_point):
    """输入起始点，终止点坐标[lat,long]（wgs84格式），输出path路径点[[long1,lat1],[long2,lat2]]（gcj02格式）"""
    #输入为wgs84，输出为gcj02
    #使用的百度地图提供的骑行路径规划api
    #url_walk = r'http://api.map.baidu.com/direction/v2/walking?output=json'  # 步行路径规划api
    #测试点[纬度，精度]，输入为wgs84
    #origin_point = [41.6526785357,123.4134637847]
    #destination_point = [41.6543619048,123.4148370757]
    url_ride = r'http://api.map.baidu.com/direction/v2/riding?'
    cod =r'&coord_type=wgs84&ret_coordtype=gcj02'
    AK = 'RvkP7mOKTxBKR6wZYrk9AU1svHCGK291' #开发者秘钥
    ak_riding = r'&ak=' + AK
    origin = r"&origin=" + str(origin_point[0]) + ',' + str(origin_point[1])  # 起点
    destination = r"&destination=" + str(destination_point[0]) + ',' + str(destination_point[1])  # 终点
    aurl_riding = url_ride + origin + destination + cod + ak_riding  #驾车规划网址
    #print(aurl_riding) 输出URL，供调试程序

    res_drive = urlopen(aurl_riding)  #打开网页
    cet_drive = res_drive.read()  #解析内容
    res_drive.close()  #关闭
    cet_drive = cet_drive.decode(encoding='utf-8')#将bytes型转换为str型

    result_riding = json.loads(cet_drive)  # json转dict
    #print(result_riding) #输出json格式，供调试
    status = result_riding['status']
    #print('查询状态码', status)
    if status == 0:  # 状态码为0：无异常
        riding_routes = result_riding['result']['routes'][0] #包含道路信息的字典
        steps = riding_routes['steps']
        step_num = len(steps)
        print("the number of steps is %d" %step_num)
        first_flag = True
        path_result = []
        for step in steps:
            path = pathstr_to_pathlist(step['path'])
            if first_flag:
                path_result = path
                first_flag = False
            else:
                del(path[0])#下一个路径的起点 与上一个路径的重点重合，所以从第二段开始要删除路径的起点
                path_result = path_result +path
        print(path_result)
        print(type(path_result[0][0]))
    else:
        print("error\n")
    return path_result
