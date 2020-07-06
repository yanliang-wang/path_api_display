# get_path

通过地图路径规划API，输入起点和终点的经纬度坐标，返回规划的路径

## 函数功能

### overview

- `coordTransform_utils.py`  含有gcj02,bd09,wgs84坐标系之间的转换
- `geo.py` 含有ENU ecef geodetic坐标系之间的转换
- `baidu_planning_riding_func.py `  通过**百度地图骑行**路径规划API来提供路径
- `gaode_planning_buxing_func.py`  通过**高德地图步行**路径规划API来提供路径
- `gaode_planning_riding_func.py`  通过**高德地图骑行**路径规划API来提供路径
- `path_client.py` 用来根据rivz里面的指定的点和设置好的起点，调用path_server的服务，生成规划轨迹，并以话题方式发布
- `path_server.py` 用来提供高德地图的步行路径规划服务

### 详细介绍

- `path_server.py`    提供服务/path_get_service服务，根据输入的起点和终点，返回起点到终点的路径（gcj02表示）
- `path_client.py`
  1. 订阅/move_base_simple/goal话题更新终点坐标
  2. 订阅/fix话题，更新起点坐标
  3. 调用/path_get_service服务，获取起点到终点的路径（gcj02表示）
  4. 调用/get_ref_point服务，获取起点坐标（wgs84表示），将起点到终点的路径（gcj02表示）转换为wgs84表示

- `baidu_planning_riding_func.py`    调用百度地图骑行规划的函数

- `gaode_planning_buxing_func.py`    调用高德地图步行规划的函数

- `gaode_planning_riding_func.py`    调用高德地图骑行规划的函数

- `geo.py`

  1. geodetic_to_ecef(lat, lon, h):地理坐标转换为大地坐标系
  2. ecef_to_geodetic(x, y, z):大地坐标系转换为大地坐标系
  3. ecef_to_enu(x, y, z, lat0, lon0, h0):大地坐标系转换为ENU坐标
  4. enu_to_ecef(xEast, yNorth, zUp, lat0, lon0, h0):ENU坐标转换为大地坐标系
  5. geodetic_to_enu(lat, lon, h, lat_ref, lon_ref, h_ref):地理坐标转换为ENU坐标
  6. enu_to_geodetic(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref):ENU坐标转换为地理坐标

- `coordTransform_utils.py`

  gcj02,bd09,wgs84坐标系之间的转换

  ```
  gcj02_to_bd09(lng, lat)
  bd09_to_gcj02(bd_lon, bd_lat)
  wgs84_to_gcj02(lng, lat)
  gcj02_to_wgs84(lng, lat)
  bd09_to_wgs84(bd_lon, bd_lat)
  wgs84_to_bd09(lon, lat)
  out_of_china(lng, lat) #判断坐标是否在中国
  ```

  

  