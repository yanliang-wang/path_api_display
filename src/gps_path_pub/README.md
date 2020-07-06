# google瓦片地图

## Options

- `Topic` is the topic of the GPS measurements.
- `Alpha` is simply the display transparency.
- `Draw Under` will cause the map to be displayed below all other geometry.
- `Zoom` is the zoom level of the map. Recommended values are 16-19, as anything smaller is *very* low resolution. 22 is the current max.
- `Blocks` number of adjacent blocks to load. rviz_satellite will load the central block, and this many blocks around the center. 8 is the current max.

- ```
  URL : http://mt2.google.cn/vt/lyrs=y&scale=2&hl=zh-CN&gl=cn&x={x}&y={y}&z={z} //含标注,卫星地图
  
  lyrs参数说明：
  
  h 街道图 //道路为白色
  m 街道图 //普通色
  p 街道图 //普通色
  r 街道图 //普通色  详细图
  s 影像无标注
  t 地形图
  y 影像含标注
  ```


不同的lyrs参数的显示结果在../../png中



更多瓦片地图资源见：[瓦片地图服务在线资源访问总结](https://www.cnblogs.com/HandyLi/p/11137367.html)

