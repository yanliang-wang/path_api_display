# path_api_display
Provide path planning from Gaode or Baidu Map API.

## 1. Build

```
$ cd path_api_display
$ catkin_make
$ source devel/setup.bash
```

## 2. Demo

```
$ roslaunch get_path demo.launch
```

This project can output the route points given the start point and destination point.

In Rviz, you can set the start point by pressing `2D Pose Estimate`(whose shortcut is `p`) and set the destination point by pressing `2D Nav Goal`(whose shortcut is `g`).

<img src="./img/rviz_plan_demo.gif" style="zoom: 50%;" />

video: [Youtube link](https://youtu.be/Dpgqhs1Kx4U), [Bilibili link](https://www.bilibili.com/video/BV1fr4y1z7Ds/) 

## 3. Reference

1. [rviz_satellite](https://github.com/gareth-cross/rviz_satellite), which is a rviz plugin that displays the map in rviz according to the tile map URL.(use `1e7b2a7` commit)
3. [coordTransform_py](https://github.com/wandergis/coordTransform_py), which is a tool provides the transformation between gcj02, bd09 and wgs84.

## 4. Attention

In order to facilitate your testing for this repository, I used the API key I applied on the official website. If you want to use this project, please apply for the API yourself.