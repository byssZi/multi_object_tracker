# MultiObjectTracker
ros下基于匈牙利匹配算法，CTRV模型的UKF算法，融合激光雷达聚类结果，ARS408毫米波雷达object结果的3D多目标融合跟踪
卡尔曼滤波实现来源于项目 https://github.com/omerwase/LIDAR-Radar-Sensor-Fusion-UKF
匈牙利匹配实现来源于项目 https://github.com/mcximing/hungarian-algorithm-cpp
# Step1
```bash
catkin_make
source devel/setup.bash
```

# Step2
```bash
roslaunch MultiObjectTracker run.launch
```
