float64 timestamp

PredictionObstacle[] prediction_obstacles

# verbatim (language="comment",  text="感知处理时刻对应的车辆位置")
geometry_msgs/Point vehicle_position

# verbatim (language="comment",  text="感知处理时刻对应的车辆姿态四元数 欧拉角和四元数坐标变换顺序为ZYX")
#只給yaw，給到w
geometry_msgs/Quaternion vehicle_quaternion



