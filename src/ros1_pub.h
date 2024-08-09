#ifndef ROS1_PUB_H
#define ROS1_PUB_H

#include <ros/time.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Geometry>
#include "MultiObjectTracker/PredictionObstacles.h"
#include "MultiObjectTracker/PredictionObstacle.h"
#include "MultiObjectTracker/PerceptionObstacle.h"
#include "MultiObjectTracker/TrajectoryPoint.h"
#include "MultiObjectTracker/DetectedObjectArray.h"
#include "MultiObjectTracker/ObjectList.h"
#include <visualization_msgs/MarkerArray.h>
#include "objecttracker.h"
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "MultiObjectTracker/Localization.h"
#include "MultiObjectTracker/Ins.h"
#include <time.h>

enum {
  POINT,
  CAR,
  TRUCK,
  PEDESTRIAN,
  MOTORCYCLE,
  BICYCLE,
  WIDE,
  RESERVED
};

enum {
  INVALID,
  PERCENT_25,
  PERCENT_50,
  PERCENT_75,
  PERCENT_90,
  PERCENT_99,
  PERCENT_99_9,
  PERCENT_100
};

class fusion{


    private:

    ros::NodeHandle nh_;
    ros::Publisher fusion_pub;
    ros::Subscriber lidar_sub;
    ros::Subscriber radar_sub;
    ros::Subscriber odom_sub;
    ros::Timer timer_;
    ObjectTracker tracker;
    Eigen::Matrix4d transform_matrix;//转换位置
    Eigen::Matrix4d transform_velocity;//转换速度
    Eigen::Matrix3d rotation_matrix;//转换速度
    Eigen::Quaterniond car_orientation;//转换姿态


    void lidar_callback (const MultiObjectTracker::DetectedObjectArray::ConstPtr &lidar_objects);
    void radar_callback (const MultiObjectTracker::ObjectList::ConstPtr &radar_objects);
    void odom_callback (const MultiObjectTracker::Localization::ConstPtr &odom);
    void timerCallback(const ros::TimerEvent& event);
    std::mutex msgs_mutex;


    std::mutex queue_mutex;
    bool stop_thread;
    double timestamp;
    double vehicle_x;
    double vehicle_y;
    double vehicle_z;
    double vehicle_yaw;
    double vehicle_v;


    public:

    fusion();
    ~fusion();
    void run();
    void process_data(const std::vector<BoundingBox3D> object_boxes);

};

#endif