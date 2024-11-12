#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H


#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include "Eigen/Dense"
#include "ukf.h"
#include "Hungarian.h"
#include <mutex>
#include "IDManager.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// 障碍物结构体
struct BoundingBox3D {
    double x;  
    double y;  
    double z;  
    double length;  
    double width;   
    double height;  
    double orientation_angle_x;
    double orientation_angle_y;
    double orientation_angle_z;
    double orientation_angle_w;      
    int label;  //"person", "car", "truck", "bus", "motorcycle", "bicycle", "other"
    double confidence;  
    double velocity_x;  
    double velocity_y;
    vector<Eigen::Vector3d> polygon;  
    enum Sensor{
        LIDAR,
        RADAR,
        RADAR_4D
    } sensor; 
};

// 跟踪目标结构体
struct Track {
    BoundingBox3D Box;
    UKF ukf;
    int id;
    int missed_count; // 记录未检测到的次数
    int birth_count; // 匹配到记录次数才会发布
    bool is_pub;
    bool radar_associated;
    bool lidar_associated;
};

// 用于跟踪的管理类
class ObjectTracker {
public:
    ObjectTracker();
    void ProcessMeasurement(const vector<BoundingBox3D>& detections);
    vector<Track> tracks_; //在匹配池中处理

private:

    IDManager id_manager_;    
    const float distance_threshold_;
    int max_missed_count_; // 最大未检测到次数
    int min_birth_count_; // 最小匹配创建次数
    void AssociateDetectionsToTracks(const vector<BoundingBox3D>& detections, vector<int>& assignments,  vector<Track>& tracks);
    void CreateNewTracks(const vector<BoundingBox3D>& detections, vector<Track>& tracks);
    void RemoveDeadTracks(vector<Track>& tracks);
    Eigen::Quaterniond yawToQuaternion(double yaw);
    std::vector<Eigen::Vector3d> calculateRadarPolygon(const BoundingBox3D& obstacle);
    std::vector<Eigen::Vector3d> calculateLidarPolygon(const BoundingBox3D& obstacle);
};

#endif