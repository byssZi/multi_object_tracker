#ifndef VISUALIZE_BOUNDINGBOX_H
#define VISUALIZE_BOUNDINGBOX_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include "ros1_pub.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
class BoundingBoxVisualizer {
public:
    BoundingBoxVisualizer();
    ~BoundingBoxVisualizer(); 

    void fusion_callback(const MultiObjectTracker::PredictionObstacles::ConstPtr &fusion_objects);

private:
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    ros::Publisher marker_id_pub;
    ros::Publisher marker_trajectory_pub;
    ros::Subscriber fusion_sub;
};
#endif