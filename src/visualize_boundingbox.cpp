#include "multi_object_tracker/visualize_boundingbox.h"


BoundingBoxVisualizer::BoundingBoxVisualizer() {
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("bounding_boxes", 1);
        fusion_sub = nh.subscribe("/MultiObjectTracker", 1, &BoundingBoxVisualizer::fusion_callback, this);
        marker_id_pub = nh.advertise<visualization_msgs::MarkerArray>("bounding_boxes_id", 1);
        marker_trajectory_pub = nh.advertise<visualization_msgs::MarkerArray>("bounding_boxes_velocity", 1);
}
BoundingBoxVisualizer::~BoundingBoxVisualizer() {}

void BoundingBoxVisualizer::fusion_callback(const multi_object_tracker::PredictionObstacles::ConstPtr &fusion_objects){
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::MarkerArray id_array;
    visualization_msgs::MarkerArray trajectory_array;
    for (const auto& object : fusion_objects->prediction_obstacles) {
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.header.frame_id = "ego_vehicle/lidar";
        marker.action = visualization_msgs::Marker::ADD;
        marker.header.stamp = ros::Time::now();
        marker.id = object.perception_obstacle.id;
        if(object.perception_obstacle.polygon.points.size() > 8){//激光雷达
            marker.pose.orientation.w = 1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            for(size_t i = 8; i < object.perception_obstacle.polygon.points.size(); i++){
                geometry_msgs::Point bottom_point, top_point;
                bottom_point.x = object.perception_obstacle.polygon.points[i].x;
                bottom_point.y = object.perception_obstacle.polygon.points[i].y;
                bottom_point.z = object.perception_obstacle.polygon.points[i].z;

                top_point.x = bottom_point.x;
                top_point.y = bottom_point.y;
                top_point.z = bottom_point.z + object.perception_obstacle.height;
                if(i % 2 == 0){
                    marker.points.push_back(bottom_point);
                    marker.points.push_back(top_point);
                }
                else{
                    marker.points.push_back(top_point);
                    marker.points.push_back(bottom_point);
                }
            }
            for(size_t i = 8; i < object.perception_obstacle.polygon.points.size(); i++){
                geometry_msgs::Point bottom_point, top_point;
                bottom_point.x = object.perception_obstacle.polygon.points[i].x;
                bottom_point.y = object.perception_obstacle.polygon.points[i].y;
                bottom_point.z = object.perception_obstacle.polygon.points[i].z;

                top_point.x = bottom_point.x;
                top_point.y = bottom_point.y;
                top_point.z = bottom_point.z + object.perception_obstacle.height;
                if(i % 2 == 0){
                    marker.points.push_back(top_point);
                    marker.points.push_back(bottom_point);
                }
                else{
                    marker.points.push_back(bottom_point);
                    marker.points.push_back(top_point);
                }
            }
            marker.color.r = 0.2;
            marker.color.g = 0.6;
            marker.color.b = 1.0;
        } else if(object.perception_obstacle.polygon.points.size() == 5){
            marker.pose.orientation.w = 1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            for(size_t i = 0; i < object.perception_obstacle.polygon.points.size(); i++){
                geometry_msgs::Point bottom_point, top_point;
                bottom_point.x = object.perception_obstacle.polygon.points[i].x;
                bottom_point.y = object.perception_obstacle.polygon.points[i].y;
                bottom_point.z = object.perception_obstacle.polygon.points[i].z - object.perception_obstacle.height/2;

                top_point.x = bottom_point.x;
                top_point.y = bottom_point.y;
                top_point.z = bottom_point.z + object.perception_obstacle.height;
                if(i % 2 == 0){
                    marker.points.push_back(bottom_point);
                    marker.points.push_back(top_point);
                }
                else{
                    marker.points.push_back(top_point);
                    marker.points.push_back(bottom_point);
                }
            }
            for(size_t i = 0; i < object.perception_obstacle.polygon.points.size(); i++){
                geometry_msgs::Point bottom_point, top_point;
                bottom_point.x = object.perception_obstacle.polygon.points[i].x;
                bottom_point.y = object.perception_obstacle.polygon.points[i].y;
                bottom_point.z = object.perception_obstacle.polygon.points[i].z - object.perception_obstacle.height/2;

                top_point.x = bottom_point.x;
                top_point.y = bottom_point.y;
                top_point.z = bottom_point.z + object.perception_obstacle.height;
                if(i % 2 == 0){
                    marker.points.push_back(top_point);
                    marker.points.push_back(bottom_point);
                }
                else{
                    marker.points.push_back(bottom_point);
                    marker.points.push_back(top_point);
                }
            }
            marker.color.r = 0.3;
            marker.color.g = 1.0;
            marker.color.b = 0.6;

        } else {
            geometry_msgs::Point pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;
            tf2::Quaternion q;
            q.setValue(
                object.perception_obstacle.orientation2.x,
                object.perception_obstacle.orientation2.y,
                object.perception_obstacle.orientation2.z,
                object.perception_obstacle.orientation2.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            if (isnan(yaw)) {
                continue;
            }
            marker.pose.position = object.perception_obstacle.position;
            marker.pose.orientation = object.perception_obstacle.orientation2;
            pos1.x = object.perception_obstacle.length / 2;
            pos1.y = object.perception_obstacle.width / 2;
            pos1.z = object.perception_obstacle.height / 2;

            pos2.x = object.perception_obstacle.length / 2;
            pos2.y = object.perception_obstacle.width / 2;
            pos2.z = -object.perception_obstacle.height / 2;

            pos3.x = object.perception_obstacle.length / 2;
            pos3.y = -object.perception_obstacle.width / 2;
            pos3.z = -object.perception_obstacle.height / 2;

            pos4.x = object.perception_obstacle.length / 2;
            pos4.y = -object.perception_obstacle.width / 2;
            pos4.z = object.perception_obstacle.height / 2;

            pos5.x = -object.perception_obstacle.length / 2;
            pos5.y = -object.perception_obstacle.width / 2;
            pos5.z = object.perception_obstacle.height / 2;

            pos6.x = -object.perception_obstacle.length / 2;
            pos6.y = -object.perception_obstacle.width / 2;
            pos6.z = -object.perception_obstacle.height / 2;

            pos7.x = -object.perception_obstacle.length / 2;
            pos7.y = object.perception_obstacle.width / 2;
            pos7.z = -object.perception_obstacle.height / 2;

            pos8.x = -object.perception_obstacle.length / 2;
            pos8.y = object.perception_obstacle.width / 2;
            pos8.z = object.perception_obstacle.height / 2;
            marker.points.push_back(pos1);
            marker.points.push_back(pos2);
            marker.points.push_back(pos3);
            marker.points.push_back(pos4);
            marker.points.push_back(pos5);
            marker.points.push_back(pos6);
            marker.points.push_back(pos7);
            marker.points.push_back(pos8);
            marker.points.push_back(pos1);
            marker.points.push_back(pos4);
            marker.points.push_back(pos3);
            marker.points.push_back(pos6);
            marker.points.push_back(pos5);
            marker.points.push_back(pos8);
            marker.points.push_back(pos7);
            marker.points.push_back(pos2);
            marker.color.r = 1.0;
            marker.color.g = 0.1;
            marker.color.b = 0.7;



        }


        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.lifetime.fromSec(0.2);
        marker_array.markers.push_back(marker);

        visualization_msgs::Marker id_marker;
        id_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        id_marker.header.frame_id = "ego_vehicle/lidar";
        id_marker.action = visualization_msgs::Marker::ADD;
        id_marker.header.stamp = ros::Time::now();
        id_marker.id = object.perception_obstacle.id;
        id_marker.text = "ID:" + std::to_string(object.perception_obstacle.id) + "\n"
                         " Position X:" + std::to_string((float) object.perception_obstacle.position.x) + "\n"
                         " Position Y:" + std::to_string((float) object.perception_obstacle.position.y);
        id_marker.pose.position = object.perception_obstacle.position;
        id_marker.pose.orientation = object.perception_obstacle.orientation2;
        id_marker.scale.z = 1;
        id_marker.color.a = 1;
        id_marker.color.r = 1.0;
        id_marker.color.g = 1.0;
        id_marker.color.b = 1.0;
        id_marker.lifetime.fromSec(0.2);
        id_array.markers.push_back(id_marker);



        visualization_msgs::Marker trajectory;
        trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        trajectory.header.frame_id = "ego_vehicle/lidar";
        trajectory.action = visualization_msgs::Marker::ADD;
        trajectory.header.stamp = ros::Time::now();
        trajectory.id = object.perception_obstacle.id;
        trajectory.pose.orientation.w = 1.0;
        trajectory.pose.orientation.x = 0.0;
        trajectory.pose.orientation.y = 0.0;
        trajectory.pose.orientation.z = 0.0;
        trajectory.color.r = 1.0;
        trajectory.color.g = 1.0;
        trajectory.color.b = 0.0;
        trajectory.scale.x = 0.1;
        trajectory.color.a = 1.0;
        trajectory.lifetime.fromSec(0.2);
        for(auto trajectory_point : object.trajectory){
            geometry_msgs::Point traject;
            traject.x = trajectory_point.x;
            traject.y = trajectory_point.y;
            traject.z = object.perception_obstacle.position.z;
            trajectory.points.push_back(traject);
        }
        trajectory_array.markers.push_back(trajectory);

    }
    marker_pub.publish(marker_array);
    marker_id_pub.publish(id_array);
    marker_trajectory_pub.publish(trajectory_array);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bounding_box_visualizer");

    BoundingBoxVisualizer visualizer;

    ros::spin(); // Process callbacks

    return 0;
}
