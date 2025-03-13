#include "multi_object_tracker/ros1_pub.h"


fusion::fusion() : stop_thread(false){
    transform_matrix = Eigen::Matrix4d::Identity();
    transform_velocity = Eigen::Matrix4d::Identity();
    rotation_matrix = Eigen::Matrix3d::Identity();
    car_orientation.x() = 0;
    car_orientation.y() = 0;
    car_orientation.z() = 0;
    car_orientation.w() = 1;
    timestamp = 0;
    vehicle_x = 0;
    vehicle_y = 0;
    vehicle_z = 0;
    vehicle_v = 0;
    vehicle_velocity_x = 0;
    vehicle_velocity_y = 0;
}

fusion::~fusion() {

}

void fusion::run(){
    lidar_sub = nh_.subscribe("/autoware_tracker/cluster/objects", 1, &fusion::lidar_callback, this);
    radar_sub = nh_.subscribe("/radar/dbscan_bbox", 1, &fusion::radar_callback, this);
    odom_sub = nh_.subscribe("/odomData", 1, &fusion::odom_callback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &fusion::timerCallback, this);
    fusion_pub = nh_.advertise<multi_object_tracker::PredictionObstacles>("/MultiObjectTracker",1);
}

void fusion::odom_callback(const multi_object_tracker::Localization::ConstPtr &odom){

    Eigen::Quaterniond q;
    q.w() = odom->location.pose.pose.orientation.w;//右前上变换到UTM
    q.x() = odom->location.pose.pose.orientation.x;
    q.y() = odom->location.pose.pose.orientation.y;
    q.z() = odom->location.pose.pose.orientation.z;
    q.normalize();
    Eigen::Matrix3d rotation_matrix(q);

    transform_matrix(0, 3) = odom->location.pose.pose.position.x;//右前上变换到UTM
    transform_matrix(1, 3) = odom->location.pose.pose.position.y;
    transform_matrix(2, 3) = 0;
    transform_matrix.block<3, 3>(0, 0) = rotation_matrix;

    car_orientation = q;

    vehicle_x = odom->location.pose.pose.position.x;
    vehicle_y = odom->location.pose.pose.position.y;
    vehicle_z = 0;
    vehicle_v = odom->original_ins.ground_speed;

    vehicle_velocity_x = odom->original_ins.east_speed;//这里是utm下的东向速度了
    vehicle_velocity_y = odom->original_ins.north_speed; //这里是utm下的北向速度了

    transform_velocity(0, 3) = vehicle_velocity_x;
    transform_velocity(1, 3) = vehicle_velocity_y;
    transform_velocity(2, 3) = 0;
    transform_velocity.block<3, 3>(0, 0) = rotation_matrix;

}

void fusion::timerCallback(const ros::TimerEvent& event){
    multi_object_tracker::PredictionObstacles tracker_objects;
    tracker_objects.timestamp = timestamp;
    tracker_objects.vehicle_position.x = vehicle_x;
    tracker_objects.vehicle_position.y = vehicle_y;
    tracker_objects.vehicle_position.z = vehicle_z;
    tracker_objects.vehicle_quaternion.w = car_orientation.w();
    tracker_objects.vehicle_quaternion.x = car_orientation.x();
    tracker_objects.vehicle_quaternion.y = car_orientation.y();
    tracker_objects.vehicle_quaternion.z = car_orientation.z();
    if(tracker.tracks_.empty()) return;
    for(auto object : tracker.tracks_)
    {
        if(object.is_pub){
            multi_object_tracker::PredictionObstacle pub_;
            pub_.is_static = 0;
            pub_.perception_obstacle.header.frame_id = 'rslidar';
            pub_.perception_obstacle.header.stamp = ros::Time::now();
            pub_.perception_obstacle.id = object.id;
            pub_.perception_obstacle.position.x = object.Box.x;
            pub_.perception_obstacle.position.y = object.Box.y;
            pub_.perception_obstacle.position.z = object.Box.z;
            pub_.perception_obstacle.width = object.Box.width;
            pub_.perception_obstacle.length = object.Box.length;
            pub_.perception_obstacle.height = object.Box.height;
            pub_.perception_obstacle.type = 100;
            pub_.perception_obstacle.velocity.x = object.Box.velocity_x;
            pub_.perception_obstacle.velocity.y = object.Box.velocity_y;
            pub_.perception_obstacle.orientation2.w = object.Box.orientation_angle_w;
            pub_.perception_obstacle.orientation2.x = object.Box.orientation_angle_x;
            pub_.perception_obstacle.orientation2.y = object.Box.orientation_angle_y;
            pub_.perception_obstacle.orientation2.z = object.Box.orientation_angle_z;
            Eigen::Quaterniond obstacle_q;
            obstacle_q.w() = object.Box.orientation_angle_w;
            obstacle_q.x() = object.Box.orientation_angle_x;
            obstacle_q.y() = object.Box.orientation_angle_y;
            obstacle_q.z() = object.Box.orientation_angle_z;
            obstacle_q.normalize();
            Eigen::Vector3d obstacle_eulerAngle = obstacle_q.matrix().eulerAngles(2,1,0);
            pub_.perception_obstacle.angle = obstacle_eulerAngle[0];
            geometry_msgs::Point32 point;
            if(object.Box.polygon.size() > 0){
                for (auto polygon : object.Box.polygon){
                    point.x = polygon[0];
                    point.y = polygon[1];
                    point.z = polygon[2];
                    pub_.perception_obstacle.polygon.points.push_back(point);
                }
            }
            multi_object_tracker::TrajectoryPoint pre_point;
            double px = object.ukf.x_(0);
            double py = object.ukf.x_(1);
            double v = object.ukf.x_(2);
            double psi = object.ukf.x_(3);
            double psi_dot = object.ukf.x_(4);
            double v_a = 0;
            double v_yawdd = 0;
            for(int tt = 0; tt < 30; tt++){
                pre_point.relative_time = (tt + 1) * 0.1;
                if (fabs(psi_dot) < 0.0001) {
			        pre_point.x = px + v*cos(psi)*(tt + 1)*0.1 + 0.5*(tt + 1)*0.1*(tt + 1)*0.1*cos(psi)*v_a;
			        pre_point.y = py + v*sin(psi)*(tt + 1)*0.1 + 0.5*(tt + 1)*0.1*(tt + 1)*0.1*sin(psi)*v_a;
		        } else {
			        pre_point.x = px + (v/psi_dot)*(sin(psi+psi_dot*(tt + 1)*0.1) - sin(psi)) + 0.5*(tt + 1)*0.1*(tt + 1)*0.1*cos(psi)*v_a;
			        pre_point.y = py + (v/psi_dot)*(-cos(psi+psi_dot*(tt + 1)*0.1) + cos(psi)) + 0.5*(tt + 1)*0.1*(tt + 1)*0.1*sin(psi)*v_a;
		        }
                pub_.trajectory.push_back(pre_point);
            }
            tracker_objects.prediction_obstacles.push_back(pub_);
        }
    }
    fusion_pub.publish(tracker_objects);
}

void fusion::lidar_callback(const multi_object_tracker::DetectedObjectArray::ConstPtr &lidar_objects){
    std::lock_guard<std::mutex> lock(msgs_mutex);
    timestamp = lidar_objects->header.stamp.toSec();
    vector<BoundingBox3D> lidar_detections;
    double veh_yaw = lidar_objects->vech_st.veh_Course;

    // 计算四元数分量
    double veh_q_w = std::cos(veh_yaw / 2.0);
    double veh_q_x = 0.0;
    double veh_q_y = 0.0;
    double veh_q_z = std::sin(veh_yaw / 2.0);

    // 构造四元数
    Eigen::Quaterniond veh_q(veh_q_w, veh_q_x, veh_q_y, veh_q_z);
    veh_q.normalize();
    Eigen::Matrix3d rotation_veh(veh_q);

    Eigen::Matrix4d transform_veh;

    transform_veh(0, 3) = lidar_objects->vech_st.veh_x;
    transform_veh(1, 3) = lidar_objects->vech_st.veh_y;
    transform_veh(2, 3) = lidar_objects->vech_st.veh_z;
    transform_veh.block<3, 3>(0, 0) = rotation_veh;

    if(lidar_objects->objects.empty()) return;
    for(auto object : lidar_objects->objects)
    {
        BoundingBox3D lidar_detection;

        Eigen::Vector4d local_position(object.pose.position.x, object.pose.position.y, object.pose.position.z, 1.0);//局部转换全局
        Eigen::Vector4d global_position_homogeneous = transform_veh * local_position;//这里transform_matrix没接收到话题时为单位矩阵
        lidar_detection.x = global_position_homogeneous[0];
        lidar_detection.y = global_position_homogeneous[1];
        lidar_detection.z = global_position_homogeneous[2];

        //lidar_detection.x = object.pose.position.x;
        //lidar_detection.y = object.pose.position.y;
        //lidar_detection.z = object.pose.position.z;

        lidar_detection.length = object.dimensions.x;
        lidar_detection.width = object.dimensions.y;
        lidar_detection.height = object.dimensions.z;

        Eigen::Quaterniond local_q, global_q;
        local_q.x() = object.pose.orientation.x;
        local_q.y() = object.pose.orientation.y;
        local_q.z() = object.pose.orientation.z;
        local_q.w() = object.pose.orientation.w;
        local_q.normalize();   
        global_q = veh_q*local_q;
        global_q.normalize();
        lidar_detection.orientation_angle_w = global_q.w();
        lidar_detection.orientation_angle_x = global_q.x();
        lidar_detection.orientation_angle_y = global_q.y();
        lidar_detection.orientation_angle_z = global_q.z();

        //lidar_detection.orientation_angle_w = object.pose.orientation.w;
        //lidar_detection.orientation_angle_x = object.pose.orientation.x;
        //lidar_detection.orientation_angle_y = object.pose.orientation.y;
        //lidar_detection.orientation_angle_z = object.pose.orientation.z;

        Eigen::Vector3d point;
        if(object.convex_hull.polygon.points.size() > 0){
            for(auto polygon : object.convex_hull.polygon.points)
            {
                Eigen::Vector4d local_polygon(polygon.x, polygon.y, polygon.z, 1.0);
                Eigen::Vector4d global_polygon_homogeneous = transform_veh * local_polygon;//局部转换全局
                point[0] = global_polygon_homogeneous[0] - lidar_detection.x;
                point[1] = global_polygon_homogeneous[1] - lidar_detection.y;
                point[2] = global_polygon_homogeneous[2] - lidar_detection.z;
                //point[0] = polygon.x - lidar_detection.x;//外包络转换成相对坐标
                //point[1] = polygon.y - lidar_detection.y;
                //point[2] = polygon.z - lidar_detection.z;
                lidar_detection.polygon.push_back(point);
            }
        }

        lidar_detection.label = 0;
        lidar_detection.confidence = object.score;
        lidar_detection.velocity_x = 0;
        lidar_detection.velocity_y = 0;
        lidar_detection.sensor = BoundingBox3D::LIDAR;
        lidar_detections.push_back(lidar_detection);
    }
    if(!stop_thread){
        stop_thread = true;
        process_data(lidar_detections);
    }
} 


void fusion::radar_callback (const multi_object_tracker::ObjectList::ConstPtr &radar_objects){
    std::lock_guard<std::mutex> lock(msgs_mutex);
    vector<BoundingBox3D> radar_detections;
    if(radar_objects->objects.empty()) return;
    for(auto object : radar_objects->objects)
    {
        BoundingBox3D radar_detection;

        Eigen::Vector4d local_position(object.position.pose.position.x, object.position.pose.position.y, object.position.pose.position.z, 1.0);
        Eigen::Vector4d global_position_homogeneous = transform_matrix * local_position;
        radar_detection.x = global_position_homogeneous(0);
        radar_detection.y = global_position_homogeneous(1);
        radar_detection.z = global_position_homogeneous(2);    

        radar_detection.width = object.width;
        radar_detection.length = object.length;
        radar_detection.height = object.height;

        Eigen::Quaterniond global_q;
        Eigen::Quaterniond local_q;
        local_q.w() = object.position.pose.orientation.w;
        local_q.x() = object.position.pose.orientation.x;
        local_q.y() = object.position.pose.orientation.y;
        local_q.z() = object.position.pose.orientation.z;
        local_q.normalize();  
        global_q = car_orientation*local_q;
        global_q.normalize();
        radar_detection.orientation_angle_w = global_q.w();
        radar_detection.orientation_angle_x = global_q.x();
        radar_detection.orientation_angle_y = global_q.y();
        radar_detection.orientation_angle_z = global_q.z();

        Eigen::Vector4d local_velocity(object.relative_velocity.twist.linear.x, object.relative_velocity.twist.linear.y, 0.0, 1.0);
        Eigen::Vector4d global_velocity_homogeneous = transform_velocity * local_velocity;  
        radar_detection.velocity_x = global_velocity_homogeneous(0);
        radar_detection.velocity_y = global_velocity_homogeneous(1);

        radar_detection.confidence = object.prob_of_exist;
        radar_detection.label = object.class_type;
        radar_detection.sensor = BoundingBox3D::RADAR;
        radar_detections.push_back(radar_detection);
    }
    if(!stop_thread){
        stop_thread = true;
        process_data(radar_detections);
    }
}


void fusion::process_data(const std::vector<BoundingBox3D> object_boxes) {
    if(object_boxes.empty()) return;
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        clock_t startTime,endTime;
        startTime=clock();				// 开始时刻
        tracker.ProcessMeasurement(object_boxes);
        endTime=clock();				// 结束时刻
        cout<<"Track time is: "<<(endTime-startTime)*1.0/CLOCKS_PER_SEC<<" s"<<endl;
        if((endTime-startTime)*1.0/CLOCKS_PER_SEC > 0.1){ //防止卡死
            tracker.tracks_.clear();
        }
        stop_thread = false;
    }
}