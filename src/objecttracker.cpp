#include "objecttracker.h"

ObjectTracker::ObjectTracker() : distance_threshold_(3), max_missed_count_(10), min_birth_count_(2) {}

void ObjectTracker::ProcessMeasurement(const vector<BoundingBox3D>& detections) { 
    vector<Track> tracks(tracks_);
    std::vector<bool> track_matched(tracks.size(), false);   // 追踪结果是否匹配
    // 检测结果
    vector<BoundingBox3D> all_detections = detections;
    // 数据关联
    vector<int> assignments(all_detections.size(), -1);
    if(assignments.empty()) return;
    AssociateDetectionsToTracks(all_detections, assignments, tracks);

    std::vector<BoundingBox3D> matched_detections;
    std::vector<BoundingBox3D> unmatched_detections;
    std::vector<Track> unmatched_tracks;

    for (size_t i = 0; i < assignments.size(); ++i) {
        if (assignments[i] >= 0) {
            // 检测结果匹配到跟踪结果
            matched_detections.push_back(all_detections[i]);
            track_matched[assignments[i]] = true; // 对应的track已匹配
        } else {
            // 检测结果未匹配到跟踪结果
            unmatched_detections.push_back(all_detections[i]);
        }
    }

    // 检查哪些跟踪结果未匹配到检测结果
    if (tracks.size() > 0) {
        for (size_t i = 0; i < tracks.size(); ++i) {
            if (tracks[i].birth_count > min_birth_count_) {
                tracks[i].is_pub = true;
                tracks[i].birth_count = 0;
            }
            if (!track_matched[i]) {//!track_matched[i]表示tracks[i]没匹配上
                unmatched_tracks.push_back(tracks[i]);
            }
            else {
                tracks[i].birth_count++;
            }
        }
    }
    std::cout<< "detections num = "<< all_detections.size() <<std::endl;
    std::cout<< "tracks num = "<< tracks.size() <<std::endl;
    std::cout<< "matched_detections_tracks num = "<< matched_detections.size() <<std::endl;
    std::cout<< "unmatched_detections num = "<< unmatched_detections.size() <<std::endl;
    std::cout<< "unmatched_tracks num = "<< unmatched_tracks.size() <<std::endl;
    // 更新跟踪目标
    for (size_t i = 0; i < assignments.size(); ++i) {
        if (assignments[i] >= 0) { //assignments[i] >= 0 表示对应all_detections[i]匹配上tracks[assignments[i]]
            switch(all_detections[i].sensor)
            {
                case BoundingBox3D::LIDAR:
                {
                    // 更新已存在的跟踪目标
                    MeasurementPackage meas_package;
                    meas_package.sensor_type_ = MeasurementPackage::LASER;
                    meas_package.raw_measurements_ = VectorXd(2);
                    meas_package.raw_measurements_ << all_detections[i].x, all_detections[i].y;
                    auto now = std::chrono::system_clock::now();
                    auto duration = now.time_since_epoch();
                    meas_package.timestamp_= std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
                    tracks[assignments[i]].ukf.ProcessMeasurement(meas_package);
                    tracks[assignments[i]].missed_count = 0; // 重置未检测到次数
                    tracks[assignments[i]].lidar_associated = true; // 激光雷达匹配到过

                    double p_x = tracks[assignments[i]].ukf.x_(0);
                    double p_y = tracks[assignments[i]].ukf.x_(1);
                    double yaw = tracks[assignments[i]].ukf.x_(3); 
                    double v = tracks[assignments[i]].ukf.x_(2);
                    double v_x = cos(yaw)*v;
                    double v_y = sin(yaw)*v;
                    Eigen::Quaterniond quaternion_ukf = yawToQuaternion(yaw);

                    tracks[assignments[i]].Box.x = p_x;//ukf更新
                    tracks[assignments[i]].Box.y = p_y;//ukf更新
                    tracks[assignments[i]].Box.z = all_detections[i].z;//信赖lidar
                    tracks[assignments[i]].Box.orientation_angle_x = all_detections[i].orientation_angle_x;//信赖lidar
                    tracks[assignments[i]].Box.orientation_angle_y = all_detections[i].orientation_angle_y;//信赖lidar
                    tracks[assignments[i]].Box.orientation_angle_z = all_detections[i].orientation_angle_z;//信赖lidar
                    tracks[assignments[i]].Box.orientation_angle_w = all_detections[i].orientation_angle_w;//信赖lidar
                    tracks[assignments[i]].Box.length = all_detections[i].length;//信赖lidar
                    tracks[assignments[i]].Box.width = all_detections[i].width;//信赖lidar
                    tracks[assignments[i]].Box.height = all_detections[i].height;//信赖lidar
                    tracks[assignments[i]].Box.label = all_detections[i].label;//信赖lidar
                    tracks[assignments[i]].Box.confidence = all_detections[i].confidence;//信赖lidar
                    tracks[assignments[i]].Box.polygon = calculateLidarPolygon(all_detections[i]);//信赖lidar
                    if(!tracks[assignments[i]].radar_associated){
                        tracks[assignments[i]].Box.velocity_x = v_x;//毫米波雷达没匹配到过，ukf更新
                        tracks[assignments[i]].Box.velocity_y = v_y;//ukf更新
                    }
                    tracks[assignments[i]].Box.sensor = BoundingBox3D::LIDAR;
                    break;
                }
                case BoundingBox3D::RADAR:
                {
                    double rho, phi, rho_dot;
                    rho = sqrt(all_detections[i].x * all_detections[i].x + all_detections[i].y * all_detections[i].y);
                    phi = atan2(all_detections[i].y, all_detections[i].x);
                    rho_dot = (all_detections[i].x * all_detections[i].velocity_x + all_detections[i].y * all_detections[i].velocity_y) / rho;
                    // angle normalization
                    while (phi > M_PI) phi -= 2.0*M_PI;
                    while (phi < -M_PI) phi += 2.0*M_PI;
                    // 更新已存在的跟踪目标
                    MeasurementPackage meas_package;
                    meas_package.sensor_type_ = MeasurementPackage::RADAR;
                    meas_package.raw_measurements_ = VectorXd(3);
                    meas_package.raw_measurements_ << rho, phi, rho_dot;
                    auto now = std::chrono::system_clock::now();
                    auto duration = now.time_since_epoch();
                    meas_package.timestamp_= std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
                    tracks[assignments[i]].ukf.ProcessMeasurement(meas_package);
                    tracks[assignments[i]].missed_count = 0; // 重置未检测到次数
                    tracks[assignments[i]].radar_associated = true; // 毫米波雷达匹配到过

                    double p_x = tracks[assignments[i]].ukf.x_(0);
                    double p_y = tracks[assignments[i]].ukf.x_(1);
                    double yaw = tracks[assignments[i]].ukf.x_(3); 
                    double v = tracks[assignments[i]].ukf.x_(2);
                    double v_x = cos(yaw)*v;
                    double v_y = sin(yaw)*v;
                    Eigen::Quaterniond quaternion_ukf = yawToQuaternion(yaw);

                    tracks[assignments[i]].Box.x = p_x;//ukf更新
                    tracks[assignments[i]].Box.y = p_y;//ukf更新
                    tracks[assignments[i]].Box.velocity_x = all_detections[i].velocity_x;//信赖radar
                    tracks[assignments[i]].Box.velocity_y = all_detections[i].velocity_y;//信赖radar
                    if(!tracks[assignments[i]].lidar_associated){
                        tracks[assignments[i]].Box.z = all_detections[i].z;//激光雷达没匹配到过，信赖radar
                        tracks[assignments[i]].Box.orientation_angle_w = quaternion_ukf.w();//ukf更新
                        tracks[assignments[i]].Box.orientation_angle_x = quaternion_ukf.x();//ukf更新
                        tracks[assignments[i]].Box.orientation_angle_y = quaternion_ukf.y();//ukf更新
                        tracks[assignments[i]].Box.orientation_angle_z = quaternion_ukf.z();//ukf更新
                        tracks[assignments[i]].Box.polygon = calculateRadarPolygon(tracks[assignments[i]].Box);//ukf更新计算polygon
                    }
                    tracks[assignments[i]].Box.sensor = BoundingBox3D::RADAR;
                    break;
                }
                case BoundingBox3D::RADAR_4D:
                {
                    break;
                }
            }
        }
    }

    // 创建新的跟踪目标
    CreateNewTracks(unmatched_detections, tracks);

    // 更新未匹配到检测结果的跟踪目标
    if(track_matched.size()>0) {
        for (size_t i = 0; i < tracks.size(); ++i) {
            if (!track_matched[i]) { //!track_matched[i]表示tracks[i]没匹配上
                tracks[i].missed_count++;
/*                 if (tracks[i].missed_count < max_missed_count_) {
                    switch(tracks[i].Box.sensor)
                    {
                        case BoundingBox3D::LIDAR:
                        {
                            MeasurementPackage meas_package;
                            meas_package.sensor_type_ = MeasurementPackage::LASER;
                            meas_package.raw_measurements_ = VectorXd(2);
                            meas_package.raw_measurements_ << tracks[i].ukf.x_(0), tracks[i].ukf.x_(1);
                            auto now = std::chrono::system_clock::now();
                            auto duration = now.time_since_epoch();
                            meas_package.timestamp_ = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
                            tracks[i].ukf.ProcessMeasurement(meas_package);
                            break;
                        }
                        case BoundingBox3D::RADAR:
                        {
                            MeasurementPackage meas_package;
                            meas_package.sensor_type_ = MeasurementPackage::RADAR;
		                    double p_x = tracks[i].ukf.x_(0);
		                    double p_y = tracks[i].ukf.x_(1);
		                    double v = tracks[i].ukf.x_(2);
		                    double yaw = tracks[i].ukf.x_(3);
                            
                            double rho = sqrt(p_x*p_x + p_y*p_y);
                            double phi = atan2(p_y, p_x);
                            // angle normalization
                            while (phi > M_PI) phi -= 2.0*M_PI;
                            while (phi < -M_PI) phi += 2.0*M_PI;
                            double rho_dot = v*(p_x*cos(yaw) + p_y*sin(yaw))/rho;
                            meas_package.raw_measurements_ = VectorXd(3);
                            meas_package.raw_measurements_ << rho, phi, rho_dot;
                            auto now = std::chrono::system_clock::now();
                            auto duration = now.time_since_epoch();
                            meas_package.timestamp_ = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
                            tracks[i].ukf.ProcessMeasurement(meas_package);
                            break;
                        }
                    }
                } */
            }
        }
    }
    // 移除死掉的跟踪目标
    RemoveDeadTracks(tracks);
    tracks_.assign(tracks.begin(), tracks.end());
}

void ObjectTracker::AssociateDetectionsToTracks(const vector<BoundingBox3D>& detections, vector<int>& assignments, vector<Track>& tracks) {
    size_t num_tracks = tracks.size();
    size_t num_detections = detections.size();
    if (num_tracks == 0 || num_detections == 0) {
        return;
    }

    // 创建二维向量 cost_matrix
    vector<vector<double>> cost_matrix(num_tracks, vector<double>(num_detections, 0.0));

    for (size_t i = 0; i < num_tracks; ++i) {
        for (size_t j = 0; j < num_detections; ++j) {
            double dx = tracks[i].Box.x - detections[j].x;
            double dy = tracks[i].Box.y - detections[j].y;
            cost_matrix[i][j] = sqrt(dx * dx + dy * dy); // 根据欧几里德距离创建
        }
    }

    HungarianAlgorithm hungarian;
    vector<int> assignment;
    double cost = hungarian.Solve(cost_matrix, assignment);

    for (size_t i = 0; i < assignment.size(); ++i) {
        if (assignment[i] >= 0 && cost_matrix[i][assignment[i]] < distance_threshold_) {
            assignments[assignment[i]] = i;
        }
    }
}

void ObjectTracker::CreateNewTracks(const vector<BoundingBox3D>& unmatched_detections, vector<Track>& tracks) {
    if (unmatched_detections.empty()) return;
    for (const auto& detection : unmatched_detections) {
        switch(detection.sensor)
        {
            case BoundingBox3D::LIDAR:
            {
                Track new_track;
                new_track.ukf = UKF();
                new_track.id = id_manager_.allocateID();
                new_track.missed_count = 0; // 初始化未检测到次数
                new_track.Box = detection;
                new_track.Box.polygon = calculateLidarPolygon(detection);
                new_track.birth_count = 0; // 初始化匹配到次数
                new_track.is_pub = false;
                new_track.lidar_associated = true;
                new_track.radar_associated = false;

                MeasurementPackage meas_package;
                auto now = std::chrono::system_clock::now();
                auto duration = now.time_since_epoch();
                meas_package.timestamp_= std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
                meas_package.sensor_type_ = MeasurementPackage::LASER; // 使用激光雷达进行初始化
                meas_package.raw_measurements_ = VectorXd(2);
                meas_package.raw_measurements_ << detection.x, detection.y;

                new_track.ukf.ProcessMeasurement(meas_package);
                tracks.push_back(new_track);
                break;
            }
            case BoundingBox3D::RADAR:
            {
                Track new_track;
                new_track.ukf = UKF();
                new_track.id = id_manager_.allocateID();
                new_track.missed_count = 0; // 初始化未检测到次数
                new_track.Box = detection;
                new_track.Box.polygon = calculateRadarPolygon(detection);//计算polygon
                new_track.birth_count = 0; // 初始化匹配到次数
                new_track.is_pub = false;
                new_track.lidar_associated = false;
                new_track.radar_associated = true;

                double rho, phi, rho_dot;
                rho = sqrt(detection.x * detection.x + detection.y * detection.y);
                phi = atan2(detection.y, detection.x);
                // angle normalization
                while (phi > M_PI) phi -= 2.0*M_PI;
                while (phi < -M_PI) phi += 2.0*M_PI;
                rho_dot = (detection.x * detection.velocity_x + detection.y * detection.velocity_y) / rho;

                MeasurementPackage meas_package;
                auto now = std::chrono::system_clock::now();
                auto duration = now.time_since_epoch();
                meas_package.timestamp_= std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
                meas_package.sensor_type_ = MeasurementPackage::RADAR; // 使用毫米波雷达进行初始化
                meas_package.raw_measurements_ = VectorXd(3);
                meas_package.raw_measurements_ << rho, phi, rho_dot;

                new_track.ukf.ProcessMeasurement(meas_package);
                tracks.push_back(new_track);
                break;
            }
        }
    }
}


void ObjectTracker::RemoveDeadTracks(vector<Track>& tracks) {
    if (tracks.empty()) return;
    for (auto it = tracks.begin(); it != tracks.end();) {
        if (it->missed_count >= max_missed_count_) {
            id_manager_.recycleID(it->id);  // 回收ID
            it = tracks.erase(it);
        } else {
            ++it;
        }
    }
}

Eigen::Quaterniond ObjectTracker::yawToQuaternion(double yaw) {
    // 计算yaw角的一半
    double half_yaw = yaw / 2.0;
    // 计算四元数分量
    double w = std::cos(half_yaw);
    double x = 0.0;
    double y = 0.0;
    double z = std::sin(half_yaw);

    // 构造四元数
    Eigen::Quaterniond q(w, x, y, z);
    return q;
}


std::vector<Eigen::Vector3d> ObjectTracker::calculateRadarPolygon(const BoundingBox3D& obstacle) {
    std::vector<Eigen::Vector3d> vertices(5);

    // 定义障碍物在局部坐标系中的顶点坐标（假设在XY平面上）
    Eigen::Vector3d local_vertices[5] = {
        Eigen::Vector3d(obstacle.length / 2, obstacle.width / 2, 0),
        Eigen::Vector3d(obstacle.length / 2, -obstacle.width / 2, 0),
        Eigen::Vector3d(-obstacle.length / 2, -obstacle.width / 2, 0),
        Eigen::Vector3d(-obstacle.length / 2, obstacle.width / 2, 0),
        Eigen::Vector3d(obstacle.length / 2, obstacle.width / 2, 0),
    };

    // 计算旋转矩阵
    Eigen::Quaterniond orientation;
    orientation.w() = obstacle.orientation_angle_w;
    orientation.x() = obstacle.orientation_angle_x;
    orientation.y() = obstacle.orientation_angle_y;
    orientation.z() = obstacle.orientation_angle_z;
    Eigen::Matrix3d rotation_matrix = orientation.normalized().toRotationMatrix();

    // 计算全局坐标系中的顶点坐标
    for (int i = 0; i < 5; ++i) {
        Eigen::Vector3d global_vertex = rotation_matrix * local_vertices[i];
        vertices[i].x() = global_vertex.x() + obstacle.x;
        vertices[i].y() = global_vertex.y() + obstacle.y;
        vertices[i].z() = global_vertex.z() + obstacle.z; // 如果需要考虑高度
    }

    return vertices;
}

std::vector<Eigen::Vector3d> ObjectTracker::calculateLidarPolygon(const BoundingBox3D& obstacle) {
    std::vector<Eigen::Vector3d> vertices;
    for(auto polygon : obstacle.polygon){
        Eigen::Vector3d global_vertex;
        global_vertex[0] = polygon[0] + obstacle.x; //相对polygon转换为绝对polygon
        global_vertex[1] = polygon[1] + obstacle.y;
        global_vertex[2] = polygon[2] + obstacle.z;
        vertices.push_back(global_vertex);
    }
    return vertices;
}


