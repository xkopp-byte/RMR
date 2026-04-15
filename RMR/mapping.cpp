#include "mapping.h"
#include <cstdio>

Mapping::Mapping(QObject *parent)
    : QObject(parent), 
        map_width_(6.02), 
        map_height_(6.81), 
        map_height_extend_(6.81),
        map_width_extend_(6.02),
        cell_size_(0.01),
        origin_x_(-0.5), // 0.0 // -0.5
        origin_y_(-2.1), // -1.6 // -2.1
        robot_start_x_(6.0),  // Starting position in world coordinates (0.5, 0.5)
        robot_start_y_(6.0),
        last_scan_pose_{robot_start_x_, robot_start_y_, 0, 0}
{
#ifndef DISABLE_MAPPING
    initializeGrid();
#endif
}

Mapping::~Mapping()
{
}

#ifndef DISABLE_MAPPING
void Mapping::initializeGrid()
{
    int width = static_cast<int>(map_width_ / cell_size_ + map_width_extend_ / cell_size_);    // 602 = 602 - 0 
    int height = static_cast<int>(map_height_ / cell_size_ + map_height_extend_ / cell_size_);  // 681 = 521 - (-160)
    // 0, -160 vo svetovych suradniciach je 0, 0 v gride
    // robot zacina na 50, 50 v svetovych suradniciach (podla https://github.com/tom-jurov/test_simulator/blob/65b67dfd964c9a59e13280c229c1e35fae0daecb/mainwindow.h)
    //
    occupancy_grid_ = cv::Mat(height, width, CV_8U, cv::Scalar(0));
}

void Mapping::recordPose(double x, double y, double phi, double timestamp)
{
    const double unwrapped_timestamp = unwrapRobotTimestamp(timestamp);
    pose_history_.push_back({x, y, phi, unwrapped_timestamp});
    if (pose_history_.size() > POSE_HISTORY_SIZE) {
        pose_history_.pop_front();
    }
}

// double Mapping::cubicInterpolate(double p0, double p1, double p2, double p3, double t) const
// {
//     
// }

Mapping::RobotPose Mapping::interpolatePose(double t) const
{
    if (pose_history_.empty()) return last_scan_pose_;
    if (pose_history_.size() == 1) return pose_history_.back();
    if (t <= pose_history_.front().timestamp) return pose_history_.front();
    if (t >= pose_history_.back().timestamp) return pose_history_.back();

    // Find bracket [p0, p1] with p0.t <= t <= p1.t
    size_t i = 0;
    while (i + 1 < pose_history_.size() && pose_history_[i + 1].timestamp < t)
    {
        i++;
    }
    const RobotPose& p0 = pose_history_[i];
    const RobotPose& p1 = (i + 1 < pose_history_.size()) ? pose_history_[i + 1] : p0;

    if (p1.timestamp == p0.timestamp) return p0;

    const double alpha = (t - p0.timestamp) / (p1.timestamp - p0.timestamp);
    const double dphi = normalizeAngleDiff(p0.phi, p1.phi);

    return 
    {
        p0.x + alpha * (p1.x - p0.x),
        p0.y + alpha * (p1.y - p0.y),
        p0.phi + alpha * dphi,
        t
    };

    
}

void Mapping::updateMapFromLidar(double robot_x, double robot_y, double robot_phi,
                                    const std::vector<LaserData>& lidar_data)
{
    if (lidar_data.empty()) return;

    QMutexLocker locker(&map_mutex_);

     if (pose_history_.empty()) 
    {
        return; 
    }

    // Use the latest robot pose timestamp as our unwrapping reference
    double reference_unwrapped_ts = pose_history_.back().timestamp;

    int num_scans = lidar_data.size();
   
    for (int i = 0; i < num_scans; i++)
    {
        double angle_rad = lidar_data[i].scanAngle * M_PI / 180.0; // Convert to radians
        double distance_m = lidar_data[i].scanDistance * 0.001;    // Convert to meters
        uint32_t raw_lidar_ts = lidar_data[i].timestamp;
        
        if (distance_m <= min_lidar_range_ || distance_m > max_lidar_range_) continue;

        // 1. Lift the wrapping LiDAR timestamp into our continuous timeline
        double lifted_ts = liftLidarTimestamp(static_cast<double>(raw_lidar_ts), reference_unwrapped_ts);

        // 2. Interpolate the exact robot pose at the moment this laser point was fired
        RobotPose exact_pose = interpolatePose(lifted_ts);

        // 3. Project the laser point into the world using the interpolated pose
        double world_angle = exact_pose.phi - angle_rad;
        double world_x = exact_pose.x + distance_m * cos(world_angle);
        double world_y = exact_pose.y + distance_m * sin(world_angle);
        
        int grid_x, grid_y;
        if (worldToGrid(world_x, world_y, grid_x, grid_y)) 
        {
            occupancy_grid_.at<uint8_t>(grid_y, grid_x) = CELL_OCCUPIED;
        }
    }

    locker.unlock();
    emit mapUpdated();
}

double Mapping::normalizeTimestamp(double timestamp) const
{
    double mod_ts = std::fmod(timestamp, TIMESTAMP_MODULO);
    if (mod_ts < 0.0) {
        mod_ts += TIMESTAMP_MODULO;
    }
    return mod_ts;
}

double Mapping::unwrapRobotTimestamp(double timestamp)
{
    const double mod_ts = normalizeTimestamp(timestamp);

    if (!has_last_robot_timestamp_) {
        has_last_robot_timestamp_ = true;
        last_robot_timestamp_mod_ = mod_ts;
        robot_timestamp_wrap_offset_ = 0.0;
        return mod_ts;
    }

    // Detect wrap-around at modulo boundary.
    if (mod_ts < last_robot_timestamp_mod_ &&
        (last_robot_timestamp_mod_ - mod_ts) > (TIMESTAMP_MODULO * 0.5)) {
        robot_timestamp_wrap_offset_ += TIMESTAMP_MODULO;
    }

    last_robot_timestamp_mod_ = mod_ts;
    return mod_ts + robot_timestamp_wrap_offset_;
}

double Mapping::liftLidarTimestamp(double timestamp_mod, double reference_unwrapped) const
{
    const double base_cycle = std::floor(reference_unwrapped / TIMESTAMP_MODULO);
    const double c0 = (base_cycle - 1.0) * TIMESTAMP_MODULO + timestamp_mod;
    const double c1 = base_cycle * TIMESTAMP_MODULO + timestamp_mod;
    const double c2 = (base_cycle + 1.0) * TIMESTAMP_MODULO + timestamp_mod;

    double best = c1;
    double best_diff = std::fabs(c1 - reference_unwrapped);

    const double diff0 = std::fabs(c0 - reference_unwrapped);
    if (diff0 < best_diff) {
        best = c0;
        best_diff = diff0;
    }

    const double diff2 = std::fabs(c2 - reference_unwrapped);
    if (diff2 < best_diff) {
        best = c2;
    }

    return best;
}

bool Mapping::worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const
{
    const int width = static_cast<int>(map_width_ / cell_size_ + map_width_extend_ / cell_size_); 
    const int height = static_cast<int>(map_height_ / cell_size_ + map_height_extend_ / cell_size_);

    const int y_shift_rows = 0;

    grid_x = static_cast<int>(std::floor((world_x - origin_x_) / cell_size_));

    const int y_cells = static_cast<int>(std::floor((world_y - origin_y_) / cell_size_));
    grid_y = (height - 1) - y_cells + y_shift_rows; // downward shift only (height - 1) - y_cells + y_shift_rows;

    return (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height);
}

// uint8_t Mapping::getOccupancy(double world_x, double world_y) const
// {
//     
// }
// 
// bool Mapping::isObstacle(double world_x, double world_y) const
// {
//     
// }

cv::Mat Mapping::getGridImage()
{
    QMutexLocker locker(&map_mutex_);
//    return occupancy_grid_.clone();
    return occupancy_grid_;
}

void Mapping::onLidarData(const std::vector<LaserData>& lidata)
{
    // updateMapFromLidar(current_robot_x_, current_robot_y_, current_robot_phi_, lidata); povodne
    QMutexLocker locker(&map_mutex_);
    
    // Calculate movement since last scan
    double pos_delta = sqrt(
        (current_robot_x_ - last_scan_pose_.x) * (current_robot_x_ - last_scan_pose_.x) +
        (current_robot_y_ - last_scan_pose_.y) * (current_robot_y_ - last_scan_pose_.y)
    );
    
    double rot_delta = fabs(normalizeAngleDiff(current_robot_phi_, last_scan_pose_.phi));
    bool moving = (pos_delta > POSITION_THRESHOLD || rot_delta > ROTATION_THRESHOLD);
    // bool moving = false;
    last_scan_pose_ = {current_robot_x_, current_robot_y_, current_robot_phi_, 0};
    
    // Only process if robot is stationary
    if (moving) {
        return;  // Robot is moving, skip this scan
    }

    locker.unlock();
    updateMapFromLidar(current_robot_x_, current_robot_y_, current_robot_phi_, lidata);
}

void Mapping::onRobotPosition(double x, double y, double phi, bool obstacle, uint32_t timestamp)
{
    QMutexLocker locker(&map_mutex_);
    current_robot_x_ = robot_start_x_ + x;
    current_robot_y_ = robot_start_y_ + y;
    current_robot_phi_ = phi * M_PI / 180.0; // Convert to radians
    recordPose(current_robot_x_, current_robot_y_, current_robot_phi_, timestamp);

    static int pose_rx_counter = 0;
    pose_rx_counter++;
    if ((pose_rx_counter % 50) == 0) {
        std::printf("[MAP] pose rx #%d: x=%.3f y=%.3f phi=%.3f ts=%u\n",
                    pose_rx_counter,
                    current_robot_x_,
                    current_robot_y_,
                    current_robot_phi_,
                    timestamp);
    }
}

double Mapping::normalizeAngleDiff(double phi1, double phi2) const
{
    double diff = phi2 - phi1;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

bool Mapping::saveMapToFile(const std::string& filename) const
{
    QMutexLocker locker(&map_mutex_);
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;
    
    fs << "occupancy_grid" << occupancy_grid_;
    fs << "width" << occupancy_grid_.cols;
    fs << "height" << occupancy_grid_.rows;
    fs.release();
    return true;
}   

// void Mapping::getOcupancyGrid(cv::Mat& grid) const
// {
//     QMutexLocker locker(&map_mutex_);
//     occupancy_grid_.copyTo(grid);
// }

#endif
