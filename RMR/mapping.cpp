#include "mapping.h"

Mapping::Mapping(QObject *parent)
    : QObject(parent), 
      map_width_(6.02), 
      map_height_(6.81), 
      cell_size_(0.01),
      origin_x_(0.0),
      origin_y_(-1.6),
      robot_start_x_(0.5),  // Starting position in world coordinates (0.5, 0.5)
      robot_start_y_(0.5),
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
    int width = static_cast<int>(map_width_ / cell_size_);    // 602 = 602 - 0
    int height = static_cast<int>(map_height_ / cell_size_);  // 681 = 521 - (-160)
    // 0, -160 vo svetovych suradniciach je 0, 0 v gride
    // robot zacina na 50, 50 v svetovych suradniciach (podla https://github.com/tom-jurov/test_simulator/blob/65b67dfd964c9a59e13280c229c1e35fae0daecb/mainwindow.h)
    //
    occupancy_grid_ = cv::Mat(height, width, CV_8U, cv::Scalar(127));  // Unknown (gray)
}

void Mapping::recordPose(double x, double y, double phi)
{
    pose_history_.push_back({x, y, phi, 0.0});
    if (pose_history_.size() > POSE_HISTORY_SIZE) {
        pose_history_.pop_front();
    }
}

// Cubic Hermite interpolation: p(t) = h0*p0 + h1*p1 + h2*m0 + h3*m1
// where m0, m1 are slopes, and t is in [0,1]
double Mapping::cubicInterpolate(double p0, double p1, double p2, double p3, double t) const
{
    
}

Mapping::RobotPose Mapping::interpolatePose(double t) const
{
    
}

void Mapping::updateMapFromLidar(double robot_x, double robot_y, double robot_phi,
                                    const std::vector<LaserData>& lidar_data)
{
    if (lidar_data.empty()) return;

    QMutexLocker locker(&map_mutex_);

    int num_scans = lidar_data.size();
   
    for (int i = 0; i < num_scans; i++)
    {
        double angle_rad = lidar_data[i].scanAngle * M_PI / 180.0; // Convert to radians
        double distance_m = lidar_data[i].scanDistance * 0.001; // Convert to meters
        if (distance_m <= 0.0 || distance_m > 5.0) continue;

        double world_angle = robot_phi - angle_rad; 
        double world_x = robot_x + distance_m * cos(world_angle);
        double world_y = robot_y + distance_m * sin(world_angle);
        int grid_x, grid_y;
        if (worldToGrid(world_x, world_y, grid_x, grid_y)) 
        {
            occupancy_grid_.at<uint8_t>(grid_y, grid_x) = CELL_OCCUPIED;  // TODO: doplnit logiku pre free/unknown
        }
    }

//     // debug print
//     cout << "Updating map with " << lidar_data.size() << " lidar points at robot pose (" 
//          << robot_x << ", " << robot_y << ", " << robot_phi * 180.0 / M_PI << " deg)\n";
// 
// 
//     for (size_t i = 0; i < static_cast<size_t>(occupancy_grid_.rows ); i++)
//     {
//         for (size_t j = 0; j < static_cast<size_t>(occupancy_grid_.cols ); j++)
//         {
//             if (occupancy_grid_.at<uint8_t>(i, j) == CELL_OCCUPIED)
//             {
//                 cout << "Occupied cell at grid (" << j << ", " << i << ")\n";
//                 cout << static_cast<int>(occupancy_grid_.at<uint8_t>(i, j)) << " ";
//             }
//         }
//     }
    locker.unlock();
    emit mapUpdated();
}

bool Mapping::worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const
{
    const int width = static_cast<int>(map_width_ / cell_size_);
    const int height = static_cast<int>(map_height_ / cell_size_);

    const int y_shift_rows = 0; // tune this number

    grid_x = static_cast<int>(std::floor((world_x - origin_x_) / cell_size_));

    const int y_cells = static_cast<int>(std::floor((world_y - origin_y_) / cell_size_));
    grid_y = (height - 1) - y_cells + y_shift_rows; // downward shift only (height - 1) - y_cells + y_shift_rows;

    return (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height);
}

uint8_t Mapping::getOccupancy(double world_x, double world_y) const
{
    
}

bool Mapping::isObstacle(double world_x, double world_y) const
{
    
}

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
    
    last_scan_pose_ = {current_robot_x_, current_robot_y_, current_robot_phi_, 0};
    
    // Only process if robot is stationary
    if (moving) {
        return;  // Robot is moving, skip this scan
    }

    locker.unlock();
    updateMapFromLidar(current_robot_x_, current_robot_y_, current_robot_phi_, lidata);
}

void Mapping::onRobotPosition(double x, double y, double phi, bool obstacle)
{
    QMutexLocker locker(&map_mutex_);
    current_robot_x_ = robot_start_x_ + x;
    current_robot_y_ = robot_start_y_ + y;
    current_robot_phi_ = phi * M_PI / 180.0; // Convert to radians
    // recordPose(x, y, phi);
    recordPose(current_robot_x_, current_robot_y_, current_robot_phi_);
}

// Helper to normalize angle difference
double Mapping::normalizeAngleDiff(double phi1, double phi2) const
{
    double diff = phi2 - phi1;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}
#endif
