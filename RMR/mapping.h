#ifndef MAPPING_H
#define MAPPING_H

#include <QObject>
#include <QMutex>
#include <deque>
#include <vector>
#include <memory>
#include <cmath>
#include "librobot/librobot.h"

#ifndef DISABLE_MAPPING
#include "opencv2/core/core.hpp"
#endif

class Mapping : public QObject {
    Q_OBJECT

    public:
        explicit Mapping(QObject *parent = nullptr);
        ~Mapping();

        void updateMapFromLidar(double robot_x, double robot_y, double robot_phi,
                                const std::vector<LaserData>& lidar_data);
        
#ifndef DISABLE_MAPPING
        uint8_t getOccupancy(double world_x, double world_y) const;
        bool isObstacle(double world_x, double world_y) const;
        cv::Mat getGridImage();
#endif
        
    signals:
        void mapUpdated();

#ifndef DISABLE_MAPPING
    public slots:
        void onLidarData(const std::vector<LaserData>& lidata);
        void onRobotPosition(double x, double y, double phi, bool obstacle);
#endif

    private:
        // Pose structure for history tracking
        struct RobotPose {
            double x, y, phi;  // position and orientation
            double timestamp;  // relative time (0 = start of scan)
        };

        // Map parameters
        double map_width_;     
        double map_height_;     
        double cell_size_;      // 0.01 m
        double robot_start_x_;  // Starting position in world coordinates
        double robot_start_y_;
        
        struct WorldPoint {
                double x;
                double y;
        };
        std::vector<WorldPoint> latest_scan_points_;
        std::vector<WorldPoint> render_scan_points_;
        cv::Mat render_grid_;
        bool grid_dirty_ = true;

        // Map storage & origin
#ifndef DISABLE_MAPPING
        cv::Mat occupancy_grid_;  // CV_8U: 0=free, 127=unknown, 255=occupied
        std::deque<RobotPose> pose_history_;  // stores poses for interpolation
        static constexpr int POSE_HISTORY_SIZE = 4;  // keep 4 poses for cubic interpolation
        
        double current_robot_x_ = 0.0;
        double current_robot_y_ = 0.0;
        double current_robot_phi_ = 0.0;

        RobotPose last_scan_pose_ = {0, 0, 0, 0};
        static constexpr double POSITION_THRESHOLD = 0.01;
        static constexpr double ROTATION_THRESHOLD = 0.01; 
#endif
        double origin_x_;
        double origin_y_;
        
        // Thread safety
        mutable QMutex map_mutex_;
        
        // Helper methods
        bool worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const;
        void initializeGrid();
        void recordPose(double x, double y, double phi);
        double normalizeAngleDiff(double phi1, double phi2) const;
        
        // Cubic Hermite interpolation
        double cubicInterpolate(double p0, double p1, double p2, double p3, double t) const;
        RobotPose interpolatePose(double t) const;
        
        // Constants
        static const uint8_t CELL_FREE = 0;
        static const uint8_t CELL_UNKNOWN = 127;
        static const uint8_t CELL_OCCUPIED = 255;
};

#endif // MAPPING_H
