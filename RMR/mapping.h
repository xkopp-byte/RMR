#ifndef MAPPING_H
#define MAPPING_H

#include <QObject>
#include <QMutex>
#include <deque>
#include <vector>
#include <memory>
#include <cmath>
#include <cstdint>
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
        void onRobotPosition(double x, double y, double phi, bool obstacle, uint32_t timestamp);
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
        double map_height_extend_;
        double map_width_extend_;
        double robot_start_x_;  // Starting position in world coordinates
        double robot_start_y_;

        double min_lidar_range_ = 0.8;
        double max_lidar_range_ = 2.0;
        
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
        static constexpr int POSE_HISTORY_SIZE = 20; 
                
        double current_robot_x_ = 0.0;
        double current_robot_y_ = 0.0;
        double current_robot_phi_ = 0.0;

        RobotPose last_scan_pose_ = {0, 0, 0, 0};
        static constexpr double POSITION_THRESHOLD = 0.1;
        static constexpr double ROTATION_THRESHOLD = 0.1; 
#endif
        double origin_x_;
        double origin_y_;
        
        // Thread safety
        mutable QMutex map_mutex_;
        
        // Helper methods
        bool worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const;
        void initializeGrid();
        void recordPose(double x, double y, double phi, double timestamp);
        double normalizeAngleDiff(double phi1, double phi2) const;
        double normalizeTimestamp(double timestamp) const;
        double unwrapRobotTimestamp(double timestamp);
        double liftLidarTimestamp(double timestamp_mod, double reference_unwrapped) const;
        
        // Cubic Hermite interpolation
        // double cubicInterpolate(double p0, double p1, double p2, double p3, double t) const;
        RobotPose interpolatePose(double t) const;
        
        // Constants
        static const uint8_t CELL_FREE = 0;
        static const uint8_t CELL_UNKNOWN = 127;
        static const uint8_t CELL_OCCUPIED = 255;

        static constexpr double TIMESTAMP_MODULO = 3600000000.0;
        bool has_last_robot_timestamp_ = false;
        double last_robot_timestamp_mod_ = 0.0;
        double robot_timestamp_wrap_offset_ = 0.0;
};

#endif // MAPPING_H
