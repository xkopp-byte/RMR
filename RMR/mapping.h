#ifndef MAPPING_H
#define MAPPING_H

#include <QObject>
#include <vector>
#include <memory>
#include "librobot/librobot.h"

#ifndef DISABLE_OPENCV
#include "opencv2/core/core.hpp"
#endif

class Mapping : public QObject {
    Q_OBJECT

    public:
        explicit Mapping(QObject *parent = nullptr);
        ~Mapping();

        void updateMapFromLidar(double robot_x, double robot_y, double robot_phi,
                                const std::vector<LaserData>& lidar_data);
    signals:


    private:
        double map_width_;
        double map_height_;
        double cell_size_;
        cv::Mat occupancy_grid_;

        double origin_x_;
        double origin_y_;

        
};


#endif // MAPPING_H
