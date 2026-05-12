#ifndef MONTECARLO_H
#define MONTECARLO_H

#include <atomic>
#include <thread>
#include <vector>
#include <stdio.h>
#include <random>
#include <cmath>
// #include "rplidar.h"
#include "librobot/librobot.h"


class MonteCarlo {
public:
    MonteCarlo();
    ~MonteCarlo();

    void findYourself();
    void startFindYourselfThread();
    void stopThread();

    std::atomic<bool> foundMyself;

    void setActualVelocity(double forw_speed, double rot_speed);
    void updateLidarData(const std::vector<LaserData>& lidarData);

private:
    std::thread mc_thread;
    std::atomic<bool> stop_flag;

    // Map dimensions
    #define MAP_WIDTH 604 + 100
    #define MAP_HEIGHT 682 + 100
    double mapMinX = 0 - 50;
    double mapMaxX = 6.02 + 50;
    double mapMinY = -1.60 - 50;
    double mapMaxY = 5.21 + 50;

    double forward_speed_;
    double rotation_speed_;

    double time_period_ = 1/40.0;

    #define MAX_GENERATIONS 1000

    std::vector<LaserData> currentLidarData_;

    struct Particle {
        double x;
        double y; 
        double theta; // orientation of the particle
        double weight; // weight of the particle 
        bool is_valid_particle; // invalid if out of bounds
        double error; // error between predicted and actual sensor readings
        // mozno sa pridaju dalsie atributy
    };

    std::vector<Particle> particles;


    FILE* map_file; 
    bool loadMap(const char* map_filename);

    char mapData[MAP_HEIGHT][MAP_WIDTH];
    
    void initParticlesGlobal(int num_particles);
    void motionUpdate(double dx, double dy, double dtheta);

    void roulette();
    void cutoff();

    double randDouble(double min, double max);
    double randGaussian(double mean, double stddev);

    bool isFreeSpace(double x, double y);

    double normalizeAngle(double angle);
};

#endif // MONTECARLO_H
