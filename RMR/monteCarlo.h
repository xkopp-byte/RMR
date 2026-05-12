#ifndef MONTECARLO_H
#define MONTECARLO_H

#include <atomic>
#include <thread>
#include <vector>
#include <stdio.h>
#include <random>
#include <cmath>


class MonteCarlo {
public:
    MonteCarlo();
    ~MonteCarlo();

    void findYourself();
    void startFindYourselfThread();
    void stopThread();

    std::atomic<bool> foundMyself;

private:
    std::thread mc_thread;
    std::atomic<bool> stop_flag;

    // Map dimensions
    #define MAP_WIDTH 604
    #define MAP_HEIGHT 682
    double mapMinX = 0;
    double mapMaxX = 6.02;
    double mapMinY = -1.60;
    double mapMaxY = 5.21;



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

    double randDouble(double min, double max);
    bool isFreeSpace(double x, double y);

    double normalizeAngle(double angle);
};

#endif // MONTECARLO_H
