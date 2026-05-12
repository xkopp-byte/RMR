#include "monteCarlo.h"
#include <iostream>
#include <chrono>

MonteCarlo::MonteCarlo() : foundMyself(true), stop_flag(false) {}

MonteCarlo::~MonteCarlo() {
    stopThread();
}

void MonteCarlo::startFindYourselfThread() {
    foundMyself = false;
    stop_flag = false;
    if (mc_thread.joinable()) {
        mc_thread.join();
    }
    mc_thread = std::thread(&MonteCarlo::findYourself, this);
}

void MonteCarlo::findYourself() {
    // // The logic of monteCarlo will be written later
    // while (!stop_flag && !foundMyself) {
    //     // Simulating the thread running and eventually setting foundMyself to true 
    //     // will be added here later.
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    if (stop_flag && foundMyself)
    {
        return;
    }
    

    loadMap("maps/finalMap.txt");
    initParticlesGlobal(100);

    std::cout << "Particles initialized: " << particles.size() << std::endl;

    double sum = 0.0;
    for (const auto& p : particles) 
    {
        sum += p.weight;
        std::cout << "Particle " << &p - &particles[0] << " - x: " << p.x << ", y: " << p.y 
                  << ", theta: " << p.theta << ", weight: " << p.weight 
                  << ", valid: " << p.is_valid_particle 
                  << ", error: " << p.error << std::endl;
    }
    std::cout << "Weight sum: " << sum << std::endl;


}

void MonteCarlo::stopThread() {
    stop_flag = true;
    if (mc_thread.joinable()) {
        mc_thread.join();
    }
}

bool MonteCarlo::loadMap(const char* map_filename) 
{
    
    map_file = fopen(map_filename, "r");
    if (!map_file) {
        printf("Error: Could not open map file %s\n", map_filename);
        return false;
    }

    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            int ch = fgetc(map_file);
            while (ch == '\n' || ch == '\r' || ch == ' ') {
                ch = fgetc(map_file); 
            }
            if (ch == EOF) break;
            
            mapData[y][x] = (char)ch;
        }
    }

    return true;
}


void MonteCarlo::initParticlesGlobal(int num_particles)
{
    particles.clear();
    particles.reserve(num_particles);
    
    for (int i = 0; i < num_particles; i++)
    {
        Particle p;
        do
        {
            p.x = randDouble(mapMinX, mapMaxX);
            p.y = randDouble(mapMinY, mapMaxY);
        }   while (!isFreeSpace(p.x, p.y));
        p.theta = randDouble(-M_PI, M_PI);
        p.weight = 1.0 / num_particles;
        p.is_valid_particle = true;
        p.error = 0.0;
        particles.push_back(p);
    }
}



double MonteCarlo::randDouble(double min, double max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(min, max);
    return dis(gen);
}

bool MonteCarlo::isFreeSpace(double x, double y)
{
    int map_x = static_cast<int>((x - mapMinX) / (mapMaxX - mapMinX) * MAP_WIDTH);
    int map_y = static_cast<int>((y - mapMinY) / (mapMaxY - mapMinY) * MAP_HEIGHT);

    if (map_x < 0 || map_x >= MAP_WIDTH || map_y < 0 || map_y >= MAP_HEIGHT)
        return false; // Out of bounds

    if (mapData[map_y][map_x] == '0')
    {
        return true; // Free space
    }
    return false; // Occupied space
}



double normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}




