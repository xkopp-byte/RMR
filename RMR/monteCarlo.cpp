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

void MonteCarlo::findYourself() // hlavna slucka
{
    if (stop_flag && foundMyself)
    {
        return;
    }
    
    
    if (!loadMap("maps/finalMap.txt"))
    {
        std::cout << "Map loading failed. Monte Carlo initialization stopped." << std::endl;
        return;
    }


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

    // fitness();
//    roulette();
//      cutoff();
//  

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
    fclose(map_file);
    return true;
}


void MonteCarlo::initParticlesGlobal(int num_particles)
{
    particles.clear();
    particles.reserve(num_particles);
    
    for (int i = 0; i < num_particles; i++)
    {
        Particle p;
        for (int attempt = 0; attempt < MAX_GENERATIONS; attempt++)
        {
            p.x = randDouble(mapMinX, mapMaxX);
            p.y = randDouble(mapMinY, mapMaxY);
            if (isFreeSpace(p.x, p.y))
            {
                break; // valid particle found
            }
        } 
        p.theta = randDouble(-M_PI, M_PI);
        p.weight = 1.0 / num_particles;
        p.is_valid_particle = true;
        p.error = 0.0;
        particles.push_back(p);
    }
}

void MonteCarlo::motionUpdate(double dx, double dy, double dtheta)
{
    for (auto& p : particles)
    {
        if (!p.is_valid_particle)
            continue;

        p.x += dx + randGaussian(0, 0.01); // add some noise
        p.y += dy + randGaussian(0, 0.01);
        p.theta += dtheta + randGaussian(0, 0.005);

        if (!isFreeSpace(p.x, p.y))
        {
            p.is_valid_particle = false; // mark as invalid if out of bounds
        }
    }
    
}



void MonteCarlo::roulette()
{
    int n = particles.size();
    if (n == 0) return;
    double max_weight = 0.0;
    for (int i = 0; i < n; ++i)
    {
        particles[i].weight = 1.0 / (particles[i].error + 1e-6);
        if (particles[i].weight > max_weight)
        {
            max_weight = particles[i].weight;
        }
    }

    std::vector<Particle> new_particles;
    new_particles.reserve(n);
    int index = static_cast<int>(randDouble(0, n - 1));
    double beta = 0.0;

    for (int i = 0; i < n; ++i)
    {
        beta += randDouble(0, 2.0 * max_weight);
        while (beta > particles[index].weight)
        {
            beta -= particles[index].weight;
            index = (index + 1) % n;
        }
        new_particles.push_back(particles[index]);
    }

    particles = new_particles;

    double weight_sum = 0.0;
    for (int i = 0; i < n; ++i)
    {
        weight_sum += particles[i].weight;
    }
    if (weight_sum > 0)
    {
        for (int i = 0; i < n; ++i)
        {
            particles[i].weight /= weight_sum;
        }
    }
}

void MonteCarlo::cutoff()
{
    int num_particles = particles.size();
    for (int i = 0; i < num_particles; ++i)
    {
        if (particles[i].weight > 10000.0)
        {
            for (int attempt = 0; attempt < MAX_GENERATIONS; attempt++)
            {
                particles[i].x = randDouble(mapMinX, mapMaxX);
                particles[i].y = randDouble(mapMinY, mapMaxY);
                if (isFreeSpace(particles[i].x, particles[i].y))
                {
                    break; // valid particle found
                }
            } 
            particles[i].theta = randDouble(-M_PI, M_PI);
            particles[i].weight = 1.0 / num_particles;
            particles[i].is_valid_particle = true;
            particles[i].error = 0.0;
        }
    }
}


double MonteCarlo::randDouble(double min, double max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(min, max);
    return dis(gen);
}

double MonteCarlo::randGaussian(double mean, double stddev)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<double> dist(mean, stddev);
    return dist(gen);
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




// SETTERS AND GETTERS

void MonteCarlo::updateLidarData(const std::vector<LaserData>& lidarData)
{
    currentLidarData_ = lidarData;

    // debug print
    // std::cout << "Lidar data updated. Number of points: " << currentLidarData_.size() << std::endl;

}

void MonteCarlo::setActualVelocity(double forw_speed, double rot_speed)
{
    forward_speed_ = forw_speed;
    rotation_speed_ = rot_speed;

    // std::cout << "Actual velocity updated - Forward: " << forw_speed << " mm/s, Rotation: " << rot_speed << " deg/s" << std::endl;
}


