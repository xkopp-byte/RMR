#ifndef MONTECARLO_H
#define MONTECARLO_H

#include <atomic>
#include <thread>

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
};

#endif // MONTECARLO_H
