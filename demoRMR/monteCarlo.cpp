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
    // The logic of monteCarlo will be written later
    while (!stop_flag && !foundMyself) {
        // Simulating the thread running and eventually setting foundMyself to true 
        // will be added here later.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MonteCarlo::stopThread() {
    stop_flag = true;
    if (mc_thread.joinable()) {
        mc_thread.join();
    }
}
