/**
 * @file PIDController.cpp
 * @author Driver - Manav Bhavesh Nagda
           Navigator - Sameer Arjun Satheesh
           Design Keeper - Ishaan Samir Parikh
 * @brief Represents a class to implement PID controller for Ackermann Kinematic model
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "PIDController.hpp"
#include <iostream>
#include <numeric>
#include <cmath>
#define M_PI 3.14159265358979323846

PIDController::PIDController(double velP, double velI, double velD, double dt,
                             double headP, double headI, double headD) {
    velKp = 1.0;
    velKi = 0.5;
    velKd = 0.3;
    deltaT = 0.1;
    headKp = 1.0;
    headKi = 0.5;
    headKd = 0.3;
}

std::vector<double> PIDController::computePID() {
    double Pvel, Ivel, Dvel;
    double Phead, Ihead, Dhead;
    std::vector<double> pidOut;

    if (velocityErrors.empty() || headingErrors.empty()) return pidOut;

    Pvel = velKp * velocityErrors.back();
    double IvelSum = std::accumulate(velocityErrors.begin(), velocityErrors.end(), 0.0);
    Ivel = velKi * IvelSum;

    if (velocityErrors.size() < 2)
        Dvel = 0;
    else
        Dvel = velKd * ((velocityErrors.back() - velocityErrors[velocityErrors.size() - 2]) / deltaT);

    double velPIDOut = Pvel + Ivel + Dvel;

    Phead = headKp * headingErrors.back();
    double IheadSum = std::accumulate(headingErrors.begin(), headingErrors.end(), 0.0);
    Ihead = headKi * IheadSum;

    if (headingErrors.size() < 2)
        Dhead = 0;
    else
        Dhead = headKd * ((headingErrors.back() - headingErrors[headingErrors.size() - 2]) / deltaT);

    double headPIDOut = Phead + Ihead + Dhead;

    pidOut.push_back(velPIDOut);
    pidOut.push_back(headPIDOut);

    return pidOut;
}

double PIDController::getVelocityProportionalConstant() {
    return velKp;
}

double PIDController::getVelocityIntegralConstant() {
    return velKi;
}

double PIDController::getVelocityDerivativeConstant() {
    return velKd;
}

double PIDController::getDeltaTime() {
    return deltaT;
}

double PIDController::getHeadingProportionalConstant() {
    return headKp;
}

double PIDController::getHeadingIntegralConstant() {
    return headKi;
}

double PIDController::getHeadingDerivativeConstant() {
    return headKd;
}

void PIDController::computeErrors(double targetVelocity, double currentVelocity, double targetHeading, double currentHeading) {
    double velocityError = targetVelocity - currentVelocity;
    std::cout << "Velocity Error: " << velocityError << "\n";

    double headingError = targetHeading - currentHeading;
    std::cout << "Heading Error: " << headingError * 180 / M_PI;

    velocityErrors.push_back(velocityError);
    headingErrors.push_back(headingError);
}
