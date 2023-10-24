/**
 * @file PIDController.cpp
 * @author Driver - Manav Bhavesh Nagda
 *         Navigator - Sameer Arjun Satheesh
 *         Design Keeper - Ishaan Samir Parikh
 * @brief Implementation of a PID controller for Ackermann Kinematic model.
 * @version 0.1
 * @date 2023
 */

#include "PIDController.hpp"
#include <iostream>
#include <numeric>
#include <cmath>

#include "PIDController.hpp"
#include <iostream>
#include <numeric>
#include <cmath>

/**
 * @brief Constructor for the PIDController class.
 * 
 * @param velP The proportional constant for velocity control.
 * @param velI The integral constant for velocity control.
 * @param velD The derivative constant for velocity control.
 * @param dt The time step.
 * @param headP The proportional constant for heading control.
 * @param headI The integral constant for heading control.
 * @param headD The derivative constant for heading control.
 */
PIDController::PIDController(double velP, double velI, double velD, double dt,
                             double headP, double headI, double headD) {
    velKp = velP;
    velKi = velI;
    velKd = velD;
    deltaT = dt;
    headKp = headP;
    headKi = headI;
    headKd = headD;
}

/**
 * @brief Computes the PID control outputs for velocity and heading.
 * 
 * @return A vector containing the computed PID values for velocity and heading.
 */
std::vector<double> PIDController::computePID() {
    double Pvel, Ivel, Dvel;
    double Phead, Ihead, Dhead;
    std::vector<double> pidOut;

    if (velocityErrors.empty() || headingErrors.empty()) return pidOut;

    // Calculate the proportional term for velocity control.
    Pvel = velKp * velocityErrors.back();
    double IvelSum = std::accumulate(velocityErrors.begin(), velocityErrors.end(), 0.0);
    // Calculate the integral term for velocity control.
    Ivel = velKi * IvelSum;

    if (velocityErrors.size() < 2)
        Dvel = 0;
    else
        // Calculate the derivative term for velocity control.
        Dvel = velKd * ((velocityErrors.back() - velocityErrors[velocityErrors.size() - 2]) / deltaT);

    // Calculate the overall PID output for velocity control.
    double velPIDOut = Pvel + Ivel + Dvel;

    // Repeat the same process for heading control.
    Phead = headKp * headingErrors.back();
    double IheadSum = std::accumulate(headingErrors.begin(), headingErrors.end(), 0.0);
    Ihead = headKi * IheadSum;

    if (headingErrors.size() < 2)
        Dhead = 0;
    else
        Dhead = headKd * ((headingErrors.back() - headingErrors[headingErrors.size() - 2]) / deltaT);

    double headPIDOut = Phead + Ihead + Dhead;

    // Store the computed PID values in the output vector.
    pidOut.push_back(velPIDOut);
    pidOut.push_back(headPIDOut);

    return pidOut;
}

/**
 * @brief Retrieves the velocity proportional constant (Kp).
 * 
 * @return The velocity proportional constant.
 */
double PIDController::getVelocityProportionalConstant() {
    return velKp;
}

/**
 * @brief Retrieves the velocity integral constant (Ki).
 * 
 * @return The velocity integral constant.
 */
double PIDController::getVelocityIntegralConstant() {
    return velKi;
}

/**
 * @brief Retrieves the velocity derivative constant (Kd).
 * 
 * @return The velocity derivative constant.
 */
double PIDController::getVelocityDerivativeConstant() {
    return velKd;
}

/**
 * @brief Retrieves the time step (deltaT).
 * 
 * @return The time step.
 */
double PIDController::getDeltaTime() {
    return deltaT;
}

/**
 * @brief Retrieves the heading proportional constant (Kp).
 * 
 * @return The heading proportional constant.
 */
double PIDController::getHeadingProportionalConstant() {
    return headKp;
}

/**
 * @brief Retrieves the heading integral constant (Ki).
 * 
 * @return The heading integral constant.
 */
double PIDController::getHeadingIntegralConstant() {
    return headKi;
}

/**
 * @brief Retrieves the heading derivative constant (Kd).
 * 
 * @return The heading derivative constant.
 */
double PIDController::getHeadingDerivativeConstant() {
    return headKd;
}

/**
 * @brief Retrieves the vector of velocity errors.
 * 
 * @return The vector containing velocity errors.
 */
const std::vector<double>& PIDController::getVelocityErrors() const {
    return velocityErrors;
}

/**
 * @brief Retrieves the vector of heading errors.
 * 
 * @return The vector containing heading errors.
 */
const std::vector<double>& PIDController::getHeadingErrors() const {
    return headingErrors;
}

/**
 * @brief Retrieves the vector of velocity errors.
 * 
 * @return The vector containing velocity errors.
 */
const std::vector<double>& PIDController::getVelocityErrors() const {
    return velocityErrors;
}

/**
 * @brief Retrieves the vector of heading errors.
 * 
 * @return The vector containing heading errors.
 */
const std::vector<double>& PIDController::getHeadingErrors() const {
    return headingErrors;
}

/**
 * @brief Computes and stores the velocity and heading errors.
 * 
 * @param targetVelocity The desired velocity.
 * @param currentVelocity The current velocity.
 * @param targetHeading The desired heading (in radians).
 * @param currentHeading The current heading (in radians).
 */
void PIDController::computeErrors(double targetVelocity, double currentVelocity, double targetHeading, double currentHeading) {
    double velocityError = targetVelocity - currentVelocity;
    std::cout << "Velocity Error: " << velocityError << "\n";

    double headingError = targetHeading - currentHeading;
    std::cout << "Heading Error: " << headingError * 180 / M_PI;

    // Store the computed errors in the respective vectors.
    velocityErrors.push_back(velocityError);
    headingErrors.push_back(headingError);
}
