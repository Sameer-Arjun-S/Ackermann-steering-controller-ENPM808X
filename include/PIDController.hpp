/**
 * @file PIDController.cpp
 * @author Driver - Manav Bhavesh Nagda
 *         Navigator - Sameer Arjun Satheesh
 *         Design Keeper - Ishaan Samir Parikh
 * @brief Implementation of a PID controller for Ackermann Kinematic model.
 * @version 0.1
 * @date 2023
 */

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <vector>

class PIDController {
public:
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
    PIDController(double velP, double velI, double velD, double dt, double headP, double headI, double headD);

    /**
     * @brief Computes the PID control outputs for velocity and heading.
     * 
     * @return A vector containing the computed PID values for velocity and heading.
     */
    std::vector<double> computePID();

    /**
     * @brief Retrieves the velocity proportional constant (Kp).
     * 
     * @return The velocity proportional constant.
     */
    double getVelocityProportionalConstant();

    /**
     * @brief Retrieves the velocity integral constant (Ki).
     * 
     * @return The velocity integral constant.
     */
    double getVelocityIntegralConstant();

    /**
     * @brief Retrieves the velocity derivative constant (Kd).
     * 
     * @return The velocity derivative constant.
     */
    double getVelocityDerivativeConstant();

    /**
     * @brief Retrieves the time step (deltaT).
     * 
     * @return The time step.
     */
    double getDeltaTime();

    /**
     * @brief Retrieves the heading proportional constant (Kp).
     * 
     * @return The heading proportional constant.
     */
    double getHeadingProportionalConstant();

    /**
     * @brief Retrieves the heading integral constant (Ki).
     * 
     * @return The heading integral constant.
     */
    double getHeadingIntegralConstant();

    /**
     * @brief Retrieves the heading derivative constant (Kd).
     * 
     * @return The heading derivative constant.
     */
    double getHeadingDerivativeConstant();

    /**
     * @brief Retrieves the vector of velocity errors.
     * 
     * @return The vector containing velocity errors.
     */
    const std::vector<double>& getVelocityErrors() const;

    /**
     * @brief Retrieves the vector of heading errors.
     * 
     * @return The vector containing heading errors.
     */
    const std::vector<double>& getHeadingErrors() const;

    /**
     * @brief Computes and stores the velocity and heading errors.
     * 
     * @param targetVelocity The desired velocity.
     * @param currentVelocity The current velocity.
     * @param targetHeading The desired heading (in radians).
     * @param currentHeading The current heading (in radians).
     */
    void computeErrors(double targetVelocity, double currentVelocity, double targetHeading, double currentHeading);

private:
    double velKp;
    double velKi;
    double velKd;
    double deltaT;
    double headKp;
    double headKi;
    double headKd;
    std::vector<double> velocityErrors;
    std::vector<double> headingErrors;
};

#endif // PID_CONTROLLER_HPP
