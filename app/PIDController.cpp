/**
 * @file PIDController.hpp
 * @author Driver: Manav Nagda, Navigator: Sameer Arjun S, Design Keeper: Ishaan Parik
 * @brief Contains the PIDController class definition.
 */

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <vector>

/**
 * @class PIDController
 * @brief A Proportional-Integral-Derivative (PID) controller for velocity and heading control.
 */
class PIDController {
public:
    /**
     * @brief Constructor for PIDController.
     * @param velP The proportional constant for velocity control.
     * @param velI The integral constant for velocity control.
     * @param velD The derivative constant for velocity control.
     * @param dt The time step.
     * @param headP The proportional constant for heading control.
     * @param headI The integral constant for heading control.
     * @param headD The derivative constant for heading control.
     */
    PIDController(double velP, double velI, double velD, double dt,
                  double headP, double headI, double headD);

    /**
     * @brief Computes the PID control outputs for velocity and heading.
     * @return A vector containing the computed PID values for velocity and heading.
     */
    std::vector<double> computePID();

    /**
     * @brief Retrieves the velocity proportional constant (Kp).
     * @return The velocity proportional constant.
     */
    double getVelocityProportionalConstant();

    /**
     * @brief Retrieves the velocity integral constant (Ki).
     * @return The velocity integral constant.
     */
    double getVelocityIntegralConstant();

    /**
     * @brief Retrieves the velocity derivative constant (Kd).
     * @return The velocity derivative constant.
     */
    double getVelocityDerivativeConstant();

    /**
     * @brief Retrieves the time step (deltaT).
     * @return The time step.
     */
    double getDeltaTime();

    /**
     * @brief Retrieves the heading proportional constant (Kp).
     * @return The heading proportional constant.
     */
    double getHeadingProportionalConstant();

    /**
     * @brief Retrieves the heading integral constant (Ki).
     * @return The heading integral constant.
     */
    double getHeadingIntegralConstant();

    /**
     * @brief Retrieves the heading derivative constant (Kd).
     * @return The heading derivative constant.
     */
    double getHeadingDerivativeConstant();

    /**
     * @brief Computes and stores the velocity and heading errors.
     * @param targetVelocity The desired velocity.
     * @param currentVelocity The current velocity.
     * @param targetHeading The desired heading (in radians).
     * @param currentHeading The current heading (in radians).
     */
    void computeErrors(double targetVelocity, double currentVelocity, double targetHeading, double currentHeading);

private:
    double velKp; /**< The proportional constant for velocity control. */
    double velKi; /**< The integral constant for velocity control. */
    double velKd; /**< The derivative constant for velocity control. */
    double deltaT; /**< The time step. */
    double headKp; /**< The proportional constant for heading control. */
    double headKi; /**< The integral constant for heading control. */
    double headKd; /**< The derivative constant for heading control. */
    std::vector<double> velocityErrors; /**< A vector to store velocity errors. */
    std::vector<double> headingErrors; /**< A vector to store heading errors. */
};

#endif // PID_CONTROLLER_HPP
