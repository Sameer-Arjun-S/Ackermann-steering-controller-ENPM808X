/**
 * @file RobotModel.cpp
 * @author Driver - Sameer Arjun S
 *         Navigator -  Ishaan Samir Parikh
 *         Design Keeper - Manav Nagda
 * @brief Implementation of a class to represent a robot model.
 * @version 0.1
 * @date 2023
 */

#include "RobotModel.hpp"
#include <cmath>

/**
 * @brief Constructor for the RobotModel class.
 * 
 * @param wheelbase The distance between the front and rear axles of the robot.
 * @param trackWidth The distance between the left and right wheels of the robot.
 * @param maxSteeringAngle The maximum allowable steering angle for the robot.
 */
RobotModel::RobotModel(double wheelbase, double trackWidth, double maxSteeringAngle)
    : wheelbase_(wheelbase), trackWidth_(trackWidth), maxSteeringAngle_(maxSteeringAngle),
      x_(0.0), y_(0.0), theta_(0.0), velocity_(0.0) {
}

/**
 * @brief Sets the initial state of the robot model.
 * 
 * @param x The initial x-coordinate of the robot.
 * @param y The initial y-coordinate of the robot.
 * @param theta The initial orientation (in radians) of the robot.
 * @param velocity The initial velocity of the robot.
 */
void RobotModel::setInitialState(double x, double y, double theta, double velocity) {
    x_ = x;
    y_ = y;
    theta_ = theta;
    velocity_ = velocity;
}

/**
 * @brief Updates the state of the robot model based on steering angle and time step.
 * 
 * @param steeringAngle The steering angle (in radians) applied to the robot's front wheels.
 * @param dt The time step for the state update.
 */
void RobotModel::updateState(double steeringAngle, double dt) {
    // Apply the Ackermann kinematic model with a maximum steering angle constraint.
    if (std::abs(steeringAngle) > maxSteeringAngle_) {
        // Limit the steering angle to the maximum allowed.
        steeringAngle = std::copysign(maxSteeringAngle_, steeringAngle);
    }

    // Calculate left and right wheel velocities based on the steering angle and velocity.
    double turningRadius = wheelbase_ / std::tan(steeringAngle);
    double leftWheelVelocity = velocity_ * (turningRadius - (trackWidth_ / 2.0)) / turningRadius;
    double rightWheelVelocity = velocity_ * (turningRadius + (trackWidth_ / 2.0)) / turningRadius;

    // Update the robot's state.
    double deltaTheta = (leftWheelVelocity - rightWheelVelocity) / trackWidth_ * dt;
    double deltaX = 0.5 * (leftWheelVelocity + rightWheelVelocity) * std::cos(theta_) * dt;
    double deltaY = 0.5 * (leftWheelVelocity + rightWheelVelocity) * std::sin(theta_) * dt;

    x_ += deltaX;
    y_ += deltaY;
    theta_ += deltaTheta;
}

/**
 * @brief Calculates the steering angle and left/right wheel velocities based on a desired turning radius.
 * 
 * @param desiredTurningRadius The desired turning radius for the robot's path.
 * @param steeringAngle The calculated steering angle (output).
 * @param velocity_ The calculated velocity (output).
 */
void RobotModel::calculateSteeringAndDriveVelocities(double desiredTurningRadius, double& steeringAngle, double& velocity) {
    // Calculate the steering angle based on the desired turning radius and Ackermann kinematic model.
    steeringAngle = std::atan(wheelbase_ / desiredTurningRadius);

    // Calculate left and right wheel velocities for the Ackermann model.
    double turningRadius = wheelbase_ / std::tan(steeringAngle);
    leftWheelVelocity = velocity_ * (turningRadius - (trackWidth_ / 2.0)) / turningRadius;
    rightWheelVelocity = velocity_ * (turningRadius + (trackWidth_ / 2.0)) / turningRadius;
}

/**
 * @brief Retrieves the current state of the robot model.
 * 
 * @param x The current x-coordinate of the robot (output).
 * @param y The current y-coordinate of the robot (output).
 * @param theta The current orientation (in radians) of the robot (output).
 * @param velocity The current velocity of the robot (output).
 */
void RobotModel::getState(double& x, double& y, double& theta, double& velocity) {
    x = x_;
    y = y_;
    theta = theta_;
    velocity = velocity_;
}