/**
 * @file RobotModel.cpp
 * @author Driver - Sameer Arjun S
           Navigator -  Ishaan Samir Parikh
           Design Keeper - Manav Nagda
 * @brief Represents a class to implement the robot model
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "RobotModel.hpp"
#include <cmath>

RobotModel::RobotModel(double wheelbase, double trackWidth, double maxSteeringAngle)
    : wheelbase_(wheelbase), trackWidth_(trackWidth), maxSteeringAngle_(maxSteeringAngle),
      x_(0.0), y_(0.0), theta_(0.0), velocity_(0.0) {
}

void RobotModel::setInitialState(double x, double y, double theta, double velocity) {
    x_ = x;
    y_ = y;
    theta_ = theta;
    velocity_ = velocity;
}

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

void RobotModel::calculateSteeringAndDriveVelocities(double desiredTurningRadius, double& steeringAngle, double& leftWheelVelocity, double& rightWheelVelocity) {
    // Calculate the steering angle based on the desired turning radius and Ackermann kinematic model.
    steeringAngle = std::atan(wheelbase_ / desiredTurningRadius);

    // Calculate left and right wheel velocities for the Ackermann model.
    double turningRadius = wheelbase_ / std::tan(steeringAngle);
    leftWheelVelocity = velocity_ * (turningRadius - (trackWidth_ / 2.0)) / turningRadius;
    rightWheelVelocity = velocity_ * (turningRadius + (trackWidth_ / 2.0)) / turningRadius;
}

void RobotModel::getState(double& x, double& y, double& theta, double& velocity) {
    x = x_;
    y = y_;
    theta = theta_;
    velocity = velocity_;
}
