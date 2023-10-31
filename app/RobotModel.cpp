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
#include<iostream>

/**
 * @brief Constructor for the RobotModel class.
 * 
 * @param wheelbase The distance between the front and rear axles of the robot.
 * @param trackWidth The distance between the left and right wheels of the robot.
 * @param maxSteeringAngle The maximum allowable steering angle for the robot.
 */
RobotModel::RobotModel(double wheelbase, double wheelRadius, double trackWidth)
    : wheelbase_(wheelbase), wheelRadius_(wheelRadius), trackWidth_(trackWidth),
      alpha_i_(0.0), alpha_o_(0.0), omega_i_(0.0), omega_o_(0.0),
      heading_(0.0), speed_(0.0),
      maxSteeringAngle_(0.0), x_(0.0), y_(0.0), theta_(0.0), velocity_(0.0) {
}

/**
 * @brief Sets the initial state of the robot model.
 * 
 * @param x The initial x-coordinate of the robot.
 * @param y The initial y-coordinate of the robot.
 * @param theta The initial orientation (in radians) of the robot.
 * @param velocity The initial velocity of the robot.
 */
void RobotModel::setInitialState(double x, double y,
                                double theta, double velocity) {
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
    // Apply the Ackermann kinematic model with a max steeringangle constraint.
    if (std::abs(steeringAngle) > maxSteeringAngle_) {
        // Limit the steering angle to the maximum allowed.
        steeringAngle = std::copysign(maxSteeringAngle_, steeringAngle);
    }

    // Calculate left and right wheel velocities based on the
    //  steering angle and velocity.
    double turningRadius = wheelbase_ / std::tan(steeringAngle);
    double leftWheelVelocity = velocity_ *
        (turningRadius - (trackWidth_ / 2.0)) / turningRadius;
    double rightWheelVelocity = velocity_ *
        (turningRadius + (trackWidth_ / 2.0)) / turningRadius;

    // Update the robot's state.
    double deltaTheta = (leftWheelVelocity - rightWheelVelocity)
                         / trackWidth_ * dt;
    double deltaX = 0.5 * (leftWheelVelocity + rightWheelVelocity)
                         * std::cos(theta_) * dt;
    double deltaY = 0.5 * (leftWheelVelocity + rightWheelVelocity)
                         * std::sin(theta_) * dt;

    x_ += deltaX;
    y_ += deltaY;
    theta_ += deltaTheta;
}

/**
 * @brief Retrieves the current state of the robot model.
 * 
 * @param x The current x-coordinate of the robot (output).
 * @param y The current y-coordinate of the robot (output).
 * @param theta The current orientation (in radians) of the robot (output).
 * @param velocity The current velocity of the robot (output).
 */
void RobotModel::getState(double& x, double& y,
                                double& theta, double& velocity) {
    x = x_;
    y = y_;
    theta = theta_;
    velocity = velocity_;
}

void RobotModel::Simulate_robot_model(double PID_heading_output,
                    double PID_velocity_output, double dt) {
    double R;
    double deltaTheta = 0;
    double newSpeed = 0;

    if (PID_heading_output > 0) {
        // Robot is executing a left turn
        R = wheelbase_ * 1 / std::tan(PID_heading_output);
        alpha_i_ = std::atan(wheelbase_ /
                            (R - (trackWidth_ / 2)));
        alpha_o_ = std::atan(wheelbase_ /
                            (R + (trackWidth_ / 2)));
        omega_o_ += PID_velocity_output;
        deltaTheta = (wheelRadius_ * omega_o_ * dt) / (R + (trackWidth_ / 2));
        omega_i_ = (deltaTheta * (R - (trackWidth_ / 2))) / (wheelRadius_ * dt);
        newSpeed = std::abs((R * deltaTheta) / dt);
    } else if (PID_heading_output < 0) {
        // Robot is executing a right turn
        R = wheelbase_ * 1 / std::tan(PID_heading_output);
        alpha_o_ = std::atan(wheelbase_ /
                        (R - (trackWidth_ / 2)));
        alpha_i_ = std::atan(wheelbase_ /
                        (R + (trackWidth_ / 2)));
        omega_i_ += PID_velocity_output;
        deltaTheta = (wheelRadius_ * omega_i_ * dt) / (R + (trackWidth_ / 2));
        omega_o_ = (deltaTheta * (R - (trackWidth_ / 2))) / (wheelRadius_ * dt);
        newSpeed = std::abs((R * deltaTheta) / dt);
    } else {
        // Robot is going straight
        alpha_o_ = 0;
        alpha_i_ = 0;
        if (omega_i_ >= omega_o_) {
            omega_o_ += PID_velocity_output;
            omega_i_ = omega_o_;
        } else {
            omega_i_ += PID_velocity_output;
            omega_o_ = omega_i_;
        }
        newSpeed = omega_i_ * wheelRadius_;
    }

    heading_ += deltaTheta;
    speed_ = newSpeed;

    std::cout << "heading: " << 180 * heading_ / M_PI << "\n";
    std::cout << "Speed: " << speed_ << "\n";
    std::cout << "**********OUTPUTS***************" << "\n";
    std::cout << "Inner steering angle: " << alpha_i_ << "\n";
    std::cout << "Outer steering angle: " << alpha_o_ << "\n";
    std::cout << "Inner wheel velocity: " << omega_i_ * wheelRadius_ << "\n";
    std::cout << "Outer wheel velocity: " << omega_o_ * wheelRadius_ << "\n";
    std::cout << "********************************" << "\n";
}

double RobotModel::getHeading() {
    return heading_;
}

double RobotModel::getSpeed() {
    return speed_;
}
