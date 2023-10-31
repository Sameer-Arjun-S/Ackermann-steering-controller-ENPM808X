/**
 * @file RobotSimulation.cpp
 * @author Driver - Ishaan Samir Parikh
 *         Navigator - Manav Bhavesh Nagda
 *         Design Keeper - Sameer Arjun Sateesh
 * @brief Represents a class for simulating a robot's control system.
 *
 * The `RobotSimulation` class allows navigation and control of a robot's motion. It integrates
 * the robot's physical behavior with a PID controller to achieve target heading and velocity.
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "RobotSimulation.hpp"
#include <iostream>
#include <cmath>
#include <vector>

/**
 * @brief Constructs a new Robot Simulation object with the specified parameters.
 *
 * This constructor initializes a robot simulation with the given characteristics, including the robot's physical properties
 * and PID controller parameters.
 *
 * @param wheelbase The wheelbase of the robot (distance between front and rear axles).
 * @param trackWidth The track width of the robot (distance between the two wheels on the same axle).
 * @param maxSteeringAngle The maximum allowed steering angle for the robot's wheels.
 * @param velP The proportional constant for velocity control.
 * @param velI The integral constant for velocity control.
 * @param velD The derivative constant for velocity control.
 * @param deltaT The time step for control calculations.
 * @param headP The proportional constant for heading control.
 * @param headI The integral constant for heading control.
 * @param headD The derivative constant for heading control.
 */
RobotSimulation::RobotSimulation(double wheelbase, double trackWidth,
                            double maxSteeringAngle,
                            double velP, double velI, double velD,
                            double deltaT,
                            double headP, double headI, double headD)
    : robot(wheelbase, trackWidth, maxSteeringAngle),
            controller(velP, velI, velD, deltaT, headP, headI, headD) {
}

/**
 * @brief Runs the robot simulation control loop.
 *
 * This function allows the user to interactively input target heading and velocity, and the robot simulation
 * attempts to control the robot's movement to reach these targets using the PID controller. The control loop
 * continues until the user exits or convergence to the target is achieved.
 */

void RobotSimulation::runSimulation(double targetHeading,
                                 double targetVelocity) {
    double currentVelocity = 0.0;
    double currentTheta = 0.0;

    // double targetHeading, targetVelocity;
    if (targetHeading == 1000.0 && targetVelocity == 1000.0) {
        // Prompt the user to enter the target heading and velocity
        std::cout << "Enter the target heading (in radians): ";
        std::cin >> targetHeading;

        std::cout << "Enter the target velocity: ";
        std::cin >> targetVelocity;
    }

    const int maxIterations = 30;
    const double convergenceThreshold = 3;  // Adjust as needed

    for (int i = 0; i < maxIterations; i++) {
        std::cout << "Iteration " << i << std::endl;
        // std::cout << "init" << currentVelocity;
        // Compute PID errors
        controller.computeErrors(targetVelocity, currentVelocity,
                                     targetHeading, currentTheta);

        // Get the PID controller outputs
        std::vector<double> controlOutputs = controller.computePID();

        // Extract control outputs
        double steeringAngle = controlOutputs[1];
        double velocityOutput = controlOutputs[0];


        // Simulate the robot model with Ackermann kinematic model
        robot.Simulate_robot_model(steeringAngle, velocityOutput,
                                     controller.getDeltaTime());

        // Simulate the robot model with Ackermann kinematic model
        robot.updateState(steeringAngle, controller.getDeltaTime());

        // Get the current state of the robot
        double currentVel = robot.getSpeed();
        std::cout << "getspeed" << currentVelocity;
        double currentHead = robot.getHeading();

        // Check for convergence
        if (fabs(targetVelocity - currentVelocity) < convergenceThreshold &&
            fabs(targetHeading - currentTheta) < convergenceThreshold) {
            std::cout << "Converged to the set points." << std::endl;
            break;}

        currentVelocity = currentVel;
        currentTheta = currentHead;
    }

    // Get the final state of the robot after convergence
    double finalX, finalY, finalTheta, finalVelocity;
    robot.getState(finalX, finalY, finalTheta, finalVelocity);
    std::cout << "Final State: x=" << finalX << " y=" << finalY <<
         " theta=" << finalTheta << " velocity=" << finalVelocity << std::endl;
}

/**
 * @brief Get the final velocity of the robot.
 *
 * @return The final velocity.
 */
double RobotSimulation::getFinalVelocity() const {
    return finalVelocity;
}
