/**
 * @file RobotSimulation.cpp
 * @author Driver - Ishaan Samir Parikh
           Navigator - Manav Bhavesh Nagda
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

RobotSimulation::RobotSimulation(double wheelbase, double trackWidth, double maxSteeringAngle,
                                 double velP, double velI, double velD, double deltaT,
                                 double headP, double headI, double headD)
    : robot(wheelbase, trackWidth, maxSteeringAngle), controller(velP, velI, velD, deltaT, headP, headI, headD) {
}

void RobotSimulation::runSimulation() {
    while (true) {
        double targetHeading, targetVelocity;

        // Stub: In a real implementation, you would obtain target values from your control system.
        targetHeading = 0.0;
        targetVelocity = 0.0;

        // Check if the user wants to exit
        if (targetHeading < 0 || targetVelocity < 0) {
            std::cout << "Exiting the control loop." << std::endl;
            break;
        }

        const int maxIterations = 1000;
        const double convergenceThreshold = 0.01; // Adjust as needed

        for (int i = 0; i < maxIterations; i++) {

            std::cout << "Iteration " << i << std::endl;

            // Get the current state of the robot
            double currentX, currentY, currentTheta, currentVelocity;

            // Stub: In a real implementation, you would obtain the current state from your robot model.
            currentX = 0.0;
            currentY = 0.0;
            currentTheta = 0.0;
            currentVelocity = 0.0;

            std::cout << "loc " << currentTheta << std::endl;

            // Stub: In a real implementation, you would calculate errors and control outputs.
            controller.computeErrors(targetVelocity, currentVelocity, targetHeading, currentTheta);

            // Stub: In a real implementation, you would compute PID control outputs and update the robot's state.
            std::vector<double> controlOutputs;

            // Stub: In a real implementation, you would extract control outputs and update the robot's state.
            double steeringAngle, leftWheelVelocity, rightWheelVelocity;

            // Stub: In a real implementation, you would update the robot's state.
            robot.updateState(steeringAngle, controller.getDeltaTime());

            // Check for convergence
            if (fabs(targetVelocity - currentVelocity) < convergenceThreshold &&
                fabs(targetHeading - currentTheta) < convergenceThreshold) {
                std::cout << "Converged to the set points." << std::endl;
                break;
            }
        }

        // Stub: In a real implementation, you would obtain the final state from your robot model.
        double finalX, finalY, finalTheta, finalVelocity;
        finalX = 0.0;
        finalY = 0.0;
        finalTheta = 0.0;
        finalVelocity = 0.0;

        std::cout << "Final State: x=" << finalX << " y=" << finalY << " theta=" << finalTheta << " velocity=" << finalVelocity << std::endl;
    }
}
