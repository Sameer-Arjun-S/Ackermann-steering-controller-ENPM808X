/**
 * @file RobotSimulation.cpp
 * @author Driver - Ishaan Samir Parikh
           Navigator - Manav Bhavesh Nagda
           Design Keeper - Sameer Arjun Sateesh
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
RobotSimulation::RobotSimulation(double wheelbase, double trackWidth, double maxSteeringAngle,
                                 double velP, double velI, double velD, double deltaT,
                                 double headP, double headI, double headD)
    : robot(wheelbase, trackWidth, maxSteeringAngle), controller(velP, velI, velD, deltaT, headP, headI, headD) {
}

/**
 * @brief Runs the robot simulation control loop.
 *
 * This function allows the user to interactively input target heading and velocity, and the robot simulation
 * attempts to control the robot's movement to reach these targets using the PID controller. The control loop
 * continues until the user exits or convergence to the target is achieved.
 */
void RobotSimulation::runSimulation() {
    while (true) {
        double targetHeading, targetVelocity;

        // Prompt the user to enter the target heading and velocity
        std::cout << "Enter the target heading (in radians): ";
        std::cin >> targetHeading;

        std::cout << "Enter the target velocity: ";
        std::cin >> targetVelocity;

        // Check if the user wants to exit
        if (targetHeading < 0 || targetVelocity < 0) {
            std::cout << "Exiting the control loop." << std::endl;
            break;
        }

        const int maxIterations = 1000;
        const double convergenceThreshold = 0.01; // Adjust as needed

        for (int i = 0; i < maxIterations; i++) {

            std::cout<< "Iteration "<< i << std::endl;
            // Get the current state of the robot
            double currentX, currentY, currentTheta, currentVelocity;
            robot.getState(currentX, currentY, currentTheta, currentVelocity);
            std::cout<< "Current Location: x=" << currentX << " y=" << currentY << " theta=" << currentTheta << " velocity=" << currentVelocity << std::endl;

            // Calculate the errors between the current state and the desired set points
            controller.computeErrors(targetVelocity, currentVelocity, targetHeading, currentTheta);

            // Compute PID control outputs
            std::vector<double> controlOutputs = controller.computePID();

            // Extract control outputs
            double steeringAngle = controlOutputs[1]; // Assuming steeringAngle is the second element
            double leftWheelVelocity, rightWheelVelocity;
            robot.calculateSteeringAndDriveVelocities(1.0 / tan(steeringAngle), steeringAngle, leftWheelVelocity, rightWheelVelocity);

            // Update the robot's state
            robot.updateState(steeringAngle, controller.getDeltaTime());
            
            // Check for convergence
            if (fabs(targetVelocity - currentVelocity) < convergenceThreshold &&
                fabs(targetHeading - currentTheta) < convergenceThreshold) {
                std::cout << "Converged to the set points." << std::endl;
                break;
            }
        }

        // Get the final state of the robot after convergence
        double finalX, finalY, finalTheta, finalVelocity;
        robot.getState(finalX, finalY, finalTheta, finalVelocity);
        std::cout << "Final State: x=" << finalX << " y=" << finalY << " theta=" << finalTheta << " velocity=" << finalVelocity << std::endl;
    }
}
