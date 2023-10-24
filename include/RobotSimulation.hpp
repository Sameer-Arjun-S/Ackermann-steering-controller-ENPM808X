#ifndef ROBOT_SIMULATION_HPP
#define ROBOT_SIMULATION_HPP

#include "PIDController.hpp" // Include the PIDController header
#include "RobotModel.hpp"    // Include the RobotModel header

class RobotSimulation {
public:
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
    RobotSimulation(double wheelbase, double trackWidth, double maxSteeringAngle,
                double velP, double velI, double velD, double deltaT,
                double headP, double headI, double headD);


    /**
     * @brief Runs the robot simulation control loop.
     *
     * This function allows the user to interactively input target heading and velocity, and the robot simulation
     * attempts to control the robot's movement to reach these targets using the PID controller. The control loop
     * continues until the user exits or convergence to the target is achieved.
     */
    void runSimulation(double targetHeading, double targetVelocity);

    /**
     * @brief Get the final x-coordinate of the robot.
     *
     * @return The final x-coordinate.
     */
    double getFinalX() const;

    /**
     * @brief Get the final y-coordinate of the robot.
     *
     * @return The final y-coordinate.
     */
    double getFinalY() const;

    /**
     * @brief Get the final orientation (theta) of the robot.
     *
     * @return The final orientation (in radians).
     */
    double getFinalTheta() const;

    /**
     * @brief Get the final velocity of the robot.
     *
     * @return The final velocity.
     */
    double getFinalVelocity() const;

private:
    RobotModel robot;
    PIDController controller;
    double finalX;
    double finalY;
    double finalTheta;
    double finalVelocity;
};

#endif // ROBOT_SIMULATION_HPP
