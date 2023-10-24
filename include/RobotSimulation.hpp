#ifndef ROBOT_SIMULATION_HPP
#define ROBOT_SIMULATION_HPP

#include "PIDController.hpp" // Include the PIDController header
#include "RobotModel.hpp"    // Include the RobotModel header

class RobotSimulation {
public:
    RobotSimulation(double wheelbase, double trackWidth, double maxSteeringAngle,
                    double velP, double velI, double velD, double deltaT,
                    double headP, double headI, double headD);

    void runSimulation(double targetHeading,double targetVelocity);

    double getFinalX() const;
    double getFinalY() const;
    double getFinalTheta() const;
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
