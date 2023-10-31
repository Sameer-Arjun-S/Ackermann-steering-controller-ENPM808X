/**
 * @file RobotModel.cpp
 * @author Driver - Sameer Arjun S
 *         Navigator -  Ishaan Samir Parikh
 *         Design Keeper - Manav Nagda
 * @brief Implementation of a class to represent a robot model.
 * @version 0.1
 * @date 2023
 */

#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

class RobotModel {
public:
    /**
     * @brief Constructor for the RobotModel class.
     * 
     * @param wheelbase The distance between the front and rear axles of the robot.
     * @param trackWidth The distance between the left and right wheels of the robot.
     * @param maxSteeringAngle The maximum allowable steering angle for the robot.
     */
    RobotModel(double wheelbase, double wheelRadius, double trackWidth);

    /**
     * @brief Sets the initial state of the robot model.
     * 
     * @param x The initial x-coordinate of the robot.
     * @param y The initial y-coordinate of the robot.
     * @param theta The initial orientation (in radians) of the robot.
     * @param velocity The initial velocity of the robot.
     */
    void setInitialState(double x, double y, double theta, double velocity);

    /**
     * @brief Updates the state of the robot model based on steering angle and time step.
     * 
     * @param steeringAngle The steering angle (in radians) applied to the robot's front wheels.
     * @param dt The time step for the state update.
     */
    void updateState(double steeringAngle, double dt);

    /**
     * @brief Retrieves the current state of the robot model.
     * 
     * @param x The current x-coordinate of the robot (output).
     * @param y The current y-coordinate of the robot (output).
     * @param theta The current orientation (in radians) of the robot (output).
     * @param velocity The current velocity of the robot (output).
     */
    void getState(double& x, double& y, double& theta, double& velocity);
    /**
     * @brief Simulates the robot model's motion based on PID controller outputs.
     *
     * This function updates the robot's state based on the provided PID controller outputs
     * for heading control and velocity control. It simulates the motion of the robot and
     * updates its heading and speed.
     *
     * @param PID_heading_output The PID controller output for heading control.
     * @param PID_velocity_output The PID controller output for velocity control.
     * @param dt The time step for simulation.
     */
    void Simulate_robot_model(double PID_heading_output, double PID_velocity_output, double dt);

    /**
     * @brief Get the current heading of the robot in radians.
     *
     * @return The current heading of the robot in radians.
     */
    double getHeading();

    /**
     * @brief Get the current speed of the robot in meters per second (m/s).
     *
     * @return The current speed of the robot in m/s.
     */
    double getSpeed();

private:
    double wheelbase_;
    double wheelRadius_;
    double trackWidth_;
    double maxSteeringAngle_;
    double x_;
    double y_;
    double theta_;
    double velocity_;
    double alpha_i_;
    double alpha_o_;
    double omega_i_;
    double omega_o_;
    double heading_;
    double speed_;

};

#endif // ROBOT_MODEL_HPP
