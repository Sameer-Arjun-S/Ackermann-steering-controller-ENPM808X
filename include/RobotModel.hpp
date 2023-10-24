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
    RobotModel(double wheelbase, double trackWidth, double maxSteeringAngle);

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
     * @brief Calculates the steering angle and left/right wheel velocities based on a desired turning radius.
     * 
     * @param desiredTurningRadius The desired turning radius for the robot's path.
     * @param steeringAngle The calculated steering angle (output).
     * @param velocity_ The calculated velocity (output).
     */
    void calculateSteeringAndDriveVelocities(double desiredTurningRadius, double& steeringAngle, double& velocity_);

    /**
     * @brief Retrieves the current state of the robot model.
     * 
     * @param x The current x-coordinate of the robot (output).
     * @param y The current y-coordinate of the robot (output).
     * @param theta The current orientation (in radians) of the robot (output).
     * @param velocity The current velocity of the robot (output).
     */
    void getState(double& x, double& y, double& theta, double& velocity);

private:
    double wheelbase_;
    double trackWidth_;
    double maxSteeringAngle_;

    double x_;
    double y_;
    double theta_;
    double velocity_;
    double leftWheelVelocity;
    double rightWheelVelocity;
};

#endif // ROBOT_MODEL_HPP
