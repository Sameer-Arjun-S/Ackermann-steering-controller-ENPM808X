#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

class RobotModel {
public:
    RobotModel(double wheelbase, double trackWidth, double maxSteeringAngle);

    void setInitialState(double x, double y, double theta, double velocity);
    void updateState(double steeringAngle, double dt);
    void calculateSteeringAndDriveVelocities(double desiredTurningRadius, double& steeringAngle, double& leftWheelVelocity, double& rightWheelVelocity);
    void getState(double& x, double& y, double& theta, double& velocity);

private:
    double wheelbase_;
    double trackWidth_;
    double maxSteeringAngle_;

    double x_;
    double y_;
    double theta_;
    double velocity_;
};

#endif // ROBOT_MODEL_HPP