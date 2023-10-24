#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <vector>

class PIDController {
public:
    PIDController(double velP, double velI, double velD, double dt, double headP, double headI, double headD);
    


    std::vector<double> computePID();
    double getVelocityProportionalConstant();
    double getVelocityIntegralConstant();
    double getVelocityDerivativeConstant();
    double getDeltaTime();
    double getHeadingProportionalConstant();
    double getHeadingIntegralConstant();
    double getHeadingDerivativeConstant();
    const std::vector<double>& getVelocityErrors() const;
    const std::vector<double>& getHeadingErrors() const;
    void computeErrors(double targetVelocity, double currentVelocity, double targetHeading, double currentHeading);

private:
    double velKp;
    double velKi;
    double velKd;
    double deltaT;
    double headKp;
    double headKi;
    double headKd;
    std::vector<double> velocityErrors;
    std::vector<double> headingErrors;
};

#endif // PID_CONTROLLER_HPP