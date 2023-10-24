 /**
 * @file test.cpp
 * @author Driver - Sameer Arjun S
 *         Navigator -  Ishaan Samir Parikh
 *         Design Keeper - Manav Nagda
 * @brief Implementation of testing methods for all the classes.
 * @version 0.1
 * @date 2023
 */
#include <gtest/gtest.h>
#include <iostream>
#include "../include/PIDController.hpp"
#include "../include/RobotModel.hpp"
#include "../include/RobotSimulation.hpp"
#define M_PI 3.14159265358979323846

/**
 * @brief This test case checks if the velocity proportional constant is set correctly.
 */
TEST(PIDControllerTest, TestVelocityProportionalConstant) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getVelocityProportionalConstant(), 1.0);
}

/**
 * @brief This test case checks if the velocity integral constant is set correctly.
 */
TEST(PIDControllerTest, TestVelocityIntegralConstant) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getVelocityIntegralConstant(), 0.5);
}

/**
 * @brief This test case checks if the velocity derivative constant is set correctly.
 */
TEST(PIDControllerTest, TestVelocityDerivativeConstant) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getVelocityDerivativeConstant(), 0.2);
}

/**
 * @brief This test case checks if the time step (delta time) is set correctly.
 */
TEST(PIDControllerTest, TestDeltaTime) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getDeltaTime(), 0.01);
}

/**
 * @brief This test case checks if the heading proportional constant is set correctly.
 */
TEST(PIDControllerTest, TestHeadingProportionalConstant) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getHeadingProportionalConstant(), 1.0);
}

/**
 * @brief This test case checks if the heading integral constant is set correctly.
 */
TEST(PIDControllerTest, TestHeadingIntegralConstant) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getHeadingIntegralConstant(), 0.5);
}

/**
 * @brief This test case checks if the heading derivative constant is set correctly.
 */
TEST(PIDControllerTest, TestHeadingDerivativeConstant) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getHeadingDerivativeConstant(), 0.2);
}

/**
 * @brief This test case checks if the computeErrors function calculates errors correctly.
 */
TEST(PIDControllerTest, TestComputeErrors) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);

    // Set specific values for testing
    double targetVelocity = 10.0;
    double currentVelocity = 5.0;
    double targetHeading = 5.0;
    double currentHeading = 2.0;

    // Call the computeErrors function with specific values
    PID.computeErrors(targetVelocity, currentVelocity, targetHeading, currentHeading);

    // Access the last elements of velocityErrors and headingErrors
    double velocityError = PID.getVelocityErrors().back();
    double headingError = PID.getHeadingErrors().back();

    // Check if the velocity error is 5.0
    EXPECT_DOUBLE_EQ(velocityError, 5.0);

    // Check if the heading error is 3.0
    EXPECT_DOUBLE_EQ(headingError, 3.0);
}

/**
 * @brief This test case checks the initial state of the RobotModel.
 */
TEST(RobotModelTest, InitialStateTest) {
    RobotModel robot(2.0, 1.0, 0.5);
    double x, y, theta, velocity;
    robot.getState(x, y, theta, velocity);
    ASSERT_DOUBLE_EQ(x, 0.0);
    ASSERT_DOUBLE_EQ(y, 0.0);
    ASSERT_DOUBLE_EQ(theta, 0.0);
    ASSERT_DOUBLE_EQ(velocity, 0.0);
}

/**
 * @brief This test case checks if the RobotModel can be set to an initial state.
 */
TEST(RobotModelTest, SetInitialStateTest) {
    RobotModel robot(2.0, 1.0, 0.5);
    robot.setInitialState(1.0, 2.0, 0.785, 3.0);  // (1.0, 2.0, 45 degrees, 3.0 m/s)
    double x, y, theta, velocity;
    robot.getState(x, y, theta, velocity);
    ASSERT_DOUBLE_EQ(x, 1.0);
    ASSERT_DOUBLE_EQ(y, 2.0);
    ASSERT_DOUBLE_EQ(theta, 0.785);
    ASSERT_DOUBLE_EQ(velocity, 3.0);
}

/**
 * @brief This test case checks if the RobotSimulation runs without exceptions.
 */
TEST(RobotSimulation, Check_Simulation_Running) {
    RobotSimulation simulation(0.5, 1.0, M_PI / 4.0, 1.0, 0.1, 0.01, 0.1, 1.0, 0.1, 0.01);
    EXPECT_NO_THROW(simulation.runSimulation(0.2, 2));
}


// TEST(RobotSimulation, Check_Simulation_FinalState) {
//     RobotSimulation simulation(0.5, 1.0, M_PI / 4.0, 1.0, 0.1, 0.01, 0.1, 1.0, 0.1, 0.01);
//     simulation.runSimulation(0.5, 2);

//     // Check if finalX, finalY, finalTheta, and finalVelocity are all zero
//     EXPECT_DOUBLE_EQ(simulation.getFinalX(), 0);
//     EXPECT_DOUBLE_EQ(simulation.getFinalY(), 0);
//     EXPECT_DOUBLE_EQ(simulation.getFinalTheta(), 0);
//     EXPECT_DOUBLE_EQ(simulation.getFinalVelocity(), 0);
// }
