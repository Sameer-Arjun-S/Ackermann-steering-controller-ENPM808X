#include <gtest/gtest.h>
#include <iostream>
#include "../include/PIDController.hpp"
#include "../include/RobotModel.hpp"
#include "../include/RobotSimulation.hpp"



TEST(PIDControllerTest, TestVelocityProportionalConstant) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getVelocityProportionalConstant(), 1.0);
}

TEST(PIDControllerTest, TestVelocityIntegralConstant) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getVelocityIntegralConstant(), 0.5);
}

TEST(PIDControllerTest, TestVelocityDerivativeConstant) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getVelocityDerivativeConstant(), 0.2);
}

TEST(PIDControllerTest, TestDeltaTime) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getDeltaTime(), 0.01);
}

TEST(PIDControllerTest, TestHeadingProportionalConstant) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getHeadingProportionalConstant(), 1.0);
}

TEST(PIDControllerTest, TestHeadingIntegralConstant) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getHeadingIntegralConstant(), 0.5);
}

TEST(PIDControllerTest, TestHeadingDerivativeConstant) {
    PIDController PID(1.0, 0.5, 0.2, 0.01, 1.0, 0.5, 0.2);
    EXPECT_DOUBLE_EQ(PID.getHeadingDerivativeConstant(), 0.2);
}