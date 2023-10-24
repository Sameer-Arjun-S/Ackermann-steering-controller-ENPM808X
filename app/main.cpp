/**
 * @file RobotSimulation.hpp
 * @brief Contains the RobotSimulation class definition.
 */
/**
 * @def M_PI
 * @brief A macro that defines the mathematical constant pi (π)
 * @author Driver: Manav Nagda, Navigator: Sameer Arjun S, Design Keeper: Ishaan Parik
 */
#define M_PI 3.14159265358979323846
#include "RobotSimulation.hpp"

/**
 * @brief The main function that runs the robot simulation
 *
 * This function creates an instance of the RobotSimulation class with appropriate parameters
 * and runs the simulation for Ackermann kinematic model
 *
 * @return An integer indicating the exit s0 indicates a successful execution of code
 */
int main() {
    // Create an instance of RobotSimulation with appropriate parameters
    RobotSimulation simulation(0.5, 1.0, M_PI / 4.0, 1.0, 0.1, 0.01, 0.1, 1.0, 0.1, 0.01);

    //Running the simulation
    simulation.runSimulation();

    return 0;
}
