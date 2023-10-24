#define M_PI 3.14159265358979323846

#include "RobotSimulation.hpp"

int main() {
    // Create an instance of RobotSimulation with appropriate parameters
    RobotSimulation simulation(0.5, 1.0, M_PI / 4.0, 1.0, 0.1, 0.01, 0.1, 1.0, 0.1, 0.01);

    // Run the simulation
    simulation.runSimulation();

    return 0;
}
