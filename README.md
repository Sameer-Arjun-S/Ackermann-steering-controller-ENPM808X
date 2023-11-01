# Ackermann-steering-controller-ENPM808X
Software development for Ackermann steering control mechanism using a PID controller for ENPM808X midterm project
![CICD Workflow status](https://github.com/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-red.svg)](https://opensource.org/licenses/MIT)
[![codecov](https://codecov.io/gh/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X/branch/development/graph/badge.svg)](https://codecov.io/gh/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X)

# Authors:
- [Sameer Arjun S](https://github.com/Sameer-Arjun-S) 
- [Manav Bhavesh Nagda](https://github.com/mvboiii)
- [Ishaan Samir Parikh](https://github.com/Ishaan1810)


## Overview
This project uses Agile Iterative process for the implementation for a robot steering control module using the Ackermann kinematic model. The module gets the input of the robot’s target heading and velocity to output the steering angle and drive wheel velocities to achieve
the required steering motion for the robot. The software team will work on this project through iterative software evolution and service processes and Agile Iterative Process (AIP) of software development will be used in the development process with Test-Driven Development approach. The team consists of 3 members and pair programming is implemented with one member working as the design keeper.
The classes of the future program and their responsibilities have been identified and their relations have been explained through UML class diagrams below.

![UML Class diagram](https://github.com/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X/assets/113264700/b5dc81d5-c95b-4990-b9db-a5b58d81ff3e)

The activity diagram for the program workflow is as follows:
![Activity Diagram](https://github.com/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X/assets/113264700/55354d49-cfa1-47d9-a410-5570a4d9a3bd)

### Agile Implementaiton Process:

- [Sprint Review](https://docs.google.com/document/d/1NmqzubGhrMsSzRBbdqmDDOxbeHE_2tC1ArEyeNob6Vo/edit)                                                  
- [Agile Iterative Process](https://docs.google.com/spreadsheets/d/1YNlWJhyofniJCe_7V67ByR8JngXJ0Lf_kJiTpSbSLZk/edit#gid=0)

## Project reports
- [Project Report Phase 0](https://github.com/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X/blob/development/Project_Report_Phase_0.pdf)
- [Project Report Phase 1](https://github.com/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X/blob/development/Project_report_Phase_1.pdf)

## Project video presentations
- [Project Video Phase 0](https://drive.google.com/drive/folders/1XxFNVwzcc_FQ3LxvkB0sat8C8gZQSr4t?usp=share_link)
- [Project Video Phase 1](https://drive.google.com/drive/folders/1FB--a8MC04ID6z6_YX9fl1sfQi5uFUO-?usp=drive_link)
- [Project Video Phase 2](https://drive.google.com/drive/folders/1qNmXCD-yAGQrZ4IVPWmvRYi_JeRbLDu3?usp=sharing)

## Implementation Results
The implementation of the PID controller for Ackermann kinematic model has been implemented, and the input values converge to the expected. However it is observed that there is an overshoot of the PID controller because of which there is a condition of the overshoot of the output values.
```
Enter the target heading (in radians): 0.8
Enter the target velocity: 20
getspeed11.2817Iteration 2
Velocity Error: 14.0482
Heading Error: -184.571heading: -396.297
Speed: 72.8368
**********OUTPUTS***************
Inner steering angle: -1.07072
Outer steering angle: -0.44126
Inner wheel velocity: 29.8832
Outer wheel velocity: 115.79
********************************
getspeed5.95179Iteration 3
Velocity Error: -52.8368
Heading Error: 442.133heading: -628.873
Speed: 71.1986
**********OUTPUTS***************
Inner steering angle: -0.228836
Outer steering angle: -0.352
Inner wheel velocity: 87.1392
Outer wheel velocity: 55.2581
********************************
getspeed72.8368Iteration 4
Velocity Error: -51.1986
Heading Error: 674.71heading: -646.219
Speed: 0.714755
**********OUTPUTS***************
Inner steering angle: -1.26727
Outer steering angle: 0.67179
Inner wheel velocity: 0.474102
Outer wheel velocity: -1.90361
********************************
getspeed71.1986Iteration 5
Velocity Error: 19.2852
Heading Error: 692.056heading: -5647.96
Speed: 363.045
**********OUTPUTS***************
Inner steering angle: -0.553819
Outer steering angle: -1.52448
Inner wheel velocity: 705.859
Outer wheel velocity: 20.2317
```
As seen above the velocity and heading errors are reducing and then overshooting to the negative plane, but again overshoot to counter effect to the positive plane. The overall trajectory is still found to reduce the error between the current and desired trajectories of the robot. Since, all the tests are passing we can safely conclude that the program is working, and needs tuning of the PID gains to optimize the output for the robot.


## Future scope
Tuning of the PID controller to converge faster to the expected trajectory of the robot, which is planned for the future scope of the project.

## Compiling and running via command line:
```
#Cloning the repository
  git clone --recursive https://github.com/Sameer-Arjun-S/Ackermann-steering-controller-      ENPM808X.git
#Configure the project and generate a native build system
  cmake -S ./ -B build/
#Compiling and building the project
  cmake --build build/ --clean-first
#Running the tests
  ./test/cpp-test
#Run the program
  ./build/app/shell-app
  Enter target heading in degrees
  Enter target speed in m/s
```



## Generating the documentation
```
# Build the documentation into the 'docs' directory using CMake:
  cmake --build build/ --target docs
# Open the documentation as a HTML file in your web browser:
  open docs/html/index.html
```
## Building for test coverage
```
# If you don't have gcovr or lcov installed, run:
  sudo apt-get install gcovr lcov
# Set the build type to Debug and WANT_COVERAGE=ON:
  cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -S ./ -B build/
# Do a clean compile, run unit test, and generate the coverage report:
  cmake --build build/ --clean-first --target all test_coverage
# Open a web browser to browse the test coverage report:
  open build/test_coverage/index.html
```
## Checking for cppcheck and cpplint
```
# if you need to install cppcheck, do
sudo apt install cppcheck

# run in the top-level project directory (eg., in cpp-boilerplate-v2/)
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" )
```
```
# You may need to install cpplint:
sudo apt install cpplint

# run in the top-level project directory (eg., in cpp-boilerplate-v2/)
cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" )
```

## Google style code verification
```
# Install Cpplint(ignore if already installed):
  sudo apt install cpplint
# Self-check Google code style conformity using Cpplint:
  cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" )
```
