# Ackermann-steering-controller-ENPM808X
Software development for Ackermann steering control mechanism using a PID controller for ENPM808X midterm project
![CICD Workflow status](https://github.com/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-red.svg)](https://opensource.org/licenses/MIT)
[![codecov](https://codecov.io/gh/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X/tree/development/graph/badge.svg)](https://codecov.io/gh/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X)

# Authors:
### Sameer Arjun S 
### Manav Bhavesh Nagda
### Ishaan Samir Parikh 

## Overview
This project uses Agile Iterative process for the implementation for a robot steering control module using the Ackermann kinematic model. The module gets the input of the robot’s target heading and velocity to output the steering angle and drive wheel velocities to achieve
the required steering motion for the robot. The software team will work on this project through iterative software evolution and service processes and Agile Iterative Process (AIP) of software development will be used in the development process with Test-Driven Development approach. The team consists of 3 members and pair programming is implemented with one member working as the design keeper.
The classes of the future program and their responsibilities have been identified and their relations have been explained through UML class diagrams below.

![UML Class diagram](https://github.com/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X/assets/113264700/b5dc81d5-c95b-4990-b9db-a5b58d81ff3e)

The activity diagram for the program workflow is as follows:
![Activity Diagram](https://github.com/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X/assets/113264700/55354d49-cfa1-47d9-a410-5570a4d9a3bd)

### Agile Implementaiton Process:

Sprint Review: (https://docs.google.com/document/d/1NmqzubGhrMsSzRBbdqmDDOxbeHE_2tC1ArEyeNob6Vo/edit)                                                  
Agile Iterative Process: (https://docs.google.com/spreadsheets/d/1YNlWJhyofniJCe_7V67ByR8JngXJ0Lf_kJiTpSbSLZk/edit#gid=0)

## Project reports
![Project Phase 0](https://github.com/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X/blob/development/Project_Report_Phase_0.pdf)
![Project Phase 1](https://github.com/Sameer-Arjun-S/Ackermann-steering-controller-ENPM808X/blob/development/Project_report_Phase_1.pdf)

## Project video presentations
Project Phase 0: (https://drive.google.com/drive/folders/1XxFNVwzcc_FQ3LxvkB0sat8C8gZQSr4t?usp=share_link)

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

## Google style code verification
```
# Install Cpplint(ignore if already installed):
  sudo apt install cpplint
# Self-check Google code style conformity using Cpplint:
  cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" )
```
