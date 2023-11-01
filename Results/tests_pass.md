sas@sas-virtual-machine:~/Ackermann-steering-controller-ENPM808X$ ctest --test-dir build/
Internal ctest changing into directory: /home/sas/Ackermann-steering-controller-ENPM808X/build
Test project /home/sas/Ackermann-steering-controller-ENPM808X/build
      Start  1: PIDControllerTest.TestVelocityProportionalConstant
 1/12 Test  #1: PIDControllerTest.TestVelocityProportionalConstant ...   Passed    0.01 sec
      Start  2: PIDControllerTest.TestVelocityIntegralConstant
 2/12 Test  #2: PIDControllerTest.TestVelocityIntegralConstant .......   Passed    0.01 sec
      Start  3: PIDControllerTest.TestVelocityDerivativeConstant
 3/12 Test  #3: PIDControllerTest.TestVelocityDerivativeConstant .....   Passed    0.01 sec
      Start  4: PIDControllerTest.TestDeltaTime
 4/12 Test  #4: PIDControllerTest.TestDeltaTime ......................   Passed    0.01 sec
      Start  5: PIDControllerTest.TestHeadingProportionalConstant
 5/12 Test  #5: PIDControllerTest.TestHeadingProportionalConstant ....   Passed    0.01 sec
      Start  6: PIDControllerTest.TestHeadingIntegralConstant
 6/12 Test  #6: PIDControllerTest.TestHeadingIntegralConstant ........   Passed    0.01 sec
      Start  7: PIDControllerTest.TestHeadingDerivativeConstant
 7/12 Test  #7: PIDControllerTest.TestHeadingDerivativeConstant ......   Passed    0.01 sec
      Start  8: PIDControllerTest.TestComputeErrors
 8/12 Test  #8: PIDControllerTest.TestComputeErrors ..................   Passed    0.01 sec
      Start  9: RobotModelTest.InitialStateTest
 9/12 Test  #9: RobotModelTest.InitialStateTest ......................   Passed    0.01 sec
      Start 10: RobotModelTest.SetInitialStateTest
10/12 Test #10: RobotModelTest.SetInitialStateTest ...................   Passed    0.01 sec
      Start 11: RobotSimulation.Check_Simulation_Running
11/12 Test #11: RobotSimulation.Check_Simulation_Running .............   Passed    0.01 sec
      Start 12: RobotSimulation.Check_Simulation_FinalState
12/12 Test #12: RobotSimulation.Check_Simulation_FinalState ..........   Passed    0.01 sec

100% tests passed, 0 tests failed out of 12

Total Test time (real) =   0.09 sec
