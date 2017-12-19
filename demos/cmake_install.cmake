# Install script for directory: /home/shinsh/ompl-1.2.1-Source/demos

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE FILE FILES
    "/home/shinsh/ompl-1.2.1-Source/demos/appUtil.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/HypercubeBenchmark.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/ThunderLightning.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/KinematicChainBenchmark.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/LTLWithTriangulation.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/RigidBodyGeometry.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/GeometricCarPlanning_Original.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/GeometricCarPlanning_tmp.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/CForestCircleGridBenchmark.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/GeometricCarPlanning.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/PlannerData.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/PlannerProgressProperties.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/RigidBodyPlanning.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/OpenDERigidBodyPlanning.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/TriangulationDemo.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/GeometricCarPlanning_Backup_170131.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/OptimalPlanning.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/RigidBodyPlanningWithControls.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/RigidBodyPlanningWithODESolverAndControls.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/RigidBodyPlanningWithIntegrationAndControls.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/HybridSystemPlanning.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/StateSampling.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/RigidBodyPlanningWithIK.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/Point2DPlanning.cpp"
    "/home/shinsh/ompl-1.2.1-Source/demos/PlannerData.py"
    "/home/shinsh/ompl-1.2.1-Source/demos/Point2DPlanning.py"
    "/home/shinsh/ompl-1.2.1-Source/demos/RigidBodyPlanningWithControls.py"
    "/home/shinsh/ompl-1.2.1-Source/demos/StateSampling.py"
    "/home/shinsh/ompl-1.2.1-Source/demos/RigidBodyPlanning.py"
    "/home/shinsh/ompl-1.2.1-Source/demos/OptimalPlanning.py"
    "/home/shinsh/ompl-1.2.1-Source/demos/RandomWalkPlanner.py"
    "/home/shinsh/ompl-1.2.1-Source/demos/RigidBodyPlanningWithODESolverAndControls.py"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/shinsh/ompl-1.2.1-Source/demos/Koules")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/shinsh/ompl-1.2.1-Source/demos/VFRRT")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "ompl")

