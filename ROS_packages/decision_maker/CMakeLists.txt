cmake_minimum_required(VERSION 2.8.3)
project(decision_maker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  message_generation
  pcl_ros
)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

##################################### OMPL  ##########################################
find_package(OMPL)
MESSAGE( STATUS "OMPL_LIBRARY_DIRS: " ${OMPL_LIBRARIES} )
MESSAGE( STATUS "OMPL_INCLUDE_DIRS: " ${OMPL_INCLUDE_DIRS} )
include(${QT_USE_FILE})
###################################################################################

find_package(Boost REQUIRED COMPONENTS signals)

option(WITH_OPENMP      "Build with OpenMP support for multithreading"                  ON)
# add OpenMP support
    if (WITH_OPENMP)
find_package(OpenMP QUIET)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
    if (NOT OPENMP_FOUND)
    set(WITH_OPENMP OFF)
    endif()
endif()

catkin_package(
#  INCLUDE_DIRS inc
#  LIBRARIES lib
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
)

set( HDR_FILES_QT_UI_NODE inc/quadtree.h)
set( SRC_FILES_QT_UI_NODE src/decision_maker_ros.cpp ${MOC_FILES})
QT4_WRAP_CPP(MOC_SRC_H_QT_UI_NODE ${HDR_FILES_QT_UI_NODE})

## Declare a cpp executable
add_executable(decision_maker ${SRC_FILES_QT_UI_NODE} ${HDR_FILES_QT_UI_NODE}
		${MOC_SRC_H_QT_UI_NODE} )

target_link_libraries(decision_maker
   ${QT_LIBRARIES}
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OMPL_LIBRARIES}
   ${boost_LIBRARIES}
)
