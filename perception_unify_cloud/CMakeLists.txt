cmake_minimum_required(VERSION 2.8.3)
project(perception_unify_cloud)
set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/drc/wrecs_common/CMake")

set(CMAKE_BUILD_TYPE RELEASE)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -march=native -mno-avx  -msse4.2 ${CMAKE_CXX_FLAGS}")
#ADD_DEFINITIONS("-Wall -Wno-unused-variable
#                   -Wno-unused-but-set-variable -Wno-unknown-pragmas")
                   
if (CMAKE_BUILD_TYPE EQUAL DEBUG)
    ADD_DEFINITIONS(-DDEBUG_MODE)
endif (CMAKE_BUILD_TYPE EQUAL DEBUG)


find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED core imgproc highgui)
find_package(catkin REQUIRED COMPONENTS	roscpp	tf wrecs_msgs)
find_package(perception_common REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include DepthMapRefinement
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_DMR
  CATKIN_DEPENDS perception_common roscpp
  DEPENDS system_lib octomap
)



include_directories(include DepthMapRefinement)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${perception_common_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(DepthMapRefinement)

## Declare a cpp library
add_library(${PROJECT_NAME}
           src/Unifier.cpp
           src/SensorFusion.cpp
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_DMR
                                      ${PCL_LIBRARIES} 
                                      ${OpenCV_LIBS}
                                      ${catkin_LIBRARIES}
                                      ${OCTOMAP_LIBRARIES}
                                      perception_common)


#cotire(${PROJECT_NAME})

add_library(${PROJECT_NAME}_drcsim
                                   src/Unifier.cpp
                                   src/SensorFusion.cpp
            )
set_property(
   TARGET ${PROJECT_NAME}_drcsim
   PROPERTY COMPILE_DEFINITIONS GAZEBO_SIMULATION
   )
target_link_libraries(${PROJECT_NAME}_drcsim ${PROJECT_NAME}_DMR
                                             ${PCL_LIBRARIES}
                                             ${OpenCV_LIBS}
                                             ${catkin_LIBRARIES}
                                             ${OCTOMAP_LIBRARIES}
                                             perception_common_drcsim)

#cotire(${PROJECT_NAME}_drcsim)

add_executable(perception_unify_cloud_node src/filter.cpp)
target_link_libraries(perception_unify_cloud_node    ${PROJECT_NAME} )
#cotire(perception_unify_cloud_node)
                                                      
add_executable(sensor_fusion_node src/sensorfusion.cpp)
target_link_libraries(sensor_fusion_node    ${PROJECT_NAME})
#cotire(sensor_fusion_node)

add_executable(ocu_pub src/ocu_publisher.cpp)
target_link_libraries(ocu_pub    ${PROJECT_NAME})
add_dependencies(ocu_pub ${catkin_EXPORTED_TARGETS})

add_executable(assembler_giant src/assemblePC_color.cpp)
target_link_libraries(assembler_giant    ${PROJECT_NAME})
add_dependencies(assembler_giant ${catkin_EXPORTED_TARGETS})

add_executable(sensor_fusion_drcsim src/sensorfusion.cpp)
set_property(
   TARGET sensor_fusion_drcsim
   PROPERTY COMPILE_DEFINITIONS GAZEBO_SIMULATION
 )
target_link_libraries(sensor_fusion_drcsim    ${PROJECT_NAME}_drcsim)
#cotire(sensor_fusion_drcsim)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS perception_unify_cloud_node sensor_fusion_node ${PROJECT_NAME} ocu_pub sensor_fusion_drcsim
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
 
 install(DIRECTORY launch/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 PATTERN ".svn" EXCLUDE)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_perception_unify_cloud.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
