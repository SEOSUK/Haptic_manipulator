################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(open_manipulator_teleop)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    open_manipulator_msgs
## ADD
    dynamixel_workbench_toolbox
    dynamixel_workbench_controllers
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
##add_service_files(FILES DockService.srv)
################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs open_manipulator_msgs 
## ADD
  dynamixel_workbench_toolbox
  dynamixel_workbench_controllers
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(open_manipulator_teleop_keyboard src/open_manipulator_teleop_keyboard.cpp)
add_dependencies(open_manipulator_teleop_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_teleop_keyboard ${catkin_LIBRARIES} )

add_executable(open_manipulator_teleop_joystick src/open_manipulator_teleop_joystick.cpp)
add_dependencies(open_manipulator_teleop_joystick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_teleop_joystick ${catkin_LIBRARIES} )

add_executable(try src/try.cpp)
add_dependencies(try ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(try ${catkin_LIBRARIES} )

add_executable(try2 src/try2.cpp)
add_dependencies(try2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(try2 ${catkin_LIBRARIES} )

add_executable(servoCtrl src/servoCtrl.cpp)
add_dependencies(servoCtrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(servoCtrl ${catkin_LIBRARIES} )

add_executable(ManipulatorCtrl src/ManipulatorCtrl.cpp)
add_dependencies(ManipulatorCtrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ManipulatorCtrl ${catkin_LIBRARIES} )

add_executable(haptic src/haptic.cpp)
add_dependencies(haptic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(haptic ${catkin_LIBRARIES} )

add_executable(KillManipulatorCtrl src/KillManipulatorCtrl.cpp)
add_dependencies(KillManipulatorCtrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(KillManipulatorCtrl ${catkin_LIBRARIES} )

add_executable(force src/force.cpp)
add_dependencies(force ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(force ${catkin_LIBRARIES} )

add_executable(try_s src/try_s.cpp)
add_dependencies(try_s ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(try_s ${catkin_LIBRARIES} )

add_executable(try_IK src/try_IK.cpp)
add_dependencies(try_IK ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(try_IK ${catkin_LIBRARIES} )

#add_executable(try23 src/try23.cpp)
#add_dependencies(try23 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(try23 ${catkin_LIBRARIES} )

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_teleop_keyboard open_manipulator_teleop_joystick
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
