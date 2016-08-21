cmake_minimum_required(VERSION 2.8.3)
project(menge_gazebo_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
set(MENGE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../Menge)
set(MENGE_SRC_DIR ${MENGE_ROOT}/src/Menge)
set(MENGE_EXE_DIR ${MENGE_ROOT}/Exe)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/menge_gazebo_plugin)
include_directories(${MENGE_SRC_DIR})
include_directories(${MENGE_SRC_DIR}/include)
include_directories(${MENGE_SRC_DIR}/MengeCore)
include_directories(${MENGE_SRC_DIR}/MengeCore/BFSM)
include_directories(${MENGE_SRC_DIR}/MengeCore/Agents)
include_directories(${MENGE_SRC_DIR}/MengeCore/Runtime)
include_directories(${MENGE_SRC_DIR}/MengeCore/SceneGraph)
include_directories(${MENGE_SRC_DIR}/MengeCore/PluginEngine)
include_directories(${MENGE_SRC_DIR}/MengeCore/Math)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../menge_common/include)
link_directories (${MENGE_EXE_DIR})
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(GAZEBO)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES menge_gazebo_plugin
#  CATKIN_DEPENDS  roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
add_library(menge_gazebo_plugin
   src/MengeDefaultPlugin.cpp
   ../menge_common/src/MengePlugin.cpp
   ../menge_common/src/MyViewer.cpp
   ../menge_common/src/ProjectSpec.cpp
)

target_link_libraries(menge_gazebo_plugin
  ${GAZEBO_LIBRARIES} mengeCore
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(menge_gazebo_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(menge_gazebo_plugin_node src/menge_gazebo_plugin_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(menge_gazebo_plugin_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(menge_gazebo_plugin_node
#   ${catkin_LIBRARIES}
# )

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
install(TARGETS menge_gazebo_plugin
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

install(DIRECTORY ${MENGE_EXE_DIR}/plugins/
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   FILES_MATCHING PATTERN "*.so"
   PATTERN ".svn" EXCLUDE
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   ${MENGE_EXE_DIR}/libmengeCore.so
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_menge_gazebo_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
