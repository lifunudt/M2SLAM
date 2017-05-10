cmake_minimum_required(VERSION 2.8.3)
project(orbslam_server)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/orbslam_server/cmake/Modules")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
        )

## System dependencies are found with CMake's conventions
find_package(OpenCV 2.4.3 REQUIRED)
find_package(Boost COMPONENTS serialization system filesystem REQUIRED)


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
add_service_files(
        FILES
        orbslam_save.srv
        orbslam_get.srv
        orbslam_muilt_save.srv
        orbslam_muilt_get.srv
        orbslam_pose_save.srv
        orbslam_pose_get.srv

)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

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
        #  LIBRARIES orbslam_server
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
        CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(orbslam_server
#   src/${PROJECT_NAME}/orbslam_server.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(orbslam_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(orbslam_server_node src/orbslam_server_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(orbslam_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(orbslam_server_node
#   ${catkin_LIBRARIES}
# )

##############################
## compile the odb project  ##
##############################

## find the odb and the pgsql

find_package(ODB REQUIRED COMPONENTS pgsql)

include(${ODB_USE_FILE})

INCLUDE_DIRECTORIES(
        ${CMAKE_SOURCE_DIR}/orbslam_server/include)

set(orbslam_server_SOURCES
        ${CMAKE_SOURCE_DIR}/orbslam_server/src/main.cpp
        ${CMAKE_SOURCE_DIR}/orbslam_server/include/database.h)

set(orbslam_server_ODB_HEADERS
        ${CMAKE_SOURCE_DIR}/orbslam_server/include/person.h
        ${CMAKE_SOURCE_DIR}/orbslam_server/include/Data_KeyFrame.h
        include/Data_MapPoint.h
        include/Serialization.h
        include/Data_TopoMapPoint.h
        include/Data_TopoKeyFrame.h)

message(STATUS "ODB_HEADERS : ${orbslam_server_ODB_HEADERS}")

odb_compile(orbslam_server_SOURCES FILES ${orbslam_server_ODB_HEADERS} DB pgsql GENERATE_QUERY GENERATE_SESSION)


add_executable(orbslam_server
        ${orbslam_server_SOURCES}
        ${orbslam_server_ODB_HEADERS})
target_link_libraries(orbslam_server
        ${ODB_LIBRARIES}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )
add_dependencies(orbslam_server orbslam_server_gencpp)
target_include_directories(orbslam_server
        PRIVATE
        ${ODB_INCLUDE_DIRS}
        ${ODB_COMPILE_OUTPUT_DIR})
target_compile_definitions(orbslam_server
        PRIVATE
        DATABASE_PGSQL)


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
# install(TARGETS orbslam_server orbslam_server_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_orbslam_server.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
