cmake_minimum_required(VERSION 2.8.3)
set(ProjectName duo3d_ros)
project(${ProjectName})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

#find_package(duo REQUIRED)


# Add DUO include and lib
include_directories(${DUO_SDK}/include)

# Detect Compiler / Arch and add libraries
if(MSVC)
  if(CMAKE_CL_64)
    set(DUOLib_ARCH x64)
  else()
    set(DUOLib_ARCH x86)
  endif()
  if(MSVC_VERSION EQUAL 1600)
    set(DUOLib_RUNTIME msvc2010)
  elseif(MSVC_VERSION EQUAL 1700)
    set(DUOLib_RUNTIME msvc2012)
  endif()
  link_directories(${DUO_SDK}/windows/${DUOLib_RUNTIME}/${DUOLib_ARCH})
  file(TO_NATIVE_PATH ${DUO_SDK}/windows/${DUOLib_RUNTIME}/${DUOLib_ARCH}/DUOLib.dll SRC)
  file(TO_NATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin/. DST)
  execute_process(COMMAND cmd /c copy /y ${SRC} ${DST})
  # Set output directory for binaries
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin )
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin )
elseif(APPLE)
  link_directories(${DUO_SDK}/osx/x64/)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
elseif(UNIX)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(DUOLib_ARCH x64)
  else()
    set(DUOLib_ARCH x86)
  endif()
  link_directories(${DUO_SDK}/linux/${DUOLib_ARCH}/)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
endif()


# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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
CATKIN_DEPENDS geometry_msgs roscpp tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})
#include_directories(${DUO_SDK})

## Declare a cpp library
# add_library(duo3d_ros
#   src/${PROJECT_NAME}/duo3d_ros.cpp
# )

## Declare a cpp executable
add_executable(duo_node src/duo.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(duo3d_ros_node duo3d_ros_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(duo_node ${catkin_LIBRARIES} )

if(WIN32)
        target_link_libraries(duo_node DUOlib)
elseif(UNIX OR APPLE)
        target_link_libraries(duo_node duo)
endif()


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
# install(TARGETS duo3d_ros duo3d_ros_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_duo3d_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
