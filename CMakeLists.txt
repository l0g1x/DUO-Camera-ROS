cmake_minimum_required(VERSION 2.8.3)
set(ProjectName duo3d_ros)
project(${ProjectName})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  camera_info_manager
  image_transport
  dynamic_reconfigure
  pcl_ros
)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

# Get DUO_SDK install path from environment variable if set
if(DEFINED ENV{DUO_SDK})
  message(STATUS "DUO SDK enviornment variable found.")
  set(DUO_SDK $ENV{DUO_SDK} )
  message(STATUS "DUO_SDK: ${DUO_SDK}")
else()
  message(STATUS "DUO SDK enviornment variable not found.")
  include(cmake/SetupDuoSDK.cmake)
  setup_duo() 
  # DUO_SDK envvar should be set after setup_duo()
  message(STATUS "DUO_SDK: ${DUO_SDK}")
endif()

set(DUO_INCLUDE_DIR ${DUO_SDK}/include)

# Detect Compiler / Arch and add libraries
if(APPLE)
  link_directories(${DUO_SDK}/osx/x64/)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
elseif(UNIX)
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL armv7l)
    set(DUOLib_ARCH arm)
  else()
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(DUOLib_ARCH x64)
    else()
      set(DUOLib_ARCH x86)
    endif()
  endif()
  set(DUO_SDK_ARCH ${DUO_SDK}/linux/${DUOLib_ARCH})
  link_directories(${DUO_SDK_ARCH}/)
#  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin/${DUOLib_ARCH} )
endif()

generate_dynamic_reconfigure_options(
  cfg/Duo.cfg
)

catkin_package( CATKIN_DEPENDS geometry_msgs roscpp tf)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(${DUO_INCLUDE_DIR})


add_executable(duo_node src/driverDUOstereo.cpp src/node.cpp)
add_executable(dense3d src/duo_dense3d.cpp)

add_dependencies(duo_node ${PROJECT_NAME}_gencfg)

# for linking duo_node to dynamic_reconfigure cfg files

set(DUO_LIB "${DUO_SDK_ARCH}/libDUO.so")
set(DENSE_LIB "${DUO_SDK_ARCH}/libDense3D.so")

target_link_libraries(duo_node ${catkin_LIBRARIES} ${DUO_LIB} )
set_target_properties(duo_node PROPERTIES COMPILE_FLAGS -g)
target_link_libraries(dense3d ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${DUO_LIB} ${DENSE_LIB} )
set_target_properties(dense3d PROPERTIES COMPILE_FLAGS -g)

#if(WIN32)
#        target_link_libraries(duo_node DUOlib)
#        target_link_libraries(dense3d DUOlib)
#elseif(UNIX OR APPLE)
#	target_link_libraries(duo_node duo)
#        target_link_libraries(dense3d duo)
#endif()

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
