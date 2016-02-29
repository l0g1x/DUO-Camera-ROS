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
  cv_bridge
)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# Add DUO include and lib
set(DUO_SDK /home/lbarnett/development/CL-DUO3D-LIN-1.0.35.226/DUOSDK/SDK)
include_directories(${DUO_SDK}/include ${PCL_INCLUDE_DIRS})

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
#add_executable(duodense3d_node src/driverDUODense3D.cpp src/driverDUOstereo.cpp src/dense3d_node.cpp)
add_executable(dense3d src/duo_dense3d.cpp)

add_dependencies(duo_node ${PROJECT_NAME}_gencfg)

# for linking duo_node to dynamic_reconfigure cfg files

set(DUO_LIB "${DUO_SDK_ARCH}/libDUO.so")
set(DENSE_LIB "${DUO_SDK_ARCH}/libDense3D.so")

target_link_libraries(duo_node ${catkin_LIBRARIES} ${DUO_LIB} )
set_target_properties(duo_node PROPERTIES COMPILE_FLAGS -g)
target_link_libraries(dense3d ${catkin_LIBRARIES} ${PCL_LIBRARIES})
set_target_properties(dense3d PROPERTIES COMPILE_FLAGS -g)
#target_link_libraries(duodense3d_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
set_target_properties(dense3d PROPERTIES COMPILE_FLAGS -g)

if(WIN32)
        target_link_libraries(duo_node DUOlib)
        target_link_libraries(dense3d DUOlib)
#        target_link_libraries(duodense3d_node DUOlib)
elseif(UNIX OR APPLE)
	target_link_libraries(duo_node DUO Dense3D)
        target_link_libraries(dense3d DUO Dense3D)        
#        target_link_libraries(duodense3d_node DUO Dense3D)        
endif()

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
