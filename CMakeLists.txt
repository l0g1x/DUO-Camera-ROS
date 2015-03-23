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

# Add DUO include and lib
include_directories(${DUO_SDK}/include ${PCL_INCLUDE_DIRS})

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

generate_dynamic_reconfigure_options(
  cfg/Duo.cfg
)

catkin_package( CATKIN_DEPENDS geometry_msgs roscpp tf)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(duo_node src/driverDUOstereo.cpp src/node.cpp)
add_executable(dense3d src/duo_dense3d.cpp)

add_dependencies(duo_node ${PROJECT_NAME}_gencfg)

# for linking duo_node to dynamic_reconfigure cfg files

target_link_libraries(duo_node ${catkin_LIBRARIES} )
set_target_properties(duo_node PROPERTIES COMPILE_FLAGS -g)
target_link_libraries(dense3d ${catkin_LIBRARIES} ${PCL_LIBRARIES})
set_target_properties(dense3d PROPERTIES COMPILE_FLAGS -g)

if(WIN32)
        target_link_libraries(duo_node DUOlib)
        target_link_libraries(dense3d DUOlib)
elseif(UNIX OR APPLE)
	target_link_libraries(duo_node duo)
        target_link_libraries(dense3d duo)
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
