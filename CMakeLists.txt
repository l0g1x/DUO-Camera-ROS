cmake_minimum_required(VERSION 2.8.3)
set(ProjectName duo3d_ros)
project(${ProjectName})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/DUO3D.cfg
)

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

catkin_package( CATKIN_DEPENDS geometry_msgs roscpp tf dynamic_reconfigure)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(duo_node src/duo.cpp)

# add_dependencies(duo3d_ros_node duo3d_ros_generate_messages_cpp)

# for linking duo_node to dynamic_reconfigure cfg files
add_dependencies(duo_node ${PROJECT_NAME}_gencfg)

target_link_libraries(duo_node ${catkin_LIBRARIES} )

if(WIN32)
        target_link_libraries(duo_node DUOlib)
elseif(UNIX OR APPLE)
        target_link_libraries(duo_node duo)
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
