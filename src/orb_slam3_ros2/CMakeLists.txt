cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros2)

# TODO add this note https://github.com/mikeferguson/ros2_cookbook/blob/main/pages/cmake.md

# TODO depricate this line 
# set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/") # Must be set to match your installation

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set optimization flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Add definition if needed
add_definitions(-DCOMPILEDWITHC17)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(matimg_custom_msg_interface REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 3.3.0 REQUIRED) # Matched with Sophus
find_package(Pangolin REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml-cpp REQUIRED) # jbeder's YAML parser and emitter library in C++

# TODO these packages are not required
# find_package(ros2_tictoc_profiler REQUIRED) # ROS 2 library only package, port of Daniel Mataurna`s tictoc_profiler package
# find_package(ros2_tftn REQUIRED) # ROS 2 package containing Dr. Rui Fan`s Three Filters to Normal surface normal estimation technique
# find_package(ros2_fast_mf REQUIRED) # ROS 2 package containing the FastMF library

# DO NOT DELETE
# Doesn't work, needs to work with very old version as discussed here https://github.com/lrse/sptam/issues/49
# find_package(G2O REQUIRED)
# if(NOT G2O_FOUND)
#    message(FATAL_ERROR "G2O not found.")
# endif()

# Header file locations
include_directories(include) # This package include
include_directories(${EIGEN3_INCLUDE_DIRS}) # Include headers for eigen3
include_directories(${Pangolin_INCLUDE_DIRS}) # include headers for pangolin
include_directories(${OpenCV_INCLUDE_DIRS}) # include headers for pangolin

#include_directories(${ros2_tftn_INCLUDE_DIRS}) # ROS2 port of TFTN
#include_directories(${ros2_tictoc_profiler_INCLUDE_DIRS}) # Tictoc timing library
#include_directories(${ros2_fast_mf_INCLUDE_DIRS}) # FastMF library

# ORB-SLAM3 includes
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/orb_slam3
  ${PROJECT_SOURCE_DIR}/orb_slam3/include
  ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
  ${ament_INCLUDE_DIRS}
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  sensor_msgs
  matimg_custom_msg_interface
  cv_bridge
  image_transport
  OpenCV
  Eigen3
  Pangolin
  yaml-cpp
)

# Redundant?
# include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/CMakeLists.txt)
# include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/CMakeLists.txt)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam3/lib)

# add_library(${PROJECT_NAME} SHARED
add_library(orb_slam3_ros2 SHARED
  orb_slam3/src/System.cc
  orb_slam3/src/Tracking.cc
  orb_slam3/src/LocalMapping.cc
  orb_slam3/src/LoopClosing.cc
  orb_slam3/src/ORBextractor.cc
  orb_slam3/src/ORBmatcher.cc
  orb_slam3/src/FrameDrawer.cc
  orb_slam3/src/Converter.cc
  orb_slam3/src/MapPoint.cc
  orb_slam3/src/KeyFrame.cc
  orb_slam3/src/Atlas.cc
  orb_slam3/src/Map.cc
  orb_slam3/src/MapDrawer.cc
  orb_slam3/src/Optimizer.cc
  orb_slam3/src/Frame.cc
  orb_slam3/src/KeyFrameDatabase.cc
  orb_slam3/src/Sim3Solver.cc
  orb_slam3/src/Viewer.cc
  orb_slam3/src/ImuTypes.cc
  orb_slam3/src/G2oTypes.cc
  orb_slam3/src/CameraModels/Pinhole.cpp
  orb_slam3/src/CameraModels/KannalaBrandt8.cpp
  orb_slam3/src/OptimizableTypes.cpp
  orb_slam3/src/MLPnPsolver.cpp
  orb_slam3/src/GeometricTools.cc
  orb_slam3/src/TwoViewReconstruction.cc
  orb_slam3/src/Config.cc
  orb_slam3/src/Settings.cc
  orb_slam3/src/SystemUtils.cc
  orb_slam3/include/System.h
  orb_slam3/include/Tracking.h
  orb_slam3/include/LocalMapping.h
  orb_slam3/include/LoopClosing.h
  orb_slam3/include/ORBextractor.h
  orb_slam3/include/ORBmatcher.h
  orb_slam3/include/FrameDrawer.h
  orb_slam3/include/Converter.h
  orb_slam3/include/MapPoint.h
  orb_slam3/include/KeyFrame.h
  orb_slam3/include/Atlas.h
  orb_slam3/include/Map.h
  orb_slam3/include/MapDrawer.h
  orb_slam3/include/Optimizer.h
  orb_slam3/include/Frame.h
  orb_slam3/include/KeyFrameDatabase.h
  orb_slam3/include/Sim3Solver.h
  orb_slam3/include/Viewer.h
  orb_slam3/include/ImuTypes.h
  orb_slam3/include/G2oTypes.h
  orb_slam3/include/CameraModels/GeometricCamera.h
  orb_slam3/include/CameraModels/Pinhole.h
  orb_slam3/include/CameraModels/KannalaBrandt8.h
  orb_slam3/include/OptimizableTypes.h
  orb_slam3/include/MLPnPsolver.h
  orb_slam3/include/GeometricTools.h
  orb_slam3/include/TwoViewReconstruction.h
  orb_slam3/include/SerializationUtils.h
  orb_slam3/include/Config.h
  orb_slam3/include/Settings.h
  orb_slam3/include/SystemUtils.h
  orb_slam3/include/CommonStructs.h
)

set_target_properties(orb_slam3_ros2 PROPERTIES VERSION "${orb_slam3_ros2_VERSION}")

ament_target_dependencies(orb_slam3_ros2
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Linking against orb_slam3_ros2 
target_link_libraries(orb_slam3_ros2
  ${ros2_tictoc_profiler_LIBRARIES}
  ${ros2_tftn_LIBRARIES} 
  ${ros2_fast_mf_LIBRARIES}  
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto
  yaml-cpp
)

# Find your .so files (replace lib1.so and lib2.so with your actual file names)
set(DBoW2_PATH "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so")
set(g2o_PATH "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so")

# Install .so files to the lib directory in the install space
install(FILES ${DBoW2_PATH} DESTINATION lib)
install(FILES ${g2o_PATH} DESTINATION lib)

# Define executables, link libraries and install

add_executable(vslam_node
  src/vslam_node.cpp
  src/vslam.cpp
)

ament_target_dependencies(vslam_node
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(vslam_node PUBLIC orb_slam3_ros2) # Link executibles with internal library

# Install all the header files in package/package/include
install (DIRECTORY include/ 
        DESTINATION include
)

# Install nodes and shared libraries
install(TARGETS vslam_node orb_slam3_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

# Tell downstream packages where to find our headers
ament_export_include_directories(include)

# Tell downstream packages our libraries to link against
ament_export_libraries(orb_slam3_ros2)

# Help downstream packages to find transitive dependencies
ament_export_dependencies(
  orb_slam3_ros2
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package() # Always the last line
