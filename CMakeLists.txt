cmake_minimum_required(VERSION 3.5)
project(pylon_instant_camera_ros1)

set (CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
endif()

find_package(catkin REQUIRED COMPONENTS
roscpp
nodelet
image_transport
camera_calibration_parsers)
find_package(image_transport REQUIRED)

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
if (NOT ${Pylon_FOUND})
    message(FATAL "Unable to find Basler Pylon SDK.")
endif()
include_directories(${catkin_INCLUDE_DIRS})
catkin_package(
  LIBRARIES pylon_instant_camera_ros1
   )


#add_library(${NODE_NAME} SHARED src/main.cpp)
add_library(pylon_instant_camera_ros1 src/main.cpp )
target_include_directories(pylon_instant_camera_ros1 PUBLIC
    ${Pylon_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(pylon_instant_camera_ros1
    ${Pylon_LIBRARIES}
    ${catkin_LIBRARIES}
)
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(pylon_instant_camera_ros1 ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS pylon_instant_camera_ros1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})