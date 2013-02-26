cmake_minimum_required(VERSION 2.6)
project(CLIENT)

# Source files
set(SRC
    helpers.cc MoveRobot.cc ConfigFileReader.cc
    PCLutils.cc OutputHandler.cc client.cc)
# Program name
set(PROG client)

# use cmake to find PCL library
find_package(PCL 1.3 REQUIRED 
  COMPONENTS common io visualization filters)
# use cmake to find OpenCV library
# if it is not found, it is included and linked manually
find_package( OpenCV REQUIRED )

# Aria headers location
include_directories(/usr/local/Aria/include/)
# ArNetworking headers location
include_directories(/usr/local/Aria/ArNetworking/include/)
# PCL headers location
include_directories(${PCL_INCLUDE_DIRS})
# OpenCV headers location
include_directories(/usr/include/opencv)

# Aria libraries location
link_directories(/usr/local/Aria/lib/)
# PCL libraries location and definitions
link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
# OpenCV libraries are in default location

# C++ flags
#set(CMAKE_CXX_FLAGS -Wall)

# Create the executable
add_executable(${PROG} ${SRC})

# Link the libraries
# Required Aria libraries
set(Aria_LIBS Aria ArNetworking pthread dl rt)
# Required OpenCV libraries
# if cmake finds OpenCV then OpenCV_LIBS will link the libraries
set(OpenCV_LIBS_CUSTOM opencv_core opencv_video)
target_link_libraries(${PROG} 
  ${Aria_LIBS}
  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${OpenCV_LIBS} ${OpenCV_LIBS_CUSTOM}
)
