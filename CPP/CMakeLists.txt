cmake_minimum_required(VERSION 2.8)
project( Apple_Juice )
find_package( OpenCV REQUIRED )
find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})

ADD_DEFINITIONS(
	-std=c++11
)

add_executable( Apple_Juice Demo.cpp )
target_link_libraries( Apple_Juice ${OpenCV_LIBS} ${CERES_LIBRARIES})
