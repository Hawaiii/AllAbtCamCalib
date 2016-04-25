cmake_minimum_required(VERSION 2.8)
project( AppleJuice )
find_package( OpenCV REQUIRED )
find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})

ADD_DEFINITIONS(
	-std=c++11
)

add_executable( AppleJuice Demo.cpp AppleJuice.cc Graycode.cpp)
target_link_libraries( AppleJuice ${OpenCV_LIBS} ${CERES_LIBRARIES})
