cmake_minimum_required(VERSION 2.8)
set(CMAKE_PREFIX_PATH "/usr/local/Cellar/opencv/2.4.12_2/share/OpenCV/; /usr/local/share/Ceres/; ${CMAKE_PREFIX_PATH}")

project( Apple_Juice )
find_package( OpenCV REQUIRED )
find_package(Ceres REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${ARMADILLO_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

ADD_DEFINITIONS(
	-std=c++11
)

add_executable( AppleJuice Demo.cpp AppleJuice.cc Graycode.cpp)
target_link_libraries( AppleJuice ${OpenCV_LIBS} ${ARMADILLO_LIBRARIES} ${CERES_LIBRARIES} )
