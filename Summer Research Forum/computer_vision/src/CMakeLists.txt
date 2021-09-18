cmake_minimum_required(VERSION 2.8)
project( FruitDet )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( FruitDet fruit_det.cpp )
target_link_libraries( FruitDet ${OpenCV_LIBS} )

