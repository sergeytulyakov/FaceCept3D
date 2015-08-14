cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(HEAD_POSE_ESTIMATION "HeadPoseEstimationFramework")

cmake_policy(SET CMP0015 OLD)

project(TemplateCreator)

FILE (GLOB SRC_LIST *.cpp *.h)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ../${HEAD_POSE_ESTIMATION})
link_directories(${PCL_LIBRARY_DIRS} ${CMAKE_BINARY_DIR} ../${HEAD_POSE_ESTIMATION}-build)
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${HEAD_POSE_ESTIMATION})

