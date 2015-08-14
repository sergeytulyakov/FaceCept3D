cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(HeadPoseEstimationFramework)

FILE (GLOB_RECURSE SRC_LIST *.cpp *.h)
foreach (SRC ${SRC_LIST})
	string (FIND ${SRC} "WindowsOnly" FOUND)
	if (NOT ${FOUND} EQUAL "-1")
		list (REMOVE_ITEM SRC_LIST ${SRC})
	endif()
endforeach ()

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR})
link_directories(${PCL_LIBRARY_DIRS} ${CMAKE_BINARY_DIR} )
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS})


