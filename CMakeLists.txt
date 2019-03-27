cmake_minimum_required (VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(THREADS_PREFER_PTHREAD_FLAG ON)

project (BaboCam)

find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(realsense2 REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "\tversion ${OpenCV_VERSION}")

file(GLOB babocam_SRC
		"src/*.hpp"
		"src/*.cpp"
		"src/socket/*.cpp"
		"src/socket/*.hpp"
		"src/color/*.cpp"
		"src/color/*.hpp"
		"src/interfaces/*.hpp"
)

add_executable(BaboCam ${babocam_SRC})

target_link_libraries(BaboCam ${OpenCV_LIBS} Threads::Threads realsense2)
