cmake_minimum_required (VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(THREADS_PREFER_PTHREAD_FLAG ON)

project (BaboCam)

find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(nlohmann_json REQUIRED)
#find_package(serial REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "\tversion ${OpenCV_VERSION}")

file(GLOB_RECURSE babocam_SRC
		"src/*.hpp"
		"src/*.cpp"
)

add_executable(BaboCam ${babocam_SRC})

target_link_libraries(BaboCam ${OpenCV_LIBS} Threads::Threads realsense2 serial nlohmann_json::nlohmann_json)
