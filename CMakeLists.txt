project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++1z)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/Map.cpp src/PathPlanner.cpp src/Behavior.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.15.0/lib)

# uws - I don't want to install this with sudo!
include_directories(~/Documents/uni.uni/udacity/selfdriving_cars/term_2/projects/_res/uWebSockets/include)
link_directories(~/Documents/uni.uni/udacity/selfdriving_cars/term_2/projects/_res/uWebSockets/build)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS python2.7)
