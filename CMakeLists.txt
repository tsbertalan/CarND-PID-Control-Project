project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


set(sources src/PID.cpp src/main.cpp)
add_executable(pid ${sources})
target_link_libraries(pid z ssl uv uWS)

set(sources_twiddle src/PID.cpp src/twiddle.cpp src/twiddle_main.cpp)
add_executable(twiddle ${sources_twiddle})
target_link_libraries(twiddle z ssl uv uWS)

set(CMAKE_BUILD_TYPE Debug)
