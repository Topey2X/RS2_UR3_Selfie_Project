cmake_minimum_required(VERSION 3.0.2)
project(optimizeCplusplus)
set(BUILD_TESTS ON)

# add_executable(nearestLine nearestLines.cpp)
add_executable(nearestLine2 main.cpp nearestLineEndpoint.cpp)
# add_library(functions function.cpp)
# target_link_libraries(optimization functions)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()