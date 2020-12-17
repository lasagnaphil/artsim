cmake_minimum_required(VERSION 3.14)
project(artsim)

find_package(glm CONFIG REQUIRED)
set(CMAKE_CXX_STANDARD 17)

# We're only using Eigen temporarily for now, we're probably going to remove it in the future
find_package(Eigen3 CONFIG REQUIRED)

add_library(artsim src/artsim.cpp src/dynamics.cpp src/example_articulations.cpp include/artsim/articulation_state.h)
target_include_directories(artsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(artsim PUBLIC glm Eigen3::Eigen)

if (ARTSIM_BUILD_DEMOS)
    add_subdirectory(demo)
endif()
if (ARTSIM_BUILD_TESTS)
    add_subdirectory(tests)
endif()