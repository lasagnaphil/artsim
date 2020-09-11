cmake_minimum_required(VERSION 3.15)
project(artsim)

find_package(glm REQUIRED)
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

# We're only using Eigen temporarily for now, we're probably going to remove it in the future
if (NOT TARGET Eigen3)
    find_package(Eigen3 REQUIRED)
endif()

add_library(artsim src/artsim.cpp)
target_include_directories(artsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(artsim PUBLIC glm glmx Eigen3::Eigen)

if (ARTSIM_BUILD_DEMOS)
    add_subdirectory(demo)
endif()
if (ARTSIM_BUILD_TESTS)
    add_subdirectory(tests)
endif()