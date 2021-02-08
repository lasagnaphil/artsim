cmake_minimum_required(VERSION 3.14)
project(artsim)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(GLM REQUIRED)
set(CMAKE_CXX_STANDARD 17)

# We're only using Eigen temporarily for now, we're probably going to remove it in the future
find_package(Eigen3 CONFIG REQUIRED)

# For URDF exports
find_package(TinyXML2 REQUIRED)

option(ARTSIM_USE_DOUBLE "Use double instead of float" ON)
option(ARTSIM_USE_ASAN "Use Address Sanitizer" OFF)

if (ARTSIM_USE_ASAN)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -O1")
endif()

add_subdirectory(libs/bullet_collision)

add_library(artsim
        src/artsim.cpp src/dynamics.cpp src/example_articulations.cpp
        src/contacts.cpp
        src/soft_body.cpp
        src/utils/urdf.cpp)

target_include_directories(artsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${GLM_INCLUDE_DIRS})
target_link_libraries(artsim PUBLIC Eigen3::Eigen tinyxml2::tinyxml2 bullet_collision)
target_compile_definitions(artsim PRIVATE EIGEN_DONT_PARALLELIZE)

if (ARTSIM_USE_DOUBLE)
    target_compile_definitions(artsim PUBLIC ARTSIM_USE_DOUBLE)
endif()

if (ARTSIM_BUILD_DEMOS)
    add_subdirectory(demo)
endif()
if (ARTSIM_BUILD_TESTS)
    add_subdirectory(tests)
endif()