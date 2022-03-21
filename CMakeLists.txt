cmake_minimum_required(VERSION 3.14)
project(artsim)

list(APPEND VCPKG_OVERLAY_TRIPLETS ${CMAKE_SOURCE_DIR}/vcpkg)
set(VCPKG_TARGET_TRIPLET x64-windows-llvm CACHE STRING "VCPKG Target Triplet to use")
set(VCPKG_HOST_TRIPLET x64-windows-llvm CACHE STRING "VCPKG Host Triplet to use")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 17)

option(ARTSIM_USE_LOGGING "Log simulation info to stdout" OFF)
option(ARTSIM_USE_DOUBLE "Use double instead of float" ON)
option(ARTSIM_USE_OPENMP "Use OpenMP" ON)
option(ARTSIM_USE_ASAN "Use Address Sanitizer" OFF)
option(ARTSIM_BUILD_RENDERER "Build renderer" OFF)
option(ARTSIM_BUILD_DEMOS "Build demos" OFF)
option(ARTSIM_BULID_TESTS "Build tests" OFF)

if (ARTSIM_USE_ASAN)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
endif()

add_subdirectory(libs)

file(GLOB ARTSIM_SOURCES
        src/*.cpp
        src/math/*.cpp
        src/anim/*.cpp
        src/utils/*.cpp
        src/utils/pymesh/*.cpp)

add_library(artsim ${ARTSIM_SOURCES})

target_include_directories(artsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(ARTSIM_LIB
        tbb
        glm::glm
        Eigen3::Eigen
        pugixml::pugixml
        cereal
        mshio
        bullet_collision
        fastsvd
        tinyobjloader
        span-lite
        fmt
        stdc++fs
        tracy)

target_link_libraries(artsim PUBLIC ${ARTSIM_LIB})

# target_compile_definitions(artsim PRIVATE EIGEN_DONT_PARALLELIZE)

if (ARTSIM_USE_LOGGING)
    target_compile_definitions(artsim PUBLIC ARTSIM_USE_LOGGING)
endif()
if (ARTSIM_USE_DOUBLE)
    target_compile_definitions(artsim PUBLIC ARTSIM_USE_DOUBLE)
endif()

if (ARTSIM_BUILD_RENDERER OR ARTSIM_BUILD_DEMOS)
    add_subdirectory(renderer)
endif()
if (ARTSIM_BUILD_DEMOS)
    add_subdirectory(demo)
endif()
if (ARTSIM_BUILD_TESTS)
    add_subdirectory(tests)
endif()
add_subdirectory(benchmark)
