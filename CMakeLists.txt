cmake_minimum_required(VERSION 3.14)
project(artsim)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 17)

find_package(GLM REQUIRED)

find_package(Eigen3 REQUIRED)

# For URDF exports
find_package(TinyXML2 REQUIRED)

find_package(TBB REQUIRED)

option(ARTSIM_USE_LOGGING "Log simulation info to stdout" OFF)
option(ARTSIM_USE_DOUBLE "Use double instead of float" ON)
option(ARTSIM_USE_OPENMP "Use OpenMP" ON)
option(ARTSIM_USE_ASAN "Use Address Sanitizer" OFF)
option(ARTSIM_BUILD_RENDERER "Build renderer" OFF)
option(ARTSIM_BUILD_DEMOS "Build demos" OFF)
option(ARTSIM_BULID_TESTS "Build tests" OFF)

if (ARTSIM_USE_OPENMP)
    find_package(OpenMP REQUIRED)
endif()
if (ARTSIM_USE_ASAN)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -O0")
endif()

add_subdirectory(libs)

file(GLOB ARTSIM_SOURCES
        src/*.cpp
        src/math/*.cpp
        src/collision/*.cpp
        src/anim/*.cpp
        src/utils/*.cpp
        src/utils/pymesh/*.cpp)

add_library(artsim ${ARTSIM_SOURCES})

target_include_directories(artsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${GLM_INCLUDE_DIRS})
set(ARTSIM_LIB
        tbb
        Eigen3::Eigen
        pugixml::pugixml
        bullet_collision
        fastsvd
        Discregrid
        tinyobjloader
        std_simd
        span-lite
        stdc++fs
        tracy)

if (ARTSIM_USE_OPENMP)
    set(ARTSIM_LIB ${ARTSIM_LIB} OpenMP::OpenMP_CXX)
endif()

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