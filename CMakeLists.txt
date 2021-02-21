cmake_minimum_required(VERSION 3.14)
project(artsim)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 17)

find_package(GLM REQUIRED)

# We're only using Eigen temporarily for now, we're probably going to remove it in the future
find_package(Eigen3 CONFIG REQUIRED)

# For URDF exports
find_package(TinyXML2 REQUIRED)

option(ARTSIM_USE_DOUBLE "Use double instead of float" OFF)
option(ARTSIM_USE_OPENMP "Use OpenMP" ON)
option(ARTSIM_USE_ASAN "Use Address Sanitizer" OFF)
option(ARTSIM_BUILD_RENDERER "Build renderer" OFF)
option(ARTSIM_BUILD_DEMOS "Build demos" OFF)
option(ARTSIM_BULID_TESTS "Build tests" OFF)

if (ARTSIM_USE_OPENMP)
    find_package(OpenMP REQUIRED)
endif()
if (ARTSIM_USE_ASAN)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -O1")
endif()

add_subdirectory(libs/bullet_collision)
add_subdirectory(libs/fastsvd)

add_library(artsim
        src/artsim.cpp src/dynamics.cpp
        src/contacts.cpp
        src/soft_body.cpp
        src/soft_body_with_art.cpp
        src/obj_file.cpp
        src/math/fastsvd.cpp
        src/utils/example_articulations.cpp
        src/utils/urdf.cpp src/utils/xml.cpp
        src/utils/pymesh/MshLoader.cpp src/utils/pymesh/MshSaver.cpp src/utils/pymesh/IOUtils.cpp)

target_include_directories(artsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${GLM_INCLUDE_DIRS})
set(ARTSIM_LIB Eigen3::Eigen tinyxml2::tinyxml2 bullet_collision fastsvd stdc++fs)
if (ARTSIM_USE_OPENMP)
    set(ARTSIM_LIB ${ARTSIM_LIB} OpenMP::OpenMP_CXX)
endif()
target_link_libraries(artsim PUBLIC ${ARTSIM_LIB})
target_compile_definitions(artsim PRIVATE EIGEN_DONT_PARALLELIZE)

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