set(SPAN_LITE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/span-lite)
add_library(span-lite INTERFACE)
target_include_directories(span-lite INTERFACE ${SPAN_LITE_DIR})

add_subdirectory(glm)

set(EIGEN_BUILD_DOC OFF CACHE BOOL "Enable creation of Eigen documentation")
set(BUILD_TESTING OFF CACHE BOOL "Enable creation of Eigen tests.")
add_subdirectory(eigen)

add_subdirectory(bullet_collision)

add_subdirectory(fastsvd)

if (ARTSIM_USE_DOUBLE)
    set(DISCREGRID_USE_DOUBLE ON)
endif()

# Probably not going to use anymore, use TMD single-header library instead
add_subdirectory(Discregrid)

add_subdirectory(pugixml)

set(JUST_INSTALL_CEREAL ON CACHE BOOL "")
add_subdirectory(cereal)

add_subdirectory(fmt)

set(TBB_BUILD_TESTS OFF CACHE BOOL "Build TBB tests and enable testing infrastructure")
add_subdirectory(tbb)

# tiny_obj_loader
add_library(tinyobjloader STATIC ${CMAKE_CURRENT_SOURCE_DIR}/tiny_obj_loader/tiny_obj_loader.cc)
target_include_directories(tinyobjloader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tiny_obj_loader)

# Tracy
add_library(tracy STATIC ${CMAKE_CURRENT_SOURCE_DIR}/tracy/TracyClient.cpp)
target_include_directories(tracy PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tracy)
if (ARTSIM_PROFILE)
    target_compile_definitions(tracy PUBLIC TRACY_ENABLE)
endif()
