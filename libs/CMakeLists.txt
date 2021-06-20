add_subdirectory(bullet_collision)
add_subdirectory(fastsvd)

# tiny_obj_loader
add_library(tinyobjloader STATIC ${CMAKE_CURRENT_SOURCE_DIR}/tiny_obj_loader/tiny_obj_loader.cc)
target_include_directories(tinyobjloader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tiny_obj_loader)

# std-simd
add_library(std_simd INTERFACE)
target_include_directories(std_simd INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/std-simd-1.0.0)

# Tracy
add_library(tracy STATIC ${CMAKE_CURRENT_SOURCE_DIR}/tracy/TracyClient.cpp)
target_include_directories(tracy PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tracy)
if (ARTSIM_PROFILE)
    target_compile_definitions(tracy PUBLIC TRACY_ENABLE)
endif()
