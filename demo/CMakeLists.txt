add_custom_target(artsim_copy_resource_folder)
add_custom_command(TARGET artsim_copy_resource_folder POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/resources
        ${CMAKE_CURRENT_BINARY_DIR}/resources)

add_executable(gengine_demo_spheres_pbr demo_spheres_pbr.cpp)
target_link_libraries(gengine_demo_spheres_pbr PRIVATE gengine)

add_executable(artsim_demo_basic demo_basic.cpp)
target_link_libraries(artsim_demo_basic PRIVATE gengine)

add_executable(artsim_demo_human demo_human.cpp)
target_link_libraries(artsim_demo_human PRIVATE gengine)