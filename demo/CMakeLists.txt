add_custom_target(artsim_copy_resource_folder)
add_custom_command(TARGET artsim_copy_resource_folder POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/resources
        ${CMAKE_CURRENT_BINARY_DIR}/resources)

find_package(raylib REQUIRED)

add_executable(artsim_demo_pendulum pendulum.cpp)
target_link_libraries(artsim_demo_pendulum PRIVATE artsim raylib)
#
# add_executable(artsim_demo_contacts contacts.cpp)
# target_link_libraries(artsim_demo_contacts PRIVATE gengine artsim)
# add_dependencies(artsim_demo_contacts artsim_copy_resource_folder)
#
# add_executable(artsim_demo_pd_test pd_test.cpp)
# target_link_libraries(artsim_demo_pd_test PRIVATE gengine artsim)
