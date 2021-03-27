add_executable(test_artsim_core test_main.cpp core/test_arena.cpp)
target_include_directories(test_artsim_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_artsim_core PRIVATE artsim)

add_executable(test_artsim test_main.cpp
        math/test_common.cpp math/test_se3.cpp
        dynamics/test_dynamics.cpp)
target_include_directories(test_artsim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_artsim PRIVATE artsim)
