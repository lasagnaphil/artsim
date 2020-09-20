add_executable(artsim_test test_main.cpp math/test_se3.cpp dynamics/test_dynamics.cpp)
target_include_directories(artsim_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(artsim_test PRIVATE artsim)
