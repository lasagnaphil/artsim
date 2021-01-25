file(GLOB_RECURSE BULLET_COLLISION_SRCS BulletCollision/*.cpp)
file(GLOB_RECURSE BULLET_LINEARMATH_SRCS LinearMath/*.cpp)

add_library(bullet_collision STATIC ${BULLET_COLLISION_SRCS} ${BULLET_LINEARMATH_SRCS})
target_include_directories(bullet_collision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
if (ARTSIM_USE_DOUBLE)
    target_compile_definitions(bullet_collision PUBLIC BT_USE_DOUBLE_PRECISION)
endif()
