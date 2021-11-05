find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

set(FASTSVD_SRCS src/fastsvd.cpp)
add_library(fastsvd STATIC ${FASTSVD_SRCS})
target_include_directories(fastsvd PUBLIC include)

option(FASTSVD_USE_SSE "Use SSE for fastsvd" OFF)
option(FASTSVD_USE_AVX "Use AVX for fastsvd" OFF)
option(FASTSVD_USE_AVX512 "Use AVX512 for fastsvd" OFF)

target_compile_options(fastsvd PRIVATE -march=native -mtune=native -O3)

if (FASTSVD_USE_SSE)
    target_compile_definitions(fastsvd PRIVATE USE_SSE_IMPLEMENTATION)
elseif (FASTSVD_USE_AVX)
    target_compile_definitions(fastsvd PRIVATE USE_AVX_IMPLEMENTATION)
elseif (FASTSVD_USE_AVX512)
    target_compile_definitions(fastsvd PRIVATE USE_AVX512_IMPLEMENTATION)
else()
    target_compile_definitions(fastsvd PRIVATE USE_SCALAR_IMPLEMENTATION)
endif()

target_link_libraries(fastsvd PUBLIC Threads::Threads)

add_executable(fastsvd_test_unit test/Singular_Value_Decomposition_Unit_Test.cpp)
target_link_libraries(fastsvd_test_unit PRIVATE fastsvd)

add_executable(fastsvd_test_streaming test/Singular_Value_Decomposition_Streaming_Test.cpp)
target_link_libraries(fastsvd_test_streaming PRIVATE fastsvd OpenMP::OpenMP_CXX)
