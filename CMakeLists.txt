cmake_minimum_required(VERSION 3.1)
project(potc)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-reorder -Wno-sign-compare")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=memory")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")

find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

add_library(potc_lib STATIC src/der_visitor.cc src/opt_fuse.cc src/ir_visitor.cc src/opt_unused.cc src/opt_duplicate.cc src/parser.cc src/gen_print.cc src/opt_redundant_decl.cc src/opt_arithmetic.cc src/opt_check.cc src/gen_spline.cc src/opt_thread.cc src/opt_vectorize.cc src/opt_shorten.cc src/opt_fold_invalid.cc src/opt_invariant.cc src/opt_param.cc src/opt_kokkos_pushdown.cc)

add_executable(potc src/main.cc)
target_link_libraries(potc potc_lib)



enable_testing()

if( GTest_FOUND )

add_executable(potc_test src/main_test.cc)
target_include_directories(potc_test PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(potc_test potc_lib ${GTEST_BOTH_LIBRARIES} Threads::Threads)
add_test(potc_test potc_test)

add_executable(spline_test src/spline_test.cc)
target_include_directories(spline_test PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(spline_test ${GTEST_BOTH_LIBRARIES} Threads::Threads)
add_test(spline_test spline_test)

add_executable(format_test src/gen_internal_test.cc)
target_include_directories(format_test PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(format_test ${GTEST_BOTH_LIBRARIES} Threads::Threads)
add_test(format_test format_test)

endif()
