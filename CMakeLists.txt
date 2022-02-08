cmake_minimum_required(VERSION 3.4)
project(projection CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

option(BUILD_TESTS OFF CACHE)
option(BUILD_EXAMPLES ON CACHE)

find_package(Eigen3 REQUIRED NO_MODULE)

add_subdirectory(extern)
add_subdirectory(src)

# if (BUILD_EXAMPLES)
#     add_subdirectory(examples)
# endif()

if (BUILD_TESTS)
    message("HERE")
    add_subdirectory(tests)
endif()
