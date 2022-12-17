cmake_minimum_required(VERSION 3.22)

project(count-osm)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

option(COUNT_OSM_UNIT_TESTS "Enable unit tests" OFF)
option(COUNT_OSM_INTEGRATION_TESTS "Enable integration tests" OFF)
option(COUNT_OSM_BENCHMARKS "Enable benchmarks" OFF)

if(NOT UNIX) # this include apple
    message(FATAL_ERROR "Not meant for non *nix")
endif()

# disable stuff we don't want
add_compile_options(-fno-rtti)
add_compile_options(-fno-exceptions)
add_compile_options(-Wall -Wextra -Werror -Wpedantic)

include(CTest)

add_library(count-osm INTERFACE include/count-osm/count-osm.h)
target_include_directories(count-osm INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src>
    $<INSTALL_INTERFACE:include>)
add_library(count-osm::count-osm ALIAS count-osm)

add_executable(count-osm-app src/main.cpp)
target_link_libraries(count-osm-app count-osm)

if(COUNT_OSM_UNIT_TESTS)
    enable_testing()
    add_subdirectory(test/unit)
endif()

if(COUNT_OSM_BENCHMARKS)
    add_subdirectory(benchmark)
endif()

if(COUNT_OSM_INTEGRATION_TESTS)
    add_subdirectory(test/integration)
endif()
