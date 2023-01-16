cmake_minimum_required(VERSION 3.22)

project(count-osm-unit-tests)

find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE
    numbers_tests.cpp
)
target_precompile_headers(${PROJECT_NAME} PRIVATE cosm_tests.pch.h)
target_link_libraries(${PROJECT_NAME} PRIVATE count-osm::count-osm)
target_link_libraries(${PROJECT_NAME} PRIVATE GTest::gtest_main)
set_target_properties(${PROJECT_NAME} PROPERTIES
    UNITY_BUILD ON
    UNITY_BUILD_MODE BATCH
    UNITY_BUILD_BATCH_SIZE 4
)

