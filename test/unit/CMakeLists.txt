cmake_minimum_required(VERSION 3.22)

project(count-osm-unit-tests)

find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME} phony_unit_test.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE count-osm::count-osm)
target_link_libraries(${PROJECT_NAME} PRIVATE GTest::gtest_main)
