cmake_minimum_required(VERSION 3.22)

project(count-osm-integration-tests)

add_executable(${PROJECT_NAME} phony_integration_test.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE count-osm::count-osm)
