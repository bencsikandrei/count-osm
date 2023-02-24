cmake_minimum_required(VERSION 3.22)

project(count-osm-integration-tests)

find_package(libdeflate REQUIRED)
find_package(ZLIB REQUIRED)

add_executable(${PROJECT_NAME} phony_integration_test.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE count-osm::count-osm)
target_link_libraries(${PROJECT_NAME} PRIVATE libdeflate::libdeflate)
