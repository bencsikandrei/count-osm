cmake_minimum_required(VERSION 3.22)

project(count-osm-benchmarks)

find_package(benchmark REQUIRED)

add_executable(${PROJECT_NAME} phony_benchmark.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE count-osm::count-osm)
target_link_libraries(${PROJECT_NAME} PRIVATE benchmark::benchmark)
