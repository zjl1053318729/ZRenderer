cmake_minimum_required(VERSION 3.24)

project(ZRender)

set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(assimp REQUIRED)

find_path(STB_INCLUDE_DIRS "stb_c_lexer.h")

add_executable(ZRender src/main/ZRender.cpp)

target_link_libraries(ZRender PRIVATE Eigen3::Eigen)
target_link_libraries(ZRender PRIVATE TBB::tbb TBB::tbbmalloc TBB::tbbmalloc_proxy)
target_link_libraries(ZRender PRIVATE assimp::assimp)
target_include_directories(ZRender PRIVATE ${STB_INCLUDE_DIRS})