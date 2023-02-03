cmake_minimum_required(VERSION 3.24)

project(ZRenderer)

set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(assimp REQUIRED)

find_path(STB_INCLUDE_DIRS "stb_c_lexer.h")

add_executable(
        ZRenderer src/main/ZRenderer.cpp
        src/core/camera.hpp
        src/core/Parser.hpp src/core/Scene.hpp src/accelerator/BVHAccel.hpp src/core/geometry.hpp src/core/ZRender.hpp src/core/intersection.hpp src/core/primitive.hpp src/shape/shape.hpp src/core/transform.hpp src/shape/sphere.hpp src/shape/triangle.hpp)

target_link_libraries(ZRenderer PRIVATE Eigen3::Eigen)
target_link_libraries(ZRenderer PRIVATE TBB::tbb TBB::tbbmalloc TBB::tbbmalloc_proxy)
target_link_libraries(ZRenderer PRIVATE assimp::assimp)
target_include_directories(ZRenderer PRIVATE ${STB_INCLUDE_DIRS})