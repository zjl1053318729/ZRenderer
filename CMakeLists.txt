cmake_minimum_required(VERSION 3.24)

project(ZRenderer)

set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(assimp REQUIRED)

find_path(STB_INCLUDE_DIRS "stb_c_lexer.h")

add_executable(
        ZRenderer src/main/main.cpp
        src/core/camera.hpp
        src/core/camera.cpp
        src/core/Parser.hpp
        src/core/Parser.cpp
        src/core/scene.hpp
        src/core/scene.cpp
        src/accelerator/BVHAccel.hpp
        src/accelerator/BVHAccel.cpp
        src/core/geometry.hpp
        src/core/geometry.cpp
        src/core/ZRender.hpp
        src/core/interaction.hpp
        src/core/interaction.cpp
        src/core/primitive.hpp
        src/core/primitive.cpp
        src/shape/shape.hpp
        src/shape/shape.cpp
        src/core/transform.hpp
        src/core/transform.cpp
        src/shape/sphere.hpp
        src/shape/sphere.cpp
        src/shape/triangle.hpp
        src/shape/triangle.cpp
        src/shape/plyRead.hpp
        src/shape/plyRead.cpp
        src/core/spectrum.hpp
        src/core/spectrum.cpp
        src/core/perspective.hpp
        src/core/perspective.cpp
        src/sampler/sampler.hpp
        src/sampler/sampler.cpp
        src/core/orthographic.hpp
        src/core/orthographic.cpp
        src/sampler/halton.hpp
        src/sampler/halton.cpp
        src/sampler/RNG.hpp
        src/sampler/RNG.cpp
        src/sampler/LowDiscrepancy.hpp
        src/sampler/LowDiscrepancy.cpp
        src/sampler/sampling.hpp
        src/sampler/sampling.cpp
        src/Integrator/Integrator.hpp
        src/Integrator/Integrator.cpp
        src/core/buffer.hpp
        src/core/buffer.cpp
        src/sampler/clockRand.hpp
        src/sampler/clockRand.cpp
        src/material/reflection.hpp
        src/material/reflection.cpp
        src/material/material.hpp
        src/material/material.cpp
        src/material/MatteMaterial.hpp
        src/material/MatteMaterial.cpp
        src/texture/texture.hpp
        src/texture/texture.cpp
        src/texture/ConstantTexture.hpp
        src/texture/ConstantTexture.cpp src/core/ZRender.cpp)

target_include_directories(ZRenderer PRIVATE ${STB_INCLUDE_DIRS})
target_link_libraries(ZRenderer PRIVATE
        Eigen3::Eigen
        TBB::tbb TBB::tbbmalloc TBB::tbbmalloc_proxy
        assimp::assimp
        )