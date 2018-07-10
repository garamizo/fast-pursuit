cmake_minimum_required(VERSION 2.8.3)
project(pursuit)

add_compile_options(-std=c++11)

include_directories(
  include
  include/GamePhysics
  include/Pursuit
)

add_library(pursuit_lib
  include/GamePhysics/Geometry3D.cpp
  include/GamePhysics/matrices.cpp
  include/GamePhysics/vectors.cpp
  include/Pursuit/Map.cpp
  include/Pursuit/Pursuit.cpp
)

add_executable(benchmark src/benchmark.cpp)
target_link_libraries(benchmark
  pursuit_lib
)

add_executable(tree_benchmark src/tree_benchmark.cpp)
target_link_libraries(tree_benchmark
  pursuit_lib
)
