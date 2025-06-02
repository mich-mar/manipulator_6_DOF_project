cmake_minimum_required(VERSION 3.10)
project(ik_solver)

set(CMAKE_CXX_STANDARD 20)

# Dodaj ścieżkę do biblioteki Eigen
include_directories(/usr/include/eigen3)

add_executable(ik_solver main.cpp)
