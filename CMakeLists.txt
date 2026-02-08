cmake_minimum_required(VERSION 3.16)
project(ballistic6dof LANGUAGES CXX)

#using C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#making the executable
add_executable(ballistic6dof src/main.cpp src/atmosphere.cpp src/aero.cpp src/projectile.cpp src/solver.cpp src/report.cpp)

target_include_directories(ballistic6dof PRIVATE include)
#eigen 
target_include_directories(ballistic6dof PRIVATE "C:/Users/camde/Desktop/eigen-5.0.0")