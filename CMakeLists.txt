cmake_minimum_required(VERSION 3.16)
project(ballistic6dof LANGUAGES CXX)

#using C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#making the executable
add_executable(ballistic6dof src/main.cpp src/atmosphere.cpp src/aero.cpp src/projectile.cpp src/solver.cpp src/report.cpp)

target_include_directories(ballistic6dof PRIVATE include)
#eigen — set EIGEN_DIR to your local Eigen path, e.g. cmake .. -DEIGEN_DIR=/path/to/eigen
set(EIGEN_DIR "" CACHE PATH "Path to Eigen header-only library")
if(EIGEN_DIR)
    target_include_directories(ballistic6dof PRIVATE "${EIGEN_DIR}")
else()
    find_package(Eigen3 REQUIRED NO_MODULE)
    target_link_libraries(ballistic6dof PRIVATE Eigen3::Eigen)
endif()