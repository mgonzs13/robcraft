cmake_minimum_required(VERSION 3.10)
project(robcraft_bringup
    VERSION 0.1.0
    DESCRIPTION "Bringup package of Robcraft"
    LANGUAGES CXX
)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
