cmake_minimum_required(VERSION 3.2)
project(pose2d)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
)

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Config
)
install(
    EXPORT ${PROJECT_NAME}Config
    DESTINATION share/${PROJECT_NAME}Config/cmake
    FILE ${PROJECT_NAME}Config.cmake
)