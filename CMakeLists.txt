cmake_minimum_required(VERSION 3.23)

project(RobotiC LANGUAGES CXX VERSION 1.0.0)
configure_file(RobotiCConfig.h.in RobotiC.h)

if(MSVC)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED)

target_sources(${PROJECT_NAME}
    PRIVATE
        transforms.cpp
    PUBLIC
        transforms.h
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
)


include_directories(${GTEST_INCLUDE_DIRS})

add_executable(robotic_unittests)
target_sources(robotic_unittests
    PRIVATE
        transforms_unittests.cpp
    )
target_link_libraries(robotic_unittests PRIVATE
          ${GTEST_LIBRARIES}
     ${GTEST_MAIN_LIBRARIES}
    ${PROJECT_NAME}
    )
