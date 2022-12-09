cmake_minimum_required(VERSION 3.23)

project(RobotiC LANGUAGES CXX VERSION 1.0.0)
configure_file(RobotiCConfig.h.in RobotiC.h)

if(MSVC)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

##### MAIN PROJECT #######

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME} SHARED)

target_sources(${PROJECT_NAME}
    PRIVATE
        transforms.cpp
        robotic_utils.cpp
        kinematics.cpp
    PUBLIC
        transforms.h
        robotic_utils.h
        kinematics.h
)

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
)

##### VIZUALIZATION PROJECT #######

find_package(Qt6 COMPONENTS Widgets Concurrent Gui 3DRender 3DInput 3DExtras 3DCore REQUIRED)

add_executable(RobotiCViz
    main.cpp
    vizscene.h
    vizscene.cpp
    se3viz.h
    se3viz.cpp
    trackballcameracontroller.cpp
    trackballcameracontroller.h
    serialarmviz.cpp
    serialarmviz.h
)

target_link_libraries(RobotiCViz
    Qt6::Widgets
    Qt6::Gui
    Qt6::3DRender
    Qt6::3DCore
    Qt6::3DInput
    Qt6::3DExtras
    ${PROJECT_NAME}
    Eigen3::Eigen
)

##### GTEST PROJECT #######

include_directories(${GTEST_INCLUDE_DIRS})

add_executable(robotic_unittests)
target_sources(robotic_unittests
    PRIVATE
        transforms_unittests.cpp
        utils_unittests.cpp
        toolsforunittests.cpp
        kinematics_unittests.cpp
    )
target_link_libraries(robotic_unittests PRIVATE
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
    Eigen3::Eigen
    ${PROJECT_NAME}
    )
