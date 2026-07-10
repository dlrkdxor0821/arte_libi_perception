cmake_minimum_required(VERSION 3.16)
project(libi_perception_viewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Core Gui Qml Quick Network REQUIRED)

add_executable(viewer
    src/main.cpp
    src/PerceptionClient.cpp
    src/PerceptionClient.h
    src/FrameImageProvider.h
    qml.qrc
)
target_link_libraries(viewer PRIVATE
    Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick Qt5::Network)
