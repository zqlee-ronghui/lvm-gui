cmake_minimum_required(VERSION 3.10)
project(lvm-gui)

option(USE_PCL_VIEWER OFF)

set(CMAKE_CXX_STANDARD 14)

set(AUTOGEN_TARGETS_FOLDER ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(USE_PCL_VIEWER)
  add_definitions(-DUSE_PCL_VIEWER)
endif()

#find_package(VTK REQUIRED)
find_package(ProtoBuf REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Sql REQUIRED)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS} 
    ${lvm-gui_BINARY_DIR}/${PROJECT_NAME}_autogen/include)

#include(${VTK_USE_FILE})

add_subdirectory(third-party)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third-party/json/include)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/mainwindow.cpp
    src/config.cpp
    src/databaseui.cpp
    src/about.cpp
    proto/message.pb.cc)
target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::Network
    Qt5::Sql
#    ${VTK_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    mysqlclient
    nlohmann_json)
