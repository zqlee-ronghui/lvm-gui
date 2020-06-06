cmake_minimum_required(VERSION 3.10)
project(lvm-gui)

set(CMAKE_CXX_STANDARD 14)

set(AUTOGEN_TARGETS_FOLDER ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

#find_package(VTK REQUIRED)
find_package(ProtoBuf REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5Widgets REQUIRED )
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS} 
    ${lvm-gui_BINARY_DIR}/${PROJECT_NAME}_autogen/include)

#include(${VTK_USE_FILE})
message(STATUS ======================${lvm-gui_BINARY_DIR})


add_executable(${PROJECT_NAME}
    src/main.cpp
    src/mainwindow.cpp
    src/config.cpp
    proto/message.pb.cc)
target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::Network
#    ${VTK_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    mysqlclient)
