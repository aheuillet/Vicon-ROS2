cmake_minimum_required(VERSION 3.5)
project(nexus_interface)

add_executable(client main.cpp utils.cpp)

add_library(DataStream SHARED IMPORTED GLOBAL)
set_target_properties(DataStream PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../../DataStreamSDK/libViconDataStreamSDK_CPP.so)

target_link_libraries(client DataStream)