# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
#
cmake_minimum_required (VERSION 3.0)
#设置生成的VS项目文件结构和本地文件目录一样
function(assign_source_group)
    foreach(_source IN ITEMS ${ARGN})
        if (IS_ABSOLUTE "${_source}")
            file(RELATIVE_PATH _source_rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_source}")
        else()
            set(_source_rel "${_source}")
        endif()
        get_filename_component(_source_path "${_source_rel}" PATH)
        string(REPLACE "/" "\\" _source_path_msvc "${_source_path}")
        source_group("${_source_path_msvc}" FILES "${_source}")
    endforeach()
endfunction(assign_source_group)
 
function(my_add_executable)
    foreach(_source IN ITEMS ${ARGN})
        assign_source_group(${_source})
    endforeach()
    add_executable(${ARGV})
endfunction(my_add_executable)

#工程名称
project(M300_SDK)
#C++98  C99
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)
IF (UNIX)
# 添加对gdb的支持
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

ELSEIF (WIN32)
#多线程
set(CMAKE_CXX_FLAGS_DEBUG "/MT")
#强制文件输出
#set(CMAKE_EXE_LINKER_FLAGS_DEBUG       "/FORCE")
ENDIF ()
#指定生成debug模式
set(CMAKE_CONFIGURATION_TYPES Debug)
#所有的头文件目录
include_directories ("${PROJECT_SOURCE_DIR}/sdk")
include_directories ("${PROJECT_SOURCE_DIR}/samples")
include_directories ("${PROJECT_SOURCE_DIR}/3rdparty")
#加载指定目录下的指定格式文件集
file(GLOB_RECURSE SOURCE_FILES ${PROJECT_SOURCE_DIR}/sdk/*.cpp  ${PROJECT_SOURCE_DIR}/sdk/*.c  )
file(GLOB_RECURSE HEADER_FILES  ${PROJECT_SOURCE_DIR}/sdk/*.h  ${PROJECT_SOURCE_DIR}/sdk/*.hpp ${PROJECT_SOURCE_DIR}/3rdparty/readerwriterqueue/*.h  ${PROJECT_SOURCE_DIR}/3rdparty/readerwriterqueue/*.hpp)
IF (UNIX)
  #加载指定目录下需要删除的指定格式文件集
  #file(GLOB_RECURSE REMOVE_SOURCES  "${PROJECT_SOURCE_DIR}/sdk/service/LidarCheckService_win32.cpp")
  #file(GLOB_RECURSE REMOVE_HEADERS  "${PROJECT_SOURCE_DIR}/sdk/win32/*")
  #获取最终编译需要的文件集
  #list(REMOVE_ITEM SOURCE_FILES ${REMOVE_SOURCES})
  #list(REMOVE_ITEM HEADER_FILES ${REMOVE_HEADERS})
  
  source_group("Include" FILES ${HEADER_FILES})
  #编译生成可执行文件demo
  my_add_executable(PointCloudAndImu ${SOURCE_FILES} ${HEADER_FILES} ${PROJECT_SOURCE_DIR}/samples/PointCloudAndImu.cpp)
  my_add_executable(CommandControl ${SOURCE_FILES} ${HEADER_FILES} ${PROJECT_SOURCE_DIR}/samples/CommandControl.cpp)
  my_add_executable(Upgrade ${SOURCE_FILES} ${HEADER_FILES} ${PROJECT_SOURCE_DIR}/samples/Upgrade.cpp)
  my_add_executable(PlaybackPcap ${SOURCE_FILES} ${HEADER_FILES} ${PROJECT_SOURCE_DIR}/samples/PlaybackPcap.cpp)
  #添加链接库pthread
  target_link_libraries(PointCloudAndImu m pthread )
  target_link_libraries(CommandControl m pthread )
  target_link_libraries(Upgrade m pthread )
  target_link_libraries(PlaybackPcap m pthread )
ELSEIF (WIN32)
  #同上
  #file(GLOB_RECURSE REMOVE_SOURCES  "${PROJECT_SOURCE_DIR}/sdk/uart.c")
  #file(GLOB_RECURSE REMOVE_HEADERS  "${PROJECT_SOURCE_DIR}/sdk/linux/*")
  
  #list(REMOVE_ITEM SOURCE_FILES ${REMOVE_SOURCES})
  #list(REMOVE_ITEM HEADER_FILES ${REMOVE_HEADERS})
  
  source_group("Include" FILES ${HEADER_FILES})

  my_add_executable(PointCloudAndImu ${SOURCE_FILES} ${HEADER_FILES} ${PROJECT_SOURCE_DIR}/samples/PointCloudAndImu.cpp)
  my_add_executable(CommandControl ${SOURCE_FILES} ${HEADER_FILES} ${PROJECT_SOURCE_DIR}/samples/CommandControl.cpp)
  my_add_executable(Upgrade ${SOURCE_FILES} ${HEADER_FILES} ${PROJECT_SOURCE_DIR}/samples/Upgrade.cpp)
  my_add_executable(PlaybackPcap ${SOURCE_FILES} ${HEADER_FILES} ${PROJECT_SOURCE_DIR}/samples/PlaybackPcap.cpp)
ENDIF ()
