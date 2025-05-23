cmake_minimum_required(VERSION 3.0.2)
project(pacecat_m300)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
)

#find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  CustomMsg.msg
  CustomPoint.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}

)
set(PACECAT_SDK_PATH "./sdk/")
FILE(GLOB PACECAT_SDK_SRC
   "${PACECAT_SDK_PATH}/*.cpp",
   "${PACECAT_SDK_PATH}/*.c"
)
add_executable(${PROJECT_NAME}_node  ./src/node.cpp ${PACECAT_SDK_SRC})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

