cmake_minimum_required(VERSION 3.8)
project(pacecat_m300_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pacecat_m300_inter REQUIRED)

FILE(GLOB PACECAT_SDK_SRC  "./sdk/*.cpp" "./sdk/*.h")

add_executable(driver src/driver.cpp ${PACECAT_SDK_SRC})
add_executable(client src/client.cpp ${PACECAT_SDK_SRC})
target_include_directories(driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  driver
  "rclcpp"
  "sensor_msgs"
  "pacecat_m300_inter"
)
ament_target_dependencies(
  client
  "rclcpp"
  "sensor_msgs"
  "pacecat_m300_inter"
)

install(TARGETS driver
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
