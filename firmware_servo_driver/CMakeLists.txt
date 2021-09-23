cmake_minimum_required(VERSION 3.5)
project(firmware_servo_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}/home/yang/pigpio-master/util)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(pigpio REQUIRED)

add_library(firmware_servo_driver STATIC src/pwm_servo_driver.cpp)

target_include_directories(firmware_servo_driver PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(firmware_servo_driver ${pigpio_LIBRARY} ${pigpiod_if_LIBRARY} ${pigpiod_if2_LIBRARY} ${pigpiod_if_LIBRARY} pigpiod_if2)

ament_target_dependencies(
  firmware_servo_driver
  pigpio
  
)

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS firmware_servo_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)





add_executable(remote_test src/remote_test.cpp)

ament_target_dependencies(remote_test pigpio)

target_include_directories(remote_test PRIVATE ${pigpio_INCLUDE_DIR}/include)

target_link_libraries(remote_test ${pigpio_LIBRARY} ${pigpiod_if_LIBRARY} ${pigpiod_if2_LIBRARY} ${pigpiod_if_LIBRARY} pigpiod_if2 firmware_servo_driver)

install(TARGETS
remote_test
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  firmware_servo_driver
)

ament_package()