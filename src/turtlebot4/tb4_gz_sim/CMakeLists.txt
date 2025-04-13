cmake_minimum_required(VERSION 3.8)
project(tb4_gz_sim)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY configs launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
