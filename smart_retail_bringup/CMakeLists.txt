cmake_minimum_required(VERSION 3.8)
project(smart_retail_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files (even if empty, ensure index is created)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Explicitly install package.xml so ament indexes it
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
