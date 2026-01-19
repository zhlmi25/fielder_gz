cmake_minimum_required(VERSION 3.5)
project(fielder_gz)

find_package(ament_cmake REQUIRED)

# Install URDF and meshes
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Optional: install config files (e.g., RViz)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY world/
  DESTINATION share/${PROJECT_NAME}/world
)
install(
  DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
)

ament_package()
