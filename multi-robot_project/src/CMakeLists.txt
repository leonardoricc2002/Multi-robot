cmake_minimum_required(VERSION 3.8)
project(multi-robot_project)

# Standard C++ settings
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

# Trova i pacchetti richiesti
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED) # AGGIORNATO: Usa l'integrazione Gazebo Ignition


install(DIRECTORY world
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY materials
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
