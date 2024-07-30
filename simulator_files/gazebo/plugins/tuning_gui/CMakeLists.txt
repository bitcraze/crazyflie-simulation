cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

project(TuningPlugin)

set(CMAKE_AUTOMOC ON)

find_package(gz-sim8 REQUIRED COMPONENTS gui)

# Find gz-transport
find_package(gz-transport13 REQUIRED COMPONENTS log parameters)

#--------------------------------------
# Find gz-msgs
find_package(gz-msgs10 REQUIRED)

QT5_ADD_RESOURCES(resources_RCC ${PROJECT_NAME}.qrc)

add_library(${PROJECT_NAME} SHARED
  ${PROJECT_NAME}.cc
  ${resources_RCC}
)
target_link_libraries(${PROJECT_NAME}
  PRIVATE gz-sim8::gui
)
