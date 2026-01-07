cmake_minimum_required(VERSION 3.8)
project(jainbot)

# Recommended: C++17 for <filesystem> and modern APIs
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------- dependencies ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcutils REQUIRED)      
find_package(Threads REQUIRED)      # <-- add this

# libgpiod (C library) for EN/DIR GPIO
find_library(GPIOD_LIB gpiod REQUIRED)    # <-- add this

# -------- target (shared plugin) ----------
add_library(${PROJECT_NAME} SHARED
  src/jainbot_hw.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rcutils
)




# link libgpiod
target_link_libraries(${PROJECT_NAME}
  ${GPIOD_LIB}
  Threads::Threads                              # <-- add this
)

# -------- plugin export ----------
pluginlib_export_plugin_description_file(hardware_interface jainbot_plugin.xml)

# -------- install targets & assets ----------
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES jainbot_plugin.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf/   DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# If you added any helper scripts later, install them like:
# install(PROGRAMS scripts/robot_description_pub.py scripts/twist_to_stamped.py
#   DESTINATION lib/${PROJECT_NAME})

# -------- ament exports ----------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# -------- tests (unchanged) ----------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
