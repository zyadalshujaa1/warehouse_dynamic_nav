cmake_minimum_required(VERSION 3.8)
project(predictive_nav)

# The critic shares xtensor objects with the installed MPPI controller, so it
# must be compiled with the same xtensor/xsimd settings and CPU flags as
# nav2_mppi_controller (Humble 1.1.x CMakeLists.txt), otherwise the memory
# layout of the tensors can differ.
set(XTENSOR_USE_TBB 0)
set(XTENSOR_USE_OPENMP 0)
set(XTENSOR_USE_XSIMD 1)

find_package(ament_cmake REQUIRED)
find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mno-avx512f" COMPILER_SUPPORTS_AVX512)
check_cxx_compiler_flag("-msse4.2" COMPILER_SUPPORTS_SSE4)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)
if(COMPILER_SUPPORTS_AVX512)
  add_compile_options(-mno-avx512f)
endif()
if(COMPILER_SUPPORTS_SSE4)
  add_compile_options(-msse4.2)
endif()
if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()
if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()
add_compile_options(-O3 -finline-limit=10000000 -ffp-contract=fast -ffast-math -mtune=generic)

add_library(predictive_critics SHARED src/dynamic_obstacle_critic.cpp)
target_include_directories(predictive_critics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${xsimd_INCLUDE_DIRS})
target_compile_options(predictive_critics PUBLIC -fconcepts)
target_link_libraries(predictive_critics xtensor xtensor::optimize xtensor::use_xsimd)
ament_target_dependencies(predictive_critics
  rclcpp nav_msgs nav2_mppi_controller nav2_costmap_2d nav2_core pluginlib tf2 tf2_ros)

install(TARGETS predictive_critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(nav2_mppi_controller predictive_critics.xml)
ament_package()
