cmake_minimum_required(VERSION 3.5)
project(project_tracker)

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

add_executable(filter src/filter.cpp)

target_include_directories(filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# find dependencies

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(cv2 REQUIRED)

# set(CMAKE_PREFIX_PATH /usr/share/doc/libtorch3c2)
# find_package(Torch REQUIRED)

set(TORCH_BASE_PATH "$ENV{VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}")
message(STATUS TORCH_BASE_PATH=${TORCH_BASE_PATH})

set(Torch_INCLUDE_DIR "${TORCH_BASE_PATH}/include")
set(Torch_LIBRARIES "${TORCH_BASE_PATH}/lib")
# target_link_libraries is to be preferred
#link_directories(${Torch_LIBRARIES})
find_library(LIBRARY_TORCH TH HINTS ${Torch_LIBRARIES})

# set(cv2_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
find_package(OpenCV REQUIRED)
# set(CV2_BASE_PATH "$ENV{VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}")
# message(STATUS CV2_BASE_PATH=${CV2_BASE_PATH})

# set(CV2_INCLUDE_DIR "${CV2_BASE_PATH}/include")
# set(CV2_LIBRARIES "${CV2_BASE_PATH}/lib")
# find_library(LIBRARY_CV2 TH HINTS ${CV2_LIBRARIES})


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package (PCL 1.8 REQUIRED)
find_package(pcl_conversions REQUIRED)

ament_target_dependencies(filter rclcpp sensor_msgs)

set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_include_directories( filter PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries( filter ${PCL_LIBRARIES})

install(TARGETS filter
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/mock_pub.py
  ${PROJECT_NAME}/cluster.py
  ${PROJECT_NAME}/mock_image_pub.py
  ${PROJECT_NAME}/object_detection.py
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

ament_package()
