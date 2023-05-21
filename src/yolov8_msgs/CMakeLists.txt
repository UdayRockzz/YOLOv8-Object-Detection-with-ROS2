cmake_minimum_required(VERSION 3.5)
project(yolov8_msgs)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED) #since we are using standard messages we  need to find their definitionthe find package command enables thecompiler to find a package providedexternal to the project and load its package specific details
find_package(rosidl_default_generators REQUIRED) # it defines the list of default generators which ensures that they are installed as dependencies
rosidl_generate_interfaces(${PROJECT_NAME} #to generate a message we are using ros_idl_generate interfaces method
    "msg/InferenceResult.msg"
    "msg/Yolov8Inference.msg"
    DEPENDENCIES std_msgs
 )
ament_export_dependencies(rosidl_default_runtime) #the Ament export dependencies exports dependencies to Downstream packages this is necessary so that the user of the library does not have to call find package method for those dependencies too ros_idl_default runtime provides C make functionality for finding and adding runtime dependencies for ros idl packages

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
