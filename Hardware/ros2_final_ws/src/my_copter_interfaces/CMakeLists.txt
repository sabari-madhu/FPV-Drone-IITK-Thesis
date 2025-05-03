cmake_minimum_required(VERSION 3.8)
project(my_copter_interfaces)

#
# 1.  Find the generator package that will turn .action files into code
#
find_package(rosidl_default_generators REQUIRED)

#
# 2.  List your interface files
#     (Create the “action” folder and place Move.action inside it)
#
set(action_files
  "action/Move.action"     # <‑‑ add more .action/.msg/.srv here if needed
)

#
# 3.  Generate the code (C++, Python, etc.)
#
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES builtin_interfaces        # add geometry_msgs std_msgs … if referenced in your files
)

#
# 4.  Export runtime dependency so other packages can `import` / `#include`
#
ament_export_dependencies(rosidl_default_runtime)

#
# 5.  Standard ament macro
#
ament_package()
