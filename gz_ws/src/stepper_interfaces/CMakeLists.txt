cmake_minimum_required(VERSION 3.8)
project(stepper_interfaces)


find_package(rosidl_default_generators REQUIRED)


set(MSG_FILES
"msg/StepperCommand.msg"
)

set(ACTION_FILES
"action/SetDepth.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
${MSG_FILES}
${ACTION_FILES}
)


ament_package()
