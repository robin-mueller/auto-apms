# ROS2 interfaces for testing
set(ACTION_SOURCES
    action/Test.action)
set(MESSAGE_SOURCES
    )
set(SERVICE_SOURCES
    )

rosidl_generate_interfaces(${PROJECT_NAME} ${ACTION_SOURCES} ${SERVICE_SOURCES} ${MESSAGE_SOURCES})