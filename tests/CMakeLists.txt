add_executable(lrclexec_tests integration.cpp)
target_link_libraries(lrclexec_tests PRIVATE lrclexec::lrclexec example_interfaces::example_interfaces__rosidl_typesupport_cpp)
target_compile_options(lrclexec_tests PRIVATE -Wall -Wextra -Wpedantic -Werror=return-type)
add_test(NAME lrclexec_integration COMMAND lrclexec_tests)
set_tests_properties(lrclexec_integration PROPERTIES TIMEOUT 60
    ENVIRONMENT "ROS_DOMAIN_ID=211;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_LOG_DIR=/tmp/lrclexec-test-logs")

find_package(rosgraph_msgs REQUIRED)
add_executable(lrclexec_adapter_tests adapters.cpp)
target_link_libraries(lrclexec_adapter_tests PRIVATE lrclexec::lrclexec
    example_interfaces::example_interfaces__rosidl_typesupport_cpp
    rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_cpp)
target_compile_options(lrclexec_adapter_tests PRIVATE -Wall -Wextra -Wpedantic -Werror=return-type)
add_test(NAME service_ros_time_adapters COMMAND lrclexec_adapter_tests)
set_tests_properties(service_ros_time_adapters PROPERTIES TIMEOUT 30
    ENVIRONMENT "ROS_DOMAIN_ID=217;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_LOG_DIR=/tmp/lrclexec-adapter-logs")

add_executable(lrclexec_signal_unit signal_unit.cpp)
target_link_libraries(lrclexec_signal_unit PRIVATE lrclexec::lrclexec)
add_test(NAME signal_mask_lifetime COMMAND lrclexec_signal_unit)
set_tests_properties(signal_mask_lifetime PROPERTIES TIMEOUT 10)

add_executable(lrclexec_shutdown_fixture shutdown_fixture.cpp)
target_link_libraries(lrclexec_shutdown_fixture PRIVATE lrclexec::lrclexec
    example_interfaces::example_interfaces__rosidl_typesupport_cpp)
find_program(UV_EXECUTABLE uv REQUIRED)
add_test(NAME signal_scope_shutdown COMMAND ${UV_EXECUTABLE} run --no-project --no-managed-python
    ${CMAKE_CURRENT_SOURCE_DIR}/signal_shutdown.py $<TARGET_FILE:lrclexec_shutdown_fixture>)
set_tests_properties(signal_scope_shutdown PROPERTIES TIMEOUT 30
    ENVIRONMENT "ROS_DOMAIN_ID=213;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_LOG_DIR=/tmp/lrclexec-shutdown-logs;UV_CACHE_DIR=/tmp/lrclexec-uv-cache")
if(LRCLEXEC_INSTALL AND LRCLEXEC_BUNDLE_LEXEC_HEADERS)
    add_test(NAME installed_package_consumer COMMAND ${UV_EXECUTABLE} run --no-project --no-managed-python
        ${CMAKE_CURRENT_SOURCE_DIR}/package_consumer.py ${CMAKE_COMMAND} ${PROJECT_BINARY_DIR}
        ${PROJECT_SOURCE_DIR} ${LRCLEXEC_LEXEC_SOURCE_DIR})
    set_tests_properties(installed_package_consumer PROPERTIES TIMEOUT 180
        ENVIRONMENT "ROS_DOMAIN_ID=215;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_LOG_DIR=/tmp/lrclexec-consumer-logs;UV_CACHE_DIR=/tmp/lrclexec-uv-cache")
endif()
