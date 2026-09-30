add_executable(lrclexec_tests integration.cpp)
target_link_libraries(lrclexec_tests PRIVATE lrclexec::lrclexec example_interfaces::example_interfaces__rosidl_typesupport_cpp)
target_compile_options(lrclexec_tests PRIVATE -Wall -Wextra -Wpedantic -Werror=return-type)
add_test(NAME lrclexec_integration COMMAND lrclexec_tests)
set_tests_properties(lrclexec_integration PROPERTIES TIMEOUT 60
    ENVIRONMENT "ROS_DOMAIN_ID=211;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_LOG_DIR=/tmp/lrclexec-test-logs")
