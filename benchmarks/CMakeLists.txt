find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
add_executable(lrclexec_load load.cpp)
target_link_libraries(lrclexec_load PRIVATE lrclexec::lrclexec
    example_interfaces::example_interfaces__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp)
target_compile_options(lrclexec_load PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Werror=return-type)
if(LRCLEXEC_BUILD_TESTS)
    add_executable(lrclexec_histogram_test histogram_test.cpp)
    target_link_libraries(lrclexec_histogram_test PRIVATE lrclexec::lrclexec)
    add_test(NAME measurement_histogram COMMAND lrclexec_histogram_test)
    foreach(mode IN ITEMS single multi events)
        add_test(NAME load_${mode}_smoke COMMAND ${UV_EXECUTABLE} run --no-project --no-managed-python
            ${CMAKE_CURRENT_SOURCE_DIR}/smoke.py $<TARGET_FILE:lrclexec_load> ${mode})
        set_tests_properties(load_${mode}_smoke PROPERTIES TIMEOUT 40
            ENVIRONMENT "ROS_DOMAIN_ID=230;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_LOG_DIR=/tmp/lrclexec-load-logs;UV_CACHE_DIR=/tmp/lrclexec-uv-cache;RCUTILS_LOGGING_USE_STDOUT=0"
            RESOURCE_LOCK lrclexec_load_dds LABELS "load")
    endforeach()
endif()
