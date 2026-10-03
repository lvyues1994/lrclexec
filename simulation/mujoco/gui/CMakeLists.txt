find_package(Qt5 5.15 REQUIRED COMPONENTS Gui Widgets)
add_library(mujoco_gui_components OBJECT GuiClient.cpp SceneView.cpp Workbench.cpp)
target_include_directories(mujoco_gui_components PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(mujoco_gui_components PUBLIC QT_NO_KEYWORDS)
target_link_libraries(mujoco_gui_components PUBLIC mujoco_physics Qt5::Widgets Qt5::Gui
    PRIVATE mujoco::mujoco lrclexec::lrclexec
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp nav_msgs::nav_msgs__rosidl_typesupport_cpp
    visualization_msgs::visualization_msgs__rosidl_typesupport_cpp std_msgs::std_msgs__rosidl_typesupport_cpp
    std_srvs::std_srvs__rosidl_typesupport_cpp)
target_compile_options(mujoco_gui_components PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
add_executable(mujoco_gui main.cpp)
target_link_libraries(mujoco_gui PRIVATE mujoco_gui_components lrclexec::lrclexec)
set_target_properties(mujoco_gui PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/..)
if(LRCLEXEC_BUILD_TESTS)
    find_package(Qt5 REQUIRED COMPONENTS Test)
    add_executable(mujoco_gui_test gui_test.cpp)
    target_link_libraries(mujoco_gui_test PRIVATE mujoco_gui_components Qt5::Test)
    add_executable(mujoco_gui_live_test gui_live_test.cpp)
    target_link_libraries(mujoco_gui_live_test PRIVATE mujoco_gui_components Qt5::Test lrclexec::lrclexec)
    add_executable(mujoco_gui_client_test client_test.cpp)
    target_link_libraries(mujoco_gui_client_test PRIVATE mujoco_gui_components mujoco_goal_inbox mujoco_scene_protocol
        std_srvs::std_srvs__rosidl_typesupport_cpp)
    add_test(NAME mujoco_gui_client COMMAND mujoco_gui_client_test)
    set_tests_properties(mujoco_gui_client PROPERTIES TIMEOUT 20 ENVIRONMENT "ROS_DOMAIN_ID=198;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
