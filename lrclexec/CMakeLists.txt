add_library(lrclexec_lexec_headers INTERFACE)
set_target_properties(lrclexec_lexec_headers PROPERTIES EXPORT_NAME lexec_headers)
target_link_libraries(lrclexec_lexec_headers INTERFACE "$<BUILD_INTERFACE:lexec::lexec>")
target_compile_features(lrclexec_lexec_headers INTERFACE cxx_std_17)
if(LRCLEXEC_BUNDLE_LEXEC_HEADERS)
    target_include_directories(lrclexec_lexec_headers INTERFACE
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/lrclexec/vendor>")
endif()

add_library(lrclexec STATIC src/ExecutionContext.cpp src/SignalStop.cpp)
add_library(lrclexec::lrclexec ALIAS lrclexec)
target_include_directories(lrclexec PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_compile_features(lrclexec PUBLIC cxx_std_17)
target_link_libraries(lrclexec PUBLIC lrclexec_lexec_headers rclcpp::rclcpp rclcpp_action::rclcpp_action Threads::Threads)
target_compile_options(lrclexec PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror=return-type)
if(LRCLEXEC_SANITIZE)
    target_compile_options(lrclexec PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(lrclexec PUBLIC -fsanitize=address,undefined)
endif()
if(LRCLEXEC_INSTALL)
    install(TARGETS lrclexec lrclexec_lexec_headers EXPORT lrclexecTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    if(LRCLEXEC_BUNDLE_LEXEC_HEADERS)
        install(DIRECTORY "${LRCLEXEC_LEXEC_SOURCE_DIR}/include/lexec"
            DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/lrclexec/vendor")
    endif()
endif()
