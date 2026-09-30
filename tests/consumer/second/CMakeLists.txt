find_package(lrclexec CONFIG REQUIRED)
add_executable(second ../main.cpp)
target_link_libraries(second PRIVATE lrclexec::lrclexec)
