find_package(lrclexec CONFIG REQUIRED)
add_executable(first ../main.cpp)
target_link_libraries(first PRIVATE lrclexec::lrclexec)
