# CMakeList.txt : CMake project for engine, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.3.1)

find_package(SDL2 REQUIRED)


# Add source to this project's executable.
add_executable (engine WIN32 "main.cpp" "main.h" "ball.h" "ball.cpp")

target_link_libraries(engine SDL2::SDL2 SDL2::SDL2main)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET engine PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
