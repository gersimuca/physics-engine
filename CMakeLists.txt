cmake_minimum_required(VERSION 3.16)
project(CarPhysicsEngine VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_FLAGS_DEBUG   "-g -O0 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Qt6
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Charts)
qt_standard_project_setup()

# Google Test
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    add_subdirectory(/usr/src/googletest ${CMAKE_BINARY_DIR}/googletest EXCLUDE_FROM_ALL)
endif()
enable_testing()

# Physics library (header-only style, compiled as static lib)
add_library(physics_core STATIC
        physics/Vector3.cpp
        physics/RigidBody.cpp
        physics/Tire.cpp
        physics/Suspension.cpp
        physics/Aerodynamics.cpp
        physics/Drivetrain.cpp
        physics/Chassis.cpp
        physics/Vehicle.cpp
        physics/PhysicsEngine.cpp
)

target_include_directories(physics_core PUBLIC
        ${CMAKE_SOURCE_DIR}
)

# Qt UI Application
qt_add_executable(car_physics_ui
        ui/main.cpp
        ui/MainWindow.cpp
        ui/MainWindow.h
        ui/SimulationView.cpp
        ui/SimulationView.h
        ui/TelemetryPanel.cpp
        ui/TelemetryPanel.h
        ui/DashboardWidget.cpp
        ui/DashboardWidget.h
        ui/VehicleControlPanel.cpp
        ui/VehicleControlPanel.h
)

target_link_libraries(car_physics_ui PRIVATE
        physics_core
        Qt6::Core
        Qt6::Widgets
        Qt6::Charts
)

set_target_properties(car_physics_ui PROPERTIES
        WIN32_EXECUTABLE TRUE
        MACOSX_BUNDLE    TRUE
)

# Test suite
add_executable(run_tests
        tests/test_vector3.cpp
        tests/test_tire.cpp
        tests/test_suspension.cpp
        tests/test_aerodynamics.cpp
        tests/test_drivetrain.cpp
        tests/test_chassis.cpp
        tests/test_vehicle.cpp
        tests/test_physics_engine.cpp
)

target_link_libraries(run_tests PRIVATE
        physics_core
        GTest::gtest
        GTest::gtest_main
)

target_include_directories(run_tests PRIVATE
        ${CMAKE_SOURCE_DIR}
)

include(GoogleTest)
gtest_discover_tests(run_tests)