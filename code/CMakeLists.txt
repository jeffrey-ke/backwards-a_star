cmake_minimum_required(VERSION 3.5)
project(planning_pset1)

set(CMAKE_CXX_STANDARD 17)

# Add the main executable.
add_executable(run_test
	src/planner.cpp
	src/runtest.cpp)

target_compile_options(run_test
	PRIVATE
	$<$<CONFIG:Release>:-O3>
	$<$<CONFIG:Debug>:-g -O0 -fsanitize=address,undefined>
)
target_link_options(run_test
	PRIVATE
	$<$<CONFIG:Debug>:-fsanitize=address,undefined>
)
# Have the executable depend on the header files.
target_include_directories(run_test PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Find and link OpenCV
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
target_include_directories(run_test PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(run_test ${OpenCV_LIBS})
target_include_directories(run_test PRIVATE ${Boost_INCLUDE_DIRS})


# Define the absolute path to relevant directories.
target_compile_definitions(run_test PRIVATE MAPS_DIR="${CMAKE_SOURCE_DIR}/maps")
target_compile_definitions(run_test PRIVATE OUTPUT_DIR="${CMAKE_SOURCE_DIR}/output")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
