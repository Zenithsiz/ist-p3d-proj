cmake_minimum_required(VERSION 3.31.2)
project(lab1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options("-Wall" "-Wextra" "-Wpedantic" "-Wstrict-aliasing")
add_compile_options("-g" "-fno-omit-frame-pointer")
add_compile_options("-fopenmp")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	add_compile_options("-fsanitize=address,undefined,leak" "-fstrict-aliasing")
	add_link_options("-fsanitize=address,undefined,leak" "-fstrict-aliasing")
endif()

# Binary
file(GLOB_RECURSE sources_lib DistributionRayTracer/*.cpp)
add_executable("lab1" ${sources_lib} ${test_src})
target_include_directories("lab1" PRIVATE "DistributionRayTracer/")
target_link_libraries("lab1" -lomp -lGL -lGLEW -lGLU -lglut -lIL)
set_property(TARGET "lab1" PROPERTY CPP_STANDARD 23)
