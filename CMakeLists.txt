cmake_minimum_required(VERSION 3.21)
project(wheel_control_sim VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
if(MSVC)
    # MSVC使用/std:c11编译同一份C99兼容源码。
    set(CMAKE_C_STANDARD 11)
endif()
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_library(wheel_core
    src/moving_average.c
    src/pi_controller.c
    src/wheel_model.c
    src/simulation.c
)
target_include_directories(wheel_core PUBLIC include)

add_executable(wheel_sim src/main.c)
target_link_libraries(wheel_sim PRIVATE wheel_core)
add_executable(wheel_tests tests/test_main.c)
target_link_libraries(wheel_tests PRIVATE wheel_core)

foreach(target wheel_core wheel_sim wheel_tests)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX /utf-8)
        target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
endforeach()
if(NOT MSVC)
    target_link_libraries(wheel_core PUBLIC m)
endif()

enable_testing()
add_test(NAME core_and_closed_loop COMMAND wheel_tests)
add_test(NAME simulation_csv COMMAND wheel_sim --output simulation.csv)
add_test(NAME reject_unknown_option COMMAND wheel_sim --unknown)
set_tests_properties(reject_unknown_option PROPERTIES WILL_FAIL TRUE)
add_test(NAME reject_missing_output_path COMMAND wheel_sim --output)
set_tests_properties(reject_missing_output_path PROPERTIES WILL_FAIL TRUE)
