cmake_minimum_required(VERSION 3.16)
project(physics_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# A plain `cmake -B build` used to produce an unoptimized build. Default to Release.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(PHYS_WERROR "Treat warnings as errors" OFF)
option(PHYS_SANITIZE "Build with AddressSanitizer + UndefinedBehaviorSanitizer" OFF)
option(PHYS_BUILD_DEMO "Build the SDL2 demo if SDL2 and SDL2_ttf are found" ON)

# Warning flags applied to every first-party target (not to third_party/).
add_library(phys_warnings INTERFACE)
if(MSVC)
    target_compile_options(phys_warnings INTERFACE /W4)
else()
    target_compile_options(phys_warnings INTERFACE -Wall -Wextra -Wpedantic)
    if(PHYS_WERROR)
        target_compile_options(phys_warnings INTERFACE -Werror)
    endif()
endif()

if(PHYS_SANITIZE)
    # float-cast-overflow is not part of GCC's -fsanitize=undefined group, so name it.
    set(PHYS_SAN_FLAGS -fsanitize=address,undefined,float-cast-overflow
                       -fno-sanitize-recover=all -fno-omit-frame-pointer)
    add_compile_options(${PHYS_SAN_FLAGS})
    add_link_options(${PHYS_SAN_FLAGS})
endif()

# ---------------------------------------------------------------------------
# physics_core: the simulation. No SDL, no I/O.
add_library(physics_core STATIC
    core/src/scene.cpp
    core/src/world.cpp
    core/src/scenes.cpp
)
target_include_directories(physics_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core/include)
target_link_libraries(physics_core PRIVATE phys_warnings)

# ---------------------------------------------------------------------------
# physics_tests: doctest 2.4.11 (vendored single header, third_party/doctest).
add_executable(physics_tests
    tests/test_main.cpp
    tests/test_world.cpp
    tests/test_audit_bugs.cpp
    tests/test_springs.cpp
    tests/test_scenes.cpp
)
target_include_directories(physics_tests SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
target_link_libraries(physics_tests PRIVATE physics_core phys_warnings)

enable_testing()
add_test(NAME physics_tests COMMAND physics_tests)

# ---------------------------------------------------------------------------
# Benchmarks (never run in CI).
string(TOUPPER "${CMAKE_BUILD_TYPE}" _bt)
set(PHYS_BUILD_INFO "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}, ${CMAKE_BUILD_TYPE}: ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${_bt}}")
string(STRIP "${PHYS_BUILD_INFO}" PHYS_BUILD_INFO)
add_executable(physics_bench bench/bench_main.cpp)
target_link_libraries(physics_bench PRIVATE physics_core phys_warnings)
target_compile_definitions(physics_bench PRIVATE PHYS_BENCH_BUILD_INFO="${PHYS_BUILD_INFO}")


# ---------------------------------------------------------------------------
# physics_demo: optional SDL2 front end.
if(PHYS_BUILD_DEMO)
    if(APPLE)
        execute_process(COMMAND brew --prefix OUTPUT_VARIABLE HOMEBREW_PREFIX
                        OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(HOMEBREW_PREFIX)
            list(APPEND CMAKE_PREFIX_PATH "${HOMEBREW_PREFIX}")
        endif()
    endif()
    find_package(SDL2 QUIET)
    find_package(SDL2_ttf QUIET)
    if(SDL2_FOUND AND SDL2_ttf_FOUND)
        add_executable(physics_demo demo/main.cpp demo/graphics.cpp)
        target_link_libraries(physics_demo PRIVATE physics_core phys_warnings)
        if(TARGET SDL2::SDL2)
            target_link_libraries(physics_demo PRIVATE SDL2::SDL2)
        else()
            target_include_directories(physics_demo PRIVATE ${SDL2_INCLUDE_DIRS})
            target_link_libraries(physics_demo PRIVATE ${SDL2_LIBRARIES})
        endif()
        if(TARGET SDL2_ttf::SDL2_ttf)
            target_link_libraries(physics_demo PRIVATE SDL2_ttf::SDL2_ttf)
        else()
            target_link_libraries(physics_demo PRIVATE SDL2_ttf)
        endif()
        if(APPLE AND HOMEBREW_PREFIX)
            target_include_directories(physics_demo PRIVATE "${HOMEBREW_PREFIX}/include")
        endif()
    else()
        message(STATUS "SDL2/SDL2_ttf not found: skipping physics_demo")
    endif()
endif()
