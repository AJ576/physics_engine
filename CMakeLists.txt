cmake_minimum_required(VERSION 3.16)
project(physics_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# Help CMake find Homebrew libraries on macOS
if(APPLE)
    execute_process(COMMAND brew --prefix OUTPUT_VARIABLE HOMEBREW_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(HOMEBREW_PREFIX)
        list(APPEND CMAKE_PREFIX_PATH "${HOMEBREW_PREFIX}")
    endif()
endif()

# Find SDL2 and SDL2_ttf
find_package(SDL2 REQUIRED)
find_package(SDL2_ttf REQUIRED)

# Ensure include path is set (SDL2/SDL2_ttf headers live in include/SDL2/)
set(EXTRA_INCLUDES)
if(APPLE AND HOMEBREW_PREFIX)
    list(APPEND EXTRA_INCLUDES "${HOMEBREW_PREFIX}/include")
endif()

# Source files
set(SOURCES
    main.cpp
    physics.cpp
    graphics.cpp
    rigidBody.cpp
    spring.cpp
)

add_executable(physics_engine ${SOURCES})

target_include_directories(physics_engine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EXTRA_INCLUDES}
)

# Link SDL2 libraries - use imported targets when available, else fallback
if(TARGET SDL2::SDL2)
    target_link_libraries(physics_engine PRIVATE SDL2::SDL2)
else()
    target_include_directories(physics_engine PRIVATE ${SDL2_INCLUDE_DIRS})
    target_link_libraries(physics_engine PRIVATE ${SDL2_LIBRARY})
endif()

if(TARGET SDL2_ttf::SDL2_ttf)
    target_link_libraries(physics_engine PRIVATE SDL2_ttf::SDL2_ttf)
else()
    target_include_directories(physics_engine PRIVATE ${SDL2_TTF_INCLUDE_DIRS})
    target_link_libraries(physics_engine PRIVATE ${SDL2_TTF_LIBRARY})
endif()
