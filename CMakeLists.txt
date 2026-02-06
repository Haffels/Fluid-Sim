cmake_minimum_required(VERSION 3.20)
project(fluidSim)

set(CMAKE_CXX_STANDARD 17)

# Set SFML 2 paths (Homebrew)
set(SFML_ROOT /opt/homebrew/opt/sfml@2)

# Find SFML components
find_package(SFML 2.5 COMPONENTS system window graphics REQUIRED)

# Add your executable
add_executable(fluidSim main.cpp)

# Link SFML libraries
target_link_libraries(fluidSim PRIVATE sfml-graphics sfml-window sfml-system)

# Include SFML headers
target_include_directories(fluidSim PRIVATE ${SFML_INCLUDE_DIR})