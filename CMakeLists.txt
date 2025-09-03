cmake_minimum_required(VERSION 3.12)
project(CH_Demo)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add executable
add_executable(ch_demo
    src/main.cpp
    src/experiment1.cpp
    src/experiment2.cpp
    src/experiment3.cpp
    src/data_parser.cpp
    src/unified_contraction_hierarchy.cpp
    src/customizable_contraction_hierarchy.cpp
)

# When using WSL-built libraries on Windows, we may need some special handling
if(WIN32)
    # Add Windows-specific libraries if needed
    target_link_libraries(ch_demo PRIVATE wsock32 ws2_32)
    
    # Some build configurations might need this to handle Linux-built libraries
    if(MSVC)
        set_target_properties(ch_demo PROPERTIES LINK_FLAGS "/IGNORE:4049 /IGNORE:4217")
    endif()
endif()
