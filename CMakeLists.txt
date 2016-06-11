cmake_minimum_required(VERSION 3.1)
project(PathPlanner CXX)

# Setup external libs
execute_process(COMMAND git submodule update --init)

# Import packages
find_package(GNUPLOT REQUIRED)
if(${GNUPLOT_FOUND})
    add_definitions(-DENABLE_GNUPLOT)
    add_definitions(-DGNUPLOT_BIN="${GNUPLOT_EXECUTABLE}")
else()
    message(WARNING "Gnuplot not found! No chart output will be provided")
endif()

# Setup C++ sources
file(GLOB BIN_SOURCES src/*.cpp)
set(DOCOPT_SOURCES libs/docopt/docopt.cpp)

# Setup include directories
include_directories(src libs libs/atoms/include)

# Setup compiler
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED on)

# Suppression of "unknown pragma" warning on GCC due to Docopt
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unknown-pragmas") # Code uses #pragma mark
endif()

# Setup binaries
add_executable(pathplanner demo/demo.cpp ${BIN_SOURCES} ${DOCOPT_SOURCES})
file(COPY demo/style.gnuplot DESTINATION .)
