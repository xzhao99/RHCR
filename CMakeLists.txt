project(lifelong)

cmake_minimum_required (VERSION 3.20)
set(CMAKE_CXX_STANDARD 17) # use std::filesystem

IF(NOT CMAKE_BUILD_TYPE)
    #SET(CMAKE_BUILD_TYPE "DEBUG")
    SET(CMAKE_BUILD_TYPE "RELEASE")
    #SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
    #SET(CMAKE_BUILD_TYPE "MINSIZEREL")
ENDIF()

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")

# Find Boost
find_package( Boost REQUIRED COMPONENTS program_options system)
message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")


add_executable(lifelong ${SOURCES} )

target_include_directories(lifelong PRIVATE ${Boost_INCLUDE_DIRS} )
target_link_libraries(lifelong PRIVATE ${Boost_LIBRARIES})