cmake_minimum_required(VERSION 3.0)
project(aeb-tracker)

set(CMAKE_CXX_STANDARD 17)

# Set the path to the YAML-CPP library and include directories
find_package(yaml-cpp REQUIRED)

# Add the path to the directory containing FindOpenCV.cmake to CMAKE_MODULE_PATH
list(APPEND CMAKE_MODULE_PATH "/path/to/cmake/modules")

# Import OpenCV
find_package(OpenCV REQUIRED)

add_executable(aeb-tracker kalman.cpp main.cpp)

set(EXECUTABLES_LIST
    aeb-tracker
)

foreach(EXEC_NAME ${EXECUTABLES_LIST})
    target_include_directories(${EXEC_NAME} PRIVATE
        include
        ${MODULE_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}  # Add YAML-CPP include directory
    )

    if(UNIX) 
        message(STATUS "Unix detected")
        target_link_libraries(${EXEC_NAME}
            ${MODULE_LIBS}
            ${OpenCV_LIBS}
            pthread
            yaml-cpp
        )
    endif()

    if(WIN32) 
        message(STATUS "Windows detected")
        target_link_libraries(${EXEC_NAME}
            ${MODULE_LIBS}
            ${OpenCV_LIBS}
            ${YAML_CPP_LIBRARY}  # Link against YAML-CPP library
            pthreadVC3
        )
    endif()
endforeach()
