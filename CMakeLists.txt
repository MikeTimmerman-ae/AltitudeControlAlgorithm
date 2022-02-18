cmake_minimum_required(VERSION 3.0.0)
project(ControlSoftware VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(ControlSoftware main.cpp)

add_subdirectory(src)

# DOWNLOAD ALL THE SUBMODULES
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

# CHECK ALL THE SUBMODULES
if (NOT EXISTS "&{PROJECT_SOURCE_DIR}/libraries/eigen/CMakeLists.text")
    message(FATA_ERROR "The eigen submodule was not downloaded! GIT_SUBMODULE was turned off or failed.")
endif()
if (NOT EXISTS "&{PROJECT_SOURCE_DIR}/libraries/qpOASES/CMakeLists.text")
    message(FATA_ERROR "The qpOASES submodule was not downloaded! GIT_SUBMODULE was turned off or failed.")
endif()

add_subdirectory(libraries/eigen)
add_subdirectory(libraries/qpOASES)

target_include_directories(${PROJECT_NAME}
    PUBLIC src
    PUBLIC libraries/eigen
    PUBLIC libraries/qpOASES/include
)

target_link_directories(${PROJECT_NAME}
    PUBLIC src
    PUBLIC libraries/eigen
    PUBLIC libraries/qpOASES
)

target_link_libraries(${PROJECT_NAME} eigen qpOASES controller dynamics rbf helpers controllerB simulate)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)