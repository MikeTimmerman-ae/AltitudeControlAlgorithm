##
##     Filename:  src/CMakeLists.txt
##     Author:    Mike Timmerman
##     Version:   3.0
##     Date:      2022
##

# Add controller.cpp

add_library(controller controller.cpp)

target_include_directories(controller
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/qpOASES/include
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(controller
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/qpOASES
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(controller eigen qpOASES)


# Add dynamics.cpp

add_library(dynamics dynamics.cpp)

target_include_directories(dynamics
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(dynamics
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(dynamics eigen)