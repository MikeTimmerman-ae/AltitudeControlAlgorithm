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

# Add controllerB.cpp

add_library(controllerB controllerB.cpp)

target_include_directories(controllerB
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/qpOASES/include
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(controllerB
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/qpOASES
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(controllerB eigen qpOASES)

# Add dynamics.cpp

add_library(dynamics dynamics.cpp)

target_include_directories(dynamics
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(dynamics
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(dynamics eigen)


# Add rbf.cpp

add_library(rbf rbf.cpp)

target_include_directories(rbf
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(rbf
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(rbf eigen)


# Add helpers.cpp

add_library(helpers helpers.cpp)

target_include_directories(helpers
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/qpOASES/include
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_directories(helpers
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/qpOASES
    PUBLIC ${CMAKE_SOURCE_DIR}/libraries/eigen
)

target_link_libraries(helpers eigen qpOASES)