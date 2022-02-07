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
