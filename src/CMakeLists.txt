set(HEADER_LIST "${projection_SOURCE_DIR}/include/projection/camera.hpp")
add_library(${PROJECT_NAME} STATIC camera.cpp ${HEADER_LIST})
target_include_directories(${PROJECT_NAME} PUBLIC ../include/projection)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
