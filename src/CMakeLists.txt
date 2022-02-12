set(HEADER_LIST "${projection_SOURCE_DIR}/include/projection/camera.hpp"
    # "${projection_SOURCE_DIR}/include/projection/utils.h"
    # "${projection_SOURCE_DIR}/include/projection/quaternion.h"
    # "${projection_SOURCE_DIR}/include/projection/se3.h"
)
add_library(${PROJECT_NAME} STATIC camera.cpp ${HEADER_LIST})
target_include_directories(${PROJECT_NAME} PUBLIC ../include/projection)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen rigidTransformation)
