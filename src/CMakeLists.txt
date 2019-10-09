set(SOURCE_FILE_IMAGE_TO_VECTOR ${SOURCE_FILE_IMAGE_TO_VECTOR} ${PROJECT_SOURCE_DIR}/src/image_to_vector/cross_points_detector.cc)
set(SOURCE_FILE_IMAGE_TO_VECTOR ${SOURCE_FILE_IMAGE_TO_VECTOR} ${PROJECT_SOURCE_DIR}/src/image_to_vector/graph_creator.cc)
set(SOURCE_FILE_IMAGE_TO_VECTOR ${SOURCE_FILE_IMAGE_TO_VECTOR} ${PROJECT_SOURCE_DIR}/src/image_to_vector/image_to_vector.cc)
set(SOURCE_FILE_IMAGE_TO_VECTOR ${SOURCE_FILE_IMAGE_TO_VECTOR} ${PROJECT_SOURCE_DIR}/src/image_to_vector/keypoint.cc)
set(SOURCE_FILE_IMAGE_TO_VECTOR ${SOURCE_FILE_IMAGE_TO_VECTOR} ${PROJECT_SOURCE_DIR}/src/image_to_vector/line_segment.cc)

set(SOURCE_FILE_SHP ${SOURCE_FILE_SHP} ${PROJECT_SOURCE_DIR}/src/shp/get_shp_elements.cc)
set(SOURCE_FILE_SHP ${SOURCE_FILE_SHP} ${PROJECT_SOURCE_DIR}/src/shp/shp.cc)

set(SOURCE_FILE_GH ${SOURCE_FILE_GH} ${PROJECT_SOURCE_DIR}/src/geometric_hashing/geometric_hashing.cc)

set(SOURCE_FILE_LOCATOR ${SOURCE_FILE_LOCATOR} ${PROJECT_SOURCE_DIR}/src/locator/geo_locator.cc)
set(SOURCE_FILE_LOCATOR ${SOURCE_FILE_LOCATOR} ${PROJECT_SOURCE_DIR}/src/locator/geometric_hashing_locator.cc)

set(SOURCE_FILE_UTIL ${SOURCE_FILE_UTIL} ${PROJECT_SOURCE_DIR}/src/util/file_system.cc)
set(SOURCE_FILE_UTIL ${SOURCE_FILE_UTIL} ${PROJECT_SOURCE_DIR}/src/util/thin_image.cc)
set(SOURCE_FILE_UTIL ${SOURCE_FILE_UTIL} ${PROJECT_SOURCE_DIR}/src/util/draw_function.cc)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# auto geolocalize
add_executable(auto_geolocalize ${SOURCE_FILE_IMAGE_TO_VECTOR} 
                         ${SOURCE_FILE_GH} 
                         ${SOURCE_FILE_LOCATOR} 
                         ${SOURCE_FILE_UTIL} 
                         ${SOURCE_FILE_SHP}
                         ${PROJECT_SOURCE_DIR}/src/test/auto_geolocalize.cc)
TARGET_LINK_LIBRARIES(auto_geolocalize ${REQUIRED_LIBRARIES} )
