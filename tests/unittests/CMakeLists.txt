add_executable(testPushBroomProjections testPushBroomProjections.cpp)
target_link_libraries(testPushBroomProjections ${PROCESSING_LIB} Qt${QT_VERSION_MAJOR}::Test)
add_test(TestPushBroomProjections testPushBroomProjections)

add_executable(testGeoTools testGeoTools.cpp)
target_link_libraries(testGeoTools ${GEO_LIB} Qt${QT_VERSION_MAJOR}::Test)
add_test(TestGeoTools testGeoTools)

add_executable(testPushBroomRelativeOffsetsEstimators testPushBroomRelativeOffsetsEstimators.cpp)
target_link_libraries(testPushBroomRelativeOffsetsEstimators ${PROCESSING_LIB} Qt${QT_VERSION_MAJOR}::Test)
add_test(TestPushBroomRelativeOffsetsEstimators testPushBroomRelativeOffsetsEstimators)

