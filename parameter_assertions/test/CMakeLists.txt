add_rostest_gtest(test_assertions test_assertions.test test_assertions.cpp)
target_link_libraries(test_assertions ${catkin_LIBRARIES} parameter_assertions ${GTEST_LIBRARIES})
