catkin_add_gtest( simalution_data_test simalution_data_test.cc ${proc_navigation_FILES} )
target_link_libraries(simalution_data_test ${catkin_LIBRARIES}
    ${lib_atlas_LIBRARIES} ${Eigen_LIBRARIES} )
