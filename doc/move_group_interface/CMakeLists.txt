add_executable(move_group_interface_cpp src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_cpp DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

