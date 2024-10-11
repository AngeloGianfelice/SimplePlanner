add_executable(simple_planner simple_planner.cpp)
target_link_libraries(simple_planner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

