add_executable(oflow_3d_generator 
    main.cpp
    oflow_3d_generator.cpp
)

include_directories(oflow_3d_generator
#     ${PCL_INCLUDE_DIRS}/pcl-1.7
#     ${OpenCV_INCLUDE_DIR}
#     ${Boost_INCLUDE_DIR}
#     ${LIBELAS_INCLUDE_DIRS}
#     /opt/ros/hydro/include
)

target_link_libraries(oflow_3d_generator
#   ${EIGEN3_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenCV_LIBS}
# #   ${CUDA_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
)