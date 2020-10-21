
ADD_EXECUTABLE(hello_test hello_test.cpp)
TARGET_LINK_LIBRARIES( hello_test myslam ${THIRD_PARTY_LIBS})


ADD_EXECUTABLE(map_test map_test.cpp)
TARGET_LINK_LIBRARIES( map_test myslam ${THIRD_PARTY_LIBS})


ADD_EXECUTABLE(plotter_test plotter_test.cpp)
TARGET_LINK_LIBRARIES( plotter_test myslam ${THIRD_PARTY_LIBS})

