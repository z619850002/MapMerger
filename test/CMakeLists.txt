
ADD_EXECUTABLE(hello_test hello_test.cpp)
TARGET_LINK_LIBRARIES( hello_test myslam ${THIRD_PARTY_LIBS})


ADD_EXECUTABLE(map_test map_test.cpp)
TARGET_LINK_LIBRARIES( map_test myslam ${THIRD_PARTY_LIBS})


ADD_EXECUTABLE(plotter_test plotter_test.cpp)
TARGET_LINK_LIBRARIES( plotter_test myslam ${THIRD_PARTY_LIBS})

ADD_EXECUTABLE(map_simulator_test map_simulator_test.cpp)
TARGET_LINK_LIBRARIES( map_simulator_test myslam ${THIRD_PARTY_LIBS})


ADD_EXECUTABLE(optimizer_test optimizer_test.cpp)
TARGET_LINK_LIBRARIES( optimizer_test myslam ${THIRD_PARTY_LIBS})

