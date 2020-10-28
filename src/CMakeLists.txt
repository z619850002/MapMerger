ADD_LIBRARY( myslam SHARED
	./keyframe/keyframe.cpp
	./mappoint/mappoint.cpp
	./camera/camera.cpp
	./map/map.cpp
	./simulator/map_simulator.cpp
	./realpoint/realpoint.cpp
)

TARGET_LINK_LIBRARIES( myslam
	${THIRD_PARTY_LIBS}
)