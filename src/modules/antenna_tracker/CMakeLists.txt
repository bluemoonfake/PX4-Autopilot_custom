px4_add_module(
	MODULE modules__antenna_tracker
	MAIN antenna_tracker
	SRCS
		antenna_tracker_main.cpp
		antenna_tracker.hpp
		tracker_geo.cpp
		tracker_geo.hpp
		tracker_controller.hpp
		tracker_servo_mapper.hpp
		tracker_setpoint_planner.hpp
	DEPENDS
		px4_work_queue
	)

px4_add_unit_gtest(
	SRC TrackerMathTest.cpp
	EXTRA_SRCS tracker_geo.cpp
)
